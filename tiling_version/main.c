// TPU v2 (tiled) bare-metal driver. PS writes DDR buffers + CSRs once, hits start, polls done.
#include <stdio.h>
#include <string.h>
#include "xparameters.h"
#include "xil_types.h"
#include "xil_io.h"
#include "xil_cache.h"

/* CSR base address: taken from xparameters.h if Vitis generated it for the block named tpu_0,
   otherwise falls back to the address set in the Address Editor (0x43C00000). */
#ifndef TPU_BASE
  #if   defined(XPAR_TPU_0_BASEADDR)
    #define TPU_BASE XPAR_TPU_0_BASEADDR
  #elif defined(XPAR_TPU_0_S_AXI_BASEADDR)
    #define TPU_BASE XPAR_TPU_0_S_AXI_BASEADDR
  #elif defined(XPAR_TPU_ACCELERATOR_TOP_0_BASEADDR)
    #define TPU_BASE XPAR_TPU_ACCELERATOR_TOP_0_BASEADDR
  #else
    #define TPU_BASE 0x43C00000u
  #endif
#endif

/* CSR map (axil_csr.sv) */
#define CSR_CTRL   0x00   /* W: bit0 = start (ignored while busy)           */
#define CSR_STATUS 0x04   /* R: [0]busy [1]done [2]mm2s_err [3]s2mm_err     */
#define CSR_A      0x08
#define CSR_W      0x0C
#define CSR_C      0x10
#define CSR_LDA    0x14   /* row strides in BYTES */
#define CSR_LDW    0x18
#define CSR_LDC    0x1C
#define CSR_M      0x20
#define CSR_K      0x24
#define CSR_NC     0x28
#define CSR_R      0x2C   /* rows per A block                               */
#define CSR_CYCLES 0x30
#define CSR_INFO   0x34   /* [7:0]=0x03 [15:8]=P [31:16]=MAX_R              */

/* Matrix size: M = K = N = DIM (must be a multiple of 8). Set DIM before the includes to change it,
   e.g.  #define DIM 8   for an 8x8 test. */
#ifndef DIM
#define DIM 32
#endif
#define M_DIM  DIM
#define K_DIM  DIM
#define N_DIM  DIM
#define R_ROWS (DIM < 16 ? DIM : 16)   /* rows per A block */

static u8  A[M_DIM][K_DIM] __attribute__((aligned(64)));
static u8  W[K_DIM][N_DIM] __attribute__((aligned(64)));
static u32 C[M_DIM][N_DIM] __attribute__((aligned(64)));

/* How many rows/cols of each matrix to print (clamped to DIM). */
#ifndef SHOW_N
#define SHOW_N 32
#endif

static void show_u8(const char *name, const u8 *m, int dim) {
    int n = SHOW_N < dim ? SHOW_N : dim;
    printf("--- %s (%dx%d) ---\n\r", name, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) printf("%3u ", m[i * dim + j]);
        printf("\n\r");
    }
    printf("\n\r");
}
static void show_u32(const char *name, const u32 *m, int dim) {
    int n = SHOW_N < dim ? SHOW_N : dim;
    printf("--- %s (%dx%d) ---\n\r", name, n, n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) printf("%6lu ", (unsigned long)m[i * dim + j]);
        printf("\n\r");
    }
    printf("\n\r");
}

static inline void   wr(u32 off, u32 v) { Xil_Out32(TPU_BASE + off, v); }
static inline u32    rd(u32 off)        { return Xil_In32(TPU_BASE + off); }

int main(void) {
    u32 info = rd(CSR_INFO);
    u32 P = (info >> 8) & 0xFF, max_r = info >> 16;
    printf("TPU v2 @0x%08x: P=%u MAX_R=%u\n\r", (unsigned)TPU_BASE, P, max_r);
    if ((info & 0xFF) != 0x03 || K_DIM % P || N_DIM % P || R_ROWS > max_r) {
        printf("ERROR: bad ID or dims not multiples of P\n\r"); return -1;
    }

    for (int i = 0; i < M_DIM; i++) for (int j = 0; j < K_DIM; j++) A[i][j] = (i * 3 + j) & 0x1F;
    for (int i = 0; i < K_DIM; i++) for (int j = 0; j < N_DIM; j++) W[i][j] = (i + j * 5) & 0x1F;
    memset(C, 0xA5, sizeof C);

    /* make DDR visible to the DataMover */
    Xil_DCacheFlushRange((UINTPTR)A, sizeof A);
    Xil_DCacheFlushRange((UINTPTR)W, sizeof W);
    Xil_DCacheFlushRange((UINTPTR)C, sizeof C);

    show_u8("Activation A (input)", &A[0][0], DIM);
    show_u8("Weight W (input)",     &W[0][0], DIM);

    wr(CSR_A, (u32)(UINTPTR)A);  wr(CSR_W, (u32)(UINTPTR)W);  wr(CSR_C, (u32)(UINTPTR)C);
    wr(CSR_LDA, K_DIM);          wr(CSR_LDW, N_DIM);          wr(CSR_LDC, N_DIM * 4);
    wr(CSR_M, M_DIM); wr(CSR_K, K_DIM); wr(CSR_NC, N_DIM); wr(CSR_R, R_ROWS);

    wr(CSR_CTRL, 1);                                   /* one start for the whole job */
    u32 st, to = 10000000;
    do { st = rd(CSR_STATUS); } while (!(st & 0x2) && --to);
    if (!to)      { printf("ERROR: timeout (status=%x)\n\r", st); return -2; }
    if (st & 0xC) { printf("ERROR: DMA error (status=%x)\n\r", st); return -3; }
    printf("done in %u cycles\n\r", rd(CSR_CYCLES));

    Xil_DCacheInvalidateRange((UINTPTR)C, sizeof C);   /* see S2MM results */

    show_u32("Output C (TPU result buffer)", &C[0][0], DIM);

    int bad = 0;
    for (int i = 0; i < M_DIM; i++) for (int j = 0; j < N_DIM; j++) {
        u32 ref = 0;
        for (int k = 0; k < K_DIM; k++) ref += A[i][k] * W[k][j];
        if (C[i][j] != ref) { if (bad++ < 8) printf("mismatch C[%d][%d]=%u ref=%u\n\r", i, j, C[i][j], ref); }
    }
    printf(bad ? "FAIL (%d bad)\n\r" : "PASS\n\r", bad);
    return bad;
}
