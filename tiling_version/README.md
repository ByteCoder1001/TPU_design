# TPU v2 and AXI DataMover — Design Guide

Target: ZedBoard (Zynq-7000, xc7z020clg484-1), fabric clock 100 MHz.
Scope: how the **AXI DataMover** IP and the **tpu_accelerator_top** IP work, how they connect, and how software drives them.

> Status: the TPU RTL was simulated with a *behavioral model* of the DataMover (iverilog). It has **not** been synthesized, placed, or run on hardware, and has not been tested against the real DataMover IP. Items marked **(verify)** come from memory of Xilinx documentation and should be checked against the IP's product guide (PG022) in your Vivado version.

---

## 1. System overview

```
                 +---------------------------- Zynq PS ----------------------------+
                 |   ARM core (Vitis app)            DDR3 controller               |
                 |        |  M_AXI_GP0                  ^  S_AXI_HP0 (64-bit)      |
                 +--------|-----------------------------|-------------------------+
                          v                             |
                    [ ic_ctl ]                     [ ic_mem ]
                          |  AXI4-Lite               ^        ^   AXI4 (full), bursts
                          v                          |        |
   +----------------------------------------+   M_AXI_MM2S  M_AXI_S2MM
   |        tpu_0  (tpu_accelerator_top)    |        |        |
   |                                        |   +----+--------+----+
   |  axil_csr   registers  <-- s_axi       |   |   AXI DataMover  |
   |  tpu_fsm    tile sequencer             |   |   (dm_0)         |
   |  dm_fetch   --- MM2S cmd -------------------->S_AXIS_MM2S_CMD  |
   |             <-- MM2S data (64b) ------------- M_AXIS_MM2S      |
   |             <-- MM2S status ----------------- M_AXIS_MM2S_STS  |
   |  acc_flush  --- S2MM cmd -------------------->S_AXIS_S2MM_CMD  |
   |             --- S2MM data (64b) ------------->S_AXIS_S2MM      |
   |             <-- S2MM status ----------------- M_AXIS_S2MM_STS  |
   |  weight_bram / act_bram / accumulators      |   +--------------+
   |  compute_core (systolic array)         |
   +----------------------------------------+
```

Roles:
- **PS (software)** only writes a few registers and polls a status bit. It never touches matrix data through the AXI-Lite port.
- **TPU IP** decides *what* to move (addresses, sizes) and *when* to compute.
- **DataMover** does the actual DDR transfers on command.

Compared with v1 (3× AXI BRAM controller + AXI GPIO, software copies every word into BRAM), v2 lets the hardware pull matrices from DDR and push results back, which is what makes matrices larger than the array possible (tiling).

---

## 2. Part A — The AXI DataMover IP

### 2.1 What it is
The AXI DataMover (`axi_datamover`) is a Xilinx IP that performs high-throughput memory-mapped transfers under the control of **command streams**, without any CPU or register interface. It has two independent channels:

| Channel | Direction | Used here for |
|---|---|---|
| **MM2S** (Memory-Mapped to Stream) | DDR → stream | Fetching the weight matrix W and activation blocks A |
| **S2MM** (Stream to Memory-Mapped) | stream → DDR | Writing result rows of C |

### 2.2 Interfaces and protocols

| Port group | Protocol | Width | Direction (from DataMover) |
|---|---|---|---|
| `M_AXI_MM2S`, `M_AXI_S2MM` | AXI4 memory-mapped (full, bursts) | 64-bit data, 32-bit address | master to DDR (via interconnect) |
| `S_AXIS_MM2S_CMD`, `S_AXIS_S2MM_CMD` | AXI4-Stream | 72 bits | slave: receives commands |
| `M_AXIS_MM2S_STS`, `M_AXIS_S2MM_STS` | AXI4-Stream | 8 bits | master: returns status |
| `M_AXIS_MM2S` | AXI4-Stream | 64-bit data (+TKEEP, TLAST) | master: read data out |
| `S_AXIS_S2MM` | AXI4-Stream | 64-bit data (+TKEEP, TLAST) | slave: write data in |

There is **no AXI4-Lite port** on the DataMover. It is controlled only through the command streams. The only AXI4-Lite interface in this design is the TPU's own register block (`tpu_0/s_axi`).

Full AXI4 (not AXI-Lite) is required on the memory side because bursts are what make DDR transfers fast; AXI-Lite would transfer one word per transaction. The PS HP0 port is AXI3, and `ic_mem` converts between AXI4 and AXI3 automatically.

### 2.3 Configuration used in this design

| Setting | Value | Reason |
|---|---|---|
| MM2S and S2MM channels | both enabled | read inputs, write outputs |
| Address width | 32 | matches ZedBoard DDR address space and the 72-bit command |
| MM2S / S2MM memory-map data width | 64 | = one buffer word (8 elements × 8 bits) |
| MM2S / S2MM stream data width | 64 | same |
| Max burst length | 16 beats | 16 × 8 B = 128 B per burst |
| BTT field width | 23 bits | the RTL builds commands with a 23-bit byte count (max 8 MB − 1 per command) |
| Unaligned transfers (DRE) | off | RTL assumes 8-byte-aligned addresses and sizes |
| Indeterminate BTT | off | every transfer has a known size |
| Asynchronous clocks | off | everything runs on FCLK_CLK0 |
| Advanced signals | off | unused |

### 2.4 Command word (72 bits)
Both `dm_fetch` and `acc_flush` build the command as
`{4'h0, 4'h0, addr[31:0], 1'b0, 1'b1, 6'd0, 1'b1, btt[22:0]}` **(verify layout in PG022)**:

| Bits | Field | Value used | Meaning |
|---|---|---|---|
| 71:68 | reserved | 0 | |
| 67:64 | TAG | 0 | echoed back in status (unused) |
| 63:32 | SADDR | start byte address | where to read (MM2S) / write (S2MM) |
| 31 | DRR | 0 | no re-alignment |
| 30 | EOF | 1 | marks the last command of a packet; S2MM expects TLAST to match |
| 29:24 | DSA | 0 | |
| 23 | Type | 1 | INCR (incrementing) addressing |
| 22:0 | BTT | byte count | bytes to transfer for this command |

### 2.5 Status word (8 bits) **(verify)**
| Bits | Field | Meaning |
|---|---|---|
| 3:0 | TAG | copy of command tag |
| 4 | INTERR | internal error |
| 5 | DECERR | AXI decode error (bad address) |
| 6 | SLVERR | AXI slave error |
| 7 | OKAY | transfer completed cleanly |

One status word is returned per command. The TPU sets its error flag when any of bits 6:4 is 1.

### 2.6 Transfer flow

**MM2S (read):**
1. TPU puts a command on `S_AXIS_MM2S_CMD` (valid/ready handshake).
2. DataMover issues AXI read bursts to DDR.
3. Data comes out on `M_AXIS_MM2S`; the TPU accepts a beat only when it is ready to store it (`tready` is the back-pressure).
4. A status word appears on `M_AXIS_MM2S_STS` after the last beat.

**S2MM (write):**
1. TPU sends a command (address + byte count) on `S_AXIS_S2MM_CMD` **before** the data.
2. TPU streams the data on `S_AXIS_S2MM` with TKEEP all ones and TLAST on the final beat of the command.
3. DataMover writes AXI bursts to DDR.
4. A status word is returned on `M_AXIS_S2MM_STS`.

### 2.7 Clocks, resets, and pins to wire
- Clock pins: `m_axi_mm2s_aclk`, `m_axi_s2mm_aclk`, `m_axis_mm2s_cmdsts_aclk`, `m_axis_s2mm_cmdsts_awclk` (note the unusual `awclk` spelling on the S2MM side) → all to FCLK_CLK0.
- Reset pins: `m_axi_mm2s_aresetn`, `m_axi_s2mm_aresetn`, `m_axis_mm2s_cmdsts_aresetn`, `m_axis_s2mm_cmdsts_aresetn` → `rst_0/peripheral_aresetn`.
- Outputs `mm2s_err` / `s2mm_err` are left unconnected; the TPU decodes errors from the status streams.

### 2.8 Constraints this imposes on the system
- Every address and size must be a multiple of 8 bytes (no DRE).
- A single command moves at most 2²³ − 1 bytes.
- DataMover accesses DDR directly and is **not cache coherent**: software must flush caches before start and invalidate the result buffer after done.
- The DataMover address spaces must be mapped to the PS DDR window (0x0000_0000–0x1FFF_FFFF on the ZedBoard).

---

## 3. Part B — The TPU IP (`tpu_accelerator_top`)

### 3.1 Module hierarchy

```
tpu_accelerator_top
├── axil_csr            AXI4-Lite registers: config, start, status, cycle counter
├── tpu_fsm             tile sequencer (loop nest, ping-pong control)
├── dm_fetch            MM2S front end: loads W, then A row-blocks
├── weight_bram         dual_port_bram: whole W matrix (128 KB default)
├── act_bram            dual_port_bram: 2 banks of A (64 KB each)
├── compute_core        systolic array datapath
│   ├── skew_buffer     staggers activation rows
│   ├── systolic_array_core  N×N grid of tpu_pe
│   └── deskew_buffer   re-aligns output columns
└── acc_flush           2-bank accumulator + S2MM write-back
    └── sdp_bram ×2     accumulator storage
```

Parameters: `N` (array size, 8), `DATA_WIDTH` (8), `ACC_WIDTH` (32), `WB_AW` (14), `AB_AW` (13), `MAX_R` (256).

### 3.2 Ports
- `clk`, `aresetn` (active low)
- `s_axi_*`: AXI4-Lite slave, 6-bit address (64-byte register space)
- `m_axis_mm2s_cmd`, `s_axis_mm2s`, `s_axis_mm2s_sts`: connect to DataMover MM2S
- `m_axis_s2mm_cmd`, `m_axis_s2mm`, `s_axis_s2mm_sts`: connect to DataMover S2MM
- `irq`: level output equal to the done flag

Stream data width is fixed at 64 bits, so the design is verified for N = 8 only.

### 3.3 The compute datapath (unchanged from v1)

**Processing element (`tpu_pe`)** holds one 8-bit weight.
- `load_weight`: stores the incoming value as its weight and passes the input down unchanged, so weights shift down a column.
- `valid`: computes `psum_out = psum_in + act_in × weight`, and passes the activation to the right.
- Multiplication is **unsigned**.

**Systolic array:** N × N PEs. Activations flow left → right, partial sums flow top → bottom. A **skew buffer** delays activation row *i* by *i* cycles so that data meets the right partial sum, and a **deskew buffer** re-aligns the outputs so a complete output row appears together.

**Weight-stationary:** weights stay in the PEs while activation rows stream through.

**Weight loading:** in `compute_core` the weight bytes enter at the top of the array while `load_weight` is high. N cycles load N rows, **last row first**, because each new row pushes earlier ones further down. This is why the sequencer reads W rows in descending order.

**Latency:** a tile's last output appears `2N − 1` cycles after its last input row, so the sequencer drains for `2N − 1` cycles (15 at N = 8).

### 3.4 On-chip memories

**Weight buffer** (`weight_bram`, 64-bit × 2^`WB_AW` words, default 16384 words = 128 KB)
- Written by the DMA stream, one 64-bit word = 8 consecutive bytes of one W row.
- Layout: word index = `row × (NC/N) + column_tile`. Byte *j* of a word is the weight for array column *j*.
- Holds the **entire** W matrix, loaded once per job.

**Activation buffer** (`act_bram`, 2 banks × 2^`AB_AW` words = 2 × 64 KB)
- Bank bit + word index = `row × (K/N) + k_tile`. Byte *i* of a word is the activation for array row *i*.
- Double-buffered: while the array works on one bank, DMA fills the other with the next row-block.

**Accumulator** (`acc_flush`, 2 banks × `MAX_R` rows × 256 bits)
- One entry = one output row segment (8 × 32-bit sums).
- First reduction tile (`ki == 0`): write the new sums. Later tiles: read the old value, add, write back (read-modify-write, one row per cycle).
- Double-buffered: one bank accumulates the current output tile while the other is flushed to DDR.

### 3.5 Tiling: what the hardware does

Matrix sizes: A is M × K, W is K × NC, C is M × NC. The array handles an N × N tile of W at a time.

- `tk = K / N` reduction tiles, `tn = NC / N` output-column tiles.
- A is processed in **row-blocks** of R rows (the last block may be shorter).

```
fetch W (whole K×NC) into weight_bram                      [one DMA]
fetch first A block into act bank 0
for each A row-block (R rows):
    start fetching the NEXT block into the other bank       [overlaps compute]
    for ni = 0 .. tn-1:                 # output column tile
        for ki = 0 .. tk-1:             # reduction tile
            LOAD  : shift W tile (ki, ni) into the array         N cycles
            FEED  : stream R rows of A[:, ki-tile]               R cycles
            DRAIN : wait for the pipeline to empty               2N-1 cycles
                    results added into the accumulator bank
        FLUSH: write R rows of C[:, ni-tile] to DDR (S2MM)       overlaps next tile
done when the last flush and all write statuses are back
```

Cost per tile ≈ `R + 3N` cycles (1 setup + N load + R feed + 2N−1 drain), so array utilization ≈ `R / (R + 3N)`: about 25 % at R = 8, 73 % at R = 64, 91 % at R = 256. Larger R amortizes the weight reload, which is why R is a software parameter.

### 3.6 Sequencer states (`tpu_fsm`)

| State | Action |
|---|---|
| `S_IDLE` | wait for `start`; latch `tk`, `tn`, addresses; issue the W fetch |
| `S_WF` | wait for W to finish loading; issue fetch of first A block |
| `S_ABLK` | wait for the current A block; swap banks; start fetching the next block; reset `ni`, `ki` |
| `S_SETUP` | compute W-buffer start address for tile (ki, ni); reset counters |
| `S_LOAD` | read N W rows (descending) into the array |
| `S_FEED` | read R activation words (stride `tk`) into the array |
| `S_DRAIN` | `2N − 1` cycles for the pipeline to empty, accumulate results |
| `S_FLUSH` | start write-back of the finished output tile (waits for the previous flush) |
| `S_FIN` | wait for all writes to be acknowledged, pulse `done` |

BRAM reads have one cycle of latency, so `ld_q`/`fd_q` are the registered versions of the load/feed states, aligning the data with its valid strobe.

### 3.7 `dm_fetch` (MM2S front end)
- One module serves both W and A: `sel_a` selects the destination BRAM, `bank` selects the A bank.
- **Contiguous case** (`stride == row_bytes`): a single command for `rows × row_bytes` bytes.
- **Strided case:** one command per row, address advanced by `stride`.
- Incoming stream beats are written to consecutive BRAM word addresses starting at 0.
- Asserts `busy` until the last beat is stored; sets `err` if a status word reports an error.

### 3.8 `acc_flush` (accumulator and S2MM back end)
- Write-back runs one **row per S2MM command**: 32 bytes = 4 beats of 64 bits, TLAST on the 4th.
- Row address = `c_addr + row × ldc + ni × 32` (computed by `tpu_fsm` and `acc_flush`).
- Tracks commands sent versus status words received; `wr_idle` is true when all writes are acknowledged, which is what lets the job finish only after data is really in DDR.

### 3.9 Register map (AXI4-Lite, base 0x43C00000, 64 bytes)

| Offset | Name | Access | Description |
|---|---|---|---|
| 0x00 | CTRL | W | bit 0 = start (ignored if busy) |
| 0x04 | STATUS | R | [0] busy, [1] done, [2] MM2S error, [3] S2MM error |
| 0x08 | A_ADDR | R/W | DDR address of A |
| 0x0C | W_ADDR | R/W | DDR address of W |
| 0x10 | C_ADDR | R/W | DDR address of C |
| 0x14 | LDA | R/W | A row stride, bytes |
| 0x18 | LDW | R/W | W row stride, bytes |
| 0x1C | LDC | R/W | C row stride, bytes |
| 0x20 | M | R/W | rows of A |
| 0x24 | K | R/W | columns of A = rows of W |
| 0x28 | NC | R/W | columns of W / C |
| 0x2C | R | R/W | rows per A block |
| 0x30 | CYCLES | R | clock cycles of the last job |
| 0x34 | ID | R | `{MAX_R[15:0], N[7:0], 0x03}` |
| 0x38 | WBUF_BYTES | R | weight buffer size |
| 0x3C | ABUF_BYTES | R | bytes per activation bank |

The done flag clears at the next start. Registers must not be changed while the job is running (the sequencer re-reads some of them every block).

### 3.10 Job constraints

| Rule | Default limit |
|---|---|
| K and NC multiples of N | 8 |
| M | any value |
| K × NC | ≤ 128 KB |
| R | ≤ 256 and R × K ≤ 64 KB |
| Addresses and strides | 8-byte aligned; LDA ≥ K, LDW ≥ NC, LDC ≥ NC × 4 |
| A, W | unsigned 8-bit |
| C | 32-bit, row-major |

### 3.11 Software sequence
1. Fill A, W in DDR; flush data cache for A, W, C.
2. Write A/W/C addresses, strides, M, K, NC, R to the registers.
3. Write 1 to CTRL.
4. Poll STATUS until bit 1 (done) is set, with a timeout; check error bits.
5. Invalidate the data cache for C and read the results.

### 3.12 Worked example
M = 20, K = 16, NC = 16, R = 8, contiguous matrices:
- `tk = 2`, `tn = 2`, row-blocks of 8, 8, 4 rows.
- W: one MM2S command of 256 bytes (32 beats). A: one command per block (128, 128, 64 bytes).
- Tiles computed: 3 blocks × 2 × 2 = 12 weight loads.
- Output: 20 rows × 2 column tiles = 40 S2MM commands of 32 bytes; row *r*, tile *ni* goes to `C + r × 64 + ni × 32`.
- Simulated run (with the behavioral DataMover): 461 cycles, result matches a software matmul.

### 3.13 Resource estimate (not synthesized)
About 272 KB of on-chip RAM: 128 KB weights + 2 × 64 KB activations + 2 × 8 KB accumulators, roughly 110 BRAM36 of the 140 on the Z7020. Reduce `WB_AW`, `AB_AW`, or `MAX_R` if it does not fit.

---

## 4. Known limitations
- Verified only with N = 8 and a behavioral DataMover; not synthesized or run on hardware.
- The full W matrix must fit on chip; larger W needs weight tiling in DDR (future work).
- Unsigned arithmetic only; no bias, activation function, or quantization stage.
- A DMA error sets a flag but does not abort the job; software must check the status bits and use a timeout.
- Stream width is fixed at 64 bits, so other array sizes need a width converter or a different DataMover width.
- Single array: no multi-core scaling.
