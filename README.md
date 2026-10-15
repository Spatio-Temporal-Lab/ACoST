# ACoST: Adaptive Error-Bounded Compression for Streaming Trajectories

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

This repository contains the algorithm implementations and experiment sources for **Adaptive Error-Bounded Compression for Streaming Trajectories**.

## Overview

CoST and ACoST are one-pass, pointwise error-bounded compressors for trajectory streams. They process samples in order, preserve one reconstructed point per input point, and bound the reconstruction error of longitude and latitude. CoST combines motion prediction, residual quantization, and integer coding with a cost-based choice of prediction mode. ACoST adds adaptive motion prediction using a CTRA-family model, adaptive history weighting, interval-dependent acceleration damping, and conditional zero-prediction selection.

The evaluation includes six real-world trajectory datasets, algorithm and component comparisons, sensitivity studies, and an STM32 prototype. Spatial compression ratio excludes timestamps. E6 additionally measures on-board processing, UART payload, and physical-link end-to-end latency.

## Results Summary

| Metric | ACoST | CoST | Serf-Qt | Elf | VOLTCom | OPW-TR |
|---|---:|---:|---:|---:|---:|---:|
| CR (ε=10⁻⁵) | 18.45 | 16.52 | 12.84 | 3.32 | 2.40 | 2.65 |
| CT (PC, μs/pt) | 0.100 | 0.120 | 0.037 | 0.078 | 0.002 | 0.137 |
| CT (STM32, μs/pt) | 256 | 139 | 46 | 72 | 41 | 182 |
| E2E @300 baud (ms) | 60.42 | 61.49 | 249.42 | 520.88 | 17089.80 | 11734.93 |

## Project Structure

```text
ACoST/
├── algorithm/                 # CoST and ACoST implementations
├── baselines/                  # Serf, Elf, Gorilla, and trajectory baselines
│   ├── elf/
│   ├── gorilla/
│   ├── serf/
│   └── trajcompress/
├── data/                       # Local dataset location; data are not tracked by Git
├── embedded/                   # STM32F103 firmware and embedded benchmark sources
├── experiments/
│   ├── e1_overall/              # Overall comparison
│   ├── e2_ablation/             # Component and predictor ablations
│   ├── e3_epsilon/              # Error-bound sensitivity
│   ├── e4_sampling/             # Berlin sampling-interval sensitivity
│   └── e5_tau/                  # ACoST parameter sensitivity
├── results/
│   └── result.xlsx              # E1–E6 experiment tables
├── utils/                       # Shared bit-stream and coding utilities
├── LICENSE
└── README.md
```

## Quick Start

### Requirements

- C++17 compiler (`g++`)
- Bash on Linux/macOS, WSL, or Git Bash on Windows
- The six datasets listed below
- For embedded builds: `arm-none-eabi-gcc` and the required STM32 board/programmer

### Download the datasets

Download the dataset archive from [Baidu Netdisk](https://pan.baidu.com/s/1t4kQjU3GmckF6lMExCaLaA?pwd=7amz) (access code: `7amz`). It contains the six evaluation datasets and Berlin datasets sampled at different intervals. Extract them outside version control and set `DATA_ROOT` to the directory containing the dataset folders.

The current E1 runner expects these folder keys under `DATA_ROOT`:

```text
Geolife/  WorldTrace/  T-dirve/  ais/  trajair/  Berlin/
```

The `T-dirve` spelling is the key currently used by the benchmark source. Keep the Berlin interval-sampled data available for E4 and E5.

### Run E1

From the repository root, build and run the overall benchmark:

```bash
DATA_ROOT=/path/to/data bash experiments/e1_overall/run.sh
```

The benchmark compares 11 methods, including Raw, across six datasets. It builds outside the repository and writes CSV output to the sibling `results/` directory by default. Set `OUTPUT=/path/to/file.csv` to change the output path; use `--help` for benchmark options.

### Run E2–E5

Each experiment provides a shell entry point that builds its C++ benchmark. E3–E5 can be run with:

```bash
DATA_ROOT=/path/to/data bash experiments/e3_epsilon/run.sh
DATA_ROOT=/path/to/data bash experiments/e4_sampling/run.sh
DATA_ROOT=/path/to/data bash experiments/e5_tau/run.sh
```

E2 supports `family`, `predictor`, and `common-ton` modes. Its argument format is `run.sh {family|predictor|common-ton} METHOD DATASET [EPSILON TAU T_ON] < binary_trajectory_stream`; invoking it without arguments prints the usage text. E3–E5 write CSV files to the sibling `results/` directory by default; set `OUTPUT` to override the destination.

### Build the embedded firmware

```bash
make -C embedded
```

E6 uses two STM32 boards. One board compresses the trajectory and transmits it over UART; the other receives and processes the stream. Connect UART TX-to-RX and RX-to-TX with a shared GND, then connect the transmitting board to the host through USB-TTL. Select the dataset, algorithm, and baud rate, run the trajectory, and record compression ratio, compression/decompression time, and end-to-end latency. Repeat for each tested baud rate. The E6 measurements are listed in [`results/result.xlsx`](results/result.xlsx).

## Datasets

Download the six evaluation datasets and Berlin interval-sampled data from [Baidu Netdisk](https://pan.baidu.com/s/1t4kQjU3GmckF6lMExCaLaA?pwd=7amz) (access code: `7amz`). The paper's PC evaluation uses these six datasets:

| Dataset | Object type | Sampling interval | Trajectories | Points |
|---|---|---:|---:|---:|
| GeoLife | Mixed transport | 1–5 s | 17,784 | 24,876,978 |
| WorldTrace | Car | 1 s processed release | 2.45 M | 880 M |
| T-Drive | Taxi | Mean 177 s | 10,336 | 17,662,983 |
| AIS | Vessel | Mean 39 s | 6,000 | 2,825,332 |
| TrajAir | Aircraft | 1 s | 4,000 | 1,614,108 |
| Berlin (SimRa) | Bicycle | 1 s | 4,438 | 2,231,686 |

For TrajAir, only longitude and latitude are used. WorldTrace refers to the processed one-second release. The dataset archive also includes Berlin datasets sampled at intervals from 5 s to 80 s. The counts above are per-dataset paper statistics; no combined point total is reported.

## Experiments

| ID | Experiment | Scope reported in the result tables |
|---|---|---|
| E1 | Overall performance | 11 methods, including Raw, across 6 datasets; compression ratio and PC compression/decompression time. |
| E2 | Ablation and implementation comparison | ACoST component ablations on sampled Berlin; vector versus arc-integral extrapolation; CoST-CTRA predictor comparison. |
| E3 | Error-bound sensitivity | ACoST, CoST, Serf-Qt, and Serf-XOR over five error bounds on all six datasets. |
| E4 | Sampling-interval sensitivity | The four E3 methods on Berlin at the original interval and 5, 10, 20, 40, and 80 s. |
| E5 | Parameter sensitivity | ACoST onset-threshold and damping-constant settings on the 20 s and 40 s Berlin samples. |
| E6 | Embedded-system validation | ACoST, CoST, Serf-Qt, Elf, VOLTCom, and OPW-TR on STM32; compression ratio, on-board compression/decompression time, and UART end-to-end latency at multiple baud rates. |

The E1–E6 result tables are in [`results/result.xlsx`](results/result.xlsx); PC and embedded measurements are reported separately.

## Citation

```bibtex
@article{zhouAdaptiveErrorBoundedCompression2026,
  title   = {Adaptive Error-Bounded Compression for Streaming Trajectories},
  author  = {Zhou, Qinqin and Chen, Meibin and Xu, Zihang and Li, Zheng and Li, Ruiyuan and He, Huajun and Zheng, Yu},
  journal = {The VLDB Journal}
}
```

## License

This project is licensed under the MIT License; see [LICENSE](LICENSE). Dataset and third-party component terms remain subject to their respective providers and licenses.
