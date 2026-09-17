# CPU benchmark summary

Values are medians across the indexed runs. CPU 100% means one fully occupied core.
The Point-LIO row is the controlled point-wise path in the shared ROS2 binary.

| Dataset | Profile | Runs | Frame mean / p95 (ms) | CPU mean / p95 (%) | RSS HWM (MiB) | ATE / RPE 1s (m) | Speedup | Gate |
|---|---|---:|---:|---:|---:|---:|---:|---|
| HILTI 2021 UZH tracking area run 2 | Batch-LIO representative CPU (experimental) | 3 | 2.311 / 3.525 | 3.43 / 7.98 | 62.4 | 0.217 / 0.084 | 3.62x | pass |
| HILTI 2021 UZH tracking area run 2 | Batch-LIO serial | 3 | 7.345 / 12.984 | 8.38 / 15.97 | 61.5 | 0.212 / 0.090 | 1.14x | pass |
| HILTI 2021 UZH tracking area run 2 | Point-LIO control | 3 | 8.361 / 14.628 | 9.41 / 15.97 | 61.7 | 0.228 / 0.099 | 1.00x | baseline |
| HILTI 2022 exp14 basement 2 | Batch-LIO representative CPU (experimental) | 3 | 1.374 / 2.229 | 3.84 / 7.99 | 84.9 | 0.103 / 0.078 | 3.54x | pass |
| HILTI 2022 exp14 basement 2 | Batch-LIO serial | 3 | 4.594 / 8.704 | 6.90 / 11.98 | 83.0 | 0.107 / 0.088 | 1.06x | pass |
| HILTI 2022 exp14 basement 2 | Point-LIO control | 3 | 4.863 / 8.615 | 7.12 / 11.98 | 81.9 | 0.109 / 0.091 | 1.00x | baseline |
| TIERS OutdoorRoad cut 1 (Ouster) | Batch-LIO representative CPU (experimental) | 3 | 7.384 / 8.128 | 12.11 / 15.98 | 97.2 | 0.185 / 0.093 | 4.90x | pass |
| TIERS OutdoorRoad cut 1 (Ouster) | Batch-LIO serial | 3 | 32.774 / 49.013 | 36.04 / 55.90 | 96.2 | 0.183 / 0.089 | 1.10x | pass |
| TIERS OutdoorRoad cut 1 (Ouster) | Point-LIO control | 3 | 36.191 / 53.045 | 37.85 / 59.87 | 94.5 | 0.182 / 0.089 | 1.00x | baseline |

## Relative to Point-LIO control

| Dataset | Profile | Latency | CPU mean | RSS HWM | ATE | RPE 1s |
|---|---|---:|---:|---:|---:|---:|
| HILTI 2021 UZH tracking area run 2 | Batch-LIO representative CPU (experimental) | -72.4% | -63.6% | +1.2% | -4.7% | -15.3% |
| HILTI 2021 UZH tracking area run 2 | Batch-LIO serial | -12.2% | -11.0% | -0.2% | -7.0% | -9.1% |
| HILTI 2022 exp14 basement 2 | Batch-LIO representative CPU (experimental) | -71.7% | -46.1% | +3.7% | -5.2% | -14.8% |
| HILTI 2022 exp14 basement 2 | Batch-LIO serial | -5.5% | -3.1% | +1.4% | -1.5% | -3.6% |
| TIERS OutdoorRoad cut 1 (Ouster) | Batch-LIO representative CPU (experimental) | -79.6% | -68.0% | +2.8% | +1.9% | +3.9% |
| TIERS OutdoorRoad cut 1 (Ouster) | Batch-LIO serial | -9.4% | -4.8% | +1.8% | +0.6% | -0.4% |
