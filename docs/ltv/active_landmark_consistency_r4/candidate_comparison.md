# R4 development candidate comparison

All rows use the unchanged acceptance and same R3B config, inputs and immutable runtime. C01=0.04rad; C02=0.02rad. Only angular limits change.

| Candidate | Sequence | G-ready | V-ready | Longest joint (s) | Raw V RMSE (m/s) | Contract |
|---|---|---:|---:|---:|---:|---|
| C01 | V2_03 | 37.15% | 30.40% | 4.55 | 0.329357 | NOT_MET |
| C01 | V2_02 | 71.01% | 68.93% | 12.80 | 0.253455 | MEETS_SEQUENCE_CONTRACT |
| C02 | V2_03 | 39.09% | 35.27% | 6.30 | 0.329387 | MEETS_SEQUENCE_CONTRACT |
| C02 | V2_02 | 70.87% | 69.33% | 12.80 | 0.253584 | MEETS_SEQUENCE_CONTRACT |

C02 selected from development only. C01 fails the5s joint criterion on V2_03. C02 passes both development sequences; this is not formal all11 or full Passive success. No backend change is supported by the observed normal-data fit outcomes.

Formal freeze awaits the budget decision and dependency/confirmation audit. Existing mathematical/gain/readiness/main VIO contracts remain unchanged.
