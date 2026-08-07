# Scripts

## `train_logistic_regression.py`

Regenerates the on-device "needs water soon" model header at
`lib/MoistureML/MoistureMLModel.h`.

```bash
pip install numpy scikit-learn
python scripts/train_logistic_regression.py                # synthetic data
python scripts/train_logistic_regression.py --csv data.csv # real telemetry
```

CSV columns (any order): `moisture_pct`, `temperature_c`, `humidity_pct`,
`lux`, `hours_since_water`, `needs_water_within_6h` (0/1).

The emitted header contains standardisation means / stds, the fitted logistic
regression coefficients, and the decision threshold - all `constexpr float`.
Inference on the ESP32 is a single dot product plus a sigmoid.
