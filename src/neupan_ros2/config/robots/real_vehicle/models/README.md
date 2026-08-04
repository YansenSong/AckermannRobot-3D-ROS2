# DUNE Model

Place the trained DUNE checkpoint file here:

```
models/dune_model_5000.pth
```

The file must be a valid PyTorch checkpoint (`.pth`) trained for the geometry
in `vehicle_config/config/real_vehicle.yaml` (Ackermann kinematics).

Copy from your training machine:
```bash
scp user@training-pc:/path/to/dune_model_5000.pth models/
```
