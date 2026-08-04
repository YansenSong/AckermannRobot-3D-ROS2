# DUNE Model

Place the trained DUNE checkpoint file here:

```
models/dune_model_5000.pth
```

The file must be a valid PyTorch checkpoint (`.pth`) trained for this vehicle's geometry (0.97m wheelbase, Ackermann kinematics).

Copy from your training machine:
```bash
scp user@training-pc:/path/to/dune_model_5000.pth models/
```
