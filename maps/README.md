# Maps

Each LIO-SAM map export is stored in a timestamped directory:

```text
maps/YYYYMMDD_HHMMSS/
├── GlobalMap.pcd
├── CornerMap.pcd
├── SurfMap.pcd
├── trajectory.pcd
├── transformations.pcd
├── map.pgm
└── map.yaml
```

Use the directory name when starting navigation:

```bash
bash scripts/nav_hdl_neupan.sh YYYYMMDD_HHMMSS
```
