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

The Map Management page lists each directory directly under `maps/` that
contains a map export. A map with `map.yaml` and its image can be loaded into
the running 2D map server; a point-cloud-only export is shown until its 2D map
is generated. UI-managed grouped maps remain under `maps/ui/`.

Use the directory name when starting navigation:

```bash
bash scripts/nav_hdl_neupan.sh YYYYMMDD_HHMMSS
```
