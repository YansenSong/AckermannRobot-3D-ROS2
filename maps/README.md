# Maps

该目录只保存实车采集或由实车点云生成的地图。

建议每次建图导出到独立目录：

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

二维地图可使用：

```bash
bash scripts/pcd_to_map.sh maps/YYYYMMDD_HHMMSS
```

仿真 `mini.world` 对应地图不在 `real-vehicle-integration` 分支维护。
