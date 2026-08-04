# 实车 DUNE 模型

当前目录中的权重是针对本项目真实阿克曼车辆训练的 NeuPAN/DUNE checkpoint：

```
models/dune_model_5000.pth
```

文件信息：

- 大小：25,013 字节
- SHA-256：`f522ab03c5c16d3059a617e557f45967ef875a20590013c0aa413d14c911e99a`
- 运动学类型：阿克曼

权重训练时使用的车辆几何必须与
`vehicle_config/config/real_vehicle.yaml` 保持一致。修改车长、车宽、轴距或模型输入
定义后，需要确认是否重新训练模型。
