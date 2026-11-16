# fsb_perception

**Amaç:** YOLOv8 (ultralytics) ile kamera görüntüsünde insan/anomali/ihlal tespiti.

**Durum:** İskelet (Aşama 1, ament_python). Düğüm Aşama 6'da yazılacak.

## Launch (planlanan)
```bash
ros2 launch fsb_perception perception.launch.py use_sim_time:=true
```

## Topic'ler (planlanan)
- Abone: `/camera/image_raw` (`sensor_msgs/Image`)
- Yayın: `/perception/detections` (`vision_msgs/Detection2DArray`), `/perception/image_annotated`, `/security/alerts`

## Parametreler (planlanan)
`model_path`, `confidence`, `device` (cpu/cuda), `target_classes`
