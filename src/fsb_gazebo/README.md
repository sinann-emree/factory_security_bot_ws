# fsb_gazebo

**Amaç:** Gazebo Harmonic (gz-sim 8) fabrika dünyası, `ros_gz_bridge` yaml'ı ve simülasyon launch dosyası.

**Durum:** İskelet (Aşama 1). İçerik Aşama 3–4'te yazılacak.

## Dizinler
- `worlds/` — `factory.sdf`
- `models/` — dünya modelleri
- `config/` — `gz_bridge.yaml` (`/clock`, `/scan`, `/camera/*`, `/imu`)
- `launch/` — `sim.launch.py`

## Launch (planlanan)
```bash
ros2 launch fsb_gazebo sim.launch.py
```

## Kurallar
- Yalnızca `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`; Gazebo Classic yok.
- `/clock` yalnızca köprüden gelir.
