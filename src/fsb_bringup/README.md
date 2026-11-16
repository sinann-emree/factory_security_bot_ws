# fsb_bringup

**Amaç:** Tek giriş noktası `bringup.launch.py`. `use_sim` argümanı ile Gazebo veya gerçek robot yolunu seçer; `use_sim_time` buradan tüm düğümlere iletilir.

**Durum:** İskelet (Aşama 1). Launch dosyası Aşama 3'te yazılacak.

## Launch (planlanan)
```bash
ros2 launch fsb_bringup bringup.launch.py use_sim:=true
ros2 launch fsb_bringup bringup.launch.py use_sim:=false
```

## Argümanlar
- `use_sim` (bool, varsayılan `true`)
- `use_sim_time` = `use_sim`
