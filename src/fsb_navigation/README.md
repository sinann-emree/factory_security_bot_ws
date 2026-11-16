# fsb_navigation

**Amaç:** SLAM Toolbox (haritalama), AMCL (lokalizasyon) ve Nav2 parametreleri, kaydedilmiş haritalar.

**Durum:** İskelet (Aşama 1). İçerik Aşama 5'te yazılacak.

## Dizinler
- `config/` — `slam_toolbox_mapping.yaml`, `nav2_params.yaml`
- `maps/` — `factory.yaml` / `factory.pgm`
- `launch/`

## Launch (planlanan)
```bash
ros2 launch fsb_navigation slam.launch.py use_sim_time:=true
ros2 launch fsb_navigation navigation.launch.py map:=<map.yaml> use_sim_time:=true
```

## Kural
- `map→odom` aynı anda yalnızca SLAM Toolbox **veya** AMCL tarafından yayınlanır.
