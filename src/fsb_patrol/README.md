# fsb_patrol

**Amaç:** Nav2 `FollowWaypoints`/`NavigateThroughPoses` ile döngüsel devriye; `/security/alerts` olaylarına tepki (dur, uyar, kaydet).

**Durum:** İskelet (Aşama 1, ament_python). Düğüm Aşama 5–6'da yazılacak.

## Launch (planlanan)
```bash
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true
```

## Parametreler (planlanan)
- `config/waypoints.yaml` — `map` çerçevesinde devriye noktaları
- `loop` (bool), `wait_at_waypoint` (s)
