# fsb_patrol

**Amaç:** Nav2 `FollowWaypoints` ile döngüsel devriye. (Aşama 6: `/security/alerts` olaylarına tepki — dur, uyar, kaydet.)

**Durum:** Aşama 5 — devriye düğümü çalışıyor.

## Düğüm: `patrol_node` (`fsb_patrol/patrol_node.py`)
Her tur için bir `nav2_msgs/action/FollowWaypoints` hedefi gönderir (`number_of_loops=0`), sonuç gelince
bir sonraki turu başlatır. Böylece her tur ayrı loglanır (süre, kaçırılan waypoint'ler, hata kodu).
Bir tur başarısız olsa da devriye durmaz; sonraki tur denenir. Bitince `DEVRİYE BİTTİ: x/y tur başarılı` yazar.

## Launch
Nav2 çalışırken (`ros2 launch fsb_bringup bringup.launch.py localization:=amcl`):
```bash
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true            # sonsuz
ros2 launch fsb_patrol patrol.launch.py use_sim_time:=true loops:=10  # 10 tur
```

## Parametreler (`config/patrol.yaml`)
| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `waypoints` | 4 nokta (aşağıda) | düz liste `[x, y, yaw, ...]` (m, m, rad), en az 3 nokta |
| `frame_id` | `map` | waypoint çerçevesi |
| `loops` | `0` | tur sayısı, 0 = sonsuz (launch argümanı `loops` ile ezilir) |
| `action_name` | `follow_waypoints` | Nav2 waypoint_follower action adı |

Varsayılan rota (factory.sdf engellerinden ≥ 1 m boşluklu):
W1 (4.5, 0) → W2 (3, 3) → W3 (-3, -3) → W4 (-4.5, 0) → W1 …
