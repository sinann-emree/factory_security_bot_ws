# fsb_gazebo

**Amaç:** Gazebo Harmonic (gz-sim 8) fabrika dünyası, `ros_gz_bridge` yaml'ı ve simülasyon launch dosyası.

**Durum:** Aşama 3 tamam (dünya, spawn, /clock köprüsü). Sensörler Aşama 4.

## Dizinler
- `worlds/factory.sdf` — 20×14 m salon, 3 m duvarlar; raf sıraları y=±3.0 / ±5.5 (4×0.8×2 m);
  ara koridorlar x∈[-4,-2], [2,4]; 4 kolon; doğu ucunda paletler. Ana koridor |y|<2.6 m boş, robot (0,0)'da başlar.
  Sistemler: Physics (1 ms adım), UserCommands (spawn için gerekli), SceneBroadcaster.
- `models/` — (boş) harici modeller için
- `config/gz_bridge.yaml` — şu an yalnızca `/clock`; `/scan`, `/camera/*`, `/imu` Aşama 4
- `launch/sim.launch.py` — gz sim + bridge + `ros_gz_sim create` (`/robot_description` topic'inden)

## Launch
Genelde `fsb_bringup` üzerinden. Doğrudan (robot_state_publisher ayrıca çalışıyor olmalı):
```bash
ros2 launch fsb_gazebo sim.launch.py headless:=true
```

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `world` | `worlds/factory.sdf` | SDF dünya |
| `headless` | `false` | `true` → `gz sim -s` (GUI yok) |
| `x`,`y`,`z`,`yaw` | `0`,`0`,`0.01`,`0` | Spawn pozu |

## Kurallar
- Yalnızca `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`; Gazebo Classic yok.
- `/clock` yalnızca köprüden gelir.
