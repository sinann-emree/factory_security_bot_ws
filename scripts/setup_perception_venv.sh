#!/usr/bin/env bash
# fsb_perception için Python sanal ortamı: torch + ultralytics (opencv-python OLMADAN) ve
# YOLOv8n-pose ağırlıklarının indirilmesi. Neden ayrı venv: src/fsb_perception/README.md
# "Python ortamı" bölümü (pip OpenCV'nin apt OpenCV 4.6'yı gölgeleyip cv_bridge'i bozması).
#
# Kullanım:
#   scripts/setup_perception_venv.sh                 # ~/fsb_venv + ~/yolov8n-pose.pt
#   VENV=~/baska_venv MODEL_DIR=~/modeller scripts/setup_perception_venv.sh
#   TORCH_INDEX=https://download.pytorch.org/whl/cpu scripts/setup_perception_venv.sh   # yalnız CPU
#
# Yan etkiler:
#   - $VENV dizini oluşturulur (varsayılan ~/fsb_venv, --system-site-packages). Varsa yeniden kullanılır.
#   - PyPI'dan indirme: torch (CUDA'lı varsayılan tekerlek birkaç GB), torchvision, ultralytics vb.
#   - venv'den setuptools kaldırılır (torch'un çektiği ≥80 sürümü colcon ile uyumsuz; colcon zaten
#     venv etkin değilken çalıştırılır).
#   - $MODEL_DIR/yolov8n-pose.pt indirilir (~6.8 MB, Ultralytics GitHub sürümlerinden). Lisans:
#     AGPL-3.0, bkz. ATTRIBUTIONS.md.
#   - ~/.local'a ve sistem Python'una dokunmaz. Ultralytics kendi ayar dosyasını
#     ~/.config/Ultralytics/ altına yazabilir.
set -euo pipefail

VENV="${VENV:-$HOME/fsb_venv}"
MODEL_DIR="${MODEL_DIR:-$HOME}"
MODEL_NAME="${MODEL_NAME:-yolov8n-pose.pt}"
TORCH_INDEX="${TORCH_INDEX:-}"
# pip ~/.local'daki paketleri "kurulu" saymasın; çalışma zamanında launch dosyası da aynısını yapar.
export PYTHONNOUSERSITE=1
# Ultralytics eksik paketleri kendiliğinden kurmaya çalışmasın (opencv-python getirebilir).
export YOLO_AUTOINSTALL=false

log() { printf '[setup_perception_venv] %s\n' "$*"; }

if [[ ! -x "$VENV/bin/python3" ]]; then
  log "venv oluşturuluyor: $VENV (--system-site-packages)"
  if python3 -c 'import ensurepip' 2>/dev/null; then
    python3 -m venv "$VENV" --system-site-packages
  else
    # python3-venv/ensurepip yoksa sistemdeki pip ile devam edilir
    python3 -m venv "$VENV" --system-site-packages --without-pip
  fi
else
  log "mevcut venv kullanılıyor: $VENV"
fi
PY="$VENV/bin/python3"

PIP_EXTRA=()
[[ -n "$TORCH_INDEX" ]] && PIP_EXTRA=(--index-url "$TORCH_INDEX" --extra-index-url https://pypi.org/simple)

log "torch ve ultralytics bağımlılıkları kuruluyor"
"$PY" -m pip install "${PIP_EXTRA[@]}" torch torchvision
"$PY" -m pip install polars nvidia-ml-py ultralytics-thop ultralytics-platform
log "ultralytics kuruluyor (--no-deps: opencv-python gelmesin)"
"$PY" -m pip install --no-deps ultralytics
if "$PY" -m pip show setuptools >/dev/null 2>&1 && \
   [[ "$("$PY" -c 'import setuptools,sys; sys.stdout.write(setuptools.__file__)')" == "$VENV"* ]]; then
  log "venv içindeki setuptools kaldırılıyor"
  "$PY" -m pip uninstall -y setuptools
fi

mkdir -p "$MODEL_DIR"
if [[ -f "$MODEL_DIR/$MODEL_NAME" ]]; then
  log "model zaten var: $MODEL_DIR/$MODEL_NAME"
else
  log "model indiriliyor: $MODEL_DIR/$MODEL_NAME"
  (cd "$MODEL_DIR" && "$PY" -c "from ultralytics import YOLO; YOLO('$MODEL_NAME')")
fi

log "doğrulama (cv2 apt OpenCV 4.x olmalı, cv_bridge import edilmeli)"
set +u; source /opt/ros/jazzy/setup.bash; set -u
"$PY" -c "import cv2, ultralytics; from cv_bridge import CvBridge; \
print('cv2', cv2.__version__, cv2.__file__); print('ultralytics', ultralytics.__version__)"
log "tamam. Çalıştırma: ros2 launch fsb_bringup bringup.launch.py ... perception:=true"
