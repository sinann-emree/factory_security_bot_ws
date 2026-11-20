# Firmware — Andino (harici bağımlılık)

Bu doküman, robotun Arduino Nano'sunda çalışacak motor/enkoder firmware'inin nasıl temin edildiğini, nasıl derlenip yükleneceğini ve nasıl değiştirileceğini tanımlar.
Proje kuralı: Andino deposundaki dosyalar bu reponun `src/` dizinine kopyalanmaz; firmware sabitlenmiş bir commit'ten harici bağımlılık olarak kullanılır.

> **Durum (2026-11-19):** Bu doküman yazılırken derleme ve yükleme **çalıştırılmadı**. Makinede Docker ve PlatformIO kurulu değil. Aşağıdaki komutlar upstream dokümantasyonundan alındı ve doğrulanmadı. İlk gerçek derlemeden sonra bu not güncellenmeli.

---

## 1. Sabitlenen sürüm

| Alan | Değer | Kaynak |
|---|---|---|
| Depo | https://github.com/Ekumen-OS/andino | `~/reference/andino` → `git remote -v` |
| Yerel kopya | `~/reference/andino` (çalışma ağacı temiz) | `git status` |
| **Sabit commit** | `974264bdefd6ab939ba487ffa95ce3c1525285ff` | `git log -1` |
| Commit tarihi / başlığı | 2026-11-02, "[andino_firmware]: Bump gcovr from 8.6 to 8.8 in /andino_firmware/docker (#380)" | `git log -1` |
| Firmware alt dizini | `andino_firmware/` | — |
| Firmware sürümü | `0.2.0` | `andino_firmware/package.xml` `<version>`; `scripts/version.py` bunu `ANDINO_FW_VERSION` makrosu olarak gömer |
| PlatformIO env | `nanoatmega328` | `andino_firmware/platformio.ini` `[env:nanoatmega328]` |
| Board paketi | `boards/arduino_nano/andino/board.h` yalnızca `#include "../../arduino_uno/andino/board.h"` içerir. Nano, Uno ile aynı pin ve config'i kullanır. | `boards/arduino_nano/andino/board.h:35`; `README.md:74` |
| Seri hız | 57600 baud | `platformio.ini:38` `monitor_speed`; `README.md:131` |

Commit'i değiştirmek bilinçli bir karardır. Bunun için bu tablo güncellenir, yamalar yeni commit'e karşı tekrar test edilir ve değişiklik ayrı bir commit olarak atılır.

**Sabitlenmeyen bağımlılıklar:**
- `[base_atmelavr]` içinde `platform = atmelavr` sürümsüz (`platformio.ini:51`). STM32 için sürüm sabitlenmiş, AVR için sabitlenmemiş. Bu nedenle AVR toolchain'i derleme zamanına göre değişebilir.
- Önerilen çözüm: İlk başarılı derlemede kullanılan sürüm `pio pkg list` çıktısından okunup bu dokümana yazılmalı. Gerekirse sürüm bir yamayla sabitlenebilir. **Doğrulanmadı.**
- Kütüphaneler sabit: Adafruit BNO055 1.6.4, BusIO 1.17.4, Unified Sensor 1.1.15 (`platformio.ini:43-45`).
- Docker imajındaki PlatformIO sürümü `platformio==6.2.0` (`docker/requirements.txt`).

---

## 2. Neden `src/` içine kopyalanmıyor?

- Proje kuralı: Andino deposundan `src/` içine dosya kopyalanmaz; kullanılan ölçü ve parametrelerin kaynağı yorumlarda belirtilir.
- Firmware ROS paketi değildir; AVR çapraz derlemesi (PlatformIO) gerekir ve colcon ile derlenmez. Bu yüzden harici bağımlılık olarak tutulur.
- Firmware'in **kaynağı** her zaman `~/reference/andino` @ sabit commit olur. **Değişiklikler** yalnızca `src/fsb_hardware/firmware/patches/*.patch` olarak tutulur.
- `src/fsb_hardware` paketi henüz yok. İlk yama gerektiğinde oluşturulacak; şu an yama yok.

---

## 3. Derleme

Derleme, referans deposunu kirletmemek için **workspace dışındaki** geçici bir dizinde yapılır. `.pio/` derleme çıktısı da bu dizinde oluşur.

Upstream compose dosyası `..` dizinini `/workspace` olarak bağlar (`docker/compose.yaml`). Bu yüzden komut doğrudan referans deposunda çalıştırılırsa `.pio/` dizini `~/reference/andino/andino_firmware` içinde oluşur (`docker/README.md:132`).

### 3.1 Sabit commit'ten derleme ağacı çıkarma

```bash
FW_COMMIT=974264bdefd6ab939ba487ffa95ce3c1525285ff
FW_BUILD=~/fsb_firmware_build          # workspace ve src/ dışında
rm -rf "$FW_BUILD" && mkdir -p "$FW_BUILD"
git -C ~/reference/andino archive "$FW_COMMIT" andino_firmware | tar -x -C "$FW_BUILD"
cd "$FW_BUILD/andino_firmware"
```

`git archive` yalnızca commit'teki içeriği verir. Yerel değişiklikler ve `.git` dizini derleme ağacına girmez.

### 3.2 Yamaları uygulama (varsa)

```bash
PATCH_DIR=~/factory_security_bot_ws/src/fsb_hardware/firmware/patches
if ls "$PATCH_DIR"/*.patch >/dev/null 2>&1; then
  for p in "$PATCH_DIR"/*.patch; do        # dosya adları 0001-, 0002- ... ile sıralı
    git apply --check -p2 "$p" && git apply -p2 "$p" || { echo "YAMA BAŞARISIZ: $p"; exit 1; }
  done
fi
```

- `-p2` seçeneği, yamalar depo kökünden (`a/andino_firmware/...`) üretildiği içindir.
- `git apply --check`, hedef satırlar beklenen yerde değilse yamayı reddeder. Bu, projedeki "yama ancak hedef metin beklenen sayıda bulunursa uygulanır" ilkesinin firmware karşılığıdır.

### 3.3 Derleme — seçenek A: Docker (upstream önerisi)

Kaynak: `README.md:54`, `docker/README.md:29-42`.

```bash
cd "$FW_BUILD/andino_firmware"
docker compose -f docker/compose.yaml build                         # imaj (bir kez)
docker compose -f docker/compose.yaml run --rm dev pio run -e nanoatmega328
```

Çıktı: `.pio/build/nanoatmega328/firmware.hex`. Bu yol PlatformIO'nun standart çıktı yoludur; bu projede **doğrulanmadı**.

### 3.4 Derleme — seçenek B: yerel venv (Docker'sız)

```bash
python3 -m venv ~/.venvs/pio && . ~/.venvs/pio/bin/activate
pip install platformio==6.2.0             # docker/requirements.txt ile aynı sürüm
cd "$FW_BUILD/andino_firmware"
pio run -e nanoatmega328
```

Bu yol upstream'de belgelenmemiştir ve **doğrulanmadı**.

---

## 4. Nano'ya yükleme (flash)

### 4.1 WSL2'de USB erişimi

- WSL2 USB cihazlarını varsayılan olarak görmez. Windows tarafında `usbipd-win` ile cihaz WSL'ye bağlanmalıdır (https://learn.microsoft.com/windows/wsl/connect-usb).
- Windows'ta (yönetici) çalıştırılacak komutlar: `usbipd list` → `usbipd bind --busid <id>` → `usbipd attach --wsl --busid <id>`.
- Linux tarafında kontrol: `ls /dev/ttyUSB* /dev/ttyACM*`. Arduino için genelde `/dev/ttyUSB0` görünür (`README.md:184`).
- Kullanıcının seri porta erişimi için: `sudo usermod -aG dialout $USER`, sonra oturumu yeniden aç.
- CH340 klon Nano'larda `brltty` paketi portu ele geçirebilir. Bu durumda `sudo apt remove brltty` gerekebilir. Bu, genel Ubuntu bilgisidir; bu makinede **doğrulanmadı**.

### 4.2 Yükleme komutu

Docker ile (`docker/README.md:83-96`). Compose `privileged: true` ve `/dev:/dev` ile çalışır:

```bash
docker compose -f docker/compose.yaml run --rm dev pio run -e nanoatmega328 --target upload
```

venv ile:

```bash
pio run -e nanoatmega328 --target upload --upload-port /dev/ttyUSB0
```

### 4.3 Bootloader (eski/yeni) — doğrulanmadı

- `nanoatmega328` env'i PlatformIO'nun Nano board tanımını kullanır. Board sayfası bootloader türünü yazmıyor; ayrı bir "New Bootloader" board'una (`nanoatmega328new`) link veriyor (https://docs.platformio.org/en/latest/boards/atmelavr/nanoatmega328.html).
- CH340'lı klon Nano'ların bir kısmı için ürün açıklamalarında "ATmega328P (Old Bootloader)" seçilmesi öneriliyor.
- Belirti: Yükleme `avrdude: stk500_recv(): programmer is not responding` hatasıyla düşerse bootloader uyumsuzluğu olasıdır. Bu durumda env'e `upload_speed` ayarı veya `board = nanoatmega328new` gerekir. Değişiklik bir yamayla yapılmalı (§5).
- Hangi kartın geleceği bilinmediği için bu madde **doğrulanmadı**.

### 4.4 Yükleme sonrası doğrulama

Kaynak: `README.md:158-186`.

```bash
pio device monitor -b 57600              # her komut CR (\r) ile bitmeli
#   ver           -> 0.2.0
#   getenc        -> "<sol> <sağ>"
#   setpwm 80 80  -> motorlar döner (tekerlekler havada!)
#   setspd 0 0
python tools/hw_check.py --port /dev/ttyUSB0   # upstream donanım doğrulama aracı
```

---

## 5. Yama politikası

1. Yama dizini: `src/fsb_hardware/firmware/patches/`. Dosyalar `NNNN-kisa-aciklama.patch` olarak adlandırılır ve sırayla uygulanır.
2. Üretim adımları:
   - Derleme ağacında (`$FW_BUILD`) geçici bir `git init` + ilk commit oluştur.
   - Değişikliği yap.
   - `git diff --relative` ile ya da depo kökünden `git diff` ile yamayı çıkar.
   - Başlığa amacı, kaynağı ve hangi sabit commit'e karşı üretildiğini yaz.
3. Yamalar `git apply --check` ile kontrol edilmeden uygulanmaz (§3.2).
4. `~/reference/andino` içinde **hiçbir dosya** düzenlenmez.
5. Olası ilk yama adayları:
   - **PWM tavanı (Plan A, DG01D-E için gerekli):** DG01D-E 3–9 V'tur; 3S + L298N ile tam PWM 9 V'u aşabilir. Plan B (N20 12 V) için gerekmez. Bkz. `docs/HARDWARE.md`. İlgili sabit `Motor::kMaxSpeed{255}`, `include/andino/drivers/motor.h:105`.
   - **Varsayılan PID kazançları:** `boards/arduino_uno/andino/board.h:85`.
   - **Bootloader / upload ayarı:** §4.3.

---

## 6. PID kazançları: derleme zamanı mı, çalışma zamanı mı?

**İkisi de.** Kaynaktan doğrulandı:

| Yol | Ayrıntı | Kaynak |
|---|---|---|
| Derleme zamanı (açılış değeri) | `kAppConfig{3000, 30, {30, 10, 0, 10}}` → auto-stop 3000 ms, PID 30 Hz, `{kp, kd, ki, ko} = {30, 10, 0, 10}` | `boards/arduino_uno/andino/board.h:85` (Nano bunu include eder, `boards/arduino_nano/andino/board.h:35`); alan açıklaması `include/andino/app/app.h:51-58` ("they can be changed with a command") |
| Çalışma zamanı (seri komut) | `setpid kp kd ki ko` → `[OK]` | Komut adı `include/andino/app/commands.h:83`; kayıt `src/app/app.cpp:99`; işleyici `src/app/app.cpp:224-240`; README tablo `README.md:127-128,149` |

Çalışma zamanı komutunun özellikleri (`src/app/app.cpp:224-240`):
- Tam olarak 4 argüman alır. Her biri `int16_t` olmalı ve sıkı `parse_int` ile okunur. Ondalık sayı veya fazla/eksik argüman `Invalid arguments` hatası verir. Testler: `test/app/test_app/app_test.cpp:347-407`; örneğin `setpid 40000 ...` ve `ko = 0` reddedilir.
- `ko > 0` zorunludur, çünkü bölen olarak kullanılır.
- Aynı kazanç iki tekerleğe birden uygulanır.
- **Kalıcı değildir.** Kaynakta EEPROM kullanımı bulunmadı (grep). Yeniden başlatmada `kAppConfig` değerlerine döner. Kalıcı ayar için `board.h:85`'i değiştiren bir yama gerekir.
- PID çıkışı ±`Motor::kMaxSpeed` (255) ile sınırlanır (`include/andino/app/app.h:90-93`).

**Host tarafı uyarısı:**
- `andino_base/src/serial_mcu.cpp:114-116` içindeki `set_pid_tuning_gains` değerleri `ss << kp` ile gönderir. Tip float ise ondalıklı metin üretebilir ve firmware bunu reddeder.
- `diffdrive_andino.cpp` içinde bu fonksiyona çağrı bulunmadı (grep).
- Kendi hardware interface'imizde PID gönderilecekse değerler **tamsayıya yuvarlanmalıdır**.

---

## 7. Lisans

| Bileşen | Lisans | Kaynak |
|---|---|---|
| Andino firmware | BSD 3-Clause, "Copyright (c) 2023, Ekumen Inc." | `andino_firmware/LICENSE`; `package.xml:9` ("BSD Clause 3") |
| ros_arduino_bridge kökenli dosyalar | BSD (Patrick Goebel, 2012); dosya başlıklarında ayrıca yer alır | ilgili `src/`/`include/` dosya başlıkları |
| Adafruit BNO055 1.6.4 | MIT | GitHub API (adafruit/Adafruit_BNO055) |
| Adafruit BusIO 1.17.4 | MIT | GitHub API (adafruit/Adafruit_BusIO) |
| Adafruit Unified Sensor 1.1.15 | Apache-2.0 | GitHub API (adafruit/Adafruit_Sensor) |

- Andino deposunun kökünde LICENSE dosyası yoktur; lisans paket bazındadır.
- BSD-3'ün ikinci maddesi gereği, ikili (`firmware.hex`) dağıtılırsa telif bildirimi ve lisans metni dokümantasyonla birlikte verilmelidir.
- Yama dosyaları türev çalışmadır; başlıklarına orijinal BSD bildirimi korunarak eklenmelidir.

---

## 8. Yapılacaklar / doğrulanmadı

- [ ] Docker veya venv kurulup `pio run -e nanoatmega328` ile ilk derleme yapılacak; AVR platform sürümü kaydedilecek.
- [ ] Gelen Nano kartın bootloader türü (eski/yeni) ve USB çipi (CH340/FT232) belirlenecek.
- [ ] WSL2 `usbipd` ile port erişimi denenecek.
- [ ] `ver`, `getenc`, `setpwm`, `setspd` ve `tools/hw_check.py` ile donanım doğrulaması yapılacak.
- [ ] Plan A (DG01D-E) motorlarla PWM tavanı yaması yazılıp test edilecek; Plan B (N20 12 V) seçilirse yama gerekmez (`docs/HARDWARE.md`).
