# APEX Kinematics Lab v3.1

META-1 hexapod için firmware geometrisiyle hizalı yarı-dijital ikiz, 3D IK/gait
ve sensör yerleşimi doğrulama arayüzü.

## Çalıştırma

```bash
cd apex_kinematics_lab
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python -m uvicorn app:app --host 0.0.0.0 --port 8090
```

Aç: `http://localhost:8090`

## Notlar

- `simulation`, `live_overlay` ve `live_drive` modları bulunur.
- Canlı veri yalnızca Command Center `:8080` üzerinden okunur; tarayıcı ESP32'ye
  ayrıca sorgu göndermez.
- Live Twin'deki servo açıları fiziksel encoder geri bildirimi değil, firmware'in
  command-state değerleridir.
- Servo açıları ile geometrik/seri-zincir eklem açıları ayrı sözleşmeler olarak
  ele alınır.
- Tripod varsayılanı: `LF + LR + RM` ve karşı fazda `RF + RR + LM`.
- Servo eksenleri her bacak için `+X, -X, +Y, -Y, +Z, -Z` seçilebilir.
- Sensör mount değerleri ve gövde dinamikleri fiziksel ölçüm/CAD ile
  doğrulanana kadar `provisional` kabul edilir.
- Bu fizik motoru değildir; IK, frame, gait ve kalibrasyon doğrulama stüdyosudur.
