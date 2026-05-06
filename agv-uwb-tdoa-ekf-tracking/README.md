# AGV UWB-TDOA EKF Tracking Simulation

Bu proje, dikdörtgen bir pist alanında hareket eden bir AGV/araç için **UWB TDOA ölçümleri** ve **Extended Kalman Filter (EKF)** kullanarak konum kestirimi yapar.

Projede amaç, önce basit ve anlaşılır bir senaryo kurup daha sonra sensör geometrisi, gürültü, hareket modeli ve filtre parametreleri üzerinde geliştirme yapabilecek sağlam bir temel oluşturmaktır.

---

## Problem Tanımı

- Gözetleme alanı dikdörtgen olarak modellenmiştir.
- Dış dikdörtgenin köşelerine 4 adet UWB sensörü yerleştirilmiştir.
- Araç, dış alanın içinde kalan ve çevresi **1200 metre** olan bir iç dikdörtgen üzerinde hareket eder.
- Araç, iç dikdörtgenin sol alt köşesinden başlar.
- Hareket sabit hızlıdır.
- Araç düz kenarlarda **20 m/s** hızla ilerler.
- Durum vektörü şu şekildedir:

```text
x = [konum_x, konum_y, hız_x, hız_y]^T
```

---

## Senaryo Geometrisi

Bu başlangıç senaryosunda kullanılan varsayılan ölçüler:

| Eleman | Değer |
|---|---:|
| Dış alan genişliği | 500 m |
| Dış alan yüksekliği | 300 m |
| İç pist genişliği | 400 m |
| İç pist yüksekliği | 200 m |
| İç pist çevresi | 1200 m |
| Araç hızı | 20 m/s |
| Örnekleme aralığı | 0.2 s |

Dış sensör noktaları:

```text
S0 = (0,   0)
S1 = (500, 0)
S2 = (500, 300)
S3 = (0,   300)
```

İç pist köşeleri:

```text
P0 = (50,  50)
P1 = (50,  250)
P2 = (450, 250)
P3 = (450, 50)
```

Kartezyen koordinat sisteminde saat yönünde hareket için araç sol alt köşeden önce yukarı, sonra sağa, sonra aşağı, sonra sola ilerler.

---

## TDOA Ölçüm Modeli

TDOA, sensörlere ulaşan sinyal zamanları arasındaki farkı kullanır. Bu projede sayısal kararlılık için ölçümler doğrudan **mesafe farkı** cinsinden modellenmiştir.

Referans sensör `S0` seçilmiştir. Ölçüm vektörü:

```text
z = [d1 - d0, d2 - d0, d3 - d0]^T
```

Burada:

```text
di = sqrt((x - S_i_x)^2 + (y - S_i_y)^2)
```

---

## EKF Modeli

Tahmin adımı için sabit hızlı hareket modeli kullanılır:

```text
x_k = F x_{k-1} + w
```

```text
F = [[1, 0, dt, 0 ],
     [0, 1, 0,  dt],
     [0, 0, 1,  0 ],
     [0, 0, 0,  1 ]]
```

Ölçüm modeli doğrusal olmadığı için EKF kullanılır. Ölçüm Jacobian matrisi her adımda güncellenir.

---

## Kurulum

```bash
pip install -r requirements.txt
```

---

## Çalıştırma

Ana dizindeyken:

```bash
python agv-uwb-tdoa-ekf-tracking/src/agv_tracking_simulation.py
```

Çalıştırınca konsolda hata metrikleri yazdırılır ve sonuç grafiği `outputs/` klasörüne kaydedilir.

---

## Çıktılar

Kod çalıştığında şu dosya oluşur:

```text
outputs/agv_uwb_tdoa_ekf_result.png
```

Grafikte şunlar görülür:

- Gerçek araç yolu
- EKF ile kestirilen yol
- UWB sensörleri
- İç pist ve dış gözetleme alanı

---

## Dosya Yapısı

```text
agv-uwb-tdoa-ekf-tracking/
├── README.md
├── requirements.txt
├── src/
│   └── agv_tracking_simulation.py
├── docs/
│   └── report.md
└── assets/
    └── agv_simulation_schematic.svg
```

---

## Geliştirme Fikirleri

Bu temel proje üzerine şu geliştirmeler eklenebilir:

1. Sensör sayısını artırmak
2. Sensör yerleşimini optimize etmek
3. Ölçümleri saniye cinsinden gerçek TDOA formatına çevirmek
4. NLOS/ölçüm sıçraması etkilerini eklemek
5. Hareket modelini dönüş dinamiği içerecek şekilde geliştirmek
6. RMSE ve hata elipsi analizi eklemek
7. Gerçek veriyle çalışacak modül oluşturmak

---

## Not

Bu proje eğitim ve simülasyon amaçlıdır. Gerçek UWB sistemlerinde zaman senkronizasyonu, NLOS etkileri, anten gecikmeleri, kalibrasyon ve sensör yerleşim hataları ayrıca modellenmelidir.
