# AGV UWB-TDOA EKF Takip Projesi Raporu

## 1. Amaç

Bu projenin amacı, dikdörtgen bir pist alanında hareket eden bir aracın konumunu UWB tabanlı TDOA ölçümleriyle tahmin etmek ve bu tahmini Extended Kalman Filter kullanarak iyileştirmektir.

Bu çalışma, ileride pist gözetleme, araç takibi, sensör yerleşimi ve konum kestirimi gibi konulara genişletilebilecek sade bir başlangıç modeli olarak hazırlanmıştır.

---

## 2. Sistem Tanımı

Sistem üç ana bileşenden oluşur:

1. Hareket eden araç
2. Dış dikdörtgenin köşelerine yerleştirilen UWB sensörleri
3. TDOA ölçümlerini işleyen EKF algoritması

Araç, dış gözetleme alanının içinde kalan bir iç dikdörtgen üzerinde sabit hızla hareket eder. Dış dikdörtgenin köşelerinde bulunan sensörler araca ait sinyalin varış zamanı farklarından konum hakkında bilgi üretir.

---

## 3. Hareket Modeli

Araç için kullanılan durum vektörü:

```text
x = [konum_x, konum_y, hız_x, hız_y]^T
```

Sabit hızlı hareket modeli:

```text
x_k = F x_{k-1} + w
```

Burada `w`, model belirsizliğini temsil eden süreç gürültüsüdür.

Geçiş matrisi:

```text
F = [[1, 0, dt, 0 ],
     [0, 1, 0,  dt],
     [0, 0, 1,  0 ],
     [0, 0, 0,  1 ]]
```

---

## 4. Geometri

Dış alan 500 m × 300 m olarak seçilmiştir. Sensörler dış alanın dört köşesine yerleştirilmiştir.

İç pist 400 m × 200 m boyutundadır. Bu nedenle pist çevresi:

```text
2 × (400 + 200) = 1200 m
```

Araç bu iç pist üzerinde saat yönünde hareket eder.

---

## 5. TDOA Ölçüm Modeli

TDOA, sensörlere ulaşan sinyal zamanları arasındaki farkı kullanır. Simülasyonda zaman farkı yerine eşdeğer mesafe farkı kullanılmıştır.

Referans sensör `S0` olarak seçilmiştir.

Ölçüm modeli:

```text
z = [d1 - d0, d2 - d0, d3 - d0]^T
```

Burada `di`, aracın ilgili sensöre olan uzaklığıdır.

Bu ölçüm modeli doğrusal değildir. Çünkü uzaklık ifadesi karekök içerir:

```text
di = sqrt((x - S_i_x)^2 + (y - S_i_y)^2)
```

Bu nedenle klasik Kalman filtresi yerine Extended Kalman Filter kullanılır.

---

## 6. Extended Kalman Filter

EKF iki temel adımdan oluşur:

### 6.1 Tahmin Adımı

Bu adımda araç konumu ve hızı, sabit hızlı hareket modeliyle bir sonraki zamana taşınır.

### 6.2 Güncelleme Adımı

Bu adımda TDOA ölçümü alınır. Ölçüm modeli doğrusal olmadığı için her zaman adımında ölçüm fonksiyonunun Jacobian matrisi hesaplanır.

Filtre, modelden gelen tahmin ile sensör ölçümünden gelen bilgiyi birleştirir.

---

## 7. Sonuçların Yorumlanması

Kod çalıştırıldığında gerçek araç yolu ile EKF tahmini aynı grafikte gösterilir. Sensörler, iç pist ve dış gözetleme alanı da grafikte yer alır.

Konum kestirim başarısı için iki metrik hesaplanır:

- RMSE
- Ortalama mutlak konum hatası

Düşük hata değerleri, EKF'in TDOA ölçümlerini kullanarak araç konumunu başarılı şekilde takip ettiğini gösterir.

---

## 8. Geliştirme Önerileri

Bu proje şu yönlerde geliştirilebilir:

1. TDOA ölçümlerini saniye cinsinden gerçek zaman farkı formatına çevirmek
2. Sensör saat senkronizasyon hatalarını eklemek
3. NLOS etkisini modellemek
4. Araç dönüşlerinde hız geçişlerini daha gerçekçi yapmak
5. Hata elipsi ve kovaryans görselleştirmesi eklemek
6. Sensör yerleşimi optimizasyonu yapmak
7. Gerçek UWB verisiyle test etmek

---

## 9. Genel Değerlendirme

Bu proje, UWB-TDOA ve EKF tabanlı konum kestirimi konusunu öğrenmek için sade, okunabilir ve geliştirilebilir bir simülasyon altyapısı sunar. Özellikle sensör geometrisinin konum kestirimi üzerindeki etkisini görmek ve Kalman filtresi mantığını uygulamalı anlamak için uygun bir başlangıçtır.
