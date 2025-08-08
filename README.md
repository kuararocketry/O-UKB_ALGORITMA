Kuara Roket Takımı Aviyonik Birimi olarak, Orta İrtifa Roket Yarışması kapsamında geliştirilen STM32F407 tabanlı özgün uçuş kontrol bilgisayarımızın yazılımı.

Bu projede NEO-6M GPS, MPU9250 IMU ve MS5611 altimetre modülleri kullanılmıştır. GPS modülünün ve RS232 portunun sürekli dinlenmesi için Interrupt kullanılmıştır. Yer istasyonu ile haberleşme için E22-900T22D modülü kullanılmaktadır. 

ROKETSAN tarafından gerçekleştirilen UKB yazılımı için de veriler Big Endian formatına dönüştürülmüş olup ilgili eke göre SUT ve SİT testlerini başarıyla tamamlamıştır. 

Not: GPS modülü olarak NEO 6M, NEO 7M, NEO 8M modülleri kullanılabilir. NMEA mesaj formatına uygun olanve kullanılan "lwgps" kütüphanesi sayesinde bu diğer iki GPS modülünü de kullanma imkanı sunulmaktadır.

Not 2: Yer istasyonuna verilerin indirilmesi için bir kod yazılmamıştır. Yer istasyonunda bulunan USB-TTL Dönüştürücü'nün LoRa ile bağlantısı yapılmış olup veriler bu şekilde alınmıştır. 
Ayrıca bir alıcı kodu yazılması için Callback fonksiyonunun yazılması ve interrupt kullanılması yeterli olacaktır.

Not 3: RF Settings uygulaması üzerinde gerekli düzenlemeleri yaptıktan sonra kodun içerisinde adres_h, adres_l ve kanal değişkenlerine "HEX" olarak ilgili değişiklikler yazılmalı. Alıcı için de Callback fonksiyonu ile proje oluşturulabilir ve interrupt açılmalıdır.
