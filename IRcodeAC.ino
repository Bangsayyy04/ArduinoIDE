#include <Arduino.h>
#include <IRremote.hpp>

const int IR_RECEIVE_PIN = 11;

void setup() {
  Serial.begin(115200);
  
  // Tunggu Serial Monitor terbuka
  while (!Serial); 

  Serial.println("==================================================");

  Serial.println("   PROGRAM PEREKAM KODE REMOTE AC (RAW DATA)      ");
  Serial.println("==================================================");
  Serial.print("Mendengarkan sinyal IR pada Pin ");
  Serial.println(IR_RECEIVE_PIN);
  Serial.println("Cara Pakai:");
  Serial.println("1. Arahkan remote AC langsung ke sensor.");
  Serial.println("2. Tekan tombol ON atau atur suhu ke 25, 20, atau 16.");
  Serial.println("3. Copy array angka yang muncul di bawah.");
  Serial.println("--------------------------------------------------\n");
  
  // Memulai penerima IR dengan indikator LED bawaan Arduino (Pin 13 akan berkedip saat menerima)
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  // Jika ada sinyal IR yang masuk
  if (IrReceiver.decode()) {
    Serial.println("\n\n>>> SINYAL TERDETEKSI! <<<");
    
    // Cetak info singkat protokolnya (sebagai informasi tambahan)
    IrReceiver.printIRResultShort(&Serial);
    
    Serial.println("\n// --- COPY KODE DI BAWAH INI MULAI DARI SINI ---");
    
    // Fungsi khusus IRremote v3 untuk mencetak Raw Data dalam format Array C++
    IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, true);
    
    Serial.println("// --- COPY SAMPAI SINI ---");
    Serial.println("--------------------------------------------------");
    
    // Jeda sebentar agar tidak merekam sinyal yang memantul ganda
    delay(1000); 
    
    // Bersihkan buffer, siap menerima sinyal berikutnya
    IrReceiver.resume();
  }
}