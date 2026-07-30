#include <Arduino.h>
#include <IRremote.hpp>

const int IR_RECEIVE_PIN = 11;

void setup() {
  Serial.begin(115200);
  
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
  
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}

void loop() {
  if (IrReceiver.decode()) {
    Serial.println("\n\n>>> SINYAL TERDETEKSI! <<<");
    
    IrReceiver.printIRResultShort(&Serial);
    
    Serial.println("\n// --- COPY KODE DI BAWAH INI MULAI DARI SINI ---");
    
    IrReceiver.compensateAndPrintIRResultAsCArray(&Serial, true);
    
    Serial.println("// --- COPY SAMPAI SINI ---");
    Serial.println("--------------------------------------------------");
    
    delay(1000); 
    
    IrReceiver.resume();
  }
}
