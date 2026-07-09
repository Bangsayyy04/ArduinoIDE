#include <IRremote.hpp>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// =========================================================================
// 1. DEKLARASI PIN, SENSOR, DAN VARIABEL GLOBAL
// =========================================================================
#define DHTTYPE DHT22
DHT dht1(2, DHTTYPE); DHT dht2(3, DHTTYPE);
DHT dht3(4, DHTTYPE); DHT dht4(5, DHTTYPE); 

LiquidCrystal_I2C lcd(0x27, 16, 2);
#define IR_SEND_PIN 10
#define RELAY_HEATER_1 30
#define RELAY_HEATER_2 31

const float SETPOINT_SUHU = 25.0;
const float SETPOINT_KELEMBABAN = 50.0;

float t1=0, h1=0, t2=0, h2=0, t3=0, h3=0, t4=0, h4=0;
float avgSuhu = 0.0, avgLembab = 0.0;
float last_eSuhu = 0.0, last_eLembab = 0.0;

unsigned long lastFuzzyTime = 0;
unsigned long lastLcdTime = 0;
int lcdState = 0; 

// --- VARIABEL KENDALI AKTUATOR & PROTEKSI (FINAL) ---
int currentACTemp = 24;
int currentHeaterState = 0; 
unsigned long lastACChange = 0;     
unsigned long lastHeaterChange = 0;

// TIMERS PROTEKSI INDUSTRI
const unsigned long STARTUP_LOCK   = 60000; // 1 Menit Heater Dilarang Nyala di awal
const unsigned long COOLDOWN_AC    = 300000; // 5 Menit Jeda Kompresor AC
const unsigned long COOLDOWN_HEATER= 180000;  // 1 Menit Jeda Relay Heater (Anti Chattering)

// =========================================================================
// 2. KODE IR RAW AC PANASONIC
// =========================================================================
const unsigned int ON_16[] = 
{
 3030,1670, 380,1170, 380,1170, 380,470, 380,470, 380,470, 380,1120, 380,520, 
 330,470, 380,1170, 380,1170, 380,520, 330,1170, 380,470, 430,420, 330,1170, 
 380,1170, 380,520, 330,1170, 380,1170, 380,470, 380,470, 380,1170, 330,520, 
 330,520, 330,1170, 380,470, 380,470, 380,470, 380,470, 380,470, 330,470, 
 380,470, 380,470, 380,470, 380,470, 380,470, 380,470, 330,470, 380,520, 
 330,470, 380,470, 380,470, 380,1170, 330,520, 330,470, 380,1170, 380,470, 
 380,470, 380,1170, 380,1170, 380,470, 380,470, 330,470, 380,470, 380,470, 
 380,470, 380,1170, 380,1170, 380,1170, 330,1220, 330,470, 380,470, 380,470, 
 380,470, 380,470, 380,470, 330,520, 330,470, 380,470, 430,420, 380,1170, 
 380,470, 380,470, 330,520, 330,520, 330,520, 330,470, 380,470, 380,470, 
 330,520, 330,520, 330,470, 380,470, 380,520, 330,470, 380,470, 380,470, 
 330,470, 380,470, 380,470, 380,470, 380,470, 380,470, 380,470, 380,420, 
 380,520, 330,520, 330,470, 380,470, 380,470, 380,470, 330,470, 430,420, 
 380,470, 380,1170, 380,1170, 380,520, 330,1170, 380,470, 380,470, 330,520, 
 330,1170, 380
};

const unsigned int ON_18[] = 
{
  3030,1720, 380,1170, 380,1170, 380,470, 380,470, 380,470, 330,1170, 430,420, 
  380,520, 330,1170, 380,1170, 380,470, 380,1170, 380,470, 330,520, 380,1120, 
  380,1220, 330,470, 380,1170, 380,1120, 430,470, 380,470, 330,1220, 380,470, 
  330,470, 380,1170, 380,470, 380,470, 380,470, 330,470, 380,470, 430,420, 
  380,520, 330,470, 380,470, 380,470, 330,470, 380,520, 330,470, 380,470, 
  380,470, 380,470, 380,470, 380,1120, 430,470, 330,520, 330,1170, 380,470, 
  380,470, 380,1170, 330,1220, 330,470, 430,470, 330,470, 380,470, 380,470, 
  380,470, 380,1170, 380,420, 380,1220, 330,1170, 380,470, 380,470, 380,470, 
  380,470, 330,520, 330,520, 330,470, 380,470, 380,470, 380,470, 380,1170, 
  380,470, 330,520, 330,470, 380,470, 380,470, 380,470, 380,470, 330,520, 
  330,520, 380,420, 380,520, 330,470, 380,470, 380,470, 330,520, 330,470, 
  380,520, 330,520, 330,470, 380,470, 380,470, 330,470, 380,470, 380,470, 
  380,520, 330,470, 380,470, 380,470, 380,420, 380,470, 380,470, 380,470, 
  380,470, 380,1170, 380,470, 380,420, 380,1220, 380,470, 330,470, 380,470, 
  380,1170, 380
};

const unsigned int ON_20[] = 
{
  3030,1720, 330,1220, 330,1220, 330,470, 380,520, 330,470, 380,1170, 380,470, 
  330,570, 330,1170, 330,1220, 380,420, 380,1220, 330,520, 380,420, 380,1170, 
  330,1220, 330,520, 380,1170, 330,1220, 330,520, 330,520, 330,1170, 380,470, 
  380,470, 330,1220, 330,520, 330,520, 330,520, 330,520, 330,470, 380,470, 
  380,470, 380,470, 330,520, 330,520, 330,470, 380,470, 380,520, 330,470, 
  330,520, 330,520, 330,520, 330,1170, 380,470, 380,470, 330,1220, 330,520, 
  380,470, 330,1220, 330,1220, 380,470, 330,470, 330,520, 380,470, 330,520, 
  380,470, 330,1220, 330,1170, 380,470, 380,1170, 380,470, 330,520, 330,520, 
  330,520, 330,520, 330,470, 380,470, 380,470, 330,520, 380,470, 330,1220, 
  330,520, 330,470, 330,520, 380,470, 330,470, 380,520, 380,470, 330,520, 
  330,470, 380,470, 330,520, 380,470, 330,470, 380,520, 330,520, 330,470, 
  380,470, 380,470, 380,470, 380,470, 380,470, 330,470, 380,470, 380,470, 
  380,470, 330,520, 330,520, 330,520, 330,470, 380,520, 330,470, 330,520, 
  380,470, 330,1170, 380,1220, 330,1220, 380,470, 330,470, 380,520, 
  330,520, 330,1170, 380
};

const unsigned int ON_22[] = 
{
  3080,1670, 380,1170, 380,1170, 380,470, 380,470, 330,470, 380,1170, 380,470, 
  380,470, 380,1170, 380,1170, 380,470, 330,1220, 330,470, 380,470, 380,1170, 
  380,1170, 380,470, 380,1120, 430,1170, 380,470, 330,470, 380,1170, 380,520, 
  330,470, 380,1170, 380,470, 380,470, 330,470, 380,470, 380,520, 330,520, 
  330,470, 380,470, 380,470, 330,520, 330,470, 380,470, 380,470, 380,470, 
  380,470, 380,470, 330,470, 380,1170, 380,470, 380,470, 380,1170, 380,470, 
  380,470, 330,1170, 380,1170, 380,470, 380,470, 380,470, 380,470, 380,470, 
  330,520, 330,1170, 380,470, 380,470, 330,1220, 380,470, 380,470, 380,470, 
  330,470, 380,470, 380,470, 380,470, 380,470, 380,470, 380,470, 330,1170, 
  430,420, 380,470, 380,470, 380,470, 380,470, 380,470, 330,470, 380,470, 
  380,520, 330,470, 380,470, 380,470, 380,470, 330,470, 380,470, 380,520, 
  330,470, 380,470, 380,470, 330,520, 330,470, 380,470, 380,470, 380,470, 
  380,470, 380,470, 330,470, 380,470, 380,470, 380,470, 380,470, 380,470, 
  330,520, 330,1170, 380,470, 380,1170, 380,470, 380,470, 380,470, 380,470, 
  380,1170, 380
};

const unsigned int ON_24[] = 
{
  3080,1720, 330,1220, 280,1220, 380,470, 380,470, 330,520, 380,1170, 330,520, 
  330,520, 330,1220, 330,1170, 380,470, 380,1170, 330,520, 380,470, 330,1220, 
  330,1220, 330,520, 330,1170, 380,1170, 380,470, 380,470, 330,1220, 330,520, 
  330,520, 330,1220, 330,470, 380,470, 380,470, 330,520, 330,520, 330,470, 
  330,520, 380,470, 380,470, 380,470, 330,520, 380,470, 330,520, 330,520, 
  330,470, 380,470, 330,520, 330,1220, 330,470, 380,520, 330,1170, 380,470, 
  380,470, 380,1170, 380,1170, 330,520, 330,520, 330,470, 380,520, 330,470, 
  380,470, 380,1170, 380,1170, 330,1170, 380,470, 380,520, 330,470, 380,470, 
  330,520, 330,520, 330,470, 380,520, 330,520, 280,520, 380,470, 380,1170, 
  380,470, 330,470, 380,520, 330,470, 380,470, 330,520, 380,470, 330,520, 
  330,520, 330,470, 380,520, 330,470, 380,470, 330,520, 330,520, 380,470, 
  330,470, 380,520, 330,470, 380,470, 380,470, 330,520, 330,520, 330,520, 
  280,570, 330,470, 380,470, 380,470, 330,470, 380,470, 380,520, 330,520, 
  330,470, 380,1170, 380,1170, 330,520, 380,470, 330,470, 380,520, 330,470, 
  380,1170, 380
};

// Prototipe fungsi
void kirimKodeAC(int suhu);

// Fungsi Trapesium (TrapMF) Fuzzy
float trap(float x, float a, float b, float c, float d) {
  if (x <= a || x >= d) return 0;
  if (x >= b && x <= c) return 1;
  if (x > a && x < b) return (x - a) / (b - a);
  return (d - x) / (d - c);
}

// =========================================================================
// 3. PENGATURAN AWAL SISTEM (SETUP)
// =========================================================================
void setup() {

  // Baca sensor 1x untuk inisialisasi Delta Error
  t1 = dht1.readTemperature(); h1 = dht1.readHumidity();
  if(!isnan(t1) && !isnan(h1)) {
     last_eSuhu = t1 - SETPOINT_SUHU;
     last_eLembab = h1 - SETPOINT_KELEMBABAN;
  }

  Serial.begin(115200);
  
  dht1.begin(); dht2.begin(); dht3.begin(); dht4.begin();
  
  lcd.init(); lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Memulai Sistem");
  lcd.setCursor(0, 1); lcd.print("Menyalakan AC...");

  pinMode(IR_SEND_PIN, OUTPUT);
  digitalWrite(IR_SEND_PIN, LOW);
  IrSender.begin(IR_SEND_PIN);
  
 // --- KONFIGURASI RELAY ACTIVE HIGH ---
  pinMode(RELAY_HEATER_1, OUTPUT);
  pinMode(RELAY_HEATER_2, OUTPUT);
  digitalWrite(RELAY_HEATER_1, LOW); // MATI di awal
  digitalWrite(RELAY_HEATER_2, LOW); // MATI di awal
  currentHeaterState = 0;
  
  delay(1000); 
  
  // Nyalakan AC awal ke 24C untuk pemanasan kompresor (1x tembak)
  kirimKodeAC(24); 
  currentACTemp = 24; 
  
  delay(3000); lcd.clear();
  
  lastFuzzyTime = millis();
  lastLcdTime = millis();
  
  // Penanda Waktu Nyala (Heater dikunci selama 1 menit pertama)
  lastHeaterChange = millis();
}

// =========================================================================
// 4. LOOPING UTAMA (KENDALI KESELURUHAN)
// =========================================================================
void loop() {
  unsigned long now = millis();

  // --- BLOK A: UPDATE LAYAR LCD (Setiap 3 Detik) ---
  if (now - lastLcdTime >= 3000) {
    lastLcdTime = now;
    lcdState++;
    if (lcdState > 2) lcdState = 0; 

    lcd.clear();
    if (lcdState == 0) {
      lcd.setCursor(0, 0); lcd.print("T1:"); lcd.print(t1, 1); lcd.print(" H1:"); lcd.print(h1, 0); lcd.print("%");
      lcd.setCursor(0, 1); lcd.print("T2:"); lcd.print(t2, 1); lcd.print(" H2:"); lcd.print(h2, 0); lcd.print("%");
    } else if (lcdState == 1) {
      lcd.setCursor(0, 0); lcd.print("T3:"); lcd.print(t3, 1); lcd.print(" H3:"); lcd.print(h3, 0); lcd.print("%");
      lcd.setCursor(0, 1); lcd.print("T4:"); lcd.print(t4, 1); lcd.print(" H4:"); lcd.print(h4, 0); lcd.print("%");
    } else if (lcdState == 2) {
      lcd.setCursor(0, 0); lcd.print("AvT:"); lcd.print(avgSuhu, 1); lcd.print(" AC:"); lcd.print(currentACTemp);
      lcd.setCursor(0, 1); lcd.print("AvH:"); lcd.print(avgLembab, 1); lcd.print(" Ht:"); lcd.print(currentHeaterState);
    }
  }

  // --- BLOK B: PEMBACAAN SENSOR & FUZZY LOGIC (Setiap 10 Detik) ---
  if (now - lastFuzzyTime >= 10000) { 
    lastFuzzyTime = now;
    
    // 1. Baca 4 Sensor DHT22
    t1 = dht1.readTemperature(); h1 = dht1.readHumidity();
    t2 = dht2.readTemperature(); h2 = dht2.readHumidity();
    t3 = dht3.readTemperature(); h3 = dht3.readHumidity();
    t4 = dht4.readTemperature(); h4 = dht4.readHumidity();
    
    // 2. Kalkulasi Rata-Rata (Spatial Averaging)
    int validT = 0, validH = 0;
    float sumT = 0, sumH = 0;
    if(!isnan(t1)){ sumT += t1; validT++; } if(!isnan(h1)){ sumH += h1; validH++; }
    if(!isnan(t2)){ sumT += t2; validT++; } if(!isnan(h2)){ sumH += h2; validH++; }
    if(!isnan(t3)){ sumT += t3; validT++; } if(!isnan(h3)){ sumH += h3; validH++; }
    if(!isnan(t4)){ sumT += t4; validT++; } if(!isnan(h4)){ sumH += h4; validH++; }
    
    if (validT > 0) avgSuhu = sumT / validT;
    if (validH > 0) avgLembab = sumH / validH;
    
    // --- TAMBAHKAN FAILSAFE INI ---
    if (validT == 0 || validH == 0) {
        // Jika tidak ada data sensor valid, matikan semua aktuator demi keamanan!
        digitalWrite(RELAY_HEATER_1, LOW);
        digitalWrite(RELAY_HEATER_2, LOW);
        currentHeaterState = 0;
        
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("SYSTEM ERROR!");
        lcd.setCursor(0, 1); lcd.print("SENSOR DISCONNECT");
        return; // Hentikan kalkulasi fuzzy loop ini
    }
    // =========================================================================
    // KENDALI AC (SUHU): Tahan 16°C sampai 26.5°C, lalu Rem Bertahap
    // =========================================================================
    float eT = avgSuhu - SETPOINT_SUHU;
    float deT = eT - last_eSuhu;

    float T_E_NB = trap(eT, -20.0, -20.0, -1.0, -0.5); 
    float T_E_NS = trap(eT, -1.0, -0.5, -0.2, 0.0);
    float T_E_Z  = trap(eT, -0.2, 0.0, 0.5, 1.0);      
    float T_E_PS = trap(eT, 0.5, 1.0, 1.5, 2.0);   
    float T_E_PB = trap(eT, 1.5, 2.0, 20.0, 20.0); 

    // Toleransi deT dilebarkan agar noise sensor tidak membuat AC loncat-loncat
    float T_dE_NB = trap(deT, -5.0, -5.0, -0.2, -0.1);
    float T_dE_NS = trap(deT, -0.2, -0.1, -0.05, 0.05);
    float T_dE_Z  = trap(deT, -0.05, 0.05, 0.1, 0.2); 
    float T_dE_PS = trap(deT, 0.1, 0.2, 0.3, 0.5);
    float T_dE_PB = trap(deT, 0.3, 0.5, 5.0, 5.0);       

    float T_E[] = {T_E_NB, T_E_NS, T_E_Z, T_E_PS, T_E_PB};
    float T_dE[] = {T_dE_NB, T_dE_NS, T_dE_Z, T_dE_PS, T_dE_PB};
    
    int outMatAC[5][5] = {
      {24, 24, 24, 24, 22}, 
      {24, 24, 24, 22, 22}, 
      {22, 22, 20, 18, 18}, 
      {20, 18, 16, 16, 16}, 
      {16, 16, 16, 16, 16}  
    };
    
    float numT = 0, denT = 0;
    for(int i=0; i<5; i++) {
      for(int j=0; j<5; j++) {
        float w = min(T_E[i], T_dE[j]);
        numT += w * outMatAC[i][j];
        denT += w;
      }
    }
    float targetAC = (denT > 0) ? (numT / denT) : 24.0;
    
    // Histeresis AC (Diperlebar batasnya agar tidak chattering 16-18-16)
    int AC_Output_Final = currentACTemp; 
    if (abs(targetAC - currentACTemp) > 1.2) { 
        // Hanya ganti target suhu jika perintah fuzzy bergeser lebih dari 1.2 derajat
        if (targetAC <= 16.5) AC_Output_Final = 16;
        else if (targetAC > 16.5 && targetAC <= 18.5) AC_Output_Final = 18;
        else if (targetAC > 18.5 && targetAC <= 20.5) AC_Output_Final = 20;
        else if (targetAC > 20.5 && targetAC <= 22.5) AC_Output_Final = 22;
        else if (targetAC > 22.5) AC_Output_Final = 24;
    }

    // =========================================================================
    // KENDALI HEATER: MODIFIKASI RENTANG (Ide: 100-80% Lvl 2 | 80-60% Lvl 1 | <60% Mati)
    // =========================================================================
    float eH = avgLembab - SETPOINT_KELEMBABAN; // Error Target 50%
    float deH = eH - last_eLembab;

    // Titik Trapesium digeser ke batas 10 (Target 60%) dan 30 (Target 80%)
    float H_E_NB = trap(eH, -50.0, -50.0, -25.0, -15.0); 
    float H_E_NS = trap(eH, -25.0, -15.0, -5.0, 0.0);    
    float H_E_Z  = trap(eH, -5.0, -2.0, 8.0, 12.0);      // < 60% = Mati
    float H_E_PS = trap(eH, 8.0, 12.0, 28.0, 32.0);      // 60-80% = Lvl 1
    float H_E_PB = trap(eH, 28.0, 32.0, 50.0, 50.0);     // 80-100% = Lvl 2

    // Toleransi deH dilebarkan dari noise
    float H_dE_NB = trap(deH, -20.0, -20.0, -2.0, -1.0);
    float H_dE_NS = trap(deH, -2.0, -1.0, -0.2, 0.2);
    float H_dE_Z  = trap(deH, -0.2, 0.2, 1.0, 2.0);
    float H_dE_PS = trap(deH, 1.0, 2.0, 4.0, 8.0);
    float H_dE_PB = trap(deH, 4.0, 8.0, 20.0, 20.0);

    float H_E[] = {H_E_NB, H_E_NS, H_E_Z, H_E_PS, H_E_PB};
    float H_dE[] = {H_dE_NB, H_dE_NS, H_dE_Z, H_dE_PS, H_dE_PB};
    
    int outMatHeater[5][5] = {
      
 {0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0}, 
      {0, 0, 0, 0, 0}, 
      {0, 0, 1, 1, 1}, 
      {1, 1, 2, 2, 2}     };
    
    float numH = 0, denH = 0;
    for(int i=0; i<5; i++) {
      for(int j=0; j<5; j++) {
        float w = min(H_E[i], H_dE[j]);
        numH += w * outMatHeater[i][j];
        denH += w;
      }
    }
    float targetHeater = (denH > 0) ? (numH / denH) : 0;
    
    int Heater_Output_Final;
    if (abs(targetHeater - currentHeaterState) > 0.65) { 
      Heater_Output_Final = round(targetHeater);
    } else {
      Heater_Output_Final = currentHeaterState;
    }

    // =========================================================================
    // EKSEKUSI HARDWARE DENGAN SISTEM PENGAMAN
    // =========================================================================
    
    if (AC_Output_Final != currentACTemp) {
      if (now - lastACChange >= COOLDOWN_AC || lastACChange == 0) {
        kirimKodeAC(AC_Output_Final);
        currentACTemp = AC_Output_Final;
        lastACChange = now;
      }
    }

    if (Heater_Output_Final != currentHeaterState) {
      if (now > STARTUP_LOCK) { 
        if (now - lastHeaterChange >= COOLDOWN_HEATER || lastHeaterChange == 0) {
          if (Heater_Output_Final == 0) {
            digitalWrite(RELAY_HEATER_1, LOW); 
            digitalWrite(RELAY_HEATER_2, LOW); 
          } else if (Heater_Output_Final == 1) {
            digitalWrite(RELAY_HEATER_1, HIGH); 
            digitalWrite(RELAY_HEATER_2, LOW);  
          } else { 
            digitalWrite(RELAY_HEATER_1, HIGH); 
            digitalWrite(RELAY_HEATER_2, HIGH); 
          }
          currentHeaterState = Heater_Output_Final;
          lastHeaterChange = now;
        }
      }
    }
    
    // =========================================================================
    // PENGIRIMAN DATA 16 PARAMETER KE MATLAB
    // =========================================================================
    // Format: Suhu, Lembab, AC, Heater, eT, deT, eH, deH, T1, H1, T2, H2, T3, H3, T4, H4
    Serial.print(avgSuhu);          Serial.print(",");
    Serial.print(avgLembab);        Serial.print(",");
    Serial.print(currentACTemp);    Serial.print(",");
    Serial.print(currentHeaterState); Serial.print(",");
    Serial.print(eT);               Serial.print(",");
    Serial.print(deT);              Serial.print(",");
    Serial.print(eH);               Serial.print(",");
    Serial.print(deH);              Serial.print(",");
    Serial.print(t1);               Serial.print(",");
    Serial.print(h1);               Serial.print(",");
    Serial.print(t2);               Serial.print(",");
    Serial.print(h2);               Serial.print(",");
    Serial.print(t3);               Serial.print(",");
    Serial.print(h3);               Serial.print(",");
    Serial.print(t4);               Serial.print(",");
    Serial.println(h4);
    
    last_eSuhu = eT;
    last_eLembab = eH;
  }
}

// =========================================================================
// FUNGSI PENGIRIMAN IR RAW AC PANASONIC
// =========================================================================
void kirimKodeAC(int suhu) {
  int khz = 38; 
  int jumlahTembakan = 2; // Tembak 2x agar AC tidak budeg menerima sinyal

  if (suhu == 16) {
    int length = sizeof(ON_16) / sizeof(ON_16[0]);
    for(int i = 0; i < jumlahTembakan; i++) { IrSender.sendRaw(ON_16, length, khz); delay(500); }
  } else if (suhu == 18) {
    int length = sizeof(ON_18) / sizeof(ON_18[0]);
    for(int i = 0; i < jumlahTembakan; i++) { IrSender.sendRaw(ON_18, length, khz); delay(500); }
  } else if (suhu == 20) {
    int length = sizeof(ON_20) / sizeof(ON_20[0]);
    for(int i = 0; i < jumlahTembakan; i++) { IrSender.sendRaw(ON_20, length, khz); delay(500); }
  } else if (suhu == 22) {
    int length = sizeof(ON_22) / sizeof(ON_22[0]);
    for(int i = 0; i < jumlahTembakan; i++) { IrSender.sendRaw(ON_22, length, khz); delay(500); }
  } else if (suhu == 24) {
    int length = sizeof(ON_24) / sizeof(ON_24[0]);
    for(int i = 0; i < jumlahTembakan; i++) { IrSender.sendRaw(ON_24, length, khz); delay(500); }
  }
}
