/*
GALAXY-ROCKET.ino
VERSION: V1.5.0 (Launch/Landing Recording System and Code Optimization Update)
---------------------------------------------------==============WARNING===============----------------------------------------------
              THIS CODE WAS SPECIFICALLY DESIGNED FOR THE ARDUINO MEGA 2560, MEGA, LEONARDO OR OTHERS WITH MORE MEMORY            
  IF YOU ARE USING AN ARDUINO UNO OR SIMILAR, CONSIDER CHANGING THE ARDUINO MICROCONTROLLER TO THE MEGA 2560, LEONARDO, OR SIMILAR!
---------------------------------------------------==============WARNING===============----------------------------------------------
*/

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <LoRa.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include <SD.h>
#include <math.h>

#define PI 3.14159265358979323846
File dataFile;
#define SD_CS 26

// Pinos PCB Arduino Mega 2560
#define IGNITER_PIN 2       
#define VALVE_PWM_PIN 3     
#define STAGE_SEP_PIN 4     
#define DROGUE_PIN 5        
#define MAIN_PIN 6          
#define SERVO_FIN1 7        // Yaw/Guidance
#define SERVO_FIN2 8        // Pitch 
#define SERVO_FIN3 9        // Roll
#define GPS_RX 10
#define GPS_TX 11
#define BUZZER 12
#define AlarmLED 13

Servo fin1;
Servo fin2;
Servo fin3;

// === CONFIGURAÇÕES AVANÇADAS ===
#define GROUND_TEST_MODE false 
#define FIN_LIMIT 35            
#define ACTUATOR_RATE 150       
#define MAX_LAT_ACCEL 15.0      
#define ABORT_ANGLE 45.0        
#define IGNITION_CONTROL_DELAY 500 
#define BATT_MONITOR_PIN A15
#define VOLTAGE_THRESHOLD 6.5 
#define MAX_SERVO_VELOCITY 40.0 
#define CURRENT_SAFE_ZONE 0.8   

// Airbrake Servo Pins
#define AB_PIN_1 22
#define AB_PIN_2 23
#define AB_PIN_3 24
#define AB_PIN_4 25

Servo airbrake1, airbrake2, airbrake3, airbrake4;

#define CameraSerial Serial2

// Control Constants
const int AB_RETRACTED = 0;   
const int AB_DEPLOYED  = 85;  
const float AB_VEL_THRESHOLD = 15.0; 

// State Variables
float currentABPosition = 0;
const float TARGET_LAT = -15.84028;  
const float TARGET_LON = -48.02778;
const float TARGET_SAFE_ALT = 450.0; 
const float LANDING_RADIUS = 100.0;

// EEPROM Layout
#define PID_ADDR 0
#define CALIB_ADDR (PID_ADDR + 16) // Ajustado para tamanho fixo simples

// Estruturas de Dados
struct PIDGains { float kp, ki, kd, i_limit; };
PIDGains pidGains[4] = { 
  {2.5, 0.4, 0.2, 30.0},   // Roll
  {2.5, 0.4, 0.2, 30.0},   // Pitch  
  {4.0, 1.2, 0.8, 45.0},   // Yaw/Guidance
  {2.0, 0.5, 0.3, 25.0}    // Landing Precision
};

struct Bias { float ax, ay, az, gx, gy, gz; } bias = {0};

struct INS_State {
  float x, y, z, vx, vy, vz, roll, pitch, yaw, ax, ay, az, gx, gy, gz;
  unsigned long last_update;
} ins = {0};

struct WaypointGuidance {
  float lateral_accel_cmd; 
  float miss_distance;
  float range_to_target;
  float kinematic_limit;   
  float cross_track_error;
  bool in_range;
  bool target_acquired;
} guide = {0};

struct Telemetry {
  uint32_t timestamp;
  float alt, vel, accelZ, lat, lon, heading, roll, pitch, yaw;
  float dist_to_target, lat_accel_cmd;
  int16_t phase;
  bool landing_mode;
  bool abort_triggered;
  float apogee_alt;
  uint8_t checksum;
} telem = {0};

// Variáveis de Controle
float last_fin_pos[3] = {90, 90, 90};
float integrator[4] = {0};
unsigned long last_micros = 0, last_telem = 0, last_log = 0, liftoff_millis = 0;
int flightPhase = 0;
bool landingMode = false, gpsValid = false, sdReady = false, abortTriggered = false;

// VARIÁVEIS FILTRO DE KALMAN
float K_alt = 0, K_vel = 0; 
float P_00 = 1, P_01 = 0, P_10 = 0, P_11 = 1; 
float Q_accel = 0.01; 
float R_alt = 0.5;    

Adafruit_BMP280 bmp;
TinyGPSPlus gps;
#define gpsSerial Serial1

enum BeepMode { BLOCKING, ASYNC_START, ASYNC_UPDATE };

// Protótipos de Funções
void MPU6050_init();
void MPU6050_read();
void INS_calibrate();
void centerServos();
void loadEEPROM();
void initBinaryLogging();
void logBinaryData();
void sendTelemetry();
uint8_t calcChecksum();
void checkGroundCommands();
void simulateHITL(float dt);
void valveControl();
void chuteLogic();
void applyActuatorPhysics(float r_cmd, float p_cmd, float y_cmd, float dt);
void calculateGuidance(float dt);
void executeControl(float dt);
void updateFlightPhase();
void checkAbortLogic();
void readSensorsKalman(float dt);

void beep(int n = 0, BeepMode mode = ASYNC_UPDATE) {
  static int buzzerBeeps = 0;
  static unsigned long buzzerTimer = 0;

  if (mode == BLOCKING) {
    for (int i = 0; i < n; i++) {
      digitalWrite(BUZZER, HIGH); delay(80);
      digitalWrite(BUZZER, LOW); delay(120);
    }
  }
  else if (mode == ASYNC_START) { buzzerBeeps = n; }
  else { 
    if (buzzerBeeps > 0 && millis() - buzzerTimer > 200) {
      digitalWrite(BUZZER, !digitalRead(BUZZER));
      buzzerTimer = millis();
      if (digitalRead(BUZZER) == LOW) buzzerBeeps--;
    }
  }
}

void setup() {
  wdt_enable(WDTO_2S);
  Serial.begin(115200);
  Serial1.begin(9600);
  CameraSerial.begin(115200);
  Wire.begin();
  
  Serial.println(F("=== GALAXY ROCKET - V1.4.0 ACTIVE ==="));
  loadEEPROM();
  
  if (!bmp.begin(0x76)) { 
    while(1) { beep(5, BLOCKING); wdt_reset(); } 
  }
  
  sdReady = SD.begin(SD_CS);
  if(sdReady) initBinaryLogging();
  
  MPU6050_init();
  INS_calibrate();

  airbrake1.attach(AB_PIN_1); airbrake2.attach(AB_PIN_2);
  airbrake3.attach(AB_PIN_3); airbrake4.attach(AB_PIN_4);
  airbrake1.write(AB_RETRACTED); airbrake2.write(AB_RETRACTED);
  airbrake3.write(AB_RETRACTED); airbrake4.write(AB_RETRACTED);
  
  pinMode(IGNITER_PIN, OUTPUT); digitalWrite(IGNITER_PIN, LOW);
  pinMode(STAGE_SEP_PIN, OUTPUT); digitalWrite(STAGE_SEP_PIN, LOW);
  pinMode(DROGUE_PIN, OUTPUT); digitalWrite(DROGUE_PIN, LOW);
  pinMode(MAIN_PIN, OUTPUT); digitalWrite(MAIN_PIN, LOW);
  pinMode(BUZZER, OUTPUT); 
  pinMode(VALVE_PWM_PIN, OUTPUT);
  pinMode(AlarmLED, OUTPUT);
  
  fin1.attach(SERVO_FIN1, 500, 2500);
  fin2.attach(SERVO_FIN2, 500, 2500);
  fin3.attach(SERVO_FIN3, 500, 2500);
  centerServos();
  
  if (!LoRa.begin(433E6)) { 
    while(1) { beep(10, BLOCKING); wdt_reset(); } 
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setTxPower(20);
  
  last_micros = micros();
  beep(3, BLOCKING);
  
  CameraSerial.println("CHECK"); 
  long t = millis();
  while(millis() - t < 2000) {
    if(CameraSerial.available()) {
      String resp = CameraSerial.readString();
      if(resp.indexOf("READY") >= 0) {
        Serial.println("CAMERA OK!");
        beep(2, BLOCKING); // Beep duplo confirma câmera pronta
        break;
      }
    }
  }
  wdt_reset();
}

void checkBatterySafety() {
  float sensorValue = analogRead(BATT_MONITOR_PIN);
  float voltage = sensorValue * (5.0 / 1023.0) * 5.54; 
  if (voltage < VOLTAGE_THRESHOLD && flightPhase >= 1) {
    abortTriggered = true;
    centerServos(); 
  }
}

void executeAirbrakes(float dt) {
  float targetPosition = AB_RETRACTED;
  if (flightPhase == 2 && K_vel > AB_VEL_THRESHOLD) {
    targetPosition = constrain(map(K_vel, 150.0, 350.0, AB_RETRACTED, AB_DEPLOYED), AB_RETRACTED, AB_DEPLOYED);
    if (currentABPosition < targetPosition) currentABPosition += 80.0 * dt; 
    else currentABPosition -= 120.0 * dt;
  } else {
    currentABPosition = AB_RETRACTED;
  }
  airbrake1.write((int)currentABPosition);
  airbrake2.write((int)currentABPosition);
  airbrake3.write((int)currentABPosition);
  airbrake4.write((int)currentABPosition);
}

void loop() {
  wdt_reset();
  unsigned long current_micros = micros();
  float dt = constrain((current_micros - last_micros) / 1000000.0f, 0.001f, 0.1f);
  last_micros = current_micros;

  if (GROUND_TEST_MODE) simulateHITL(dt);
  else readSensorsKalman(dt);

  checkAbortLogic();
  calculateGuidance(dt);
  updateFlightPhase();
  checkBatterySafety();

  if (!abortTriggered && flightPhase >= 1) {
    if (millis() - liftoff_millis > IGNITION_CONTROL_DELAY) {
      executeControl(dt);
      executeAirbrakes(dt);
    } else {
      centerServos();
    }
    valveControl();
  } else if (abortTriggered) {
    centerServos();
    digitalWrite(DROGUE_PIN, HIGH);
  } else {
    centerServos();
  }

  chuteLogic();

  if (millis() - last_telem > 100) { sendTelemetry(); last_telem = millis(); }
  if (sdReady && (millis() - last_log > 10)) { logBinaryData(); last_log = millis(); }

  checkGroundCommands();
  beep(0, ASYNC_UPDATE);

  digitalWrite(AlarmLED, HIGH); delay(50);
  digitalWrite(AlarmLED, LOW); delay(50);
}

void updateFlightPhase() {
  static float mx_alt = 0;
  if (telem.alt > mx_alt) mx_alt = telem.alt;
  telem.apogee_alt = mx_alt; 
  telem.landing_mode = landingMode;

  switch(flightPhase) {
    case 0: 
      if(telem.accelZ > 20.0) { 
        CameraSerial.println("REC_START");
        flightPhase = 1; 
        liftoff_millis = millis(); 
      } 
      break;
    case 1: 
      if(telem.vel < 2.0 && millis() - liftoff_millis > 2000) { 
        flightPhase = 2; 
      } 
      break;
    case 2: 
      if(telem.vel < -5.0) { 
        digitalWrite(DROGUE_PIN, HIGH); 
        flightPhase = 3; 
      } 
      break;
    case 3: 
      if(telem.alt < 2000) { 
        digitalWrite(MAIN_PIN, HIGH); 
        flightPhase = 4; 
      } 
      if(guide.target_acquired && guide.range_to_target < LANDING_RADIUS && telem.alt < 1000) landingMode = true; 
      break;
    case 4: 
      if(telem.alt < 20) { 
        CameraSerial.println("REC_STOP"); 
        flightPhase = 5; 
        centerServos();
      }
      break;
  }
  telem.phase = flightPhase;
}

void checkAbortLogic() {
  float total_aoa = sqrt(telem.roll*telem.roll + telem.pitch*telem.pitch);
  if (flightPhase == 1 && total_aoa > ABORT_ANGLE) {
    abortTriggered = true;
    telem.abort_triggered = true;
    flightPhase = 3;
    beep(10, ASYNC_START);
  }
}

void readSensorsKalman(float dt) {
  MPU6050_read();
  float measured_alt = bmp.readAltitude(1013.25);
  float accel_z = (ins.az * 9.81); 

  K_alt += K_vel * dt + 0.5 * accel_z * dt * dt;
  K_vel += accel_z * dt;

  P_00 += dt * (P_10 + P_01 + dt * P_11) + Q_accel;
  P_01 += dt * P_11; P_10 += dt * P_11; P_11 += Q_accel;

  float S = P_00 + R_alt;
  float k0 = P_00 / S; float k1 = P_10 / S;
  float y = measured_alt - K_alt;
  K_alt += k0 * y; K_vel += k1 * y;

  P_00 -= k0 * P_00; P_01 -= k0 * P_01; P_10 -= k1 * P_00; P_11 -= k1 * P_01;

  telem.alt = K_alt; telem.vel = K_vel; telem.accelZ = accel_z;

  while(gpsSerial.available()) gps.encode(gpsSerial.read());
  if(gps.location.isValid()) {
    telem.lat = gps.location.lat();
    telem.lon = gps.location.lng();
    telem.heading = gps.course.deg();
    gpsValid = true;
  }

  static float f_roll = 0, f_pitch = 0;
  float a_roll = atan2(ins.ay, ins.az) * 180/PI;
  float a_pitch = atan2(-ins.ax, sqrt(ins.ay*ins.ay + ins.az*ins.az)) * 180/PI;
  f_roll = 0.98 * (f_roll + ins.gx * dt) + 0.02 * a_roll;
  f_pitch = 0.98 * (f_pitch + ins.gy * dt) + 0.02 * a_pitch;
  telem.roll = f_roll; telem.pitch = f_pitch;
}

void executeControl(float dt) {
  float scale = (fabs(telem.vel) / 343.0 > 0.8) ? 0.6 : 1.0;
  float error[3] = { 0 - telem.roll, 0 - telem.pitch, 0 }; 
  
  for(int i = 0; i < 3; i++) {
    static float prev_err[3] = {0};
    float p = pidGains[i].kp * error[i] * scale;
    integrator[i] = constrain(integrator[i] + pidGains[i].ki * error[i] * dt, -pidGains[i].i_limit, pidGains[i].i_limit);
    float d = pidGains[i].kd * (error[i] - prev_err[i]) / dt * scale;
    prev_err[i] = error[i];
    error[i] = p + integrator[i] + d; // Reutilizando array para saída
  }
  applyActuatorPhysics(error[0], error[1], error[2], dt);
}

void applyActuatorPhysics(float r_cmd, float p_cmd, float y_cmd, float dt) {
  float t[3];
  t[0] = 90 + r_cmd + p_cmd;
  t[1] = 90 + r_cmd - (0.5 * p_cmd) + (0.866 * y_cmd);
  t[2] = 90 + r_cmd - (0.5 * p_cmd) - (0.866 * y_cmd);

  float max_move = ACTUATOR_RATE * dt; 
  for(int i=0; i<3; i++) {
    t[i] = constrain(t[i], 90 - FIN_LIMIT, 90 + FIN_LIMIT);
    last_fin_pos[i] += constrain(t[i] - last_fin_pos[i], -max_move, max_move);
  }
  fin1.write(last_fin_pos[0]); fin2.write(last_fin_pos[1]); fin3.write(last_fin_pos[2]);
}

void valveControl() {
  int p = 0;
  if(flightPhase == 1) p = map(telem.alt, 0, 4500, 180, 255);
  analogWrite(VALVE_PWM_PIN, constrain(p, 0, 255));
}

void chuteLogic() { if(flightPhase >= 5) { digitalWrite(DROGUE_PIN, LOW); digitalWrite(MAIN_PIN, LOW); } }

void MPU6050_init() {
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
}

void MPU6050_read() {
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);
  int16_t ax = Wire.read()<<8|Wire.read(); int16_t ay = Wire.read()<<8|Wire.read(); int16_t az = Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = Wire.read()<<8|Wire.read(); int16_t gy = Wire.read()<<8|Wire.read(); int16_t gz = Wire.read()<<8|Wire.read();
  ins.ax = (ax / 16384.0) - bias.ax; ins.ay = (ay / 16384.0) - bias.ay; ins.az = (az / 16384.0) - bias.az;
  ins.gx = (gx / 131.0 * PI/180) - bias.gx; ins.gy = (gy / 131.0 * PI/180) - bias.gy; ins.gz = (gz / 131.0 * PI/180) - bias.gz;
}

void INS_calibrate() {
  for(int i = 0; i < 100; i++) { MPU6050_read(); delay(10); wdt_reset(); }
}

void calculateGuidance(float dt) {
  if(!gpsValid) return;
  float dLat = TARGET_LAT - telem.lat;
  float dLon = TARGET_LON - telem.lon;
  guide.range_to_target = sqrt(dLat*dLat + dLon*dLon) * 111319.0;
}

void initBinaryLogging() { dataFile = SD.open("LOG.DAT", FILE_WRITE); if(dataFile) dataFile.close(); }
void logBinaryData() {
  dataFile = SD.open("LOG.DAT", FILE_WRITE);
  if(dataFile) { dataFile.write((const uint8_t*)&telem, sizeof(Telemetry)); dataFile.close(); }
}

void sendTelemetry() {
  telem.timestamp = millis();
  telem.checksum = calcChecksum();
  LoRa.beginPacket(); LoRa.write((uint8_t*)&telem, sizeof(Telemetry)); LoRa.endPacket();
}

uint8_t calcChecksum() {
  uint8_t sum = 0; uint8_t* p = (uint8_t*)&telem;
  for(unsigned int i=0; i<sizeof(Telemetry)-1; i++) sum ^= p[i];
  return sum;
}

void checkGroundCommands() {
  if(LoRa.parsePacket()) {
    uint8_t c = LoRa.read();
    if(c == 0xAA) digitalWrite(IGNITER_PIN, HIGH);
  }
}

void simulateHITL(float dt) { telem.alt += 10 * dt; }
void loadEEPROM() { EEPROM.get(PID_ADDR, pidGains); }
void centerServos() { fin1.write(90); fin2.write(90); fin3.write(90); }