// NOT TESTED //
#include <Wire.h>

#define PIDMAX  400 // maximalni povolena hodnota v PID cyklu aby se zabránilo překompenzaci
#define MPU 0x68 // Adresa sensor chipu MPU6050 I2C

float AccError[3] = {-0.044, 0.046, 0.029}, GyroError[3] = {3.694, 0.399, -0.306}; // odstranění konstantních chyb z měření sensoru     

float Acc[3];
float Gyro[3];
float accAngleX, accAngleY, gyroAngle[2];
float Angle[3], anglePID[3], desAngle[3], ratePID[3], desRate[3];
float dt, currentTime, previousTime;
float ESCCommand[4], channel_vol[4], channel[4]; 
unsigned long ESCTimer[4], generalTimer;
bool Armed = false, change = true, change1 = false, STOP = false; // STOP zastaví motory pokud náklon dronu je moc vysoký
float dk;

float batteryVoltage = 1140.0;

bool D8_state, D9_state, D10_state, D11_state, D12_state; //high
unsigned long current_time, timer_1, timer_2, timer_3, timer_4;

int c = 0;
float loop_time;

void setup() {

  PCICR |= B00000001;
//  PCMSK0 |= B00000001;
  PCMSK0 |= B00000010;
  PCMSK0 |= B00000100;
  PCMSK0 |= B00001000;
  PCMSK0 |= B00010000;
  
  Serial.begin(57600);
  
  DDRC |= B00001110; // LED_green = 15, LED_yellow = 16, LED_red = 17;
  DDRD |= B00111100;
  
  Wire.begin();
  TWBR = 12; 
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);                                  
  Wire.write(0x1A); // DLPF config                                                
  Wire.write(0x03); // 3 is 44Hz attenuation for acc and 42Hz attenuation for gyro, with 4.8ms of delay
  Wire.endTransmission();  

  analogReference(INTERNAL);
  Serial.println(batteryVoltage);
  
  delay(250);
  currentTime = micros();
}
void loop() {
    
  for (int i = 0; i < 4; i++){
    channel[i] = channel_vol[i];
  }
  
  for (int i = 0; i < 2; i++){
    if ((Angle[i] > 45 || Angle[i] < -45) && STOP)STOP = true;
  }    

  convert_input();

  ratePID[0] = calculate_pitch_rate_PID(Gyro[0], 0); // Vypočítání PID hodnot na odstranění změřené chyby v náklonu
  ratePID[1] = calculate_roll_rate_PID(Gyro[1], 0);
  ratePID[2] = calculate_yaw_rate_PID(Gyro[2], 0);

  batteryVoltage = batteryVoltage * 0.92 + analogRead(7) * 0.0984375; // Měření napětí baterie + komplementární filter ((1260/1024) * 0.08)

  if (Armed)motor_mixing_algo();
  else for(int i = 0; i < 4; i++)ESCCommand[i] = 1000;

  if(micros() - currentTime > 3000)PORTC |= B00000100;
  while(micros() - currentTime < 3000);

  previousTime = currentTime;
  currentTime = micros();
  dt = (currentTime - previousTime) / 1000000;
  
  PORTD |= B00000000;
  for (int i = 0; i < 4; i++)ESCTimer[i] = ESCCommand[i] + currentTime;
  
  read_sensors();

  while (PORTD >= B00111100){
    generalTimer = micros();
    if(ESCTimer[0] <= generalTimer)PORTD &= B11111011;
    if(ESCTimer[1] <= generalTimer)PORTD &= B11110111;
    if(ESCTimer[2] <= generalTimer)PORTD &= B11101111;
    if(ESCTimer[3] <= generalTimer)PORTD &= B11011111;
  }
  
  if (channel_vol[0] <= 1050 && channel_vol[1] >= 1950 && !change){
    Armed = !Armed;
    change = true;
    PORTC ^= B00000010;
  } else if (channel_vol[1] <= 1600 and change)change = false;

  if (channel_vol[2] < 1100 && !change1){
    dk -= 0.2;
    change1 = true;
  } else if (channel_vol[2] > 1900 && !change1){
    dk += 0.2;
    change1 = true;
  } else if (channel_vol[2] < 1510 && channel_vol[2] > 1490 && change1)change1 = false;

  if (channel_vol[3] < 1400){
    Armed = false;
    PORTC &= B11111101;
  }
    
//  if (c < 200) {
//    loop_time += dt;
//    c++;
//  } else {
//    c = 0;
////    Serial.println(loop_time/200, 5);
//    Serial.println(change1);
//    loop_time = 0;
//  }
  Serial.println(batteryVoltage);
//  if (batteryVoltage < 1170 && batteryVoltage > 1150)PORTC |= B00000100;
//  else PORTC &= B11110111;
//
//  if (batteryVoltage < 1160)PORTC |= B00000100;
//  else PORTC &= B11111011;
//  if (batteryVoltage > 1160)PORTC |= B00000010;
//  else PORTC &= B11111101;
}

void read_sensors(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Snímání akcelerace pomocí register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Požádat o 6 bytů, každá osa má 2
  for (int i = 0; i < 3; i++){
    Acc[i] = (Wire.read() << 8 | Wire.read()) / 16384.0 + AccError[i];
  }
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Data z gyroskopu získány z registru 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  for (int i = 0; i < 3; i++){
    Gyro[i] = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroError[i];
  } 
  accAngleX = (atan(Acc[1] / sqrt(pow(Acc[0], 2) + pow(Acc[2], 2))) * 180 / PI); // výpočet náklonu z naměřeného zrychlení
  accAngleY = (atan(-1 * Acc[0] / sqrt(pow(Acc[1], 2) + pow(Acc[2], 2))) * 180 / PI);
}

void calculate_angles(){ 
  Angle[0] = 0.96 * (Angle[0] + Gyro[0] * dt) + 0.04 * accAngleX; // Komplementární filter na zkombinování náklonnových hodnot z 
  Angle[1] = 0.96 * (Angle[1] + Gyro[1] * dt) + 0.04 * accAngleY;
  Angle[2] += Gyro[2] * dt;
}

void convert_input(){
  for (int i = 0; i < 3; i++){ // Upravení PWM pokynů z vysílače na rychlost změny náklonu
    desRate[i+1] = 0;
    if (channel[i+1] > 1505)desRate[i] = (channel[i+1] - 1505)/6;
    else if (channel[i+1] < 1495)desRate[i] = (channel[i+1] - 1495)/6;
  }
  
  if (channel[0] > 1700)channel[0] = 1700;
}

void motor_mixing_algo(){
  ESCCommand[0] = channel[0] - ratePID[0] + ratePID[1] - ratePID[2];
  ESCCommand[1] = channel[0] + ratePID[0] + ratePID[1] + ratePID[2];
  ESCCommand[2] = channel[0] + ratePID[0] - ratePID[1] - ratePID[2];
  ESCCommand[3] = channel[0] - ratePID[0] - ratePID[1] + ratePID[2];

  if (batteryVoltage < 1260 && batteryVoltage > 800){
    for (int i = 0; i < 4; i++){
      ESCCommand[i] += (ESCCommand[i] - 1000) * (1260 - batteryVoltage);
      if (ESCCommand[i] > 2000)ESCCommand[i] = 2000;
      else if (ESCCommand[i] < 1100)ESCCommand[i] = 1100;
    }
  }
}

float calculate_pitch_rate_PID(float Rate, float desRate){
  const int Kp = 0;
  const int Ki = 0;
  const int Kd = 0;
  float Integral, Error, previousError, PID;
  
  Error = desRate - Rate;
  Integral += Error;

  if (Integral > PIDMAX)Integral = PIDMAX;
  else if (Integral < -PIDMAX)Integral = -PIDMAX;
  
  PID = Kp * Error + Ki * Integral + Kd * (Error - previousError);
  previousError = Error;

  if (PID > PIDMAX)PID = PIDMAX;
  else if (PID < -PIDMAX)PID = -PIDMAX;

  return PID;
}

float calculate_roll_rate_PID(float Rate, float desRate){
  const int Kp = 0;
  const int Ki = 0;
  const int Kd = 0;
  float Integral, Error, previousError, PID;
  
  Error = desRate - Rate;
  Integral += Error;

  if (Integral > PIDMAX)Integral = PIDMAX;
  else if (Integral < -PIDMAX)Integral = -PIDMAX;
  
  PID = Kp * Error + Ki * Integral + Kd * (Error - previousError);
  previousError = Error;

  if (PID > PIDMAX)PID = PIDMAX;
  else if (PID < -PIDMAX)PID = -PIDMAX;

  return PID;
}

float calculate_yaw_rate_PID(float Rate, float desRate){
  const int Kp = 0;
  const int Ki = 0;
  const int Kd = 0;
  float Integral, Error, previousError, PID;
  
  Error = desRate - Rate;
  Integral += Error;

  if (Integral > PIDMAX)Integral = PIDMAX;
  else if (Integral < -PIDMAX)Integral = -PIDMAX;
  
  PID = Kp * Error + Ki * Integral + Kd * (Error - previousError);
  previousError = Error;

  if (PID > PIDMAX)PID = PIDMAX;
  else if (PID < -PIDMAX)PID = -PIDMAX;

  return PID;
}

ISR(PCINT0_vect) {
  current_time = micros();
  if (!D9_state && PINB & B00000010) {
    D9_state = true;
    timer_2 = current_time;
  } else if (D9_state && !(PINB & B00000010)) {
    D9_state = false;
    channel_vol[1] = current_time - timer_2;
  }
  if (!D10_state && PINB & B00000100) {
    D10_state = true;
    timer_3 = current_time;
  } else if (D10_state && !(PINB & B00000100)) {
    D10_state = false;
    channel_vol[0] = current_time - timer_3;
  }
  if (!D11_state && PINB & B00001000) {
    D11_state = true;
    timer_4 = current_time;
  } else if (D11_state && !(PINB & B00001000)) {
    D11_state = false;
    channel_vol[2] = current_time - timer_4;
  }
  if (!D8_state && PINB & B00010000) {
    D8_state = true;
    timer_1 = current_time;
  } else if (D8_state && !(PINB & B00010000)) {
    D8_state = false;
    channel_vol[3] = current_time - timer_1;
  }

}
