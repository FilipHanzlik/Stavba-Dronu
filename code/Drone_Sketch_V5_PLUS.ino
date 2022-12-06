#include <Wire.h>

// Rate PID gains
#define kproll 0.45
#define kiroll 0.10
#define kdroll 2.33
#define kppitch 0.45
#define kipitch 0.10
#define kdpitch 2.33
#define kpyaw 0.0
#define kiyaw 0.0
#define kdyaw 0.0

// Angle PID gains
#define kprolla 10.0
#define kirolla 10.0
#define kppitcha 0.0
#define kipitcha 0.0

#define PIDMAX  400 // maximalni povolena hodnota v PID cyklu aby se zabránilo překompenzaci
#define MPU 0x68 // Adresa sensor chipu MPU6050 I2C
#define dt 0.004

float AccError[3] = {-0.0594, 0.0187, 0.0022}, GyroError[3] = {3.328, 0.485, -0.035}; // odstranění konstantních chyb z měření sensoru     

float Acc[3] = {0.1, 0.1, 0.1};
float AccLatest[3];
float Gyro[3] = {0.1, 0.1, 0.1};
float GyroLatest[3];
float accAngleX, accAngleY, GyroLatestAngle[2];
float Angle[3], anglePID[3], desAngle[3], ratePID[3], desRate[3];
float pitch_adjust, roll_adjust;
float currentTime;
float ESCCommand[4], channel_vol[4], channel[4]; 
unsigned long ESCTimer[4], generalTimer;
float batteryVoltage = 1140;
bool Armed = false, change = true, change1 = false, STOP = false; // STOP zastaví motory pokud náklon dronu je moc vysoký
float dk;

bool D8_state, D9_state, D10_state, D11_state, D12_state, high;
unsigned long current_time, timer_1, timer_2, timer_3, timer_4;

//float loop_time;

void setup() {

  PCICR |= B00000001;
  PCMSK0 |= B00000010;
  PCMSK0 |= B00000100;
  PCMSK0 |= B00001000;
  PCMSK0 |= B00010000;
  
  //Serial.begin(57600);
  
  DDRC |= B00001110; // LED_green = 15, LED_yellow = 16, LED_red = 17;
  DDRD |= B00111100;
  DDRB |= B00000001;

  Wire.begin();
  TWBR = 12; 
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);                                      //Start communication with the address found during search.
  Wire.write(0x1B);                                                          //We want to write to the GyroLatest_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the GyroLatest

  Wire.beginTransmission(MPU);                                      //Start communication with the address found during search.
  Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();  

  Wire.beginTransmission(MPU);                                  
  Wire.write(0x1A); // DLPF config                                                
  Wire.write(0x03); // 3 is 44Hz attenuation for acc and 42Hz attenuation for GyroLatest, with 4.8ms of delay oof, probs not worth it cmon
  Wire.endTransmission();  

  analogReference(DEFAULT);

//  calculate_IMU_error();

  delay(250);
  currentTime = micros();
}
void loop() {

  if (high){
    PORTB |= B00000001;
  } else {
    PORTB &= B00000000;
  }
  high = !high;

  for (int i = 0; i < 4; i++){
    channel[i] = channel_vol[i];
  }
  
  for (int i = 0; i < 2; i++){
    if ((Angle[i] > 45 || Angle[i] < -45) && !STOP)STOP = true;
  }    

  convert_input();

  calculate_angles();
  
  ratePID[0] = calculate_roll_rate_PID(Gyro[0], 0);// desRate[0]); // Vypočítání PID hodnot na odstranění změřené chyby v náklonu // why -6???
  ratePID[1] = calculate_pitch_rate_PID(Gyro[1], 0); // desRate[1]);
  //ratePID[2] = calculate_yaw_rate_PID(Gyro[2], desRate[2]);

  if (Armed)motor_mixing_algo();
  else for(int i = 0; i < 4; i++)ESCCommand[i] = 900;

  if(micros() - currentTime > 4010)PORTC |= B00000100;
  while(micros() - currentTime < 4000);
  currentTime = micros();
  
  PORTD |= B00111100;
  for (int i = 0; i < 4; i++)ESCTimer[i] = ESCCommand[i] + currentTime;
  
  read_sensors();

  while (PORTD != B00000000){
    generalTimer = micros();
    if(ESCTimer[0] <= generalTimer)PORTD &= B11111011;
    if(ESCTimer[1] <= generalTimer)PORTD &= B11110111;
    if(ESCTimer[2] <= generalTimer)PORTD &= B11101111;
    if(ESCTimer[3] <= generalTimer)PORTD &= B11011111;
  }

  for (int i = 0; i < 3; i++){
    Gyro[i] = Gyro[i] * 0.45 + GyroLatest[i] * 0.55;
    Acc[i] = Acc[i] * 0.75 + AccLatest[i] * 0.25;
  }

  batteryVoltage = batteryVoltage * 0.97 + analogRead(7) * 0.03633;

  if (batteryVoltage <= 1000)PORTC |= B00001000;

  //Serial.println(desRate[1]);
  
  if (channel_vol[0] <= 1050 && channel_vol[1] >= 1950 && !change){
    Armed = !Armed;
    change = true;
    PORTC ^= B00000010;
  } else if (channel_vol[1] <= 1600 and change)change = false;
}

void read_sensors(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Snímání akcelerace pomocí register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Požádat o 6 bytů, každá osa má 2
  for (int i = 0; i < 3; i++){
    AccLatest[i] = (Wire.read() << 8 | Wire.read()) / 4096.0 + AccError[i];
  }
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Data z GyroLatestskopu získány z registru 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  for (int i = 0; i < 3; i++){
    GyroLatest[i] = (Wire.read() << 8 | Wire.read()) / 65.5 + GyroError[i];
  } 
}

void calculate_angles(){
  accAngleX = (atan(Acc[1] / sqrt(pow(Acc[0], 2) + pow(Acc[2], 2))) * 180 / PI); // výpočet náklonu z naměřeného zrychlení
  accAngleY = (atan(-1 * Acc[0] / sqrt(pow(Acc[1], 2) + pow(Acc[2], 2))) * 180 / PI);
  Angle[2] += Gyro[2] * dt;
  Angle[0] = 0.98 * ((Angle[0] + Gyro[0] * dt) - Angle[1] * sin(Angle[2] * 0.01745)) + 0.02 * accAngleX; // Roll
  Angle[1] = 0.98 * ((Angle[1] + Gyro[1] * dt) + Angle[0] * sin(Angle[2] * 0.01745)) + 0.02 * accAngleY; // Pitch
}

void convert_input(){
  for (int i = 0; i < 2; i++){ // Upravení PWM pokynů z vysílače na rychlost změny náklonu
    desRate[i] = 0;
    if (channel[i+2] > 1505)desRate[i] = (channel[i+2] - 1505)/6;
    else if (channel[i+2] < 1495)desRate[i] = (channel[i+2] - 1495)/6;
  }
  desRate[2] = 0;
  if (channel[1] > 1505)desRate[2] = (channel[1] - 1505)/6;
  else if (channel[1] < 1495)desRate[2] = (channel[1] - 1495)/6;
  
  if (channel[0] > 1800)channel[0] = 1800;
}

void motor_mixing_algo(){
  ESCCommand[0] = channel[0] - ratePID[0] + ratePID[2];
  ESCCommand[1] = channel[0] - ratePID[1] - ratePID[2];
  ESCCommand[2] = channel[0] + ratePID[0] + ratePID[2];
  ESCCommand[3] = channel[0] + ratePID[1] - ratePID[2];

  for (int i = 0; i < 4; i++){
    if (batteryVoltage < 1240 && batteryVoltage > 1000)ESCCommand[i] += ESCCommand[i] * ((1240 - batteryVoltage)/3600.0);
    if (ESCCommand[i] < 1100)ESCCommand[i] = 1100;
  }
}

float calculate_roll_rate_PID(float Rate, float desRate){
  const float Kp = kproll;
  const float Ki = kiroll;
  const float Kd = kdroll;
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

float calculate_pitch_rate_PID(float Rate, float desRate){
  const float Kp = kppitch;
  const float Ki = kipitch;
  const float Kd = kdpitch;
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
  const float Kp = kpyaw;
  const float Ki = kiyaw;
  const float Kd = kdyaw;
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

float calculate_roll_angle_PID(float Angle, float desAngle){
  const float Kp = kprolla;
  const float Ki = kirolla;
  float Integral, Error, previousError, PID;

  Error = desAngle = Angle;

  if (Integral > PIDMAX)Integral = PIDMAX;
  else if (Integral < -PIDMAX)Integral = -PIDMAX;
  
  PID = Kp * Error + Ki * Integral;
  previousError = Error;

  if (PID > PIDMAX)PID = PIDMAX;
  else if (PID < -PIDMAX)PID = -PIDMAX;

  return PID;
}

float calculate_pitch_angle_PID(float Angle, float desAngle){
  const float Kp = kppitcha;
  const float Ki = kipitcha;
  float Integral, Error, previousError, PID;

  Error = desAngle = Angle;

  if (Integral > PIDMAX)Integral = PIDMAX;
  else if (Integral < -PIDMAX)Integral = -PIDMAX;
  
  PID = Kp * Error + Ki * Integral;
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
