#include <Wire.h>
#include <Servo.h>

Servo motor1, motor2, motor3, motor4;

int PIDMAX = 400;
const int MPU = 0x68; // Adresa chipu MPU6050 I2C

float AccError[3] = {-0.052, 0.043, 0.029}, GyroError[3] = {3.445, 0.373, -0.394};       
 
float Acc[3];
float accArray[3][9];
float filteredGyro[3], Gyro[3], Gyro_1[3], Gyro_2[3], filteredGyro_1[3], filtered_1[3], filtered_2[3];
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float Angle[3], anglePID[3], desAngle[3], ratePID[3], desRate[3];
float dt, currentTime, previousTime;
float ESCCommand[4], channel_vol[5]; 
bool Armed = false, thStickUp = false, cntrSwitch, STOP = false; //STOP is motor brake in case angle too high

float KP = 0.0;
bool switched = false;

float gyroangle[2];

float battery_voltage;

bool D8_state, D9_state, D10_state, D11_state, D12_state;
int current_time, timer_1, timer_2, timer_3, timer_4, timer_5;

void setup() {
  
  Serial.begin(74880);
  
  motor1.attach(2, 1000, 2000);
  motor2.attach(3, 1000, 2000);
  motor3.attach(4, 1000, 2000);
  motor4.attach(5, 1000, 2000);
  
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);

  // green = 15, yellow = 16, red = 17;
  DDRC |= B00001110;

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  
  delay(1000);
}
void loop() {

  float channel[4];
    
  for (int i = 0; i < 5; i++){
    channel[i] = channel_vol[i];
  }
  
  if (Armed == true){
        
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); // Snímání akcelerace pomocí register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Požádat o 6 bytů, každá osa má 2
  
    for (int i = 0; i < 3; i++){
      Acc[i] = (Wire.read() << 8 | Wire.read()) / 16384.0 + AccError[i];
    }
    
    previousTime = currentTime;
    currentTime = micros();
    dt = (currentTime - previousTime) / 1000000; // Divide by 1000000 to get seconds
    
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
    for (int i = 0; i < 3; i++){
      Gyro[i] = (Wire.read() << 8 | Wire.read()) / 131.0 + GyroError[i];
      filteredGyro[i] = 0.3 * Gyro[i] + 0.7 * filteredGyro[i];
    }

    accAngleX = (atan(Acc[1] / sqrt(pow(Acc[0], 2) + pow(Acc[2], 2))) * 180 / PI);
    accAngleY = (atan(-1 * Acc[0] / sqrt(pow(Acc[1], 2) + pow(Acc[2], 2))) * 180 / PI);
    Angle[0] = 0.96 * (Angle[0] + Gyro[0] * dt) + 0.04 * accAngleX;
    Angle[1] = 0.96 * (Angle[1] + Gyro[1] * dt) + 0.04 * accAngleY;
    Angle[2] += Gyro[2] * dt;

    
    gyroangle[0] += Gyro[0] * dt;
    gyroangle[1] += Gyro[1] * dt;

    battery_voltage = battery_voltage * 0.92 + analogRead(7) * 0.0969696;
//
//    Serial.print(Angle[0]);
//    Serial.print(",");
//    Serial.print(Angle[1]);
//    Serial.print(",");
    Serial.println(dt, 4);

    for (int i = 0; i < 2; i++){
      if ((Angle[i] > 45 || Angle[i] < -45) && STOP){
        STOP = true;
      }
    }    

    for (int i = 0; i < 3; i++){
      desRate[i] = 0;
      if (channel[i+1] > 1505){
        desRate[i] = (channel[i+1] - 1505)/6;
      } else if (channel[i+1] < 1495){
        desRate[i] = (channel[i+1] - 1495)/6;
      }
    }

    if (channel[4] > 1900 && not switched){
      KP -= 0.1;
      switched = true;
    } else if (channel[4] < 1000 && not switched) {
      KP += 0.1;
      switched = true;
    }
    if (channel[4] < 1500 && channel[4] > 1300 && switched) {
      switched = false;
    }

    ratePID[0] = calculate_pitch_rate_PID(Gyro[0], 0);
    ratePID[1] = calculate_roll_rate_PID(Gyro[1], 0);
    ratePID[2] = calculate_yaw_rate_PID(Gyro[2], 0);

    if (channel[0] > 1700) {
      channel[0] = 1700;
    }

    if (channel[0] > 1100 and not STOP){
        ESCCommand[0] = channel[0] - ratePID[0] + ratePID[1] - ratePID[2];
        ESCCommand[1] = channel[0] + ratePID[0] + ratePID[1] + ratePID[2];
        ESCCommand[2] = channel[0] + ratePID[0] - ratePID[1] - ratePID[2];
        ESCCommand[3] = channel[0] - ratePID[0] - ratePID[1] + ratePID[2];
    } else {
      for (int i = 0; i < 4; i++){
        ESCCommand[i] = 1000;
      }
    }

    if (battery_voltage < 1240 && battery_voltage > 800){
      for (int i = 0; i < 4; i++){
        ESCCommand[i] += ESCCommand[i] * ((1240 - battery_voltage)/(float)3500);
      }
    } 
    
    motor1.writeMicroseconds(ESCCommand[0]); 
    motor2.writeMicroseconds(ESCCommand[1]); 
    motor3.writeMicroseconds(ESCCommand[2]); 
    motor4.writeMicroseconds(ESCCommand[3]);

  } else {
    desAngle[2] = pulseIn(8, HIGH);
    channel[0] = pulseIn(9, HIGH);
    if (thStickUp && channel[0] <= 1090 && desAngle[2] >= 1850){
      Armed = true;
      PORTC |= B00000010;
//      digitalWrite(LED_green, HIGH);
      desAngle[2] = 0;
    } else if (channel[0] > 1850){
      thStickUp = true;
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

  if (Integral > PIDMAX) {
    Integral = PIDMAX;
  } else if (Integral < -PIDMAX) {
    Integral = -PIDMAX;
  }
  
  PID = Kp * Error + Ki * Integral + KP * (Error - previousError);
  previousError = Error;

  if (PID > PIDMAX) {
    PID = PIDMAX;
  } else if (PID < -PIDMAX) {
    PID = -PIDMAX;
  }

  return PID;
}

float calculate_roll_rate_PID(float Rate, float desRate){
  const int Kp = 0;
  const int Ki = 0;
  const int Kd = 0;
  float Integral, Error, previousError, PID;
  
  Error = desRate - Rate;
  Integral += Error;

  if (Integral > PIDMAX) {
    Integral = PIDMAX;
  } else if (Integral < -PIDMAX) {
    Integral = -PIDMAX;
  }
  
  PID = Kp * Error + Ki * Integral + KP * (Error - previousError);
  previousError = Error;

  if (PID > PIDMAX) {
    PID = PIDMAX;
  } else if (PID < -PIDMAX) {
    PID = -PIDMAX;
  }

  return PID;
}

float calculate_yaw_rate_PID(float Rate, float desRate){
  const int Kp = 0;
  const int Ki = 0;
  const int Kd = 0;
  float Integral, Error, previousError, PID;
  
  Error = desRate - Rate;
  Integral += Error;

  if (Integral > PIDMAX) {
    Integral = PIDMAX;
  } else if (Integral < -PIDMAX) {
    Integral = -PIDMAX;
  }
  
  PID = Kp * Error + Ki * Integral + KP * (Error - previousError);;
  previousError = Error;

  if (PID > PIDMAX) {
    PID = PIDMAX;
  } else if (PID < -PIDMAX) {
    PID = -PIDMAX;
  }

  return PID;
}

ISR(PCINT0_vect) {
  current_time = micros();
  if (!D8_state && PINB & B00000001) {
    D8_state = true;
    timer_1 = current_time;
  } else if (D8_state && !(PINB & B00000001)) {
    D8_state = false;
    channel_vol[3] = current_time - timer_1;
  }
  if (!D9_state && PINB & B00000010) {
    D9_state = true;
    timer_2 = current_time;
  } else if (D9_state && !(PINB & B00000010)) {
    D9_state = false;
    channel_vol[0] = current_time - timer_2;
  }
  if (!D10_state && PINB & B00000100) {
    D10_state = true;
    timer_3 = current_time;
  } else if (D10_state && !(PINB & B00000100)) {
    D10_state = false;
    channel_vol[1] = current_time - timer_3;
  }
  if (!D11_state && PINB & B00001000) {
    D11_state = true;
    timer_4 = current_time;
  } else if (D11_state && !(PINB & B00001000)) {
    D11_state = false;
    channel_vol[2] = current_time - timer_4;
  }
  if (!D12_state && PINB & B00010000) {
    D12_state = true;
    timer_5 = current_time;
  } else if (D12_state && !(PINB & B00010000)) {
    D12_state = false;
    channel_vol[4] = current_time - timer_5;
  }
}
