void calculate_IMU_error(){
  for (int i = 0; i < 3; i++) {
    GyroError[i] = 0;
    AccError[i] = 0;
  }
  for (int i = 0; i < 5000; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    for (int j = 0; j < 3; j++){
      Acc[j] = (Wire.read() << 8 | Wire.read()) / 4096.0;
    }
    AccError[0] -= Acc[0];
    AccError[1] -= Acc[1];
    AccError[2] = 1.0 - Acc[2];
  }
  for (int i = 0; i < 3; i++){
    AccError[i] /= 5000;
    Serial.println(AccError[i], 4);
  }
  for (int i = 0; i < 10000; i++){
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    for (int i = 0; i < 3; i++){
      Gyro[i] = (Wire.read() << 8 | Wire.read()) / 65.5;
      GyroError[i] -= Gyro[i];
    }
  }
  for (int i = 0; i < 3; i++){
    GyroError[i] /= 10000;
    Serial.println(GyroError[i], 4);
  }
}
