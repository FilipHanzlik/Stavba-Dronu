void calculate_IMU_error(float AccError[3], float GyroError[3]){
  for (int i = 0; i < 300; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    
    for (int j = 0; j < 3; j++){
      Acc[j] = (Wire.read() << 8 | Wire.read()) / 16384.0;
    }
    AccError[0] -= Acc[0];
    AccError[1] -= Acc[1];
    AccError[2] = 9.80665 - Acc[2];
  }
  for (int i = 0; i < 3; i++){
    AccError[i] /= 300;
    Serial.println(AccError[i], 6);
  }
  for (int i = 0; i < 300; i++){
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Gyro data first register address 0x43
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    for (int i = 0; i < 3; i++){
      Gyro[i] = (Wire.read() << 8 | Wire.read()) / 131.0;
      GyroError[i] -= Gyro[i];
    }
  }
  for (int i = 0; i < 3; i++){
    GyroError[i] /= 300;
    Serial.println(GyroError[i], 6);
  }
}
