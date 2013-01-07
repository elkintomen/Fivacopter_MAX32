/* This file is part of the Razor AHRS Firmware */

// Output angles: yaw, pitch, roll
void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    //Calculo ByteSignos y resto bytes para el envio de los angulos
  char ByteSignosAngulos = 0;
  unsigned int rollEnGrados = TO_DEG(abs(roll))*100;
  unsigned int pitchEnGrados = TO_DEG(abs(pitch))*100;
  unsigned int yawEnGrados = TO_DEG(abs(yaw))*100;
  if (roll > 0) {
    if (pitch > 0) {
      if (yaw > 0) ByteSignosAngulos = 111;
      else ByteSignosAngulos = 110;
    }
    else {
      if (yaw > 0) ByteSignosAngulos = 101;
      else ByteSignosAngulos = 100;
    }
  }
  else {
    if (pitch > 0) {
      if (yaw > 0) ByteSignosAngulos = 11;
      else ByteSignosAngulos = 10;
    }
    else {
      if (yaw > 0) ByteSignosAngulos = 01;
      else ByteSignosAngulos = 0;
    }
  }
  byte Byte1 = highByte(rollEnGrados);
  byte Byte2 = lowByte(rollEnGrados);
  byte Byte3 = highByte(pitchEnGrados);
  byte Byte4 = lowByte(pitchEnGrados);
  byte Byte5 = highByte(yawEnGrados);
  byte Byte6 = lowByte(yawEnGrados);
      
  //Calculo ByteSignos y resto bytes para el envio del acelerometro
  char ByteSignosAcelerometro = 0;
  unsigned int AccelX = abs(accel[0])*100;
  unsigned int AccelY = abs(accel[1])*100;
  unsigned int AccelZ = abs(accel[2])*100;
  if (accel[0] > 0) {
    if (accel[1] > 0) {
      if (accel[2] > 0) ByteSignosAcelerometro = 111;
      else ByteSignosAcelerometro = 110;
    }
    else {
      if (accel[2] > 0) ByteSignosAcelerometro = 101;
      else ByteSignosAcelerometro = 100;
    }
  }
  else {
    if (accel[1] > 0) {
      if (accel[2] > 0) ByteSignosAcelerometro = 11;
      else ByteSignosAcelerometro = 10;
    }
    else {
      if (accel[2] > 0) ByteSignosAcelerometro = 01;
      else ByteSignosAcelerometro = 0;
    }
  }
  byte Byte7 = highByte(AccelX);
  byte Byte8 = lowByte(AccelX);
  byte Byte9 = highByte(AccelY);
  byte Byte10 = lowByte(AccelY);
  byte Byte11 = highByte(AccelZ);
  byte Byte12 = lowByte(AccelZ);
  
      
  Serial.print("!");
  Serial.write(ByteSignosAngulos);
  Serial.write(Byte1);
  Serial.write(Byte2);
  Serial.write(Byte3);
  Serial.write(Byte4);
  Serial.write(Byte5);
  Serial.write(Byte6);
  Serial.write(ByteSignosAcelerometro);
  Serial.write(Byte7);
  Serial.write(Byte8);
  Serial.write(Byte9);
  Serial.write(Byte10);
  Serial.write(Byte11);
  Serial.write(Byte12);
  Serial.println();
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    Serial.print("#YPR=");
    Serial.print(TO_DEG(yaw)); Serial.print(",");
    Serial.print(TO_DEG(pitch)); Serial.print(",");
    Serial.print(TO_DEG(roll)); Serial.println();
  }
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    Serial.print("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      Serial.print(accel_min[i]);
      Serial.print("/");
      Serial.print(accel_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    Serial.print("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      Serial.print(magnetom_min[i]);
      Serial.print("/");
      Serial.print(magnetom_max[i]);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    Serial.print("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gyro[i]);
      Serial.print("/");
      Serial.print(gyro_average[i] / (float) gyro_num_samples);
      if (i < 2) Serial.print("  ");
      else Serial.println();
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  Serial.print("#A-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(accel[0]); Serial.print(",");
  Serial.print(accel[1]); Serial.print(",");
  Serial.print(accel[2]); Serial.println();

  Serial.print("#M-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(magnetom[0]); Serial.print(",");
  Serial.print(magnetom[1]); Serial.print(",");
  Serial.print(magnetom[2]); Serial.println();

  Serial.print("#G-"); Serial.print(raw_or_calibrated); Serial.print('=');
  Serial.print(gyro[0]); Serial.print(",");
  Serial.print(gyro[1]); Serial.print(",");
  Serial.print(gyro[2]); Serial.println();
}

void output_sensors_binary()
{
  Serial.write((byte*) accel, 12);
  Serial.write((byte*) magnetom, 12);
  Serial.write((byte*) gyro, 12);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}

