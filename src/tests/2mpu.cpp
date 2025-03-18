// mpu calibration and smoothing
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <SimpleKalmanFilter.h>

// void ledBlink(int times, int delay_ms)
// {
//     for (int i = 0; i < times; i++)
//     {
//         digitalWrite(LED_BUILTIN, HIGH);
//         delay(delay_ms);
//         digitalWrite(LED_BUILTIN, LOW);
//         delay(delay_ms);
//     }
// }

// Adafruit_MPU6050 mpu;

// float mAccRate[3];
// float mRotRate[3];
// float mRollAngle = 0, mPitchAngle = 0;
// float kRollAngle = 0, kPitchAngle = 0;
// float accOffset[3];
// float gyroOffset[3];
// float angleOffset[2];

// // Kalman Filters for sensor data
// SimpleKalmanFilter kalmanPitch(3.5, 3.5, 0.001);
// SimpleKalmanFilter kalmanRoll(3.5, 3.5, 0.001);

// void readIMUData()
// {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     mAccRate[0] = a.acceleration.x - accOffset[0];
//     mAccRate[1] = a.acceleration.y - accOffset[1];
//     mAccRate[2] = a.acceleration.z - accOffset[2];
//     mRotRate[0] = g.gyro.x - gyroOffset[0];
//     mRotRate[1] = g.gyro.y - gyroOffset[1];
//     mRotRate[2] = g.gyro.z - gyroOffset[2];

//     mRollAngle = atan(mAccRate[1] / sqrt(pow(mAccRate[0], 2) + pow(mAccRate[2], 2))) * 180 / PI;
//     mPitchAngle = atan(mAccRate[0] / sqrt(pow(mAccRate[1], 2) + pow(mAccRate[2], 2))) * 180 / PI;

//     kRollAngle = kalmanRoll.updateEstimate(mRollAngle) - angleOffset[0];
//     kPitchAngle = kalmanPitch.updateEstimate(mPitchAngle) - angleOffset[1];
// }

// void calibrateMPU()
// {
//     Serial.println("⌛ Hold the MPU6050 steady...");

//     float sumAccXErr = 0, sumAccYErr = 0, sumAccZErr = 0;
//     float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
//     float sumRollAngle = 0, sumPitchAngle = 0;

//     float rollSum, pitchSum;
//     const int calibrationSamples = 2000;

//     digitalWrite(LED_BUILTIN, HIGH);

//     for (int i = 0; i < calibrationSamples; i++)
//     {
//         sensors_event_t a, g, temp;
//         mpu.getEvent(&a, &g, &temp);

//         sumAccXErr += a.acceleration.x - 0;
//         sumAccYErr += a.acceleration.y - 0;
//         sumAccZErr += a.acceleration.z - 9.81;

//         sumGyroX += g.gyro.x;
//         sumGyroY += g.gyro.y;
//         sumGyroZ += g.gyro.z;

//         rollSum = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI;
//         pitchSum = atan(a.acceleration.x / sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180 / PI;

//         sumRollAngle += kalmanRoll.updateEstimate(rollSum);
//         sumPitchAngle += kalmanPitch.updateEstimate(pitchSum);

//         delay(4);
//     }

//     accOffset[0] = sumAccXErr / calibrationSamples;
//     accOffset[1] = sumAccYErr / calibrationSamples;
//     accOffset[2] = sumAccZErr / calibrationSamples;

//     gyroOffset[0] = sumGyroX / calibrationSamples;
//     gyroOffset[1] = sumGyroY / calibrationSamples;
//     gyroOffset[2] = sumGyroZ / calibrationSamples;

//     angleOffset[0] = sumRollAngle / calibrationSamples;
//     angleOffset[1] = sumPitchAngle / calibrationSamples;

//     ledBlink(4, 100);

//     Serial.println("✅ Calibration Done! MPU6050 Stabilized!");
// }

// void setup(void)
// {
//     Serial.begin(115200);
//     Wire.begin(21, 22);
//     Wire.setClock(400000); // Set I2C to 400kHz for better performance

//     delay(1500);
//     // Initialize onboard LED pin
//     pinMode(LED_BUILTIN, OUTPUT);

//     // Initialize MPU6050 sensor
//     if (!mpu.begin())
//     {
//         Serial.println("MPU6050 connection failed! Restarting...");
//         ledBlink(5, 400);
//         delay(1000);
//         ESP.restart(); // Restart ESP32 if MPU6050 fails
//     }

//     Serial.println("✅ MPU6050 Connected. Starting calibration...");
//     calibrateMPU();
// }

// void loop()
// {
//     readIMUData();

//     Serial.print("MZ:");
//     Serial.print(mAccRate[2]);
//     Serial.print(" AR:");
//     Serial.print(kRollAngle);
//     Serial.print(" AP:");
//     Serial.print(kPitchAngle);
//     Serial.println();

//     delay(10);
// }