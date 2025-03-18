// using wifi & freeRTOS
// #include <Arduino.h>
// #include <WiFi.h>
// #include <ArduinoOTA.h>
// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <ESP32Servo.h>
// #include <SimpleKalmanFilter.h>
// #include <WebServer.h>
// #include "secrets.h"

// // Wifi Credentials
// const char *ssid = SECRET_SSID;
// const char *password = SECRET_PASS;
// // Web Server on port 80
// WebServer server(80);

// // FreeRTOS Task Handles
// TaskHandle_t Task1_FlightControl;
// TaskHandle_t Task2_WiFiOTA;

// // Function Declarations
// void Task_FlightControl(void *pvParameters);
// void Task_WiFiOTA(void *pvParameters);
// void handleRoot();
// void handleUpdate();

// // Define ESC and receiver pin connections
// #define ESC1_PIN 27
// #define ESC2_PIN 26
// #define ESC3_PIN 25
// #define ESC4_PIN 32
// // Receiver Channels: 1yaw, 2throttle, 3pitch, 4roll, 5aux1, 6aux2
// #define RECEIVER_YAW 19      // Ch1
// #define RECEIVER_THROTTLE 18 // Ch2
// #define RECEIVER_PITCH 17    // Ch3
// #define RECEIVER_ROLL 16     // Ch4
// #define RECEIVER_AUX1 14     // Ch5
// #define RECEIVER_AUX2 13     // Ch6
// // Create objects for ESCs and sensors
// Servo esc1, esc2, esc3, esc4;
// Adafruit_MPU6050 mpu;
// // Sensor data variables
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
// // Flight control variables
// float throttle;
// float desiredRollAngle = 0, desiredPitchAngle = 0, desiredYawRate = 0;
// float desiredRollRate = 0, desiredPitchRate = 0;
// int aux1, aux2;
// float rollInput, pitchInput, yawInput;
// // PID control variables
// float prevErrorRollRate = 0, prevErrorPitchRate = 0, prevErrorYawRate = 0;
// float integralRollRate = 0, integralPitchRate = 0, integralYawRate = 0;
// float prevErrorRollAngle = 0, prevErrorPitchAngle = 0;
// float integralRollAngle = 0, integralPitchAngle = 0;
// float kp = 0, ki = 0, kd = 0;
// // Motor speed inputs
// float motorInputs[4];

// void resetPID()
// {
//     prevErrorRollRate = 0, prevErrorPitchRate = 0, prevErrorYawRate = 0;
//     integralRollRate = 0, integralPitchRate = 0, integralYawRate = 0;
//     prevErrorRollAngle = 0, prevErrorPitchAngle = 0;
//     integralRollAngle = 0, integralPitchAngle = 0;
// }

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

// void calibrateMPU()
// {
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
// }

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

// void readReceiverInputs()
// {
//     throttle = constrain(pulseIn(RECEIVER_THROTTLE, HIGH, 25000), 900, 1800);
//     desiredRollAngle = map(pulseIn(RECEIVER_ROLL, HIGH, 25000), 1000, 2000, -30, 30);
//     desiredPitchAngle = map(pulseIn(RECEIVER_PITCH, HIGH, 25000), 1000, 2000, -30, 30);
//     desiredYawRate = map(pulseIn(RECEIVER_YAW, HIGH, 25000), 1000, 2000, -45, 45);
//     aux1 = pulseIn(RECEIVER_AUX1, HIGH, 25000) > 1500 ? 1 : 0;
//     aux2 = pulseIn(RECEIVER_AUX2, HIGH, 25000) > 1500 ? 1 : 0;

//     Serial.printf("\n | Thr: %.2f | Rol: %.2f | Pit: %.2f | Yaw: %.2f", throttle, desiredRollAngle, desiredPitchAngle, desiredYawRate);
//     Serial.printf("\n | Au1: %d | Au2: %d\n", aux1, aux2);
// }

// void calibrateESCs()
// {
//     bool calibration = true;
//     delay(500);
//     while (calibration)
//     {
//         readReceiverInputs();

//         if (aux1 == 0 && aux2 == 0)
//         {
//             Serial.println("Sending MAX throttle (2000µs)...");
//             esc1.writeMicroseconds(2000);
//             esc2.writeMicroseconds(2000);
//             esc3.writeMicroseconds(2000);
//             esc4.writeMicroseconds(2000);
//             ledBlink(20, 100);
//         }
//         else if (aux1 == 1 && aux2 == 0)
//         {
//             Serial.println("Sending MIN throttle (1000µs)...");
//             esc1.writeMicroseconds(1000);
//             esc2.writeMicroseconds(1000);
//             esc3.writeMicroseconds(1000);
//             esc4.writeMicroseconds(1000);
//             ledBlink(5, 400);
//         }
//         else if (aux1 == 1 && aux2 == 1)
//         {
//             ledBlink(10, 100);
//             calibration = false;
//         }
//     }
// }

// void pid_equation(float Error, float P, float I, float D, float &PrevError, float &PrevIterm, float &PIDOutput)
// {
//     float Pterm = P * Error;
//     float Iterm = PrevIterm + I * (Error + PrevError) * 0.004 / 2;
//     Iterm = constrain(Iterm, -400, 400);
//     float Dterm = D * (Error - PrevError) / 0.004;
//     PIDOutput = Pterm + Iterm + Dterm;
//     PIDOutput = constrain(PIDOutput, -400, 400);
//     PrevError = Error;
//     PrevIterm = Iterm;
// }

// void computePIDAngle()
// {
//     float errorRollAngle = desiredRollAngle - kRollAngle;
//     float errorPitchAngle = desiredPitchAngle - kPitchAngle;

//     pid_equation(errorRollAngle, kp, ki, kd, prevErrorRollAngle, integralRollAngle, desiredRollRate);
//     pid_equation(errorPitchAngle, kp, ki, kd, prevErrorPitchAngle, integralPitchAngle, desiredPitchRate);
// }

// void computePIDRate()
// {
//     float errorRoll = desiredRollRate - mRotRate[0];
//     float errorPitch = desiredPitchRate - mRotRate[1];
//     float errorYawRate = desiredYawRate - mRotRate[2];

//     pid_equation(errorRoll, kp, ki, kd, prevErrorRollRate, integralRollRate, rollInput);
//     pid_equation(errorPitch, kp, ki, kd, prevErrorPitchRate, integralPitchRate, pitchInput);
//     pid_equation(errorYawRate, kp, ki, kd, prevErrorYawRate, integralYawRate, yawInput);
// }

// void updateMotors()
// {
//     motorInputs[0] = throttle + rollInput - pitchInput + yawInput;
//     motorInputs[1] = throttle + rollInput + pitchInput - yawInput;
//     motorInputs[2] = throttle - rollInput + pitchInput + yawInput;
//     motorInputs[3] = throttle - rollInput - pitchInput - yawInput;

//     for (int i = 0; i < 4; i++)
//     {
//         motorInputs[i] = constrain(motorInputs[i], 1000, 2000);
//     }

//     esc1.writeMicroseconds(motorInputs[0]);
//     esc2.writeMicroseconds(motorInputs[1]);
//     esc3.writeMicroseconds(motorInputs[2]);
//     esc4.writeMicroseconds(motorInputs[3]);
// }

// void safetyCheck()
// {
//     while (throttle > 1050)
//     {
//         readReceiverInputs();
//         delay(4);
//     }
// }

// // **Main Setup Function**
// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin(21, 22);
//     Wire.setClock(400000); // Set I2C to 400kHz for better performance
//     delay(1500);

//     pinMode(LED_BUILTIN, OUTPUT);
//     digitalWrite(LED_BUILTIN, HIGH);

//     // Attach ESCs to pins
//     esc1.attach(ESC1_PIN, 1000, 2000);
//     esc2.attach(ESC2_PIN, 1000, 2000);
//     esc3.attach(ESC3_PIN, 1000, 2000);
//     esc4.attach(ESC4_PIN, 1000, 2000);
//     // Set receiver pins as input
//     pinMode(RECEIVER_THROTTLE, INPUT);
//     pinMode(RECEIVER_ROLL, INPUT);
//     pinMode(RECEIVER_PITCH, INPUT);
//     pinMode(RECEIVER_YAW, INPUT);

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
//     Serial.println("✅ Calibration Done! MPU6050 Stabilized!");

//     readReceiverInputs(); // Get user inputs
//     safetyCheck();        // Ensure throttle is at minimum before starting tasks
//     Serial.println("✅ Starting ESC Calibration...");
//     calibrateESCs(); // Ensure ESCs are calibrated before starting tasks
//     Serial.println("✅ ESCs Calibrated!");

//     digitalWrite(LED_BUILTIN, HIGH);
//     // FreeRTOS Tasks
//     xTaskCreatePinnedToCore(Task_FlightControl, "FlightControl", 10000, NULL, configMAX_PRIORITIES - 1, &Task1_FlightControl, 1); // Core 1
//     xTaskCreatePinnedToCore(Task_WiFiOTA, "WiFiOTA", 8000, NULL, 0, &Task2_WiFiOTA, 0);
// }

// // **Task 1: Flight Control (Core 1)**
// void Task_FlightControl(void *pvParameters)
// {
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = 50 / portTICK_PERIOD_MS; // Maintain 50ms loop rate

//     while (1)
//     {
//         readReceiverInputs(); // Get user inputs
//         readIMUData();        // Read sensor data
//         computePIDAngle();    // Compute stabilization control
//         computePIDRate();     // Compute rotation rate control
//         updateMotors();       // Adjust motor speeds
//         // Serial.printf("\n | Thr: %.2f | Rol: %.2f | Pit: %.2f | Yaw: %.2f", throttle, desiredRollAngle, desiredPitchAngle, desiredYawRate);
//         // Serial.printf("\n | Au1: %d | Au2: %d\n", aux1, aux2);
//         Serial.printf("\n | Kpr: %.2f | Kin: %.2f | Kde: %.2f\n ", kp, ki, kd);
//         Serial.printf("\n | Mo1: %.2f | Mo2: %.2f | Mo3: %.2f | Mo4: %.2f\n", motorInputs[0], motorInputs[1], motorInputs[2], motorInputs[3]);
//         Serial.print("\n----------------------------------------------------");

//         // Maintain precise loop timing without blocking CPU
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// // **Task 2: Wi-Fi & OTA Updates (Core 0)**
// void Task_WiFiOTA(void *pvParameters)
// {
//     WiFi.begin(ssid, password); // Connect to Wi-Fi
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         Serial.print(".");
//         delay(500);
//     }

//     Serial.print("\n✅ 📡 ESP32 IP Address: ");
//     Serial.println(WiFi.localIP());

//     ArduinoOTA.begin(); // Start OTA Update Service
//     Serial.println("✅ OTA Ready! Upload via Network.");

//     // Start Web Server
//     server.on("/", handleRoot);
//     server.on("/update", handleUpdate);
//     server.begin();
//     Serial.println("✅ Web Server Started.");

//     while (1)
//     {
//         ArduinoOTA.handle(); // Keep OTA service running
//         server.handleClient();
//         delay(50); // Allow other tasks to run
//     }
// }

// // 🌍 **Webpage for PID Tuning**
// const char HTML_PAGE[] PROGMEM = R"rawliteral(
// <!DOCTYPE html>
// <html lang="en">
// <head>
//   <meta charset="UTF-8">
//   <meta name="viewport" content="width=device-width, initial-scale=1.0">
//   <title>ESP32 PID Tuning</title>
//   <style>
//     body { font-family: Arial, sans-serif; text-align: center; margin: 20px; }
//     input { margin: 10px; padding: 5px; width: 60px; }
//     button { padding: 10px; }
//   </style>
// </head>
// <body>
//   <h2>PID Controller Tuning</h2>
//   <p>Set Kp, Ki, Kd:</p>
//   <label>Kp: <input type="number" id="kp" step="0.01" value="%KP%"></label><br>
//   <label>Ki: <input type="number" id="ki" step="0.01" value="%KI%"></label><br>
//   <label>Kd: <input type="number" id="kd" step="0.01" value="%KD%"></label><br>
//   <button onclick="updatePID()">Update</button>
//   <script>
//     function updatePID() {
//       let kp = document.getElementById("kp").value;
//       let ki = document.getElementById("ki").value;
//       let kd = document.getElementById("kd").value;

//       fetch("/update?kp=" + kp + "&ki=" + ki + "&kd=" + kd)
//         .then(response => response.text())
//         .then(data => alert("PID Updated: " + data));
//     }
//   </script>
// </body>
// </html>
// )rawliteral";

// // Serve Webpage
// void handleRoot()
// {
//     String htmlPage = HTML_PAGE;
//     htmlPage.replace("%KP%", String(kp));
//     htmlPage.replace("%KI%", String(ki));
//     htmlPage.replace("%KD%", String(kd));
//     server.send(200, "text/html", htmlPage);
// }

// // Handle PID Updates
// void handleUpdate()
// {
//     if (server.hasArg("kp"))
//         kp = server.arg("kp").toFloat();
//     if (server.hasArg("ki"))
//         ki = server.arg("ki").toFloat();
//     if (server.hasArg("kd"))
//         kd = server.arg("kd").toFloat();

//     String response = "kp=" + String(kp) + ", ki=" + String(ki) + ", kd=" + String(kd);
//     server.send(200, "text/plain", response);
// }

// void loop()
// {
//     // FreeRTOS tasks handle everything, no need for loop()
// }