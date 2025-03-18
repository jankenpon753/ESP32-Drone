// serial print over telnet & wifi
// #include <WiFi.h>
// #include <ArduinoOTA.h>
// #include <WebServer.h>

// #define ESC1_PIN 18 // Dummy ESC pin for testing
// #define ESC2_PIN 19
// #define ESC3_PIN 21
// #define ESC4_PIN 22

// const char *ssid = "MONIR";
// const char *password = "01921711046#";

// WiFiServer telnetServer(23); // Telnet server for PuTTY
// WiFiClient telnetClient;
// WebServer server(80);

// // FreeRTOS Task Handles
// TaskHandle_t Task1_FlightControl;
// TaskHandle_t Task2_WiFiOTA;
// int i = 0;

// void Task_FlightControl(void *pvParameters)
// {
//     while (1)
//     {
//         Serial.println("🚀 Flight Control Task Running...");
//         Serial.println("✅ Checking Sensor Data...");
//         Serial.println("🔄 Adjusting Motor Speeds...");
//         telnetClient.println("🚀 Flight Control Task Running...\n");
//         telnetClient.println(i++);
//         // Simulate crash
//         vTaskDelay(2000 / portTICK_PERIOD_MS); // Simulate delay
//     }
// }

// void Task_WiFiOTA(void *pvParameters)
// {
//     WiFi.begin(ssid, password);
//     while (WiFi.status() != WL_CONNECTED)
//     {
//         Serial.print(".");
//         delay(500);
//     }

//     Serial.println("\n✅ Wi-Fi Connected!");
//     Serial.print("📡 ESP32 IP Address: ");
//     Serial.println(WiFi.localIP());

//     ArduinoOTA.begin();
//     server.begin();

//     telnetServer.begin();
//     telnetServer.setNoDelay(true);
//     Serial.println("✅ Telnet Server Started (Port 23)");

//     while (1)
//     {
//         ArduinoOTA.handle();
//         server.handleClient();

//         if (telnetServer.hasClient())
//         {
//             if (telnetClient)
//                 telnetClient.stop(); // Disconnect previous client
//             telnetClient = telnetServer.available();
//             Serial.println("✅ New Telnet Client Connected!");
//         }

//         if (telnetClient && Serial.available())
//         {
//             telnetClient.write(Serial.read()); // Send Serial prints to PuTTY
//         }

//         if (telnetClient.available())
//         {
//             Serial.write(telnetClient.read()); // Echo back user input from PuTTY
//         }

//         vTaskDelay(50 / portTICK_PERIOD_MS);
//     }
// }

// void setup()
// {
//     Serial.begin(115200);

//     // Create FreeRTOS Tasks
//     xTaskCreatePinnedToCore(Task_FlightControl, "FlightControl", 5000, NULL, 1, &Task1_FlightControl, 1); // Core 1
//     xTaskCreatePinnedToCore(Task_WiFiOTA, "WiFiOTA", 8000, NULL, 1, &Task2_WiFiOTA, 0);                   // Core 0
// }

// void loop() {} // FreeRTOS does everything