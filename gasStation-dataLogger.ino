#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>

const char *ssid = "";
const char *password = "";
const char* mqttServer = "";
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";
const char* clientID = "ESP32Client";
// String clientID = "ESP32Client-" + String(random(0xffff), HEX);
const char* mqttTopic = "sensor/fuel_level";

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_ADS1115 ads;

#define liquidDensity 1  // 0.755 Gasoline

#define Vref 3.3 // 4.096
#define adc_ref 4095 // 32768

#define maxPressure_bar 1.5  // Range: 0.8 to 1.5 bar abs
#define minPressure_bar 0.8  // Range: 0 to 0.5 bar rel??

#define R 149 
#define minPressureSignal (0.004 * R * adc_ref / Vref)  // 0.64V
#define maxPressureSignal (0.02 * R * adc_ref / Vref)  // 3.2V

int16_t pressure_signal;
float pressure_voltage;
float absPressure_bar;
float relPressure_bar;
float height;


void connectToWiFi();
void connectToMQTT();
float readFuelLevel();


void setup() 
{
  Serial.begin(115200);

  delay(10);

  ads.setGain(GAIN_ONE);
  while (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    delay(1000);
  }

  connectToWiFi();

  client.setServer(mqttServer, mqttPort);
  connectToMQTT();
}


void loop() 
{
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi disconnected.");
    connectToWiFi();
  }

  if (!client.connected()) {
    Serial.println("MQTT disconnected.");
    connectToMQTT();
    synchronizeClock();
  }
  
  client.loop();

  float fuelLevel = readFuelLevel();
  if (fuelLevel < 0) {fuelLevel = 0.0;}

  char payload[10];
  snprintf(payload, sizeof(payload), "%.2f", fuelLevel);

  client.publish(mqttTopic, jsonPayload);
  Serial.println("Data published: " + String(payload));
  Serial.println();

  delay(10000);
}



void connectToWiFi() {
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(2000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
}


void connectToMQTT() {
  Serial.println("Connecting to MQTT...");
  while (!client.connected()) {
    if (client.connect(clientID, mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}


float readFuelLevel() {

  pressure_signal = ads.readADC_SingleEnded(0);

  pressure_voltage = (pressure_signal*Vref)/adc_ref;

  absPressure_bar = minPressure_bar + ((float)(pressure_signal - minPressureSignal) / (maxPressureSignal - minPressureSignal)) * (maxPressure_bar - minPressure_bar);

  relPressure_bar = absPressure_bar - 1.01325;  //Atmospheric pressure: 1.01325

  height = (relPressure_bar*100)/(9.81*liquidDensity);

  Serial.print("Analog value: ");
  Serial.print(pressure_signal); 
  Serial.print(" | Voltage: ");
  Serial.print(pressure_voltage);
  Serial.print(" V | Abs pressure: ");
  Serial.print(absPressure_bar);
  Serial.print(" bar | Rel pressure: ");
  Serial.print(relPressure_bar);
  Serial.print(" bar | Height: ");
  Serial.print(height);
  Serial.println(" m");

  return height;
}
