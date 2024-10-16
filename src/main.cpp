#include <Arduino.h>
#include <WiFi.h>
#include <Wifimanager.h>
#include <PubSubClient.h>

#define VCC               3.3 //Voltage of ESP32
#define ThermistorPIN     4   //Thermistor is connected to GPIO 4
#define MSG_BUFFER_SIZE   (50)

const char* mqtt_server = "172.203.134.8";

float ResistorValue = 100000; // 100k Ohm
float beta          = 3950;   // Beta factor
float T0            = 298.15; // 25°C in Kelvin
float R0            = 100000; // 100k Ohm at 25°C

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("Connected");
      // Once connected, publish an announcement...
      client.publish("iotfrontier/mqtt", "iotfrontier");
      // ... and resubscribe
      client.subscribe("iotfrontier/mqtt");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

float readTemp(){
  int adcValue = analogRead(ThermistorPIN);
  float voltage = (adcValue * VCC) / 4095;

  float resistance = ResistorValue * (VCC / voltage - 1);

  float tempK = 1 / ((log(resistance / R0) / beta) + 1 / T0);

  float tempCelsius = tempK - 273.15;


  return tempCelsius;
}

void wifiManager(){

  WiFiManager wifiManager;

  if(!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void temp_display(){
  float tempCelsius = readTemp();
  Serial.print("Temperature: ");
  Serial.print(tempCelsius, 1);
  Serial.println(" °C");

  char tempStr[6];
  dtostrf(tempCelsius, 4, 1, tempStr);
  client.publish("petamen/temp", tempStr);
}

void setup(){
  Serial.begin(115200);
  pinMode(ThermistorPIN, INPUT);

  wifiManager();

  client.setServer(mqtt_server, 1883);

}

void loop(){

  if (!client.connected()) {
    reconnect();
  }

  client.loop();

  temp_display();

  delay(1000);
}
