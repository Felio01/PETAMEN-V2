#include <Arduino.h>
#include <WiFi.h>
#include <PID_v1.h>
#include <Wifimanager.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <Wire.h>

#define VCC 3.264       // Voltage of ESP32
#define ThermistorPIN 4 // Thermistor is connected to GPIO 4
#define HeaterPIN 5     // Heater is connected to GPIO 5
#define stepPin 6       // Stepper motor is connected to GPIO 6
#define limit1 1
#define limit2 2
#define MSG_BUFFER_SIZE (50)

const char *mqtt_server = "172.203.134.8";

float ResistorValue = 4700; // 4k7 Ohm
float beta = 3950;          // Beta factor
float T0 = 298.15;          // 25°C in Kelvin
float R0 = 100000;          // 100k Ohm at 25°C

double setpoint = 220;
double input, output;

double Kp = 18, Ki = 0, Kd = 0;

WiFiClient espClient;
PubSubClient client(espClient);
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
Adafruit_ADS1115 ads;


// PWM configuration
const int pwmChannel = 0;
const int pwmFrequency = 400;
const int pwmResolution = 8;

// PubSubClient client(espClient);
unsigned long lastMsg = 0;
unsigned long previousMillis = 0;
const long interval = 1000;
int16_t adcValue;
float voltage;
int motorSpeed = 600;


void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str()))
    {
      Serial.println("Connected");
      // Once connected, publish an announcement...
      client.publish("iotfrontier/mqtt", "iotfrontier");
      // ... and resubscribe
      client.subscribe("testking");
      client.subscribe("petamen/motorspeed");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  String messageBuffer;
  Serial.print("Message received on topic: ");
  Serial.println(topic);

  Serial.print("Message: ");
  for (unsigned int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);

    messageBuffer += (char)payload[i];
  }
  if (String(topic) == "testking")
  {
    setpoint = messageBuffer.toFloat();
    Serial.print("Setpoint: ");
    Serial.println(setpoint);
  }

  else if (String(topic) == "petamen/motorspeed")
  {
    motorSpeed = messageBuffer.toInt();
    Serial.print("Motor speed: ");
    Serial.println(motorSpeed);
  }

  Serial.println();
}

int termNom = 100000; // Thermistor reference resistance

int refTemp = 25;

float current;

float readTemp()
{
  adcValue = ads.readADC_SingleEnded(0);
  voltage = ads.computeVolts(adcValue);

  float resistance = ResistorValue * (VCC / voltage - 1);

  float tempK = 1 / ((log(resistance / R0) / beta) + 1 / T0);

  float tempCelsius = tempK - 273.15;

  return tempCelsius;
}

void wifiManager()
{

  WiFiManager wifiManager;

  // wifiManager.resetSettings();

  if (!wifiManager.autoConnect("PETAMEN ESP32", "password"))
  {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void temp_display()
{
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    Serial.print("ADC Value:");
    Serial.println(adcValue);

    Serial.print("Voltage: ");
    Serial.println(voltage, 5);

    Serial.print("Temperature: ");
    Serial.print(input, 0);
    Serial.println(" °C");

    Serial.print("PID Output: ");
    Serial.println(output);

    char tempStr[6];
    dtostrf(input, 4, 0, tempStr);
    client.publish("petamen/temp", tempStr);
  }
}

void stepper()
{
  for (int i = 0; i < 200; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorSpeed); // Adjust delay for speed control
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorSpeed); // Adjust delay for speed control
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(ThermistorPIN, INPUT);
  pinMode(HeaterPIN, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limit1, INPUT);

  myPID.SetMode(AUTOMATIC);
  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(HeaterPIN, pwmChannel);

  // Wire.begin(8, 9);

  if (!ads.begin())
  {
    Serial.println("ADS1115 tidak terdeteksi!");
    while (1)
      ;
  }
  Serial.println("ADS1115 berhasil diinisialisasi.");

  // Clear the display buffer
  wifiManager();

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  reconnect();
}

void loop()
{

  if (!client.connected())
  {
    reconnect();
  }

  client.loop();

  if (limit1 == HIGH)
  {
    stepper();
  }

  input = readTemp();

  myPID.Compute();

  if (output < 20)
  {
    output = 20;
  }

  ledcWrite(pwmChannel, output);
  stepper();
  temp_display();
}