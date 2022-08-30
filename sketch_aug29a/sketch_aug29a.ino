#include <Arduino_LSM9DS1.h>
#include "math.h"
#include <ArduinoBLE.h>

BLEService sensorService("66df5109-edde-4f8a-a5e1-02e02a69cbd5");
BLEStringCharacteristic xSensorAngle("741c12b9-e13c-4992-8a5e-fce46dec0bff", BLERead | BLENotify,15);
BLEStringCharacteristic ySensorAngle("baad41b2-f12e-4322-9ba6-22cd9ce09832", BLERead | BLENotify,15);

float oldxAngle = 0;
float oldyAngle = 0;
long preciousMillis = 0;

void setup() {
    Serial.begin(9600);    // initialize serial communication

    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.println("Started");

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    if (!BLE.begin()) {
        Serial.println("starting BLE failed!");
        while (1);
    }

    BLE.setLocalName("Gyroscope");
    BLE.setAdvertisedService(sensorService);
    sensorService.addCharacteristic(xSensorAngle);
    sensorService.addCharacteristic(ySensorAngle);
    BLE.addService(sensorService);

    xSensorAngle.writeValue(String(0));
    ySensorAngle.writeValue(String(0));

    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");

    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.println("X\tY\tZ");
}

void loop() {
  BLEDevice central = BLE.central();
 if (central) {
  Serial.print("Connected to central: ");
  Serial.println(central.address());
  digitalWrite(LED_BUILTIN, HIGH);

  while (central.connected()) {
   //long currentMillis = millis();
   updateGyroscopeAngle();
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
 }
}

void updateGyroscopeAngle() {
  float x, y, z;
    
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z); 
        
        float xAngle = atan2(y, z)*180./PI;
        float yAngle = atan2(x, z)*180./PI;
  
      if (xAngle != oldxAngle) {
        xSensorAngle.writeValue(String(xAngle));
        oldxAngle = xAngle;
      }

      if (yAngle != oldxAngle) {
        ySensorAngle.writeValue(String(yAngle));
        oldyAngle = yAngle;
      }

        Serial.print("Roll : ");
        Serial.print(xAngle);
        Serial.print('\t');
        Serial.print("Pitch : ");
        Serial.println(yAngle);
    
    }
}
