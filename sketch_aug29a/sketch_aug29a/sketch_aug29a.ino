#include <ArduinoBLE.h>
#include <NexgenAHRS.h>

LSM9DS1 imu;
EulerAngles angles;

BLEService sensorService("66df5109-edde-4f8a-a5e1-02e02a69cbd5");
BLEStringCharacteristic xSensorAngle("741c12b9-e13c-4992-8a5e-fce46dec0bff", BLERead | BLENotify,15);
BLEStringCharacteristic ySensorAngle("baad41b2-f12e-4322-9ba6-22cd9ce09832", BLERead | BLENotify,15);
BLEByteCharacteristic flag("1a4a954a-494c-11ed-b878-0242ac120002", BLERead | BLEWrite);

byte value;
int loopFrequency = 0;
const long displayPeriod = 10;
unsigned long previousMillis = 0;
long past = 0;
long now = 0;
int vib = 3;
long time1;
float startX, startY, startZ;
float caliX, caliY, caliZ;
 
void setup() {
  // Initialise the LSM9DS1 IMU
  imu.begin();
  BLE.begin();

  //  Positive magnetic declination - Sydney, AUSTRALIA
  imu.setDeclination(12.717);
  imu.setFusionAlgorithm(SensorFusion::FUSION);
  imu.setFusionPeriod(0.01f);   // Estimated sample period = 0.01 s = 100 Hz
  imu.setFusionThreshold(0.5f); // Stationary threshold = 0.5 degrees per second
  imu.setFusionGain(7.5);       // Default Fusion Filter Gain - try 7.5 for a much quicker response

  //  Start Serial and wait for connection
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(vib, OUTPUT);

  if (imu.connected()) {
    Serial.println("LSM9DS1 IMU Connected."); 
    imu.start();
  } else {
    Serial.println("LSM9DS1 IMU Not Detected.");
    while(1);
  }

  if (!BLE.begin()) {
        Serial.println("starting BLE failed!");
        while (1);
  }

  BLE.setLocalName("Gyroscope");
  BLE.setAdvertisedService(sensorService);
  sensorService.addCharacteristic(xSensorAngle);
  sensorService.addCharacteristic(ySensorAngle);
  sensorService.addCharacteristic(flag);
  BLE.addService(sensorService);


  BLE.advertise();
  
  Serial.println("Bluetooth device active, waiting for connections...");


  
}

void loop() {

  
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LED_BUILTIN, HIGH);
    time1 = millis();

  while (central.connected()) {
   now = millis();
  //  Wait for new sample - 7 ms delay provides a 100Hz sample rate / loop frequency
  delay(7);  

      // yes, get the value, characteristic is 1 byte so use byte value
      value = 1; 
      flag.readValue(value);
      Serial.println(value);
      if (value == 0) { //0이면 허리, 1이면 목 
        // first bit corresponds to the right button
        Serial.println("허리 측정");
      }
      else {
        // second bit corresponds to the left button
        Serial.println("목 측정");
      }

  
  //  Display sensor data every displayPeriod, non-blocking.
  if (millis() - previousMillis >= displayPeriod) {
    
    angles = imu.update();//  Check for new IMU data and update angles
    
    Serial.print("Roll: ");
    Serial.print(angles.roll);
    Serial.print("\tPitch: ");
    Serial.print(angles.pitch);
    Serial.print("\tYaw: ");
    Serial.print("caliX: ");
    Serial.print(caliX);
    Serial.print("\tcaliY: ");
    Serial.println(caliY);
    
    caliX = angles.roll-startX;
    caliY = angles.pitch-startY;
    caliZ = angles.yaw-startZ;

       if(now - time1 <3000) {
      startX = angles.roll;
      startY = angles.pitch;
      startZ = angles.yaw;
      
    }else if((now - time1 > 3000)&&(now - time1 < 3300)) {
      analogWrite(vib, 255);
    }
    else{
      
    xSensorAngle.writeValue(String(caliX));
    ySensorAngle.writeValue(String(angles.pitch));
    
    if(value == 1){ //목 측정
     if((caliX >= 40)|(angles.pitch >= 30)|(angles.pitch <= -30)){
       if(now - past >= 7000){
          analogWrite(vib, 255);
       }
     }else if(((caliX > 15)&&(caliX < 40))|((angles.pitch > 15)&&(angles.pitch < 30))|((angles.pitch < -15)&&(angles.pitch > -30))){
      if(now - past >= 7000){
          analogWrite(vib, 200);
       }
     }
     else{
      analogWrite(vib, 0);
      past = now;
     }
    }
    else{ //허리 측정
         if((caliX >= 40)|(angles.pitch >= 30)|(angles.pitch <= -30)){
       if(now - past >= 7000){
          analogWrite(vib, 255);
       }
     }else if(((caliX > 15)&&(caliX < 40))|((angles.pitch > 15)&&(angles.pitch < 30))|((angles.pitch < -15)&&(angles.pitch > -30))){
      if(now - past >= 7000){
          analogWrite(vib, 200);
       }
     }
     else{
      analogWrite(vib, 0);
      past = now;
     }
    }


    loopFrequency = 0;
    previousMillis = millis();
    }
  }
  
  loopFrequency++;
   }
   digitalWrite(LED_BUILTIN, LOW);
   Serial.print("Disconnected from central: ");
   Serial.println(central.address());
   analogWrite(vib, 0);
  }
 }
