#include <CurieBle.h>
#include "CurieImu.h"

int16_t ax, ay, az;         // accelerometer values
int16_t gx, gy, gz;         // gyrometer values

BLEPeripheral blePeripheral;       // Устройство BLE
BLEService heartRateService("0000180F-0000-1000-8000-00805F9B34FB"); // Сервис. 00000000000000000000000072646372 - ID сервиса, может быть использовано любое значение размером в 32 байта

// Характеристика для посылки измерений
BLECharacteristic heartRateChar("00002A19-0000-1000-8000-00805F9B34FB",  // стандартный 16-битный ID
    BLERead | BLENotify, 1);  // BLERead | BLENotify говорит о том что характеристика будет оповещать всех клиентов о своем изменении                              // Характеристика занимает два байта (по спецификации)                              // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

int oldHeartRate = 0;  // переменная для того чтобы убедиться в том что не произойдет повторной отправки результатов
long previousMillis = 0;  // Переменная для контроля за частотой проверки данных

void setup() {
  Serial.begin(9600);   
  pinMode(13, OUTPUT);   // инициализация лампочки на борде, которая будет зажигаться, когда к Genuino подключается другое устройство
  calibrate();

  /* Задает имя для устройства*/
  blePeripheral.setLocalName("ReTrain");
  blePeripheral.setAdvertisedServiceUuid(heartRateService.uuid());  //Включает уведомления о том что наше устройство вышло в сеть и обладает определенным сервисом
  blePeripheral.addAttribute(heartRateService);   //добавляет  сервис
  blePeripheral.addAttribute(heartRateChar); // Добавляет характеристике

  /* Включает BLE */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void calibrate()
{
  
  // initialize device
  Serial.println("Initializing IMU device...");
  CurieImu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieImu.testConnection()) {
    Serial.println("CurieImu connection successful");
  } else {
    Serial.println("CurieImu connection failed");
  }
  
  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieImu.getXAccelOffset()); 
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getYAccelOffset()); 
  Serial.print("\t"); // -235
  Serial.print(CurieImu.getZAccelOffset()); 
  Serial.print("\t"); // 168
  Serial.print(CurieImu.getXGyroOffset()); 
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getYGyroOffset()); 
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getZGyroOffset());

  // To manually configure offset compensation values, 
  // use the following methods instead of the autoCalibrate...() methods below
  //    CurieImu.setXGyroOffset(220);
  //    CurieImu.setYGyroOffset(76);
  //    CurieImu.setZGyroOffset(-85);
  //    CurieImu.setXAccelOffset(-76);
  //    CurieImu.setYAccelOffset(-235);
  //    CurieImu.setZAccelOffset(168);
  
  Serial.println("About to calibrate. Make sure your board is stable and upright");
  delay(5000);
  
  // The board must be resting in a horizontal position for 
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration...");
  CurieImu.autoCalibrateGyroOffset();
  Serial.println(" Done");
  Serial.print("Starting Acceleration calibration...");
  CurieImu.autoCalibrateXAccelOffset(0);
  CurieImu.autoCalibrateYAccelOffset(0);
  CurieImu.autoCalibrateZAccelOffset(1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieImu.getXAccelOffset());
  Serial.print("\t"); // -76
  Serial.print(CurieImu.getYAccelOffset());
  Serial.print("\t"); // -2359
  Serial.print(CurieImu.getZAccelOffset());
  Serial.print("\t"); // 1688
  Serial.print(CurieImu.getXGyroOffset());
  Serial.print("\t"); // 0
  Serial.print(CurieImu.getYGyroOffset());
  Serial.print("\t"); // 0
  Serial.println(CurieImu.getZGyroOffset());

  Serial.println("Enabling Gyroscope/Acceleration offset compensation");
  CurieImu.setGyroOffsetEnabled(true);
  CurieImu.setAccelOffsetEnabled(true);
}

void loop() {
  
  
  // Проверяет, подключился ли к нам кто то:
  BLECentral central = blePeripheral.central();

  // Если подключился:
  if (central) {
    Serial.print("Connected to central: ");
    // зажигает лампочке:
    digitalWrite(13, HIGH);

    // пока к нам подключен клиент
    //передаем ему значения пульсометра каждые 200мс:
    while (central.connected()) {
      long currentMillis = millis();
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateHeartRate();
      }
    }
    // при отключении от клиента выключает лампочке:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

void updateHeartRate() {
  /*Читает данные с аналогового входа
  */
  //int heartRate = analogRead(A0);
  CurieImu.getRotation(&gx, &gy, &gz);
  //if (heartRate != oldHeartRate) {      // при изменении значения
    //Serial.print("Heart Rate is now: "); 
    //Serial.println(heartRate / 6);
    const unsigned char heartRateCharArray[1] = {(char) (gx/6)};
    heartRateChar.setValue(heartRateCharArray, 1);  // обновляет характеристику
    //oldHeartRate = heartRate;           
  
}

