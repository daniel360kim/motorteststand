// New Mexico Rocketry Reviews: Model Rocket Motor Test Stand Code V.2.4. For reuse, please fill out the contact form at https://sites.google.com/view/rocketryreviews/home
//Make sure to subscribe to our YouTube channel at:https://www.youtube.com/channel/UC0VazqJrUQiJGd_Tedn47zA?view_as=subscriber
//If you enjoyed the guide, please consider joining our Patreon membership program at: https://www.patreon.com/rocket_reviews


#include <HX711_ADC.h>
#include <EEPROM.h>

//pins:
const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 9; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
long t;

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Measuring in 10 seconds");
  delay(10000);

  LoadCell.begin();
  float calibrationValue; // paste the calibration value here
  calibrationValue = 696.0; 
#if defined(ESP8266)|| defined(ESP32)
  
#endif
  

  long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed value from the dataset:
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(i);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    float i;
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}
