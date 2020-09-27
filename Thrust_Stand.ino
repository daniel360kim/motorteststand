#include "HX711.h"
#include <LiquidCrystal.h>
//lcd pins
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// HX711 circuit wiring

const int LOADCELL_DOUT_PIN = 8;

const int LOADCELL_SCK_PIN = 9;

HX711 scale;

double mtime;
double force;
double dt;
double dp;
double impulse;
double last;
double ig;

void setup() {

  Serial.begin(57600);
  lcd.begin(16,2);
  lcd.print("Take off Calibration Weight!");
  

//not my code

int done = 0;  // Declared as a global


Serial.println("Take off calibration weight!");
delay(500);
Serial.println("First column - time elasped.");
delay(1000);
Serial.println("Second column - thrust measured in N");
delay(1000);
Serial.println("Third Column - Total Impulse");
delay(500);
Serial.println("Press G and Enter to continue");
  while(done == 0)
  {
while (Serial.available() > 0)
{
if (Serial.read() == 'G')
{
done = 1;
}
}
  }
// now we clear the serial buffer.
while(Serial.available() > 0)
  {
byte dummyread = Serial.read();
  }

//back to me

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  double taret = 80;
  scale.tare(taret);
  scale.set_scale(413.53); // paste cal val here

  mtime = micros()/1000000.f;
  force = scale.get_units()*0.009806f;
  dt = 1/80.f;
  dp = force*dt;
  impulse = impulse + dp;
  last = mtime;
  ig = mtime + 10.f;

  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
}

void loop() {

  mtime = micros()/1000000.f;
  force = scale.get_units()*0.009806f;
  dt = mtime-last;
  dp = force*dt;
  impulse = impulse + dp;
  last = mtime;
  Serial.print(mtime,10);
  Serial.print("\t");
  Serial.print(force,10);
  Serial.print("\t");
  Serial.println(impulse,10);
  
  if (ig <= mtime && ig + 2 >= mtime){
    digitalWrite(10, HIGH);
  }
  else{
    digitalWrite(10, LOW);
  }
}
