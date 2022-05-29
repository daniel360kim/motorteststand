#include <HX711.h>


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

void setup() {

  Serial.begin(2000000);

  // Initialize library with data output pin, clock input pin and gain factor.

  // Channel selection is made by passing the appropriate gain:

  // - With a gain factor of 64 or 128, channel A is selected

  // - With a gain factor of 32, channel B is selected

  // By omitting the gain factor parameter, the library

  // default "128" (Channel A) is used here.

  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  double taret = 800;
  scale.tare(taret);
  scale.set_scale(376.65f);
}

void loop() {

  Serial.println(scale.get_units(10));

}
