#include <Arduino.h>
#include <Adafruit_MPL3115A2.h>

Adafruit_MPL3115A2 mpl = Adafruit_MPL3115A2();

float pres_pa;
float temp_C;
float alt_m;

unsigned long loop_time = 0;

void setup()
{
    Serial.begin(9600);
    Serial.println("##################################");
    Serial.println("         TESTING MPL3115A2       ");
    Serial.println("##################################");
    Serial.println();
}

void loop()
{
    if (!mpl.begin())
    {
        Serial.println("Sensor not found");
        return;
    }

    pres_pa = mpl.getPressure();
    Serial.print("Ambient Pressure:    ");
    Serial.print(pres_pa / 101325, 4);
    Serial.println(" atm");

    // Altitude can be calculated directly through the following equation
    alt_m = 44330.77 * (1 - pow(pres_pa / 101326.0, 0.1902632));
    // alt_m = mpl.g}etAltitude();
    Serial.print("Barometric Altitude: ");
    Serial.print(alt_m, 2);
    Serial.println(" m");

    temp_C = mpl.getTemperature();
    Serial.print("Ambient Temperature:  ");
    Serial.print(temp_C, 2);
    Serial.println(" *C");

    Serial.println("----------------------------------");
    Serial.print("Time spent this iteration: ");
    Serial.print(millis() - loop_time);
    Serial.print(" ms");
    Serial.println("\n----------------------------------");

    loop_time = millis();
}