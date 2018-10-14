#include <NMEAGPS.h>
#include <GPSport.h>
#include <GPSfix_cfg.h>

#include <Streamers.h>

// #define NMEAGPS_INTERRUPT_PROCESSING

#ifndef NMEAGPS_INTERRUPT_PROCESSING
    #error You must define NMEAGPS_INTERRUPT_PROCESSING in NMEAGPS_cfg.h!
#endif

static NMEAGPS gps;
static gps_fix fix_data;

//--------------------------

static void GPSisr(uint8_t c)
{
    gps.handle(c);

} // GPSisr

//--------------------------

void setup()
{
    DEBUG_PORT.begin(9600);
    while (!DEBUG_PORT)
        ;

    DEBUG_PORT.print(F("NMEA_isr.INO: started\n"));
    DEBUG_PORT.print(F("fix object size = "));
    DEBUG_PORT.println(sizeof(gps.fix()));
    DEBUG_PORT.print(F("NMEAGPS object size = "));
    DEBUG_PORT.println(sizeof(gps));
    DEBUG_PORT.println(F("Looking for GPS device on " GPS_PORT_NAME));

    trace_header(DEBUG_PORT);

    DEBUG_PORT.flush();
    
    gpsPort.attachInterrupt(GPSisr);
    gpsPort.begin(9600);
}

//--------------------------

void loop()
{
    if (gps.available())
    {
        // Print all the things!
        fix_data = gps.fix();
        Serial.println(fix_data.altitude());
        trace_all(DEBUG_PORT, gps, gps.read());
    }

    if (gps.overrun())
    {
        gps.overrun(false);
        DEBUG_PORT.println(F("DATA OVERRUN: took too long to print GPS data!"));
    }
}