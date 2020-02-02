/*
  UBLOX_example2.ino
  Brian R Taylor
  brian.taylor@bolderflight.com
  2016-07-06

  Copyright (c) 2016 Bolder Flight Systems

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
  and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
**********************************************************************
**  High level Commands, for the user ~ UBLOX lib. v1.0.2 2018-03-20 *
**********************************************************************
   NOTE: gps.command(Boolean) == (true)Print Message Acknowledged on USB Serial Monitor
  end();                            // Disables Teensy serial communication, to re-enable, call begin(Baud)
  Poll_CFG_Port1(bool);             // Polls the configuration for one I/O Port, I/O Target 0x01=UART1
  Poll_NAV_PVT();                   // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution
  Poll_NAV_POSLLH();                // Polls UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
  Poll_NAV_ATT();                   // Polls UBX-NAV-ATT    (0x01 0x05) Attitude Solution
  ### Periodic Auto Update ON,OFF Command ###
  Ena_NAV_PVT(bool);                // Enable periodic auto update NAV_PVT
  Dis_NAV_PVT(bool);                // Disable periodic auto update NAV_PVT
  Ena_NAV_ATT(bool);                // Enable periodic auto update NAV_ATT
  Dis_NAV_ATT(bool);                // Disable periodic auto update NAV_ATT
  Ena_NAV_POSLLH(bool);             // Enable periodic auto update NAV_POSLLH
  Dis_NAV_POSLLH(bool);             // Disable periodic auto update NAV_POSLLH
  #### u-blox Switch off all NMEA MSGs ####
  Dis_all_NMEA_Child_MSGs(bool);    // Disable All NMEA Child Messages Command
  ### High level Command Generator ###
  SetGPSbaud(uint32_t baud, bool)   // Set UBLOX GPS Port Configuration Baud rate
  SetNAV5(uint8_t dynModel, bool)   // Set Dynamic platform model Navigation Engine Settings (0:portable, 3:pedestrian, Etc)
  SetRATE(uint16_t measRate, bool)  // Set Navigation/Measurement Rate Settings (100ms=10.00Hz, 200ms=5.00Hz, 1000ms=1.00Hz, Etc)
*/

#include "UBLOX.h"

// The elapsedMillis feature is built into Teensyduino.
// For non-Teensy boards, it is available as a library.
elapsedMillis sinceMSG_poll;

// Set this to the GPS hardware serial port you wish to use
#define GPShwSERIAL 3 // 1 = Serial1, 2 = Serial2, 3 = Serial3, 4 = Serial4 ....
uint32_t const BaudDefault = 9600; // default settings

// a uBlox object, which is on Teensy hardware
// GPS serial port
UBLOX gps(GPShwSERIAL);

// the uBlox data structure
gpsData uBloxData;

void setup() {
  // serial to display data
  Serial.begin(9600); // Teensy Serial object always communicates at 12 Mbit/sec USB speed.
  while ( !Serial && (millis() < 10000)) ; // wait until serial monitor is open or timeout 10 seconds

  // -- AutoBauding test --
  // Try communication with the GPS
  // receiver at 9600 baud, default settings
  // then set GPS UART Baud to 460800
  gps.begin(BaudDefault);                   // Enable Teensy serial communication @ 9600 baud, default settings.
  gps.SetGPSbaud(460800, false);            // Set GPS Port Baud, Possible Baud Rate Configurations 4800~9600~19200~38400~57600~115200~230400~460800
  gps.end();                                // Disables Teensy serial communication, to re-enable serial communication, call gps.begin(Baud, bool);.
  gps.begin(19200);
  gps.SetGPSbaud(460800, false);
  gps.end();
  gps.begin(38400);
  gps.SetGPSbaud(460800, false);
  gps.end();
  gps.begin(57600);
  gps.SetGPSbaud(460800, false);
  gps.end();
  gps.begin(115200);
  gps.SetGPSbaud(460800, false);
  gps.end();
  gps.begin(230400);
  gps.SetGPSbaud(460800, true);
  gps.end();
  // now start communication with the GPS
  // receiver at 460800 baud,
  gps.begin(460800);                        // Enable Teensy serial communication @ given baud rate
  gps.Poll_GPSbaud_Port1(true);             // Polls the GPS baud configuration for one I/O Port, I/O Target 0x01=UART1

  gps.Poll_NAV_PVT();                       // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution

  gps.SetRATE(200, true);                   // Navigation/Measurement Rate Settings, e.g. 100ms => 10Hz, 200 => 5.00Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
  // Possible Configurations:
  // 60=>16.67Hz, 64=>15.63Hz, 72=>13.89Hz, 80=>12.50Hz, 100=>10.00Hz, 125=>8.00Hz, 200=>5.00Hz, 250=>4.00Hz, 500=>2.00Hz
  // 800=>1.25Hz, 1000=>1.00Hz, 2000=>0.50Hz, 4000=>0.25Hz, 10000=>0.10Hz, 20000=>0.05Hz, 50000=>0.02Hz

  // NOTE: Dis_all_NMEA -strongly suggest changing RX buffer to 255 or more,*otherwise you will miss ACKs*on serial monitor
  gps.Dis_all_NMEA_Child_MSGs(false);       // Disable All NMEA Child Messages Command

  gps.SetNAV5(3, true);                     // Set Dynamic platform model Navigation Engine Settings (0:portable, 2: stationary, 3:pedestrian, Etc)
  // Possible Configurations
  // 0: portable, 2: stationary, 3: pedestrian, 4: automotive, 5: sea, 6: airborne with <1g, 7: airborne with <2g
  // 8: airborne with <4g, 9: wrist worn watch (not supported in protocol v.less than 18)

  // ### Periodic auto update ON,OFF Command ###
  //gps.Ena_NAV_PVT(true);                  // Enable periodic auto update NAV_PVT
  gps.Dis_NAV_PVT(true);                    // Disable periodic auto update NAV_PVT

  //gps.Ena_NAV_ATT(true);                  // Enable periodic auto update NAV_ATT ~ U-blox M8 from protocol version 19
  gps.Dis_NAV_ATT(true);                    // Disable periodic auto update NAV_ATT ~ ---^

  gps.Ena_NAV_POSLLH(true);                 // Enable periodic auto update NAV_POSLLH
  //gps.Dis_NAV_POSLLH(true);               // Disable periodic auto update NAV_POSLLH
}

void loop() {

  // The elapsedMillis feature is built into Teensyduino.
  // For non-Teensy boards, it is available as a library.
  // #### Polling Mechanism ####
  if (sinceMSG_poll >= 10000) { // SEND poll message request every 10 sec.
    sinceMSG_poll = 0;          // reset since

    // ### Polling Command(s) ###
    Serial.println(" SEND gps.Poll_NAV_PVT()");
    gps.Poll_NAV_PVT();                           // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution
    //gps.Poll_NAV_POSLLH();                      // Polls UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
    //gps.Poll_NAV_ATT();                         // Polls UBX-NAV-ATT (0x01 0x05) Attitude Solution

    // UBX-NAV-PVT --  Navigation Position Velocity Time Solution
    // ### UBX Protocol, Class NAV 0x01, ID 0x07 ###
    // UBX-NAV-PVT (0x01 0x07)    (Payload U-blox-M8=92, M7&M6=84)
    // iTOW;                      ///< [ms], GPS time of the navigation epoch
    // utcYear;                   ///< [year], Year (UTC)
    // utcMonth;                  ///< [month], Month, range 1..12 (UTC)
    // utcDay;                    ///< [day], Day of month, range 1..31 (UTC)
    // utcHour;                   ///< [hour], Hour of day, range 0..23 (UTC)
    // utcMin;                    ///< [min], Minute of hour, range 0..59 (UTC)
    // utcSec;                    ///< [s], Seconds of minute, range 0..60 (UTC)
    // valid;                     ///< [ND], Validity flags
    // tAcc;                      ///< [ns], Time accuracy estimate (UTC)
    // utcNano;                   ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
    // fixType;                   ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
    // flags;                     ///< [ND], Fix status flags
    // flags2;                    ///< [ND], Additional flags
    // numSV;                     ///< [ND], Number of satellites used in Nav Solution
    // lon;                       ///< [deg], Longitude
    // lat;                       ///< [deg], Latitude
    // height;                    ///< [m], Height above ellipsoid
    // hMSL;                      ///< [m], Height above mean sea level
    // hAcc;                      ///< [m], Horizontal accuracy estimate
    // vAcc;                      ///< [m], Vertical accuracy estimate
    // velN;                      ///< [m/s], NED north velocity
    // velE;                      ///< [m/s], NED east velocity
    // velD;                      ///< [m/s], NED down velocity
    // gSpeed;                    ///< [m/s], Ground Speed (2-D)
    // heading;                   ///< [deg], Heading of motion (2-D)
    // sAcc;                      ///< [m/s], Speed accuracy estimate
    // headingAcc;                ///< [deg], Heading accuracy estimate (both motion and vehicle)
    // pDOP;                      ///< [ND], Position DOP
    // headVeh;                   ///< [deg], Heading of vehicle (2-D)             #### NOTE: u-blox8 only ####
    //   magDec, magAcc --- TODO TEST
    // magDec;                    ///< [deg], Magnetic declination                 #### NOTE: u-blox8 only ####
    // magAcc;                    ///< [deg], Magnetic declination accuracy        #### NOTE: u-blox8 only ####
    Serial.print("UBX-NAV-PVT -\t");
    Serial.print(uBloxData.utcYear);  ///< [year], Year (UTC)
    Serial.print("y:");
    Serial.print(uBloxData.utcMonth); ///< [month], Month, range 1..12 (UTC)
    Serial.print("m:");
    Serial.print(uBloxData.utcDay);   ///< [day], Day of month, range 1..31 (UTC)
    Serial.print("d\t");
    Serial.print(uBloxData.utcHour);  ///< [hour], Hour of day, range 0..23 (UTC)
    Serial.print(":");
    Serial.print(uBloxData.utcMin);   ///< [min], Minute of hour, range 0..59 (UTC)
    Serial.print(":");
    Serial.print(uBloxData.utcSec);   ///< [s], Seconds of minute, range 0..60 (UTC)
    Serial.print("sec\t");
    Serial.print(uBloxData.numSV);    ///< [ND], Number of satellites used in Nav Solution
    Serial.print(" #sat\t");
    Serial.print(uBloxData.lat, 10);  ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(uBloxData.lon, 10);  ///< [deg], Longitude
    Serial.print("\t m sea:");
    Serial.print(uBloxData.hMSL);     ///< [m], Height above mean sea level
    Serial.print("\t m ell:");
    Serial.print(uBloxData.height);   ///< [m], Height above ellipsoid
    Serial.print("\t acc:");
    Serial.println(uBloxData.hAcc);   ///< [m], Horizontal accuracy estimate

    // Current Baud
    Serial.print(" Current Baud: \t");
    Serial.println(uBloxData.GpsUart1Baud); // Current UBLOX GPS Uart1 baud rate
  }

  // checking to see if a good packet has
  // been received and displaying some
  // of the packet data

  if (gps.read(&uBloxData) ) {
    // UBX-NAV-POSLLH -- Geodetic Position Solution
    //   ### UBX Protocol, Class NAV 0x01, ID 0x02 ###
    // UBX-NAV-POSLLH (0x01 0x02) (Payload U-blox-M8&M7&M6=20)
    // iTOW;                      ///< [ms], GPS time of the navigation epoch
    // lon;                       ///< [deg], Longitude
    // lat;                       ///< [deg], Latitude
    // height;                    ///< [m], Height above ellipsoid
    // hMSL;                      ///< [m], Height above mean sea level
    // hAcc;                      ///< [m], Horizontal accuracy estimate
    Serial.print("UBX-NAV-POSLLH -\t");
    Serial.print(uBloxData.iTOW);     ///< [ms], GPS time of the navigation epoch
    float i;
    i = uBloxData.iTOW;
    i = i / 1000;
    Serial.print("\t i~iTOW ");
    Serial.print(i, 3);                  ///< float iTOW
    Serial.print("\t");
    Serial.print(uBloxData.lat, 7);      ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(uBloxData.lon, 7);      ///< [deg], Longitude
    Serial.print("\t m sea:");
    Serial.print(uBloxData.hMSL);        ///< [m], Height above mean sea level
    Serial.print("\t m ell:");
    Serial.print(uBloxData.height);      ///< [m], Height above ellipsoid
    Serial.print("\t acc:");
    Serial.println(uBloxData.hAcc);      ///< [m], Horizontal accuracy estimate
  }

  /*
    // UBX-NAV-POSLLH -- Geodetic Position Solution
    if (gps.read(&uBloxData) ) {
    //   ### UBX Protocol, Class NAV 0x01, ID 0x02 ###
    // UBX-NAV-POSLLH (0x01 0x02) (Payload U-blox-M8&M7&M6=20)
    // iTOW;                      ///< [ms], GPS time of the navigation epoch
    // lon;                       ///< [deg], Longitude
    // lat;                       ///< [deg], Latitude
    // height;                    ///< [m], Height above ellipsoid
    // hMSL;                      ///< [m], Height above mean sea level
    // hAcc;                      ///< [m], Horizontal accuracy estimate
    Serial.println("UBX-NAV-POSLLH ID 0x02");
    Serial.print("UBX-NAV-POSLLH -\t");
    Serial.print(uBloxData.lat, 7);  ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(uBloxData.lon, 7);  ///< [deg], Longitude
    Serial.print("\t");
    Serial.print(uBloxData.hMSL);     ///< [m], Height above mean sea level
    Serial.print("\t");
    Serial.print(uBloxData.hAcc);   ///< [m], Horizontal accuracy estimate
    }
  */

  /*
    // UBX-NAV-PVT --  Navigation Position Velocity Time Solution
    if ( gps.read(&uBloxData) ) {
    // ### UBX Protocol, Class NAV 0x01, ID 0x07 ###
    // UBX-NAV-PVT (0x01 0x07)    (Payload U-blox-M8=92, M7&M6=84)
    // iTOW;                      ///< [ms], GPS time of the navigation epoch
    // utcYear;                   ///< [year], Year (UTC)
    // utcMonth;                  ///< [month], Month, range 1..12 (UTC)
    // utcDay;                    ///< [day], Day of month, range 1..31 (UTC)
    // utcHour;                   ///< [hour], Hour of day, range 0..23 (UTC)
    // utcMin;                    ///< [min], Minute of hour, range 0..59 (UTC)
    // utcSec;                    ///< [s], Seconds of minute, range 0..60 (UTC)
    // valid;                     ///< [ND], Validity flags
    // tAcc;                      ///< [ns], Time accuracy estimate (UTC)
    // utcNano;                   ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
    // fixType;                   ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
    // flags;                     ///< [ND], Fix status flags
    // flags2;                    ///< [ND], Additional flags
    // numSV;                     ///< [ND], Number of satellites used in Nav Solution
    // lon;                       ///< [deg], Longitude
    // lat;                       ///< [deg], Latitude
    // height;                    ///< [m], Height above ellipsoid
    // hMSL;                      ///< [m], Height above mean sea level
    // hAcc;                      ///< [m], Horizontal accuracy estimate
    // vAcc;                      ///< [m], Vertical accuracy estimate
    // velN;                      ///< [m/s], NED north velocity
    // velE;                      ///< [m/s], NED east velocity
    // velD;                      ///< [m/s], NED down velocity
    // gSpeed;                    ///< [m/s], Ground Speed (2-D)
    // heading;                   ///< [deg], Heading of motion (2-D)
    // sAcc;                      ///< [m/s], Speed accuracy estimate
    // headingAcc;                ///< [deg], Heading accuracy estimate (both motion and vehicle)
    // pDOP;                      ///< [ND], Position DOP
    // headVeh;                   ///< [deg], Heading of vehicle (2-D)             #### NOTE: u-blox8 only ####
    //   magDec, magAcc --- TODO TEST
    // magDec;                    ///< [deg], Magnetic declination                 #### NOTE: u-blox8 only ####
    // magAcc;                    ///< [deg], Magnetic declination accuracy        #### NOTE: u-blox8 only ####
    Serial.print("UBX-NAV-PVT -\t");
    Serial.print(uBloxData.utcYear);  ///< [year], Year (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcMonth); ///< [month], Month, range 1..12 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcDay);   ///< [day], Day of month, range 1..31 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcHour);  ///< [hour], Hour of day, range 0..23 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcMin);   ///< [min], Minute of hour, range 0..59 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.utcSec);   ///< [s], Seconds of minute, range 0..60 (UTC)
    Serial.print("\t");
    Serial.print(uBloxData.numSV);    ///< [ND], Number of satellites used in Nav Solution
    Serial.print("\t");
    Serial.print(uBloxData.lat, 10);  ///< [deg], Latitude
    Serial.print("\t");
    Serial.print(uBloxData.lon, 10);  ///< [deg], Longitude
    Serial.print("\t");
    Serial.println(uBloxData.hMSL);   ///< [m], Height above mean sea level
    }
  */
  /*
    // UBX-NAV-ATT -- Attitude Solution
    if (gps.read(&uBloxData) ) {
    //  ### UBX Protocol, Class NAV 0x01, ID 0x05 ###
    //  ### NOTE: U-blox M8 from protocol version 19 up to version 23.01 (only with ADR or UDR products) ###
    // UBX-NAV-ATT (0x01 0x05)    (Payload U-blox-M8=32)
    // version                    ///< [ND],  Message version (0 for this version)
    // roll                       ///< [deg], Vehicle roll.
    // pitch                      ///< [deg], Vehicle pitch.
    // heading                    ///< [deg], Heading of motion (2-D)
    // accRoll                    ///< [deg], Vehicle roll accuracy (if null, roll angle is not available).
    // accPitch                   ///< [deg], Vehicle pitch accuracy (if null, pitch angle is not available).
    // accHeading                 ///< [deg], Vehicle heading accuracy (if null, heading angle is not available).
    Serial.print("UBX-NAV-ATT ----\t");
    Serial.print(uBloxData.roll);         ///< [deg], Vehicle roll.
    Serial.print("\t");
    Serial.print(uBloxData.pitch);        ///< [deg], Vehicle pitch.
    Serial.print("\t");
    Serial.print(uBloxData.heading);      ///< [deg], Heading of motion (2-D)
    Serial.print("\t");
    Serial.print(uBloxData.accRoll);      ///< [deg], Vehicle roll accuracy (if null, roll angle is not available).
    Serial.print("\t");
    Serial.print(uBloxData.accPitch);     ///< [deg], Vehicle pitch accuracy (if null, pitch angle is not available).
    Serial.print("\t");
    Serial.println(uBloxData.accHeading); ///< [deg], Vehicle heading accuracy (if null, heading angle is not available).
    }
  */
}