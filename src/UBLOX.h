/*
  UBLOX.h
  Brian R Taylor
  brian.taylor@bolderflight.com
  2016-11-03

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

  v1.0.1
  Chris O.
  2018-02-26 Changes:
  Identify the packet Class in receive message
  Identify the packet Message ID in receive message
  Identify the packet 16Bit Message payloadSize
  Fixed the checksum calculations, now it's based on*the*received message payload.
  This fixes the compatibility issues between NEO-M8 and NEO-M7-6 GPS series.

  Addition:
  UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
   high level commands, for the user
  CFG-PRT (0x06 0x00) Get/Set Port Configuration for UART
  SetGPSbaud460800(); //Set UBLOX GPS Port Configuration to 460800Baud
  SetGPSbaud9600(); //Set UBLOX GPS Port Configuration to 9600Baud
*/

// NOTE: U-blox8, U-blox6 and 7
// UBX-NAV-PVT (0x01 0x07) offset by 4 bytes, U-blox8 msg lenght 92 bytes, U-blox6 and 7 msg lenght 84 bytes

//   ### UBX Protocol ###
// Class NAV 0x01, ID 0x02, Message Naming
// UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
// iTOW;                      ///< [ms], GPS time of the navigation epoch
// lon;                          ///< [deg], Longitude
// lat;                          ///< [deg], Latitude
// height;                    ///< [m], Height above ellipsoid
// hMSL;                    ///< [m], Height above mean sea level
// hAcc;                      ///< [m], Horizontal accuracy estimate

// Class NAV 0x01, ID 0x07
// UBX-NAV-PVT (0x01 0x07) Navigation Position Velocity Time Solution
// iTOW;                ///< [ms], GPS time of the navigation epoch
// utcYear;             ///< [year], Year (UTC)
// utcMonth;          ///< [month], Month, range 1..12 (UTC)
// utcDay;              ///< [day], Day of month, range 1..31 (UTC)
// utcHour;            ///< [hour], Hour of day, range 0..23 (UTC)
// utcMin;              ///< [min], Minute of hour, range 0..59 (UTC)
// utcSec;             ///< [s], Seconds of minute, range 0..60 (UTC)
// valid;                 ///< [ND], Validity flags
// tAcc;                 ///< [ns], Time accuracy estimate (UTC)
// utcNano;           ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
// fixType;             ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
// flags;                 ///< [ND], Fix status flags
// flags2;               ///< [ND], Additional flags
// numSV;             ///< [ND], Number of satellites used in Nav Solution
// lon;                    ///< [deg], Longitude
// lat;                     ///< [deg], Latitude
// height;               ///< [m], Height above ellipsoid
// hMSL;               ///< [m], Height above mean sea level
// hAcc;                ///< [m], Horizontal accuracy estimate
// vAcc;                ///< [m], Vertical accuracy estimate
// velN;                 ///< [m/s], NED north velocity
// velE;                 ///< [m/s], NED east velocity
// velD;                 ///< [m/s], NED down velocity
// gSpeed;                  ///< [m/s], Ground Speed (2-D)
// heading;                 ///< [deg], Heading of motion (2-D)
// sAcc;                ///< [m/s], Speed accuracy estimate
// headingAcc;                ///< [deg], Heading accuracy estimate (both motion and vehicle)
// pDOP;                ///< [ND], Position DOP
// headVeh;                 ///< [deg], Heading of vehicle (2-D)               #### NOTE: u-blox8 only ####
// --- Edit - 2018-2-25 magDec, magAcc --- TODO TEST
// magDec;                    ///< [deg], Magnetic declination                 #### NOTE: u-blox8 only ####
// magAcc;                    ///< [deg], Magnetic declination accuracy        #### NOTE: u-blox8 only ####

// Variable Type Definitions, For future UBX Protocol implementation (the data sheets are using these weird variable types)
#define U1 unsigned char    // Unsigned Char 1 0..255 1
#define RU1_3 unsigned char // Unsigned Char 1, binary floating point with 3 bit exponent, eeeb bbbb, (Value & 0x1F) << (Value>> 5), 0..(31*2^7)non-continuous, ~ 2^(Value >> 5)
#define I1 signed Char      // Signed Char 1, 2's complement -128..127 1
#define X1 uint8_t          // Bitfield 1, n/a n/a
#define U2 unsigned short   // Unsigned Short 2, 0..65535 1
#define I2 signed short     // Signed Short 2, 2's complement -32768..32767 1
#define X2 uint16_t         // Bitfield 2, n/a n/a
#define U4 unsigned long    // Unsigned Long 4, 0..4 '294'967'295 1
#define I4 double           // Signed Long 4, 2's complement -2'147'483'648 .. 2'147'483'647, 1
#define X4 uint32_t         // Bitfield 4, n/a n/a
#define R4 int32_t          // IEEE 754 Single Precision 4, -1*2^+127 .. 2^+127, ~ Value * 2^-24
#define R8 int64_t          // IEEE 754 Double Precision 8, -1*2^+1023 .. 2^+1023, ~ Value * 2^-53
#define CH char             // ASCII / ISO 8859.1 Encoding, 1

#ifndef UBLOX_h
#define UBLOX_h

#include "Arduino.h"

struct gpsData {
  unsigned long   iTOW;       // U4 /< [ms], GPS time of the navigation epoch
  unsigned short  utcYear;      // U2 /< [year], Year (UTC)
  unsigned char   utcMonth;     // U1 /< [month], Month, range 1..12 (UTC)
  unsigned char   utcDay;     // U1 /< [day], Day of month, range 1..31 (UTC)
  unsigned char   utcHour;      // U1 /< [hour], Hour of day, range 0..23 (UTC)
  unsigned char   utcMin;     // U1 /< [min], Minute of hour, range 0..59 (UTC)
  unsigned char   utcSec;     // U1 /< [s], Seconds of minute, range 0..60 (UTC)
  unsigned char   valid;      // X1 /< [ND], Validity flags
  unsigned long   tAcc;       // U4 /< [ns], Time accuracy estimate (UTC)
  long            utcNano;      // I4 /< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
  unsigned char   fixType;      // U1 /< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
  unsigned char   flags;      // X1 /< [ND], Fix status flags
  unsigned char   flags2;     // X1 /< [ND], Additional flags
  unsigned char   numSV;      // U1 /< [ND], Number of satellites used in Nav Solution
  double          lon;        // I4 /< [deg], Longitude
  double          lat;        // I4 /< [deg], Latitude
  double          height;     // I4 /< [m], Height above ellipsoid
  double          hMSL;       // I4 /< [m], Height above mean sea level
  double          hAcc;       // U4 /< [m], Horizontal accuracy estimate
  double          vAcc;       // U4 /< [m], Vertical accuracy estimate
  double          velN;       // I4 /< [m/s], NED north velocity
  double          velE;       // I4 /< [m/s], NED east velocity
  double          velD;       // I4 /< [m/s], NED down velocity
  double          gSpeed;     // I4 /< [m/s], Ground Speed (2-D)
  double          heading;      // I4 /< [deg], Heading of motion (2-D)
  double          sAcc;       // U4 /< [m/s], Speed accuracy estimate
  double          headingAcc;   // U4 /< [deg], Heading accuracy estimate (both motion and vehicle)
  double          pDOP;       // U2 /< [ND], Position DOP
  double          headVeh;      // I4 /< [deg], Heading of vehicle (2-D)         #### NOTE: u-blox8 only ####
  // Edit - 2018-2-25 magDec, magAcc
  I2 magDec;                      // I2/< [deg], Magnetic declination            #### NOTE: u-blox8 only ####
  U2 magAcc;                      // U2/< [deg], Magnetic declination accuracy   #### NOTE: u-blox8 only ####
};

class UBLOX {
  public:
    UBLOX();
    UBLOX(uint8_t bus);
    void configure(uint8_t bus);
    void begin(int baud);
    bool read(gpsData *gpsData_ptr);

    /******************************************/
    /**  high level commands, for the user    */
    /******************************************/
    void SetGPSbaud460800(); //Set UBLOX GPS Port Configuration to 460800Baud
    void SetGPSbaud9600(); //Set UBLOX GPS Port Configuration to 9600Baud

  private:
    uint8_t _bus;
    uint8_t _fpos;
    uint8_t _CurrentClass;
    bool _UBX_NAV_PVT_ID = false;
    bool _UBX_NAV_POSLLH_ID = false;
    uint16_t _MSGpayloadSize; // Message payloadSize

    // CFG-PRT (0x06 0x00) Get/Set Port Configuration for UART
    uint8_t const SET9600B_CFG_PRT[28]   = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0xB5};
    uint8_t const SET460800B_CFG_PRT[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xBC};

    static const uint8_t _payloadSize = 96;
    uint8_t _gpsPayload[_payloadSize];
    HardwareSerial* _port;
    bool parse();
    void calcChecksum(unsigned char* CK, unsigned char* payload, uint8_t length);
    void ACK_IDs(uint8_t SERc);
    void NAV_IDs(uint8_t SERIALc);
};
#endif