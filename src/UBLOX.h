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
v1.0.2
Chris O.
2018-03-01 Changes:
Identify the packet Class in receive message
Identify the packet Message ID in receive message
Identify the packet 16Bit Message payloadSize
Fixed the checksum calculations, now it's based on*the*received message payload.
This fixes the compatibility issues between NEO-M8 and NEO-M7-6 GPS series.
2018-03-01 Addition:
UBX-NAV-POSLLH - (0x01 0x02) Geodetic Position Solution
2018-03-20
UBX-NAV-ATT ---- (0x01 0x05) Attitude Solution
/---  High level commands, for the user ---/
NOTE: command(bool) == (true)Print Message Acknowledged on USB Serial Monitor
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
### u-blox Switch off all NMEA MSGs ###
Dis_all_NMEA_Child_MSGs(bool);    // Disable All NMEA Child Messages Command
### High level Command Generator ###
SetGPSbaud(uint32_t baud, bool)   // Set UBLOX GPS Port Configuration Baud rate
SetNAV5(uint8_t dynModel, bool)   // Set Dynamic platform model Navigation Engine Settings (0:portable, 3:pedestrian, Etc)
SetRATE(uint16_t measRate, bool)  // Set Navigation/Measurement Rate Settings (100ms=10.00Hz, 200ms=5.00Hz, 1000ms=1.00Hz, Etc)
*/
// UBX-NAV-POSLLH -- Geodetic Position Solution
// ### UBX Protocol, Class NAV 0x01, ID 0x02 ###
// UBX-NAV-POSLLH (0x01 0x02) (Payload U-blox-M8&M7&M6=20)
// iTOW                       ///< [ms], GPS time of the navigation epoch
// lon                        ///< [deg], Longitude
// lat                        ///< [deg], Latitude
// height                     ///< [m], Height above ellipsoid
// hMSL                       ///< [m], Height above mean sea level
// hAcc                       ///< [m], Horizontal accuracy estimate
// UBX-NAV-ATT -- Attitude Solution
// ### UBX Protocol, Class NAV 0x01, ID 0x05 ###
// ### NOTE: U-blox M8 from protocol version 19 up to version 23.01 (only with ADR or UDR products) ###
// UBX-NAV-ATT (0x01 0x05)    (Payload U-blox-M8=32)
// version                    ///< [ND],  Message version (0 for this version)
// roll                       ///< [deg], Vehicle roll.
// pitch                      ///< [deg], Vehicle pitch.
// heading                    ///< [deg], Heading of motion (2-D)
// accRoll                    ///< [deg], Vehicle roll accuracy (if null, roll angle is not available).
// accPitch                   ///< [deg], Vehicle pitch accuracy (if null, pitch angle is not available).
// accHeading                 ///< [deg], Vehicle heading accuracy (if null, heading angle is not available).
// UBX-NAV-PVT --  Navigation Position Velocity Time Solution
// ### UBX Protocol, Class NAV 0x01, ID 0x07 ###
// UBX-NAV-PVT (0x01 0x07)    (Payload U-blox-M8=92, M7&M6=84)
// iTOW                       ///< [ms], GPS time of the navigation epoch
// utcYear                    ///< [year], Year (UTC)
// utcMonth                   ///< [month], Month, range 1..12 (UTC)
// utcDay                     ///< [day], Day of month, range 1..31 (UTC)
// utcHour                    ///< [hour], Hour of day, range 0..23 (UTC)
// utcMin                     ///< [min], Minute of hour, range 0..59 (UTC)
// utcSec                     ///< [s], Seconds of minute, range 0..60 (UTC)
// valid                      ///< [ND], Validity flags
// tAcc                       ///< [ns], Time accuracy estimate (UTC)
// utcNano                    ///< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
// fixType                    ///< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
// flags                      ///< [ND], Fix status flags
// flags2                     ///< [ND], Additional flags
// numSV                      ///< [ND], Number of satellites used in Nav Solution
// lon                        ///< [deg], Longitude
// lat                        ///< [deg], Latitude
// height                     ///< [m], Height above ellipsoid
// hMSL                       ///< [m], Height above mean sea level
// hAcc                       ///< [m], Horizontal accuracy estimate
// vAcc                       ///< [m], Vertical accuracy estimate
// velN                       ///< [m/s], NED north velocity
// velE                       ///< [m/s], NED east velocity
// velD                       ///< [m/s], NED down velocity
// gSpeed                     ///< [m/s], Ground Speed (2-D)
// heading                    ///< [deg], Heading of motion (2-D)
// sAcc                       ///< [m/s], Speed accuracy estimate
// headingAcc                 ///< [deg], Heading accuracy estimate (both motion and vehicle)
// pDOP                       ///< [ND], Position DOP
// headVeh                    ///< [deg], Heading of vehicle (2-D)             #### NOTE: u-blox8 only ####
// --- magDec, magAcc --- TODO TEST
// magDec                     ///< [deg], Magnetic declination                 #### NOTE: u-blox8 only ####
// magAcc                     ///< [deg], Magnetic declination accuracy        #### NOTE: u-blox8 only ####

#ifndef UBLOX_h
#define UBLOX_h
#include "Arduino.h"
struct gpsData
{
  unsigned long iTOW; // /< [ms], GPS time of the navigation epoch
  unsigned short utcYear; // /< [year], Year (UTC)
  unsigned char utcMonth; // /< [month], Month, range 1..12 (UTC)
  unsigned char utcDay; // /< [day], Day of month, range 1..31 (UTC)
  unsigned char utcHour; // /< [hour], Hour of day, range 0..23 (UTC)
  unsigned char utcMin; // /< [min], Minute of hour, range 0..59 (UTC)
  unsigned char utcSec; // /< [s], Seconds of minute, range 0..60 (UTC)
  unsigned char valid; // /< [ND], Validity flags
  unsigned long tAcc; // /< [ns], Time accuracy estimate (UTC)
  long utcNano; // /< [ns], Fraction of second, range -1e9 .. 1e9 (UTC)
  unsigned char fixType; // /< [ND], GNSSfix Type: 0: no fix, 1: dead reckoning only, 2: 2D-fix, 3: 3D-fix, 4: GNSS + dead reckoning combined, 5: time only fix
  unsigned char flags; // /< [ND], Fix status flags
  unsigned char flags2; // /< [ND], Additional flags
  unsigned char numSV; // /< [ND], Number of satellites used in Nav Solution
  double lon; // /< [deg], Longitude
  double lat; // /< [deg], Latitude
  double height; // /< [m], Height above ellipsoid
  double hMSL; // /< [m], Height above mean sea level
  double hAcc; // /< [m], Horizontal accuracy estimate
  double vAcc; // /< [m], Vertical accuracy estimate
  double velN; // /< [m/s], NED north velocity
  double velE; // /< [m/s], NED east velocity
  double velD; // /< [m/s], NED down velocity
  double gSpeed; // /< [m/s], Ground Speed (2-D)
  double heading; // /< [deg], Heading of motion (2-D) ***
  double sAcc; // /< [m/s], Speed accuracy estimate
  double headingAcc; // /< [deg], Heading accuracy estimate (both motion and vehicle)
  double pDOP; // /< [ND], Position DOP
  double headVeh; // /< [deg], Heading of vehicle (2-D)        #### NOTE: u-blox8 only ####
  double magDec; // /< [deg], Magnetic declination            #### NOTE: u-blox8 only ####
  double magAcc; // /< [deg], Magnetic declination accuracy   #### NOTE: u-blox8 only ####
  // UBX-NAV-ATT (0x01 0x05) Length (Payload 32)
  // #### NOTE: u-blox 8 from protocol version 19 up to version 23.01 (only with ADR or UDR products) ###
  unsigned char version; // /< [ND],  Message version (0 for this version)
  double roll; // /< [deg], Vehicle roll.
  double pitch; // /< [deg], Vehicle pitch.
  // heading;     ///< [deg], heading *** see above ^^^^
  double accRoll; // /< [deg], Vehicle roll accuracy (if null, roll angle is not available).
  double accPitch; // /< [deg], Vehicle pitch accuracy (if null, pitch angle is not available).
  double accHeading; // /< [deg], Vehicle heading accuracy (if null, heading angle is not available).
  /* Returned after sending command */
  // Send command, gps.Poll_CFG_Port1(true); // true = print ACK on usb Serial Monitor, Ack/Nack Messages: i.e. as replies to CFG Input Messages.
  uint32_t GpsUart1Baud; // /< [ND], Current GPS Uart1 baud rate --
};

class UBLOX
{
  public:
    UBLOX();
    UBLOX(uint8_t bus);
    void configure(uint8_t bus);
    void begin(int baud);
    void end(); // Disables serial communication, to re-enable serial communication, call gps.begin(Baud);
    bool read(gpsData * gpsData_ptr);
    /******************************************/
    /**  High level commands, for the user    */
    /******************************************/
    void Poll_GPSbaud_Port1(bool printACK); // Polls the configuration for one I/O Port, I/O Target 0x01=UART1
    void Poll_NAV_PVT(); // Polls UBX-NAV-PVT    (0x01 0x07) Navigation Position Velocity Time Solution
    void Poll_NAV_POSLLH(); // Polls UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
    void Poll_NAV_ATT(); // Polls UBX-NAV-ATT (0x01 0x05) Attitude Solution
    // ### Periodic auto update ON,OFF command ###
    void Ena_NAV_PVT(bool printACK); // Enable periodic auto update NAV_PVT
    void Dis_NAV_PVT(bool printACK); // Disable periodic auto update NAV_PVT
    void Ena_NAV_ATT(bool printACK); // Enable periodic auto update NAV_ATT
    void Dis_NAV_ATT(bool printACK); // Disable periodic auto update NAV_ATT
    void Ena_NAV_POSLLH(bool printACK); // Enable periodic auto update NAV_POSLLH
    void Dis_NAV_POSLLH(bool printACK); // Disable periodic auto update NAV_POSLLH
    // #### u-blox Switch OFF ALL NMEA MSGs ####
    void Dis_all_NMEA_Child_MSGs(bool printACK); // Disable All NMEA Child Messages Command
    /** High level Command Generator GPS control write operations **/
    void SetGPSbaud(uint32_t GPS32baud, bool printACK); // UBX-CFG-PRT  (0x06 0x00)  Set UBLOX GPS Port Configuration Baud rate
    void SetNAV5(uint8_t dynModel, bool printACK); // UBX-CFG-NAV5 (0x06 0x24)  Set Dynamic platform model Navigation Engine Settings (0:portable, 3:pedestrian, Etc)
    void SetRATE(uint16_t measRate, bool printACK); // UBX-CFG-RATE (0x06 0x08)  Set Navigation/Measurement Rate Settings (100ms=10.00Hz, 200ms=5.00Hz, 1000ms=1.00Hz, Etc)
  private:
    uint8_t _bus;
    uint8_t _fpos;
    uint8_t _CurrentClass;
    uint8_t _portID; // I/O Port
    uint32_t _printACK;
    uint32_t _printACK_Dis_all_NMEA;
    uint32_t _printACK_PVT;
    uint32_t _printACK_ATT;
    uint32_t _printACK_POSLLH;
    uint8_t _ACKcount;
    bool _pACK2 = false;
    bool _pACK_RATE = false;
    bool _pACK_NAV5 = false;
    bool _UBX_NAV_PVT_ID = false;
    bool _UBX_NAV_POSLLH_ID = false;
    bool _UBX_NAV_ATT_ID = false;
    bool _UBX_CFG_PRT_ID = false;
    bool _UBX_ACK_ACK_ID = false;
    uint16_t _MSGpayloadSize;
    uint32_t _GpsUartBaud;
    static const uint8_t _payloadSize = 100;
    uint8_t _gpsPayload[_payloadSize];
    HardwareSerial * _port;
    bool parse();
    void calcChecksum(unsigned char * CK, unsigned char * payload, uint8_t length);
    void ACK_IDs(uint8_t SERa);
    void NAV_IDs(uint8_t SERn);
    void CFG_IDs(uint8_t SERc);
    /** High level Command GPS control write operations **/
    // Generate the uBlox command configuration
    void writeCommand(uint8_t CLASS, uint8_t ID, uint8_t PayloadLength0, uint8_t PayloadLength1, uint8_t Identifier, uint8_t Parameter0, uint8_t Parameter1, uint8_t Parameter2, uint8_t Parameter3, uint8_t reserved0) const;
    // ### Polling Commands Mechanism, MSG REQUEST ###
    // All messages that are output by the receiver in a periodic manner (i.e. messages in classes MON, NAV and RXM) can also be polled.
    uint8_t const _Poll_CFG_PRT[9] =
    {
      0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
    }; // CFG-PRT (0x06 0x00) Polls the GPS baud configuration for one I/O Port, (I/O Target # MSG) 0x01=UART1
    uint8_t const _Poll_NAV_PVT[8] =
    {
      0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19
    }; // UBX-NAV-PVT (0x01 0x07) Navigation Position Velocity Time Solution
    uint8_t const _Poll_NAV_POSLLH[8] =
    {
      0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A
    }; // UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
    uint8_t const _Poll_NAV_ATT[8] =
    {
      0xB5, 0x62, 0x01, 0x05, 0x00, 0x00, 0x06, 0x13
    }; // UBX-NAV-ATT (0x01 0x05) Attitude Solution
    // TODO
    // uint8_t const _Poll_CFG_NAV5[8] =      {0xB5, 0x62, 0x06, 0x24, 0x00, 0x00, 0x2A, 0x84};        // UBX-CFG-NAV5 (0x06 0x24) Navigation Engine Settings
    // uint8_t const _Poll_CFG_RATE[8] =      {0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30};        // UBX-CFG-RATE (0x06 0x08) Navigation/Measurement Rate Settings
    // ### Periodic auto update ON,OFF command ###
    // CFG-MSG 0x06 0x01
    uint8_t const _Ena_NAV_PVT[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51
    }; // Enable periodic auto update NAV_PVT
    uint8_t const _Dis_NAV_PVT[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x00, 0x12, 0x50
    }; // Disable periodic auto update NAV_PVT
    uint8_t const _Ena_NAV_ATT[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x05, 0x01, 0x11, 0x4D
    }; // Enable periodic auto update NAV_ATT
    uint8_t const _Dis_NAV_ATT[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x05, 0x00, 0x10, 0x4C
    }; // Disable periodic auto update NAV_ATT
    uint8_t const _Ena_NAV_POSLLH[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47
    }; // Enable periodic auto update NAV_POSLLH
    uint8_t const _Dis_NAV_POSLLH[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x00, 0x0D, 0x46
    }; // Disable periodic auto update NAV_POSLLH
    // #### NMEA- u-blox Switch OFF ALL NMEA MSGs ####
    // #### ---------------------------------------------------------------------------------------------------- NMEA--Name - Description -------------------------------------------------------- ####
    uint8_t const _Dis_NMEA_GxDTM[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x0A, 0x00, 0x04, 0x23
    }; // NMEA GxDTM (DTM 0xF0 0x0A) Datum Reference)
    uint8_t const _Dis_NMEA_GxGBS[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x09, 0x00, 0x03, 0x21
    }; // NMEA GxGBS (GBS 0xF0 0x09) GNSS Satellite Fault Detection)
    uint8_t const _Dis_NMEA_GxGGA[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F
    }; // NMEA GxGGA (GGA 0xF0 0x00) Global positioning system fix data)
    uint8_t const _Dis_NMEA_GxGLL[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11
    }; // NMEA GxGLL (GLL 0xF0 0x01) Latitude and longitude, with time of position fix and status)
    uint8_t const _Dis_NMEA_GxGNS[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x0D, 0x00, 0x07, 0x29
    }; // NMEA GxGNS (GNS 0xF0 0x0D) GNSS fix data)
    uint8_t const _Dis_NMEA_GxGRS[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x06, 0x00, 0x00, 0x1B
    }; // NMEA GxGRS (GRS 0xF0 0x06) GNSS Range Residuals)
    uint8_t const _Dis_NMEA_GxGSA[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13
    }; // NMEA GxGSA (GSA 0xF0 0x02) GNSS DOP and Active Satellites)
    uint8_t const _Dis_NMEA_GxGST[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x00, 0x01, 0x1D
    }; // NMEA GxGST (GST 0xF0 0x07) GNSS Pseudo Range Error Statistics)
    uint8_t const _Dis_NMEA_GxGSV[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15
    }; // NMEA GxGSV (GSV 0xF0 0x03) GNSS Satellites in View)
    uint8_t const _Dis_NMEA_GxRMC[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17
    }; // NMEA GxRMC (RMC 0xF0 0x04) Recommended Minimum data)
    uint8_t const _Dis_NMEA_GxVLW[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x0F, 0x00, 0x09, 0x2D
    }; // NMEA GxVLW (VLW 0xF0 0x0F) Dual ground/water distance)
    uint8_t const _Dis_NMEA_GxVTG[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19
    }; // NMEA GxVTG (VTG 0xF0 0x05) Course over ground and Ground speed)
    uint8_t const _Dis_NMEA_GxZDA[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x08, 0x00, 0x02, 0x1F
    }; // NMEA GxZDA (ZDA 0xF0 0x08) Time and Date)
    // #### PUBX- u-blox NMEA extension - Switch OFF ALL ####
    uint8_t const _Dis_PUBXe_NMEA00[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x00, 0x00, 0xFB, 0x12
    }; // PUBX 00 (Position Data)
    uint8_t const _Dis_PUBXe_NMEA01[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x01, 0x00, 0xFC, 0x14
    }; // PUBX 01 (UTM Position Data)
    uint8_t const _Dis_PUBXe_NMEA03[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x03, 0x00, 0xFE, 0x18
    }; // PUBX 03 (Satellite Data)
    uint8_t const _Dis_PUBXe_NMEA04[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x04, 0x00, 0xFF, 0x1A
    }; // PUBX 04 (Time of Day)
    uint8_t const _Dis_PUBXe_NMEA05[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x05, 0x00, 0x00, 0x1C
    }; // PUBX 05 (EKF Status)
    uint8_t const _Dis_PUBXe_NMEA06[11] =
    {
      0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF1, 0x06, 0x00, 0x01, 0x1E
    }; // PUBX 06 (GPS-only on EKF products)
    void DEBUG(); // debug print helper
};

#endif
