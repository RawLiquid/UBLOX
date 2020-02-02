/*
  UBLOX.cpp
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

// Teensy 3.0 || Teensy 3.1/3.2 || Teensy 3.5 || Teensy 3.6 || Teensy LC
#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || \
  defined(__MK66FX1M0__) || defined(__MKL26Z64__)

#include "Arduino.h"
#include "UBLOX.h"

/* default constructor */
UBLOX::UBLOX() {}

/* uBlox object, input the serial bus */
UBLOX::UBLOX(uint8_t bus) {
  _bus = bus; // serial bus
}

void UBLOX::configure(uint8_t bus) {
  _bus = bus; // serial bus
}

/* starts the serial communication */
void UBLOX::begin(int baud) {

  // initialize parsing state
  _fpos = 0;

  // select the serial port
#if defined(__MK20DX128__) || defined(__MK20DX256__) ||  defined(__MKL26Z64__) // Teensy 3.0 || Teensy 3.1/3.2 || Teensy LC

  if (_bus == 3) {
    _port = &Serial3;
  }
  else if (_bus == 2) {
    _port = &Serial2;
  }
  else {
    _port = &Serial1;
  }

#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 || Teensy 3.6

  if (_bus == 6) {
    _port = &Serial6;
  }
  else if (_bus == 5) {
    _port = &Serial5;
  }
  else if (_bus == 4) {
    _port = &Serial4;
  }
  else if (_bus == 3) {
    _port = &Serial3;
  }
  else if (_bus == 2) {
    _port = &Serial2;
  }
  else {
    _port = &Serial1;
  }

#endif

  // begin the serial port for uBlox
  _port->begin(baud);
}

/* write the uBlox data */
/******************************************/
/**  high level commands, for the user    */
/******************************************/
void UBLOX::SetGPSbaud460800()
{
  _port->write(SET460800B_CFG_PRT, sizeof(SET460800B_CFG_PRT));
}
void UBLOX::SetGPSbaud9600()
{
  _port->write(SET9600B_CFG_PRT, sizeof(SET9600B_CFG_PRT));
}

/* read the uBlox data */
bool UBLOX::read(gpsData *gpsData_ptr) {

  const double mm2m = 1.0e-3;
  const double en7 = 1.0e-7;
  const double en5 = 1.0e-5;
  const double en2 = 1.0e-2;

  union {
    unsigned long val;
    uint8_t b[4];
  } iTOW;

  union {
    unsigned short val;
    uint8_t b[2];
  } utcYear;

  union {
    unsigned long val;
    uint8_t b[4];
  } tAcc;

  union {
    long val;
    uint8_t b[4];
  } utcNano;

  union {
    long val; // I4
    uint8_t b[4];
  } lon;

  union {
    long val;
    uint8_t b[4];
  } lat;

  union {
    long val;
    uint8_t b[4];
  } height;

  union {
    long val;
    uint8_t b[4];
  } hMSL;

  union {
    unsigned long val;
    uint8_t b[4];
  } hAcc;

  union {
    unsigned long val;
    uint8_t b[4];
  } vAcc;

  union {
    long val;
    uint8_t b[4];
  } velN;

  union {
    long val;
    uint8_t b[4];
  } velE;

  union {
    long val;
    uint8_t b[4];
  } velD;

  union {
    long val;
    uint8_t b[4];
  } gSpeed;

  union {
    long val;
    uint8_t b[4];
  } heading;

  union {
    unsigned long val;
    uint8_t b[4];
  } sAcc;

  union {
    unsigned long val;
    uint8_t b[4];
  } headingAcc;

  union {
    unsigned short val;
    uint8_t b[2];
  } pDOP;

  union {
    unsigned long val;
    uint8_t b[4];
  } headVeh;

  union {
    I2 val;
    uint8_t b[2];
  } magDec;

  union {
    U2 val;
    uint8_t b[2];
  } magAcc;

  // parse the uBlox packet
  if (parse()) {

    // UBX-NAV-POSLLH (0x01 0x02) Geodetic Position Solution
    if (_UBX_NAV_POSLLH_ID == true) {
      iTOW.b[0] = _gpsPayload[4]; // GPS time of week of the navigation epoch.
      iTOW.b[1] = _gpsPayload[5];
      iTOW.b[2] = _gpsPayload[6];
      iTOW.b[3] = _gpsPayload[7];
      gpsData_ptr->iTOW = iTOW.val;

      lon.b[0] = _gpsPayload[8]; // Longitude
      lon.b[1] = _gpsPayload[9];
      lon.b[2] = _gpsPayload[10];
      lon.b[3] = _gpsPayload[11];
      gpsData_ptr->lon = lon.val * en7;

      lat.b[0] = _gpsPayload[12]; // Latitude
      lat.b[1] = _gpsPayload[13];
      lat.b[2] = _gpsPayload[14];
      lat.b[3] = _gpsPayload[15];
      gpsData_ptr->lat = lat.val * en7;

      height.b[0] = _gpsPayload[16]; // Height above ellipsoid
      height.b[1] = _gpsPayload[17];
      height.b[2] = _gpsPayload[18];
      height.b[3] = _gpsPayload[19];
      gpsData_ptr->height = height.val * mm2m;

      hMSL.b[0] = _gpsPayload[20]; // Height above mean sea level
      hMSL.b[1] = _gpsPayload[21];
      hMSL.b[2] = _gpsPayload[22];
      hMSL.b[3] = _gpsPayload[23];
      gpsData_ptr->hMSL = hMSL.val * mm2m;

      hAcc.b[0] = _gpsPayload[24]; // Horizontal accuracy estimate
      hAcc.b[1] = _gpsPayload[25];
      hAcc.b[2] = _gpsPayload[26];
      hAcc.b[3] = _gpsPayload[27];
      gpsData_ptr->hAcc = hAcc.val * mm2m;
      _UBX_NAV_POSLLH_ID = false;
    }

    // UBX-NAV-PVT (0x01 0x07) Navigation Position Velocity Time Solution
    if (_UBX_NAV_PVT_ID == true) {
      iTOW.b[0] = _gpsPayload[4];
      iTOW.b[1] = _gpsPayload[5];
      iTOW.b[2] = _gpsPayload[6];
      iTOW.b[3] = _gpsPayload[7];
      gpsData_ptr->iTOW = iTOW.val;

      utcYear.b[0] = _gpsPayload[8];
      utcYear.b[1] = _gpsPayload[9];
      gpsData_ptr->utcYear = utcYear.val;

      gpsData_ptr->utcMonth = _gpsPayload[10];
      gpsData_ptr->utcDay = _gpsPayload[11];
      gpsData_ptr->utcHour = _gpsPayload[12];
      gpsData_ptr->utcMin = _gpsPayload[13];
      gpsData_ptr->utcSec = _gpsPayload[14];
      gpsData_ptr->valid = _gpsPayload[15];

      tAcc.b[0] = _gpsPayload[16];
      tAcc.b[1] = _gpsPayload[17];
      tAcc.b[2] = _gpsPayload[18];
      tAcc.b[3] = _gpsPayload[19];
      gpsData_ptr->tAcc = tAcc.val;

      utcNano.b[0] = _gpsPayload[20];
      utcNano.b[1] = _gpsPayload[21];
      utcNano.b[2] = _gpsPayload[22];
      utcNano.b[3] = _gpsPayload[23];
      gpsData_ptr->utcNano = utcNano.val;

      gpsData_ptr->fixType = _gpsPayload[24];
      gpsData_ptr->flags = _gpsPayload[25];
      gpsData_ptr->flags2 = _gpsPayload[26];
      gpsData_ptr->numSV = _gpsPayload[27];

      lon.b[0] = _gpsPayload[28];
      lon.b[1] = _gpsPayload[29];
      lon.b[2] = _gpsPayload[30];
      lon.b[3] = _gpsPayload[31];
      gpsData_ptr->lon = lon.val * en7;

      lat.b[0] = _gpsPayload[32];
      lat.b[1] = _gpsPayload[33];
      lat.b[2] = _gpsPayload[34];
      lat.b[3] = _gpsPayload[35];
      gpsData_ptr->lat = lat.val * en7;

      height.b[0] = _gpsPayload[36];
      height.b[1] = _gpsPayload[37];
      height.b[2] = _gpsPayload[38];
      height.b[3] = _gpsPayload[39];
      gpsData_ptr->height = height.val * mm2m;

      hMSL.b[0] = _gpsPayload[40];
      hMSL.b[1] = _gpsPayload[41];
      hMSL.b[2] = _gpsPayload[42];
      hMSL.b[3] = _gpsPayload[43];
      gpsData_ptr->hMSL = hMSL.val * mm2m;

      hAcc.b[0] = _gpsPayload[44];
      hAcc.b[1] = _gpsPayload[45];
      hAcc.b[2] = _gpsPayload[46];
      hAcc.b[3] = _gpsPayload[47];
      gpsData_ptr->hAcc = hAcc.val * mm2m;

      vAcc.b[0] = _gpsPayload[48];
      vAcc.b[1] = _gpsPayload[49];
      vAcc.b[2] = _gpsPayload[50];
      vAcc.b[3] = _gpsPayload[51];
      gpsData_ptr->vAcc = vAcc.val * mm2m;

      velN.b[0] = _gpsPayload[52];
      velN.b[1] = _gpsPayload[53];
      velN.b[2] = _gpsPayload[54];
      velN.b[3] = _gpsPayload[55];
      gpsData_ptr->velN = velN.val * mm2m;

      velE.b[0] = _gpsPayload[56];
      velE.b[1] = _gpsPayload[57];
      velE.b[2] = _gpsPayload[58];
      velE.b[3] = _gpsPayload[59];
      gpsData_ptr->velE = velE.val * mm2m;

      velD.b[0] = _gpsPayload[60];
      velD.b[1] = _gpsPayload[61];
      velD.b[2] = _gpsPayload[62];
      velD.b[3] = _gpsPayload[63];
      gpsData_ptr->velD = velD.val * mm2m;

      gSpeed.b[0] = _gpsPayload[64];
      gSpeed.b[1] = _gpsPayload[65];
      gSpeed.b[2] = _gpsPayload[66];
      gSpeed.b[3] = _gpsPayload[67];
      gpsData_ptr->gSpeed = gSpeed.val * mm2m;

      heading.b[0] = _gpsPayload[68];
      heading.b[1] = _gpsPayload[69];
      heading.b[2] = _gpsPayload[70];
      heading.b[3] = _gpsPayload[71];
      gpsData_ptr->heading = heading.val * en5;

      sAcc.b[0] = _gpsPayload[72];
      sAcc.b[1] = _gpsPayload[73];
      sAcc.b[2] = _gpsPayload[74];
      sAcc.b[3] = _gpsPayload[75];
      gpsData_ptr->sAcc = sAcc.val  * mm2m;

      headingAcc.b[0] = _gpsPayload[76];
      headingAcc.b[1] = _gpsPayload[77];
      headingAcc.b[2] = _gpsPayload[78];
      headingAcc.b[3] = _gpsPayload[79];
      gpsData_ptr->headingAcc = headingAcc.val * en5;

      pDOP.b[0] = _gpsPayload[80];
      pDOP.b[1] = _gpsPayload[81];
      gpsData_ptr->pDOP = pDOP.val * 0.01L;

      headVeh.b[0] = _gpsPayload[88];
      headVeh.b[1] = _gpsPayload[89];
      headVeh.b[2] = _gpsPayload[90];
      headVeh.b[3] = _gpsPayload[91];
      gpsData_ptr->headVeh = headVeh.val * en5;

      // Edit - 2018 2 25 magDec, magAcc
      magDec.b[0] = _gpsPayload[92];
      magDec.b[0] = _gpsPayload[93];
      gpsData_ptr->magDec = magDec.val * en2;

      magAcc.b[0] = _gpsPayload[94];
      magAcc.b[0] = _gpsPayload[95];
      gpsData_ptr->magDec = magAcc.val * en2;
      _UBX_NAV_PVT_ID = false;
    }
    // return true on receiving a full packet
    return true;
  }
  else {
    // return false if a full packet is not received
    return false;
  }
}

/* parse the uBlox data */
bool UBLOX::parse() {
  // uBlox UBX header definition
  uint8_t const UBX_HEADER[] = { 0xB5, 0x62 };

  // uBlox Message   (Class, ID)    # Description #
  // UBX-NAV-PVT     (0x01 0x07)    Navigation Position Velocity Time Solution
  // UBX-NAV-POSLLH  (0x01 0x02)    Geodetic Position Solution

  // checksum calculation
  static unsigned char checksum[2];

  // read a byte from the serial port
  while ( _port->available() ) {
    uint8_t c = _port->read();

    // identify the packet header
    if ( _fpos < 2 ) {
      if ( c == UBX_HEADER[_fpos] ) {
        // Serial.print("UBX_HEADER :"), Serial.println(c, HEX); // USB debug print
        _fpos++;
      }
      else {
        _fpos = 0;
      }
    }
    // Identify the packet Class
    else if (_fpos == 2) {
      switch (c) {    // ### Name Class Description ###
        case 0x01:    // NAV 0x01 Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
          _CurrentClass = c; // Save for Message ID.
          ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // grab the payload, need for checksum
          // Serial.print("UBX_NAV_CLASS:0x0"), Serial.println(_CurrentClass, HEX); // USB debug print
          _fpos++;
          break;
        case 0x02:    // RXM 0x02 Receiver Manager Messages: Satellite Status, RTC Status
          _CurrentClass = c;
          Serial.print("UBX_RXM_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _fpos = 0;
          break;
        case 0x04:    // INF 0x04 Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
          _CurrentClass = c;
          Serial.print("UBX_INF_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _fpos = 0;
          break;
        case 0x05:    // ACK 0x05 Ack/Nack Messages: as replies to CFG Input Messages
          //Serial.print("UBX_ACK_CLASS:0x0"), Serial.print(c, HEX), Serial.println(); // USB debug print
          _CurrentClass = c;
          ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // grab the payload, need for checksum
          _fpos++;
          break;
        case 0x06:    // CFG 0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
          Serial.print("UBX_CFG_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _CurrentClass = c;
          _fpos = 0;
          break;
        case 0x0A:    // MON 0x0A Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
          Serial.print("UBX_MON_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _CurrentClass = c;
          _fpos = 0;
          break;
        case 0x0B:    // AID 0x0B AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
          Serial.print("UBX_AID_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _CurrentClass = c;
          _fpos = 0;
          break;
        case 0x0D:    // TIM 0x0D Timing Messages: Time Pulse Output, Timemark Results
          Serial.print("UBX_TIM_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _CurrentClass = c;
          _fpos = 0;
          break;
        case 0x21:    // LOG 0x21 Logging Messages: Log creation, deletion, info and retrieval
          Serial.print("UBX_LOG_CLASS:0x"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          _CurrentClass = c;
          _fpos = 0;
          break;
        default:
          // 0x03 and 0x0E to 0x20 Not Supported
          _CurrentClass = c;
          _fpos = 0;
          Serial.print("Unknown packet Class:");
          if (c > 16) {
            Serial.println("0x");
          } else {
            Serial.println("0x0");
          }
          Serial.println(c, HEX), Serial.println(); // USB debug print
          break;
      }
    }
    // Identify the packet Message ID
    else if (_fpos == 3 ) {
      switch (_CurrentClass) {
        case 0x01:    // NAV 0x01
          NAV_IDs(c); // Go grab the uBlox NAV - ID
          break;
        case 0x02:    // RXM 0x02
          _fpos = 0;
          break;
        case 0x04:    // INF 0x04
          _fpos = 0;
          break;
        case 0x05:    // ACK 0x05
          ACK_IDs(c); // Go grab the uBlox ACK - ID
          break;
        case 0x06:    // CFG 0x06
          _fpos = 0;
          break;
        case 0x0A:    // MON 0x0A
          _fpos = 0;
          break;
        case 0x0B:    // AID 0x0B
          _fpos = 0;
          break;
        case 0x0D:    // TIM 0x0D
          _fpos = 0;
          break;
        case 0x21:    // LOG 0x21
          _fpos = 0;
          break;
        default:
          // 0x03 and 0x0E to 0x20 Not Supported
          _fpos = 0;
          Serial.println("Unknown packet ID:"), Serial.println(c, HEX), Serial.println(); // USB debug print
          break;
      }
      _CurrentClass = 0x00;
    }
    // Identify the packet 16Bit Message payloadSize
    // It does not include 16Bit-Sync header, 8Bit-Class, 8Bit-ID, 16Bit-Length Field, and 16Bit-CRC fields
    else if (_fpos == 4 ) {
      ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // least significant byte (LSB)
      _fpos++;
    }
    else if (_fpos == 5 ) {
      ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // most significant byte (MSB)
      _MSGpayloadSize = (_gpsPayload[3] << 8) | _gpsPayload[2]; // Combining two uint8_t as uint16_t
      // Serial.print("_MSGpayloadSize DEC:"), Serial.println(_MSGpayloadSize, DEC); // USB debug print
      _MSGpayloadSize = _MSGpayloadSize + 4; // Fix for (CRC), Add 8Bit-Class, 8Bit-ID, 16Bit-Length Field
      _fpos++;
    }
    else {
      // grab the payload
      if ( (_fpos - 2) < _MSGpayloadSize )
        ((unsigned char*)_gpsPayload)[_fpos - 2] = c;
      _fpos++;

      // The checksum is calculated over the packet, starting and including the
      // CLASS field, ID and payload, up until, but excluding, the Checksum Field
      // (CRC) Compute checksum
      if ( (_fpos - 2) == _MSGpayloadSize ) {
        calcChecksum(checksum, _gpsPayload, _MSGpayloadSize);
      }
      else if ( (_fpos - 2) == (_MSGpayloadSize + 1) ) {
        if ( c != checksum[0] )
          _fpos = 0;
      }
      else if ( (_fpos - 2) == (_MSGpayloadSize + 2) ) {
        _fpos = 0;
        if ( c == checksum[1] ) {
          return true;
        }
      }
      else if ( _fpos > (_MSGpayloadSize + 4) ) {
        _fpos = 0;
      }
    }
  }
  return false;
}

/* uBlox checksum */
void UBLOX::calcChecksum(unsigned char* CK, unsigned char* payload, uint8_t length) {
  CK[0] = 0;
  CK[1] = 0;
  for (uint8_t i = 0; i < length; i++) {
    CK[0] += payload[i]; // CK[0] = CK[0] + payload[i];
    CK[1] += CK[0];
  }
}

/* uBlox ACK - IDs */
void UBLOX::ACK_IDs(uint8_t SERc) {
  switch (SERc) {    // ### Name ACK-Class & IDs Description, Msg Length ###
    case 0x00:          //  UBX-ACK-NAK (0x05 0x00) Message Not-Acknowledged, len[2]
      Serial.print("UBX_ACK_NAK ID:"), Serial.print(SERc, HEX), Serial.println(" Message Not-Acknowledged"), Serial.println(); // USB debug print
      //_UBX_ACK_NAK_ID = true; //TODO
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERc; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x01:          //  UBX-ACK-ACK (0x05 0x01) Message Acknowledged, len[2]
      Serial.print("UBX_ACK_ACK ID:"), Serial.print(SERc, HEX), Serial.println(" Message Acknowledged"), Serial.println(); // USB debug print
      //_UBX_ACK_ACK_ID = true; //TODO
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERc; // grab the payload, need for checksum
      _fpos++;
      break;
    default:
      // Not Supported
      _fpos = 0;
      Serial.println("Unknown packet Class ACK ID:"), Serial.println(SERc, HEX), Serial.println(); // USB debug print
      break;
  }
}

/* uBlox NAV - IDs */
void UBLOX::NAV_IDs(uint8_t SERIALc) {

  switch (SERIALc) {    // ### Name NAV-Class & IDs Description, Msg Length ###
    case 0x01:          // NAV-POSECEF 0x01 0x01 Position Solution in ECEF, U-Blox7 = len[20]
      Serial.print("UBX_NAV_POSECEF ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x02:          // NAV-POSLLH 0x01 0x02 Geodetic Position Solution, U-Blox7 = len[28]
      // Serial.print("UBX_NAV_POSLLH ID:"), Serial.println(SERIALc, HEX);; // USB print
      _UBX_NAV_POSLLH_ID = true;
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERIALc; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x03:          // NAV-STATUS 0x01 0x03 Receiver Navigation Status, U-Blox7 = len[16]
      Serial.print("UBX_NAV_STATUS ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x04:         // NAV-DOP 0x01 0x04 Dilution of precision, U-Blox7 = len[18]
      Serial.print("UBX_NAV_DOP ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x06:         // NAV-SOL 0x01 0x06 Navigation Solution Information, U-Blox7 = len[52]
      Serial.print("UBX_NAV_SOL ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x07:        // NAV-PVT 0x01 0x07 Navigation Position Velocity Time Solution, U-Blox7=len[84], U-Blox8=len[92]
      // Serial.print("UBX_NAV_PVT ID:"), Serial.println(SERIALc, HEX);; // USB debug print
      _UBX_NAV_PVT_ID = true;
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERIALc; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x11:        // NAV-VELECEF 0x01 0x11 Velocity Solution in ECEF, U-Blox7 = len[20]
      Serial.print("UBX_NAV_VELECEF ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x12:        // NAV-VELNED 0x01 0x12 Velocity Solution in NED, U-Blox7 = len[36]
      Serial.print("UBX_NAV_VELNED ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x20:        // NAV-TIMEGPS 0x01 0x20 16 Periodic/Polled GPS Time Solution, U-Blox7 = len[16]
      Serial.print("UBX_NAV_TIMEGPS ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x21:        // NAV-TIMEUTC 0x01 0x21 UTC Time Solution, U-Blox7 = len[20]
      Serial.print("UBX_NAV_TIMEUTC ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x22:       // NAV-CLOCK 0x01 0x22 Clock Solution, U-Blox7 = len[20]
      Serial.print("UBX_NAV_CLOCK ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x30:       // NAV-SVINFO U-Blox7 = len[8 + 12*numCh]
      Serial.print("UBX_NAV_SVINFO ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x31:       // NAV-DGPS 0x01 0x31 DGPS Data Used for NAV, U-Blox7 = len[16 + 12*numCh]
      Serial.print("UBX_NAV_DGPS ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x32:       // NAV-SBAS 0x01 0x32 12 + 12*cnt Periodic/Polled SBAS Status Data
      Serial.print("UBX_NAV_SBAS ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    case 0x60:       // NAV-AOPSTATUS 0x01 0x60 AssistNow Autonomous Status, U-Blox7 = len[20]
      Serial.print("UBX_NAV_AOPSTATUS ID:"), Serial.print(SERIALc, HEX), Serial.println(" Not implemented"), Serial.println(); // USB debug print
      _fpos = 0;
      break;
    default:
      // Not Supported
      _fpos = 0;
      Serial.println("Unknown packet Class NAV ID:"), Serial.println(SERIALc, HEX), Serial.println(); // USB debug print
      break;
  }
}
#endif