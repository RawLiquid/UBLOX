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

/* Note: Acknowledgment
  When messages from the Class CFG are sent to the receiver, the receiver will send an Acknowledge (ACK-ACK)
  or a Not Acknowledge (ACK-NAK) message back to the sender, depending on whether or not the message
  was processed correctly.
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
  delay(50); // wait for a bit, need for commands execution @ cold boot
  // check if the hardware serial port has room for outgoing bytes to TX buffer
  while (38 > _port->availableForWrite()) { }
  _port->write(_Poll_CFG_PRT, sizeof(_Poll_CFG_PRT));
  _port->flush();
}
// end the serial port for uBlox
void UBLOX::end() {
  _port->flush(); // Waits for the transmission of outgoing serial data to complete
  _port->end();   // Disables serial communication
}

// Generate the uBlox command configuration
void UBLOX::writeCommand(uint8_t CLASS, uint8_t ID , uint8_t PayloadLen0, uint8_t PayloadLen1, uint8_t Identifier, uint8_t reserved0, uint8_t Par0, uint8_t Par1, uint8_t Par2, uint8_t Par3) const {
  const uint8_t HeaderLength = 2; // 2 synchronization characters: 0xB5 0x62.
  const uint8_t CLASSandID = 2;   // 1-byte Message Class field and 1-byte Message ID field
  const uint8_t LenF = 2; // 2-byte 16-bit Length field
  const uint8_t CRC = 2;  // 2-byte 16-bit checksum
  uint16_t Bit16PayloadLen = (PayloadLen1 << 8) | PayloadLen0; // Combining two uint8_t as uint16_t
  uint8_t DATA [Bit16PayloadLen] {}; // Set up DATA array with size of 16 bit payload length

  switch (CLASS) { //
    case 0x00:
    case 0x01:    // NAV 0x01
    case 0x02:    // RXM 0x02
    case 0x04:    // INF 0x04
    case 0x05:    // ACK 0x05
      Serial.println("Not Supported COMMAND CLASS:"), Serial.println(CLASS, HEX);
      break;
    case 0x06:    // CLASS CFG 0x06

      // ID CFG-PRT (0x00)
      if (ID == 0x00) { // ID CFG-PRT (0x06 0x00) Port Configuration
        // SET   9600_CFG_PRT[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xC0, 0x08, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x92, 0xB5};
        // SET 460800_CFG_PRT[28] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x08, 0x07, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xBC};
        //                          |HEADER    |CLASS| ID  |16-Bit LOAD| --[0]---[1]---[2]---[3]--m[4]---[5]---[6]---[7]->b[8]--[9]--[10]--[11]<--[12]--[13]--[14]--[15]--[16]--[17]--[18]--[19]|--- CRC----|
        //                          |          |6 CFG|0 PRT|0x14 =DEC20| ^------------------ packet - payload ----------->  baudRate uint32_t  <----------------------------------------------^ |           |
        if (Bit16PayloadLen == 0x14) { // Length (Bytes) DEC 20
          if (Identifier == 0x01) {   // Port Identifier Number uart1
            DATA [0] = 0x01;          // 0 U1 - portID
            DATA [1] = 0x00;          // 1 U1 - reserved1
            DATA [2] = 0x00;          // 2 X2 - txReady
            DATA [3] = 0x00;          //   X2 - txReady
            uint32_t GPSBaud = (((Par0 << 24) | (Par1 << 16)) | (Par2 << 8)) | Par3; // Combining
            if (GPSBaud == 9600) {    // 4 X4 - mode - A bit mask describing the UART mode
              DATA [4] = 0xC0;        //        @9600=0xC0,   *b11000000‬
            } else { // NOT SURE HERE, ISSUE UBX-CFG-PRT (0x06 0x00) Bit Field mode ACCESSING UNDOCUMENTED BIT
              DATA [4] = 0xD0;        //        @460800=0xD0, *b11010000‬ U-CENTER ISSUE ?
            }
            DATA [5] = 0x08;          //   X4 - mode
            DATA [6] = 0x00;          //   X4 - mode
            DATA [7] = 0x00;          //   X4 - mode
            DATA [8] = Par0;          // 8 U4 - 32bit baudRate Bits/s Baud rate in bits/second
            DATA [9] = Par1;          //           >baudRate uint32_t< DATA [8]-[9]-[10]-[11]
            DATA [10] = Par2;         //           Little Endian:     Hex:  00--08---07---00
            DATA [11] = Par3;         //           Big Endian:        Hex:  00--07---08---00 Decimal = **460800‬baud
            DATA [12] = 0x07;         // 12 X2 - inProtoMask
            DATA [13] = 0x00;         //    X2 - inProtoMask
            DATA [14] = 0x03;         // 14 X2 - outProtoMask
            DATA [15] = 0x00;         //    X2 - outProtoMask
            DATA [16] = 0x00;         // 16 X2 - flags
            DATA [17] = 0x00;         //    X2 - flags
            DATA [18] = 0x00;         // 18 U1 - reserved1
            DATA [19] = 0x00;         // 19 U1 - reserved2
          }
        }
      } // end UBX-CFG-PRT (0x06 0x00)

      // ID CFG-RATE (0x08)
      if (ID == 0x08) { // ID CFG-RATE (0x06 0x08) Navigation/Measurement Rate Settings, e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
        if (Bit16PayloadLen == 0x06) { // Length (Bytes) DEC 6
          DATA [0] = Identifier;     // 0  U2 - measRate ms
          DATA [1] = reserved0;      //    U2 - measRate
          DATA [2] = Par0;           // 2  U2 - navRate
          DATA [3] = Par1;           //    U2 - navRate
          DATA [4] = Par2;           // 4  U2 - timeRef
          DATA [5] = Par3;           //    U2 - timeRef
        }
      } // end UBX-CFG-RATE (0x06 0x08)

      // ID CFG-NAV5 (0x24)
      if (ID == 0x24) { // ID CFG-NAV5 (0x06 0x24) Navigation Engine Settings
        if (Bit16PayloadLen == 0x24) { // Length (Bytes) DEC 36
          DATA [0] = Identifier;     // 0  X2 - mask
          DATA [1] = reserved0;      //    X2 - mask
          DATA [2] = Par0;           // 2  U1 - dynModel
          DATA [3] = 0x00;           // 3  U1 - fixMode
          DATA [4] = 0x00;           // 4  I4 - fixedAlt, Scaling 0.01
          DATA [5] = 0x00;           //    I4 - fixedAlt
          DATA [6] = 0x00;           //    I4 - fixedAlt
          DATA [7] = 0x00;           //    I4 - fixedAlt
          DATA [8] = 0x00;           // 8  U4 - fixedAltVar, Scaling 0.0001
          DATA [9] = 0x00;           //    U4 - fixedAltVar
          DATA [10] = 0x00;          //    U4 - fixedAltVar
          DATA [11] = 0x00;          //    U4 - fixedAltVar
          DATA [12] = 0x00;          // 12 I1 - minElev
          DATA [13] = 0x00;          // 13 U1 - drLimit
          DATA [14] = 0x00;          // 14 U2 - pDop, Scaling 0.1
          DATA [15] = 0x00;          //    U2 - pDop
          DATA [16] = 0x00;          // 16 U2 - tDop, Scaling 0.1
          DATA [17] = 0x00;          //    U2 - tDop
          DATA [18] = 0x00;          // 18 U2 - pAcc
          DATA [19] = 0x00;          //    U2 - pAcc
          DATA [20] = 0x00;          // 20 U2 - tAcc
          DATA [21] = 0x00;          //    U2 - tAcc
          DATA [22] = 0x00;          // 22 U1 - staticHoldThresh
          DATA [23] = 0x00;          // 23 U1 - dgnssTimeout
          DATA [24] = 0x00;          // 24 U1 - cnoThreshNumSVs
          DATA [25] = 0x00;          // 25 U1 - cnoThresh
          DATA [26] = 0x00;          // 26 U1[2] - reserved1
          DATA [27] = 0x00;          //    U1 -    reserved1
          DATA [28] = 0x00;          // 28 U2 - staticHoldMaxDist
          DATA [29] = 0x00;          //    U2 - staticHoldMaxDist
          DATA [30] = 0x00;          // 30 U1 - utcStandard
          DATA [31] = 0x00;          // 31 U1[5] - reserved2
          DATA [32] = 0x00;          //    U1 -    reserved2
          DATA [33] = 0x00;          //    U1 -    reserved2
          DATA [34] = 0x00;          //    U1 -    reserved2
          DATA [35] = 0x00;          //    U1 -    reserved2
        }
      } // end UBX-CFG-NAV5 (0x06 0x24)
      break;
    case 0x0A:    // MON 0x0A
    case 0x0B:    // AID 0x0B
    case 0x0D:    // TIM 0x0D
    case 0x10:    // ESF 0x10
    case 0x13:    // MGA 0x13
    case 0x21:    // LOG 0x21
    case 0x27:    // SEC 0x27
    case 0x28:    // HNR 0x28
    //break;
    default:
      // Not Supported
      Serial.println("Not Supported COMMAND CLASS:"), Serial.println(CLASS, HEX);
      break;
  }
  // The checksum is calculated over the packet, starting and including the
  // CLASS field, ID field and payload, up until, but excluding, the Checksum Field
  // (CRC) Compute checksum
  // NOTE: @ GPS baud 460800 CRC = 0x0C, 0xBC
  uint8_t CK_A = 0, CK_B = 0;
  CK_A = CK_A + CLASS;
  CK_B = CK_B + CK_A;
  CK_A = CK_A + ID;
  CK_B = CK_B + CK_A;
  CK_A = CK_A + PayloadLen0;
  CK_B = CK_B + CK_A;
  CK_A = CK_A + PayloadLen1;
  CK_B = CK_B + CK_A;
  for (int i = 0; i < Bit16PayloadLen ; i++) {
    CK_A = CK_A + DATA[i];
    CK_B = CK_B + CK_A;
  }
  // Generate the command
  if (Bit16PayloadLen == 0x06) { // Payload Length HEX 06 ~ DEC 6
    uint8_t COMMpayload[HeaderLength + CLASSandID + LenF + Bit16PayloadLen + CRC] = {
      /* Header */
      0xB5,           //[ 0] synchronization characters: 0xB5 0x62
      0x62,           //[ 1]
      CLASS,          //[ 2] Destination,  // B5 62 06 08 06 00 D0 07 01 00 01 00 ED BD
      ID,             //[ 3]
      PayloadLen0,    //[ 4] 8bit Payload Length
      PayloadLen1,    //[ 5] 8bit Payload Length
      (DATA [0]),     //[ 6] Payload HEX 0x06 ~ DEC 6
      (DATA [1]),     //[ 7]
      (DATA [2]),     //[ 8]
      (DATA [3]),     //[ 9]
      (DATA [4]),     //[10]
      (DATA [5]),     //[11]
      CK_A,           //[12] CRC 16-bit
      CK_B            //[13]
    };
    _port->flush(); // Waits for the transmission of outgoing serial data to complete
    _port->write(COMMpayload, sizeof(COMMpayload));
  }
  if (Bit16PayloadLen == 0x14) { // Payload Length HEX 14 ~ DEC 20
    uint8_t COMMpayload[HeaderLength + CLASSandID + LenF + Bit16PayloadLen + CRC] = {
      /* Header */
      0xB5,           //[ 0] synchronization characters: 0xB5 0x62
      0x62,           //[ 1]
      CLASS,          //[ 2] Destination
      ID,             //[ 3]
      PayloadLen0,    //[ 4] 8bit Payload Length
      PayloadLen1,    //[ 5] 8bit Payload Length
      (DATA [0]),     //[ 6] Payload HEX 14 ~ DEC 20
      (DATA [1]),     //[ 7]
      (DATA [2]),     //[ 8]
      (DATA [3]),     //[ 9]
      (DATA [4]),     //[10]
      (DATA [5]),     //[11]
      (DATA [6]),     //[12]
      (DATA [7]),     //[13]
      (DATA [8]),     //[14]
      (DATA [9]),     //[15]
      (DATA [10]),    //[16]
      (DATA [11]),    //[17]
      (DATA [12]),    //[18]
      (DATA [13]),    //[19]
      (DATA [14]),    //[20]
      (DATA [15]),    //[21]
      (DATA [16]),    //[22]
      (DATA [17]),    //[23]
      (DATA [18]),    //[24]
      (DATA [19]),    //[25]
      CK_A,           //[26] CRC 16-bit
      CK_B            //[27]
    };
    _port->flush(); // Waits for the transmission of outgoing serial data to complete
    _port->write(COMMpayload, sizeof(COMMpayload));
  }
  if (Bit16PayloadLen == 0x24) { // Payload Length HEX 24 ~ DEC 36
    uint8_t COMMpayload[HeaderLength + CLASSandID + LenF + Bit16PayloadLen + CRC] = {
      /* Header */
      0xB5,           //[ 0] synchronization characters: 0xB5 0x62
      0x62,           //[ 1]
      CLASS,          //[ 2] Destination
      ID,             //[ 3]
      PayloadLen0,    //[ 4] 8bit Payload Length
      PayloadLen1,    //[ 5] 8bit Payload Length
      (DATA [0]),     //[ 6] Payload HEX 24 ~ DEC 36
      (DATA [1]),     //[ 7]
      (DATA [2]),     //[ 8]
      (DATA [3]),     //[ 9]
      (DATA [4]),     //[10]
      (DATA [5]),     //[11]
      (DATA [6]),     //[12]
      (DATA [7]),     //[13]
      (DATA [8]),     //[14]
      (DATA [9]),     //[15]
      (DATA [10]),    //[16]
      (DATA [11]),    //[17]
      (DATA [12]),    //[18]
      (DATA [13]),    //[19]
      (DATA [14]),    //[20]
      (DATA [15]),    //[21]
      (DATA [16]),    //[22]
      (DATA [17]),    //[23]
      (DATA [18]),    //[24]
      (DATA [19]),    //[25]
      (DATA [20]),    //[26]
      (DATA [21]),    //[27]
      (DATA [22]),    //[28]
      (DATA [23]),    //[29]
      (DATA [24]),    //[30]
      (DATA [25]),    //[31]
      (DATA [26]),    //[32]
      (DATA [27]),    //[33]
      (DATA [28]),    //[34]
      (DATA [29]),    //[35]
      (DATA [30]),    //[36]
      (DATA [31]),    //[37]
      (DATA [32]),    //[38]
      (DATA [33]),    //[39]
      (DATA [34]),    //[40]
      (DATA [35]),    //[41]
      CK_A,           //[42] CRC 16-bit
      CK_B            //[43]
    };
    _port->flush(); // Waits for the transmission of outgoing serial data to complete
    _port->write(COMMpayload, sizeof(COMMpayload));
  }
}

/* write the uBlox data */
/******************************************/
/**  High level commands, for the user    */
/******************************************/

// UBX-CFG-PRT (0x06 0x00) SetGPSbaud
// Generate the configuration command's syntax
void UBLOX::SetGPSbaud(uint32_t baud32rate, bool printACK) {
  bool Supported = false;
  if ( printACK == true ) {
    Serial.print("Setting CFG-PRT (0x06 0x00) GPS UART baud rate:");
    _pACK2 = true; // print ACK to USB
  }
  // Set GPS Uart1 baud rate (32bit)
  // Possible Configurations
  switch (baud32rate) {
    case 4800:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("4800");
      }
      break;
    case 9600:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("9600");
      }
      break;
    case 19200:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("19200");
      }
      break;
    case 38400:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("38400");
      }
      break;
    case 57600:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("57600");
      }
      break;
    case 115200:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("115200");
      }
      break;
    case 230400:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("230400");
      }
      break;
    case 460800:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("460800");
      }
      break;
    default:
      Supported = false;
      _pACK2 = false;
      Serial.print("\tNot Supported :"), Serial.println(baud32rate), Serial.println("\tPossible Baud Rate Configurations");
      Serial.println("\t4800~9600~19200~38400~57600~115200~230400~460800"), Serial.println();
      break;
  }
  if ( Supported == true ) {
    // Splitting an uint32_t baud32rate
    uint8_t Par3 = ((baud32rate & 0xFF000000) >> 24); // Parameter3
    uint8_t Par2 = ((baud32rate & 0x00FF0000) >> 16);
    uint8_t Par1 = ((baud32rate & 0x0000FF00) >> 8);
    uint8_t Par0 =  (baud32rate & 0x000000FF);
    uint8_t CLASS = 0x06, ID = 0x00, Length0 = 0x14, Length1 = 0x00, Identifier = 0x01, res0 = 0; // reserved0 -- NOT IN USE
    // Identifier Port # Electrical Interface
    // 0 DDC (I²C compatible)
    // 1 UART 1
    // 3 USB
    // 4 SPI
    writeCommand(CLASS, ID, Length0, Length1, Identifier, res0, Par0, Par1, Par2, Par3);
  }
}

// UBX-CFG-RATE (0x06 0x08)  Set Navigation/Measurement Rate Settings (100ms=10.00Hz, 200ms=5.00Hz, 1000ms=1.00Hz, Etc)
// Generate the configuration command's syntax
void UBLOX::SetRATE(uint16_t measRate, bool printACK) {
  bool Supported = false;
  if ( printACK == true ) {
    Serial.print("Setting CFG-RATE (0x06 0x08) Navigation/Measurement Rate ");
    _pACK_RATE = true; // print ACK to USB
  }
  // Possible Configurations
  // _______________________________________________________________
  // 60ms    16.67Hz <-- | 192ms    5.21Hz     | 600ms    1.67Hz
  // 64ms    15.63Hz <-- | 200ms    5.00Hz <-- | 625ms    1.60Hz
  // 72ms    13.89Hz <-- | 225ms    4.44Hz     | 640ms    1.56Hz
  // 75ms    13.33Hz <-- | 240ms    4.17Hz     | 720ms    1.39Hz
  // 80ms    12.50Hz <-- | 250ms    4.00Hz <-- | 750ms    1.33Hz
  // 90ms    11.11Hz     | 288ms    3.47Hz     | 800ms    1.25Hz <--
  // 96ms    10.42Hz     | 300ms    3.33Hz     | 900ms    1.11Hz
  // 100ms   10.00Hz <-- | 320ms    3.13Hz     | 960ms    1.04Hz
  // 120ms    8.33Hz     | 360ms    2.78Hz     | 1000ms   1.00Hz <-- 
  // 125ms    8.00Hz <-- | 375ms    2.67Hz     | 2000ms   0.50Hz <-- 
  // 128ms    7.81Hz     | 400ms    2.50Hz <-- | 4000ms   0.25Hz <--
  // 144ms    6.94Hz     | 450ms    2.22Hz     | 10000ms  0.10Hz <--
  // 150ms    6.67Hz     | 480ms    2.08Hz     | 20000ms  0.05Hz <--
  // 160ms    6.25Hz     | 500ms    2.00Hz <-- | 50000ms  0.02Hz <-- 
  // 180ms    5.56Hz     | 576ms    1.74Hz     |
  // NOTE: Only rate settings marked with arrows are the implemented configuration.
  switch (measRate) {
    case 60:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("60ms 16.67Hz");
      }
      break;
    case 64:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("64ms 15.63Hz");
      }
      break;
    case 72:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("72ms 13.89Hz");
      }
      break;
    case 80:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("80ms 12.50Hz");
      }
      break;
    case 100:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("100ms 10.00Hz");
      }
      break;
    case 125:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("125ms 8.00Hz");
      }
      break;
    case 200:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("200ms 5.00Hz");
      }
      break;
    case 250:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("250ms 4.00Hz");
      }
      break;
    case 400:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("400ms 2.50Hz");
      }
      break;
    case 500:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("500ms 2.00Hz");
      }
      break;
    case 800:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("800ms 1.25Hz");
      }
      break;
    case 1000:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("1000ms 1.00Hz");
      }
      break;
    case 2000:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("2000ms 0.50Hz");
      }
      break;
    case 4000:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("4000ms 0.25Hz");
      }
      break;
    case 10000:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("10000ms 0.10Hz");
      }
      break;
    case 20000:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("20000ms 0.05Hz");
      }
      break;
    case 50000:
      Supported = true;
      if ( printACK == true ) {
        Serial.println("50000ms 0.02Hz");
      }
      break;
    default:
      Supported = false;
      _pACK_RATE = false;
      Serial.print("\tNot Supported :"), Serial.println(measRate), Serial.println("\tPossible Configurations");
      Serial.println("\t60=16.67Hz 64=15.63Hz 72=13.89Hz 80=12.50Hz 100=10.00Hz 125=8.00Hz 200=5.00Hz 250=4.00Hz 500=2.00Hz");
      Serial.println("\t800=1.25Hz 1000=1.00Hz 2000=0.50Hz 4000=0.25Hz 10000=0.10Hz 20000=0.05Hz 50000=0.02Hz "), Serial.println();
      break;
  }
  if ( Supported == true ) {
    // Splitting an uint16_t measRate
    // Identifier & res0 - 16bit Parameter defines the rate. e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz.
    uint8_t res0 = ((measRate & 0xFF00) >> 8);
    uint8_t Identifier =  (measRate & 0x00FF);
    uint8_t CLASS = 0x06, ID = 0x08, Length0 = 0x06, Length1 = 0x00;
    uint8_t Par0 = 0x01, Par1 = 0x00, Par2 = 0x01, Par3 = 0x00; // navRate and timeRef Parameters.
    writeCommand(CLASS, ID, Length0, Length1, Identifier, res0, Par0, Par1, Par2, Par3);
  }
}

// UBX-CFG-NAV5 (0x06 0x24) - Set Navigation Engine Settings
// Generate the configuration command's syntax
void UBLOX::SetNAV5(uint8_t dynModel, bool printACK) {
  bool Supported = false;
  uint8_t Par0 = 0;
  if ( printACK == true ) {
    Serial.print("Setting CFG-NAV5 (0x06 0x24) Dynamic platform model ");
    _pACK_NAV5 = true;
  }
  // Possible Configurations Dynamic platform model:
  // 0: portable
  // 2: stationary
  // 3: pedestrian
  // 4: automotive
  // 5: sea
  // 6: airborne with <1g acceleration
  // 7: airborne with <2g acceleration
  // 8: airborne with <4g acceleration
  // 9: wrist worn watch (not supported in protocol versions less than 18)
  switch (dynModel) {
    case 0:
      Supported = true;
      Par0 = 0;        // Parameter0
      if ( printACK == true ) {
        Serial.println("0: portable");
      }
      break;
    case 2:
      Supported = true;
      Par0 = 2;
      if ( printACK == true ) {
        Serial.println("2: stationary");
      }
      break;
    case 3:
      Supported = true;
      Par0 = 3;
      if ( printACK == true ) {
        Serial.println("3: pedestrian");
      }
      break;
    case 4:
      Supported = true;
      Par0 = 4;
      if ( printACK == true ) {
        Serial.println("4: automotive");
      }
      break;
    case 5:
      Supported = true;
      Par0 = 5;
      if ( printACK == true ) {
        Serial.println("5: sea");
      }
      break;
    case 6:
      Supported = true;
      Par0 = 6;
      if ( printACK == true ) {
        Serial.println("6: airborne with <1g acceleration");
      }
      break;
    case 7:
      Supported = true;
      Par0 = 7;
      if ( printACK == true ) {
        Serial.println("7: airborne with <2g acceleration");
      }
      break;
    case 8:
      Supported = true;
      Par0 = 8;
      if ( printACK == true ) {
        Serial.println("8: airborne with <4g acceleration");
      }
      break;
    case 9:
      Supported = true;
      Par0 = 9;
      if ( printACK == true ) {
        Serial.println("9: wrist worn watch (not supported in protocol Ver. less than 18");
      }
      break;
    default:
      Supported = false;
      _pACK_NAV5 = false;
      Serial.print("\tNot Supported :"), Serial.println(dynModel), Serial.println("\tPossible Configurations");
      Serial.println("\t0: portable 2: stationary 3: pedestrian 4: automotive 5: sea 6: airborne with <1g 7: airborne with <2g");
      Serial.println("\t8: airborne with <4g 9: wrist worn watch (not supported in protocol v.less than 18)"), Serial.println();
      break;
  }
  if ( Supported == true ) {
    // Identifier & res0 - 16bit Parameter bitmask. Only the masked parameters will be applied.
    uint8_t CLASS = 0x06, ID = 0x24, Length0 = 0x24, Length1 = 0x00, Identifier = B00000001, res0 = B00000000; // reserved0
    uint8_t Par1 = 0, Par2 = 0, Par3 = 0; // Parameters, Par0 set in switch case.
    writeCommand(CLASS, ID, Length0, Length1, Identifier, res0, Par0, Par1, Par2, Par3);
  }
}

void UBLOX::Poll_GPSbaud_Port1(bool printACK) {
  // CFG-PRT (0x06 0x00) Polls the configuration for one I/O Port, (I/O Target # MSG) 0x01=I/O UART1 0x06001
  while (10 > _port->availableForWrite()) { }
  _port->write(_Poll_CFG_PRT, sizeof(_Poll_CFG_PRT));
  if ( printACK == true ) {
    _printACK = 0x06001; // print ACK to USB - Clas 06, Id 00, print ACK I/O UART1 0x06001
    parse(); // force reading serial on cold boot
  }
}
void UBLOX::Poll_NAV_PVT() {
  while (12 > _port->availableForWrite()) { } // check if the hardware serial port has room for outgoing bytes to TX buffer
  _port->write(_Poll_NAV_PVT, sizeof(_Poll_NAV_PVT));
}
void UBLOX::Poll_NAV_POSLLH() {
  while (12 > _port->availableForWrite()) { }
  _port->write(_Poll_NAV_POSLLH, sizeof(_Poll_NAV_POSLLH));
}
void UBLOX::Poll_NAV_ATT() {
  _port->write(_Poll_NAV_ATT, sizeof(_Poll_NAV_ATT));
}
// #### Periodic auto update ON,OFF command ####
void UBLOX::Ena_NAV_PVT(bool printACK) { // Enable periodic auto update NAV_PVT
  while (12 > _port->availableForWrite()) { }
  _port->write(_Ena_NAV_PVT, sizeof(_Ena_NAV_PVT));
  if ( printACK == true ) {
    Serial.println("Setting Ena_NAV_PVT");
    _printACK_PVT = 0x06011; // Clas 06, Id 01, Ena 1 - print ACK Ena_NAV_PVT 0x06011
  }
}
void UBLOX::Dis_NAV_PVT(bool printACK) { // Disable periodic auto update NAV_PVT
  while (12 > _port->availableForWrite()) { }
  _port->write(_Dis_NAV_PVT, sizeof(_Dis_NAV_PVT));
  if ( printACK == true ) {
    Serial.println("Setting Dis_NAV_PVT");
    _printACK_PVT = 0x06010; // Clas 06, Id 01, Dis 0 - print ACK Dis_NAV_PVT 0x06010
  }
}
void UBLOX::Ena_NAV_ATT(bool printACK) { // Enable periodic auto update NAV_ATT
  while (12 > _port->availableForWrite()) { }
  _port->write(_Ena_NAV_ATT, sizeof(_Ena_NAV_ATT));
  if ( printACK == true ) {
    Serial.println("Setting Ena_NAV_ATT");
    _printACK_ATT = 0x06011; // Clas 06, Id 01, Ena 1
  } else {
    _printACK_ATT = 0xFF; // need for ACK_NAK, NAV_ATT Supported only on protocol version 19 (only with ADR or UDR products)
  }
}
void UBLOX::Dis_NAV_ATT(bool printACK) { // Disable periodic auto update NAV_ATT
  while (12 > _port->availableForWrite()) { }
  _port->write(_Dis_NAV_ATT, sizeof(_Dis_NAV_ATT));
  if ( printACK == true ) {
    Serial.println("Setting Dis_NAV_ATT");
    _printACK_ATT = 0x06010; // Clas 06, Id 01, Dis 0
  } else {
    _printACK_ATT = 0xFF;
  }
}
void UBLOX::Ena_NAV_POSLLH(bool printACK) { // Enable periodic auto update NAV_POSLLH
  while (12 > _port->availableForWrite()) { }
  _port->write(_Ena_NAV_POSLLH, sizeof(_Ena_NAV_POSLLH));
  if ( printACK == true ) {
    Serial.println("Setting Ena_NAV_POSLLH");
    _printACK_POSLLH = 0x06011; // Clas 06, Id 01, Ena 1
  }
}
void UBLOX::Dis_NAV_POSLLH(bool printACK) { // Disable periodic auto update NAV_POSLLH
  while (12 > _port->availableForWrite()) { }
  _port->write(_Dis_NAV_POSLLH, sizeof(_Dis_NAV_POSLLH));
  if ( printACK == true ) {
    Serial.println("Setting Dis_NAV_POSLLH");
    _printACK_POSLLH = 0x06010; // Clas 06, Id 01, Dis 0
  }
}

// #### u-blox Disable All NMEA Child Messages Command ####
void UBLOX::Dis_all_NMEA_Child_MSGs(bool printACK) {
  if ( printACK == true ) {
    // Ensure any pending writes are complete before writing new data
    while ( 38 >= _port->availableForWrite() ) {} // make sure we have at least 39 bytes available in TX buffer on start
    /*int wr;
      wr = _port->availableForWrite();
      if (wr <= 39) {
      Serial.print(" Buffer Available For Write only=");
      Serial.println(wr);
      Serial.print(" Please edit Teensyduino Core file serial"), Serial.print(_bus), Serial.println(".c, look for #define SERIAL#_TX_BUFFER_SIZE  40 // number of outgoing bytes to buffer");
      Serial.println(" This Command (Dis_all_NMEA_Child_MSGs(true) - needs 165 bytes of TX BUFFER) recommended TX buffer of 200");
      Serial.println(" I also strongly suggest changing RX buffer to 255 or more,*otherwise you will miss ACKs*on serial monitor");
      }*/

    _printACK_Dis_all_NMEA = 0x06010; // print ACK to USB - Clas 06, Id 01, print ACK Dis_all_NMEA 0x0601
    Serial.println("Dis_all_NMEA_Child_MSGs");
    Serial.println("Setting Dis_NMEA_GxDTM");
    Serial.println("Setting Dis_NMEA_GxGBS");
    Serial.println("Setting Dis_NMEA_GxGGA");
    Serial.println("Setting Dis_NMEA_GxGLL");
    Serial.println("Setting Dis_NMEA_GxGNS");
    Serial.println("Setting Dis_NMEA_GxGRS");
    Serial.println("Setting Dis_NMEA_GxGSA");
    Serial.println("Setting Dis_NMEA_GxGST");
    Serial.println("Setting Dis_NMEA_GxGSV");
    Serial.println("Setting Dis_NMEA_GxRMC");
    // Serial.println("Setting Dis_NMEA_GxVLW");   // u-blox6 -- Message Not-Acknowledged
    Serial.println("Setting Dis_NMEA_GxVTG");
    Serial.println("Setting Dis_NMEA_GxZDA");
    Serial.println("Setting Dis_PUBXe_NMEA00");
    // Serial.println("Setting Dis_PUBXe_NMEA01"); // u-blox6 -- Message Not-Acknowledged
    Serial.println("Setting Dis_PUBXe_NMEA03");
    Serial.println("Setting Dis_PUBXe_NMEA04");
    // Serial.println("Setting Dis_PUBXe_NMEA05"); // u-blox6 -- Message Not-Acknowledged
    // Serial.println("Setting Dis_PUBXe_NMEA06"); // u-blox6 -- Message Not-Acknowledged
  } else {
    _printACK_Dis_all_NMEA = 0x00;
  }
  _ACKcount = 15; // Set NMEA ACK counter #
  while (12 > _port->availableForWrite()) {} // check if the hardware serial port has room for outgoing bytes to TX buffer
  _port->write(_Dis_NMEA_GxDTM, sizeof(_Dis_NMEA_GxDTM));             //#1  / NMEA GxDTM (DTM 0xF0 0x0A) Datum Reference)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGBS, sizeof(_Dis_NMEA_GxGBS));             //#2  / NMEA GxGBS (GBS 0xF0 0x09) GNSS Satellite Fault Detection)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGGA, sizeof(_Dis_NMEA_GxGGA));             //#3  / NMEA GxGGA (GGA 0xF0 0x00) Global positioning system fix data)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGLL, sizeof(_Dis_NMEA_GxGLL));             //#4  / NMEA GxGLL (GLL 0xF0 0x01) Latitude and longitude, with time of position fix and status)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGNS, sizeof(_Dis_NMEA_GxGNS));             //#5  / NMEA GxGNS (GNS 0xF0 0x0D) GNSS fix data)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGRS, sizeof(_Dis_NMEA_GxGRS));             //#6  / NMEA GxGRS (GRS 0xF0 0x06) GNSS Range Residuals)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGSA, sizeof(_Dis_NMEA_GxGSA));             //#7  / NMEA GxGSA (GSA 0xF0 0x02) GNSS DOP and Active Satellites)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGST, sizeof(_Dis_NMEA_GxGST));             //#8  / NMEA GxGST (GST 0xF0 0x07) GNSS Pseudo Range Error Statistics)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxGSV, sizeof(_Dis_NMEA_GxGSV));             //#9  / NMEA GxGSV (GSV 0xF0 0x03) GNSS Satellites in View)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxRMC, sizeof(_Dis_NMEA_GxRMC));             //#10 / NMEA GxRMC (RMC 0xF0 0x04) Recommended Minimum data)
  while (12 > _port->availableForWrite()) {}
  // u-blox6 -- Message Not-Acknowledged
  //_port->write(_Dis_NMEA_GxVLW, sizeof(_Dis_NMEA_GxVLW));           //    / NMEA GxVLW (VLW 0xF0 0x0F) Dual ground/water distance)
  _port->write(_Dis_NMEA_GxVTG, sizeof(_Dis_NMEA_GxVTG));             //#11 / NMEA GxVTG (VTG 0xF0 0x05) Course over ground and Ground speed)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_NMEA_GxZDA, sizeof(_Dis_NMEA_GxZDA));             //#12 / NMEA GxZDA (ZDA 0xF0 0x08) Time and Date)
  while (12 > _port->availableForWrite()) {}
  // #### PUBX- u-blox NMEA extension - Switch OFF ALL ####
  _port->write(_Dis_PUBXe_NMEA00, sizeof(_Dis_PUBXe_NMEA00));         //#13 / PUBX 00 (Position Data)
  while (12 > _port->availableForWrite()) {}
  // u-blox6 -- Message Not-Acknowledged
  //_port->write(_Dis_PUBXe_NMEA01, sizeof(_Dis_PUBXe_NMEA01));       //    / PUBX 01 (UTM Position Data)
  _port->write(_Dis_PUBXe_NMEA03, sizeof(_Dis_PUBXe_NMEA03));         //#14 / PUBX 03 (Satellite Data)
  while (12 > _port->availableForWrite()) {}
  _port->write(_Dis_PUBXe_NMEA04, sizeof(_Dis_PUBXe_NMEA04));         //#15 / PUBX 04 (Time of Day)
  while (12 > _port->availableForWrite()) {}
  // u-blox6 -- Message Not-Acknowledged
  //_port->write(_Dis_PUBXe_NMEA05, sizeof(_Dis_PUBXe_NMEA05));       //    / PUBX 05 (EKF Status)
  //_port->write(_Dis_PUBXe_NMEA06, sizeof(_Dis_PUBXe_NMEA06));       //    / PUBX 06 (GPS-only on EKF products)
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
    signed short val;
    uint8_t b[2];
  } magDec;

  union {
    unsigned short val;
    uint8_t b[2];
  } magAcc;

  // UBX-NAV-ATT (0x01 0x05) Attitude Solution
  union {
    long val;
    uint8_t b[4];
  } roll;

  union {
    long val;
    uint8_t b[4];
  } pitch;

  union {
    long val;
    uint8_t b[4];
  } accRoll;

  union {
    long val;
    uint8_t b[4];
  } accPitch;

  union {
    long val;
    uint8_t b[4];
  } accHeading;

  // Current GPS Uart1 Baud
  union {
    uint32_t val;
    uint8_t b[4];
  } GpsUart1Baud;

  // parse the uBlox packet
  if (parse()) {
    // uBlox Message (Class, ID) # Description #
    // UBX-NAV-PVT   (0x01 0x07) Navigation Position Velocity Time Solution
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
      // return true on receiving a full packet
      return true;
    }

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
      // return true on receiving a full packet
      return true;
    }

    // UBX-NAV-ATT (0x01 0x05) Attitude Solution
    if (_UBX_NAV_ATT_ID == true) {
      iTOW.b[0] = _gpsPayload[4]; // GPS time of week of the navigation epoch.
      iTOW.b[1] = _gpsPayload[5];
      iTOW.b[2] = _gpsPayload[6];
      iTOW.b[3] = _gpsPayload[7];
      gpsData_ptr->iTOW = iTOW.val;

      gpsData_ptr->version = _gpsPayload[8]; // Message version (0 for this version)

      // U1[3] - reserved1 - Reserved
      // gpsData_ptr->reserved1 = _gpsPayload[9];
      // gpsData_ptr->reserved2 = _gpsPayload[10];
      // gpsData_ptr->reserved3 = _gpsPayload[11];

      roll.b[0] = _gpsPayload[12]; // Vehicle roll.
      roll.b[1] = _gpsPayload[13];
      roll.b[2] = _gpsPayload[14];
      roll.b[3] = _gpsPayload[15];
      gpsData_ptr->roll = roll.val * en5;

      pitch.b[0] = _gpsPayload[16]; // Vehicle pitch.
      pitch.b[1] = _gpsPayload[17];
      pitch.b[2] = _gpsPayload[18];
      pitch.b[3] = _gpsPayload[19];
      gpsData_ptr->pitch = pitch.val * en5;

      heading.b[0] = _gpsPayload[20]; // heading
      heading.b[1] = _gpsPayload[21];
      heading.b[2] = _gpsPayload[22];
      heading.b[3] = _gpsPayload[23];
      gpsData_ptr->heading = heading.val * en5;

      accRoll.b[0] = _gpsPayload[24]; // Vehicle roll accuracy (if null, roll angle is not available).
      accRoll.b[1] = _gpsPayload[25];
      accRoll.b[2] = _gpsPayload[26];
      accRoll.b[3] = _gpsPayload[27];
      gpsData_ptr->accRoll = accRoll.val * en5;

      accPitch.b[0] = _gpsPayload[28]; // Vehicle pitch accuracy (if null, pitch angle is not available).
      accPitch.b[1] = _gpsPayload[29];
      accPitch.b[2] = _gpsPayload[30];
      accPitch.b[3] = _gpsPayload[31];
      gpsData_ptr->accPitch = accPitch.val * en5;

      accHeading.b[0] = _gpsPayload[32]; // Vehicle heading accuracy (if null, heading angle is not available).
      accHeading.b[1] = _gpsPayload[33];
      accHeading.b[2] = _gpsPayload[34];
      accHeading.b[3] = _gpsPayload[35];
      gpsData_ptr->accHeading = accHeading.val * en5;
      _UBX_NAV_ATT_ID = false;
      // return true on receiving a full packet
      return true;
    }

    // UBX-CFG-PRT 0x06 0x00 Poll Request Polls the configuration for one I/O Port , len[1]
    // UBX-CFG-PRT 0x06 0x00 Get/Set Port Configuration for UART-USB-SPI-DDC(I²C) Port, len[20]
    if (_UBX_CFG_PRT_ID == true) {
      // Decide what to do based on payload size, length in decimal format
      uint8_t MSGpayload = (_gpsPayload[3] << 8) | _gpsPayload[2]; // Combining two uint8_t as uint16_t
      switch (MSGpayload) { // 16Bit-Length Field
        case 1:   // if MSG payload size == 0x01 Poll Request Polls the configuration for one I/O Port , len[1] DEC
          Serial.print("Poll Request Polls the configuration for one I/O Port:"), Serial.println(_gpsPayload[4]), Serial.println(); // USB debug print
          break;
        case 20:  // if MSG payload size == (HEX0x14~DEC20) Get/Set Port Configuration for UART-USB-SPI-DDC(I²C) Port, len[20]
          // Decide what to do based on _gpsPayload:[4] Payload Content-I/O portID: 0=DDC(I2C), 1=UART1, 2=UART2, 3=USB, 4=SPI, 5=reserved
          if (_gpsPayload[4] == 1) { // 1=UART1
            //  *** Example UART1 460800‬ baud ***
            // Message Structure
            //    Header,Clas,  Id, DataSize 16b,     MSG, Checksum
            // 0xB5 0x62,0x06 0x00,     20 Bytes, Payload, CK_A CK_B
            // Payload Contents:
            //       Byte Offset___Number Format______Scaling__________Name______Unit_______________________Description
            //gpsPayload:[4]  0---- U1--uint8_t-------------------- portID --------- I/O port Target Number assignment
            //gpsPayload:[5]  1---- U1--uint8_t----------------- reserved0 ---------------------------------- Reserved
            //      --------- 2---- X2-uint16_t------------------- txReady -------------------------- Bitfield txReady
            //      --------- 4---- X4-uint32_t---------------------- mode ----------------------------- Bitfield mode
            //gpsPayload:[12] 8---- U4-uint32_t------------------ baudRate ---Bits/s---------- Baudrate in bits/second
            //gpsPayload:[13] 9
            //gpsPayload:[14]10
            //gpsPayload:[15]11
            //               12---- X2-uint16_t--------------- inProtoMask --------------- Mask input protocols active
            //               14---- X2-uint16_t-------------- outProtoMask -------------- Mask output protocols active
            //      -------- 16---- U2-uint16_t----------------- reserved4 ------------------------ Always set to zero
            //      ---------18---- U2-uint16_t----------------- reserved5 ------------------------ Always set to zero
            //                 Header|CL|Id|DSize| MSG Payload                                                 |Checksum|
            // _gpsPayload:[n]       | 0| 1| 2  3| 4  5  6  7  8  9 10 11  12 13 14 15  16 17 18 18 19 20 21 22|23 24   |
            //23:46:54   0000   B5 62|06 00|14 00|01 00 00 00 D0 08 00 00 >00 08 07 00< 07 00 03 00 00 00 00 00|0C BC   |  µb........Ð................¼.
            // >baudRate uint32_t<
            // Little Endian: Hex: 0x00080700
            // Big Endian:    Hex: 0x00070800 Decimal = **460800‬baud
            GpsUart1Baud.b[0] = _gpsPayload[12];
            GpsUart1Baud.b[1] = _gpsPayload[13];
            GpsUart1Baud.b[2] = _gpsPayload[14];
            GpsUart1Baud.b[3] = _gpsPayload[15];
            gpsData_ptr->GpsUart1Baud = GpsUart1Baud.val;
            // need this for ACK_ACK print
            _GpsUartBaud = (((_gpsPayload[15] << 24) | (_gpsPayload[14] << 16)) | (_gpsPayload[13] << 8)) | _gpsPayload[12]; // Combining
          } else {
            Serial.print("I/O portID: 0=DDC(I2C), 2=UART2, 3=USB, 4=SPI, 5=reserved "), Serial.print(_gpsPayload[4]), Serial.println(" Not implemented"), Serial.println(); // USB debug print
          }
          break;
        default:
          // Not Supported
          Serial.print("Unknown CFG PRT Length Field:"), Serial.println((_MSGpayloadSize) - 4, DEC), Serial.println();
          break;
      }
      _UBX_CFG_PRT_ID = false;
      // return false even though its packets are successfully being received, not a NAV packet
      return false;
    }

    if (_UBX_ACK_ACK_ID == true) {
      uint8_t CC = 0; // use only one if statement
      switch (_gpsPayload[4]) { // UBX_ACK_ACK(0x05 0x01) response to Class CFG 0x06
        case 0x06:   // CFG 0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc
          switch (_gpsPayload[5]) {    // ### Name ID Description ###  UBX_ACK_ACK response to ID
            case 0x00: // CFG-PRT 0x06 0x00:
              if ((_pACK2 == 1) || (_printACK == 0x06001)) {
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged -");
                if ((_printACK == 0x06001) && CC == 0) {
                  CC = 1;
                  Serial.print(" Poll_GPSbaud_Port1 baudRate:"), Serial.println(_GpsUartBaud);
                  _printACK = 0x00;
                }
                if ((_pACK2 == 1) && CC == 0) {
                  CC = 1;
                  Serial.println(" CFG-PRT (0x06 0x00) Set GPS UART baud");
                  _pACK2 = 0;
                }
              } else {
                if ((_pACK2 == 0) || (_printACK == 0x00)) {
                  // Do Nothing Here
                } else { // print all Unknown CFG ACK on USB
                  Serial.print("UBX_ACK_ACK: CFG-PRT 0x06 0x00"), Serial.print(" Message Acknowledged "), Serial.println();
                }
              }
              break;

            case 0x01:   // CFG-MSG 0x06 0x01: Set Message Rate(s)
              if ((_printACK_Dis_all_NMEA == 0x06010) && CC == 0) { // Dis_all_NMEA_Child_MSGs 0x06010
                CC = 1;
                //Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged -");
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged - ");
                Serial.print("Dis_all_NMEA ");
                Serial.print("-CFG-MSG(0x06 0x01): Set disable All NMEA "), Serial.println(_ACKcount);
                _ACKcount = _ACKcount - 1;
                if (_ACKcount == 0) { // NMEA ACK counter
                  _printACK_Dis_all_NMEA = 0x00; // reset
                  //Serial.println();
                }
              }
              if ((_printACK_ATT == 0x06011 || _printACK_ATT == 0x06010) && CC == 0) {
                CC = 1;
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged - ");
                if (_printACK_ATT == 0x06011) { // Ena_NAV_ATT 0x06011
                  Serial.print("Ena_NAV_ATT "), Serial.println("-Enable periodic auto update");
                  _printACK_ATT = 0x01; // 01
                }
                if (_printACK_ATT == 0x06010) { // Dis_NAV_ATT 0x06010
                  Serial.print("Dis_NAV_ATT "), Serial.println("-Disable periodic auto update");
                  _printACK_ATT = 0x02; // 01
                }
              }
              if ((_printACK_PVT == 0x06011 || _printACK_PVT == 0x06010) && CC == 0) {
                CC = 1;
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged - ");
                if (_printACK_PVT == 0x06011) { // Ena_NAV_PVT 0x06011
                  Serial.print("Ena_NAV_PVT "), Serial.println("-Enable periodic auto update");
                  _printACK_PVT = 0x00;
                }
                if (_printACK_PVT == 0x06010) { // Dis_NAV_PVT 0x06010
                  Serial.print("Dis_NAV_PVT "), Serial.println("-Disable periodic auto update");
                  _printACK_PVT = 0x00;
                }
              }
              if ((_printACK_POSLLH == 0x06011 || _printACK_POSLLH == 0x06010) && CC == 0) {
                CC = 1;
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged - ");
                if (_printACK_POSLLH == 0x06011) { // Ena_NAV_POSLLH 0x06011
                  Serial.print("Ena_NAV_POSLLH "), Serial.println("-Enable periodic auto update");
                  _printACK_POSLLH = 0x00;
                }
                if (_printACK_POSLLH == 0x06010) { // Dis_NAV_POSLLH 0x06010
                  Serial.print("Dis_NAV_POSLLH "), Serial.println("-Disable periodic auto update");
                  _printACK_POSLLH = 0x00;
                }
              }
              break;

            case 0x02:    // CFG-INF 0x06 0x02:       Poll configuration for one protocol or Set Information message configuration
              Serial.println("-CFG-INF 0x06 0x02"), Serial.println();
              break;
            case 0x06:    // CFG-DAT 0x06 0x06:       Set User-defined Datum or Get The currently defined Datum
              Serial.println("-CFG-DAT 0x06 0x06"), Serial.println();
              break;
            case 0x09:    // CFG-CFG 0x06 0x09:       Command Clear, Save and Load configurations
              Serial.println("-CFG-CFG 0x06 0x09"), Serial.println();
              break;
            case 0x08:    // CFG-RATE 0x06 0x08:      Get/Set Navigation/Measurement Rate Settings
              //_pACK_RATE =
              if (_pACK_RATE == true) {
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged ");
                Serial.println("- CFG-RATE 0x06 0x08 Get/Set Navigation/Measurement Rate Settings");
              }
              break;
            case 0x11:    // CFG-RXM 0x06 0x11:       Get/Set RXM configuration
              Serial.println("-CFG-RXM 0x06 0x11"), Serial.println();
              break;
            case 0x13:    // CFG-ANT 0x06 0x13:       Get/Set Antenna Control Settings
              Serial.println("-CFG-ANT 0x06 0x13"), Serial.println();
              break;
            case 0x16:    // CFG-SBAS 0x06 0x16:      Get/Set SBAS Configuration
              Serial.println("-CFG-SBAS 0x06 0x16"), Serial.println();
              break;
            case 0x17:    // CFG-NMEA 0x06 0x17:      Get/Set NMEA protocol configuration, V0 (deprecated) or Extended NMEA protocol configuration V1
              Serial.println("-CFG-NMEA 0x06 0x17"), Serial.println();
              break;
            case 0x1B:    // CFG-USB 0x06 0x1B:       Get/Set USB Configuration
              Serial.println("-CFG-USB 0x06 0x1B"), Serial.println();
              break;
            case 0x1E:    // CFG-ODO 0x06 0x1E:       Get/Set Odometer, Low-speed COG Engine Settings
              Serial.println("-CFG-ODO 0x06 0x1E"), Serial.println();
              break;
            case 0x23:    // CFG-NAVX5 0x06 0x23:     Get/Set Navigation Engine Expert Settings
              Serial.println("-CFG-NAVX5 0x06 0x23"), Serial.println();
              break;
            case 0x24:    // CFG-NAV5 0x06 0x24:      Get/Set Navigation Engine Settings
              if (_pACK_NAV5 == true) {
                Serial.print("UBX_ACK_ACK:"), Serial.print(" Message Acknowledged ");
                Serial.println("- CFG-NAV5 0x06 0x24 Get/Set Navigation Engine Settings");
              }
              break;
            case 0x31:    // CFG-TP5 0x06 0x31:       Poll Time Pulse Parameters for Time Pulse 0, Poll Time Pulse Parameters or Get/Set Time Pulse Parameters
              Serial.println("-CFG-TP5 0x06 0x31"), Serial.println();
              break;
            case 0x34:    // CFG-RINV 0x06 0x34       Get/Set Contents of Remote Inventory
              Serial.println("-CFG-RINV 0x06 0x34"), Serial.println();
              break;
            case 0x39:    // CFG-ITFM 0x06 0x39:      Get/Set Jamming/Interference Monitor configuration
              Serial.println("-CFG-ITFM 0x06 0x39"), Serial.println();
              break;
            case 0x3B:    // CFG-PM2 0x06 0x3B:       Get/Set Extended Power Management configuration
              Serial.println("-CFG-PM2 0x06 0x3B"), Serial.println();
              break;
            case 0x3D:    // CFG-TMODE2 0x06 0x3D:    Get/Set Time Mode Settings 2
              Serial.println("-CFG-TMODE2 0x06 0x3D"), Serial.println();
              break;
            case 0x3E:    // CFG-GNSS 0x06 0x3E:      Get/Set GNSS system configuration
              Serial.println("-CFG-GNSS 0x06 0x3E"), Serial.println();
              break;
            case 0x47:    // CFG-LOGFILTER 0x06 0x47: Get/Set Data Logger Configuration
              Serial.println("-CFG-LOGFILTER 0x06 0x47"), Serial.println();
              break;
            case 0x53:    // CFG-TXSLOT 0x06 0x53:    Set TX buffer time slots configuration
              Serial.println("-CFG-TXSLOT 0x06 0x53"), Serial.println();
              break;
            case 0x57:    // CFG-PWR 0x06 0x57:       Set Put receiver in a defined power state.
              Serial.println("-CFG-PWR 0x06 0x57"), Serial.println();
              break;
            case 0x5C:    // CFG-HNR 0x06 0x5C:       Get/Set High Navigation Rate Settings ### u-blox 8 only ###
              Serial.println("-CFG-HNR 0x06 0x5C"), Serial.println();
              break;
            case 0x60:    // CFG-ESRC 0x06 0x60:      Get/Set External synchronization source configuration
              Serial.println("-CFG-ESRC 0x06 0x60"), Serial.println();
              break;
            case 0x61:    // CFG-DOSC 0x06 0x61:      Get/Set Disciplined oscillator configuration
              Serial.println("-CFG-DOSC 0x06 0x61"), Serial.println();
              break;
            case 0x62:    // CFG-SMGR 0x06 0x62:      Get/Set Synchronization manager configuration
              Serial.println("-CFG-SMGR 0x06 0x62"), Serial.println();
              break;
            case 0x69:    // CFG-GEOFENCE 0x06 0x69:  Get/Set Geofencing configuration
              Serial.println("-CFG-GEOFENCE 0x06 0x69"), Serial.println();
              break;
            case 0x70:    // CFG-DGNSS 0x06 0x70:     Get/Set DGNSS configuration
              Serial.println("-CFG-DGNSS 0x06 0x70"), Serial.println();
              break;
            case 0x71:    // CFG-TMODE3 0x06 0x71:    Get/Set Time Mode Settings 3
              Serial.println("-CFG-TMODE3 0x06 0x71"), Serial.println();
              break;
            case 0x84:    // CFG-FIXSEED 0x06 0x84:   Set Programming the fixed seed for host...
              Serial.println("-CFG-FIXSEED 0x06 0x84"), Serial.println();
              break;
            case 0x85:    // CFG-DYNSEED 0x06 0x85:   Set Programming the dynamic seed for the host...
              Serial.println("-CFG-DYNSEED 0x06 0x85"), Serial.println();
              break;
            case 0x86:    // CFG-PMS 0x06 0x86:       Get/Set Power Mode Setup
              Serial.println("-CFG-PMS 0x06 0x86"), Serial.println();
              break;
            default:
              // Not Supported
              Serial.print("Unknown ACK CFG response to ID:");
              if (_gpsPayload[5] > 16) {
                Serial.print("0x");
              } else {
                Serial.print("0x0");
              }
              Serial.println(_gpsPayload[5], HEX), Serial.println();
              break;
          }
          break;
        default:
          // Not Supported
          Serial.print("Unknown ACK ACK response to Class:"), Serial.println((_gpsPayload[4]), HEX), Serial.println();
          break;
      }
      _UBX_ACK_ACK_ID = false;
      // return false even though its packets are successfully being received, not a NAV packet
      return false;
    }
    // return true on receiving a full packet
    // return true;
    return true;
  } else {
    // return false if a full packet is not received
    return false;
  }
}

/* parse the uBlox data */
bool UBLOX::parse() {
  // uBlox UBX header definition
  uint8_t const UBX_HEADER[] = { 0xB5, 0x62 };

  // checksum calculation
  static unsigned char checksum[2];

  // read a byte from the serial port
  while ( _port->available() ) {
    uint8_t c = _port->read();

    // identify the packet header
    if ( _fpos < 2 ) {
      if ( c == UBX_HEADER[_fpos] ) {
        _fpos++;
      } else {
        _fpos = 0;
        //if (c == 36) { // ASCII Table Dec 36~Char $
        //  Serial.write(c); // debug print NMEA message start $ Char
        //  Serial.println(" NMEA message start $ Char"); // USB debug print
        //}
      }
    }
    // Identify the packet Class
    else if (_fpos == 2) {
      switch (c) {    // ### Name Class Description ###
        case 0x01:    // NAV 0x01 Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
          _CurrentClass = c; // Save for Message ID.
          ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // grab the payload, need for checksum
          _fpos++;
          break;
        case 0x02:    // RXM 0x02 Receiver Manager Messages: Satellite Status, RTC Status
          _CurrentClass = c;
          Serial.print("UBX_RXM_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println();
          _fpos = 0;
          break;
        case 0x04:    // INF 0x04 Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
          _CurrentClass = c;
          Serial.print("UBX_INF_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println();
          _fpos = 0;
          break;
        case 0x05:    // ACK 0x05 Ack/Nack Messages: as replies to CFG Input Messages
          _CurrentClass = c;
          ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // grab the payload, need for checksum
          _fpos++;
          break;
        case 0x06:    // CFG 0x06 Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
          _CurrentClass = c; // Save for Message ID.
          ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // grab the payload, need for checksum
          _fpos++;
          break;
        case 0x0A:    // MON 0x0A Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
          Serial.print("UBX_MON_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println();
          _CurrentClass = c;
          _fpos = 0;
          break;
        case 0x0B:    // AID 0x0B AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
          Serial.print("UBX_AID_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println();
          _CurrentClass = c;
          _fpos = 0;
          break;
        case 0x0D:    // TIM 0x0D Timing Messages: Time Pulse Output, Timemark Results
          Serial.print("UBX_TIM_CLASS:0x0"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println();
          _CurrentClass = c;
          _fpos = 0;
          break;
        // Not Supported
        // ESF 0x10 External Sensor Fusion Messages: External Sensor Measurements and Status Information
        // MGA 0x13 Multiple GNSS Assistance Messages: Assistance data for various GNSS
        case 0x21:    // LOG 0x21 Logging Messages: Log creation, deletion, info and retrieval
          Serial.print("UBX_LOG_CLASS:0x"), Serial.print(c, HEX), Serial.println(" Not implemented"), Serial.println();
          _CurrentClass = c;
          _fpos = 0;
          break;
        // Not Supported
        // SEC 0x27 Security Feature Messages
        // HNR 0x28 High Rate Navigation Results Messages: High rate time, position, speed, heading
        default:
          // Not Supported
          _CurrentClass = c;
          _fpos = 0;
          Serial.print("Unknown packet Class:");
          if (c > 16) {
            Serial.print("0x");
          } else {
            Serial.print("0x0");
          }
          Serial.println(c, HEX), Serial.println(); // USB
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
          CFG_IDs(c); // Go grab the uBlox CFG - ID
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
          Serial.println("Unknown packet ID:"), Serial.println(c, HEX), Serial.println();
          break;
      }
      _CurrentClass = 0x00;
    }
    // Identify the packet 16Bit Message payloadSize
    // It does not include 16Bit-Sync header, 8Bit-Class, 8Bit-ID, 16Bit-Length Field, and 16Bit-CRC fields
    else if (_fpos == 4 ) {
      //Serial.write(c); // USB u-center
      ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // least significant byte (LSB)
      _fpos++;
    }
    else if (_fpos == 5 ) {
      //Serial.write(c); // USB u-center
      ((unsigned char*)_gpsPayload)[_fpos - 2] = c; // most significant byte (MSB)
      _MSGpayloadSize = (_gpsPayload[3] << 8) | _gpsPayload[2]; // Combining two uint8_t as uint16_t
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

/* uBlox CFG - IDs */
void UBLOX::CFG_IDs(uint8_t SERc) {
  switch (SERc) {       // ### Name CFG-Class & IDs Description, Msg Length ###
    case 0x00:          // UBX-CFG-PRT 0x06 0x00 Poll Request Polls the configuration for one I/O Port , len[1]
      //                   CFG-PRT 0x06 0x00 Get/Set Port Configuration for UART-USB-SPI-DDC(I²C) Port, len[20]
      _UBX_CFG_PRT_ID = true; // next - decide what to do based on MSG payload size
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERc; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x01:          // UBX-CFG-MSG 0x06 0x01 Poll Request Poll a message configuration, len[2]
      //                   UBX-CFG-MSG 0x06 0x01 Get/Set Set Message Rate(s), len[8]
      //                   UBX-CFG-MSG 0x06 0x01 Get/Set Set Message Rat, len[3]
      Serial.print("UBX_CFG_MSG ID:"), Serial.print(SERc, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    default:
      // Not Supported
      _fpos = 0;
      Serial.println(" Not implemented or Unknown packet Class CFG ID:"), Serial.println(SERc, HEX), Serial.println();
      break;
  }
}

/* uBlox ACK - IDs */
// When messages from the class CFG are sent to the receiver, the receiver will send an "acknowledge" (ACK-ACK) or a "not acknowledge" (ACK-NAK) message back to the sender.
void UBLOX::ACK_IDs(uint8_t SERa) {
  uint8_t CN = 0;
  switch (SERa) {       // ### Name ACK-Class & IDs Description, Msg Length ###
    case 0x00:          //  UBX-ACK-NAK (0x05 0x00) Message Not-Acknowledged, len[2]
      //Serial.println(" NAK- "); // todo remove
      if ((_printACK == 0x06001 || _printACK_ATT == 0x01 || _printACK_ATT == 0x02) && CN == 0) {
        Serial.print("UBX_ACK_NAK:"), Serial.print(" Message Not-Acknowledged ");
        if (_printACK == 0x06001) {
          CN = 1;
          Serial.println("- CFG_PRT baudRate"), Serial.println();
          _printACK = 0x00; // reset
        }
        if ((_printACK_ATT == 0x01) && CN == 0) { // Ena_NAV_ATT 0x06011
          CN = 1;
          Serial.println("- Ena_NAV_ATT, ATT Supported only on protocol v. 19 (only with ADR or UDR products)");
          _printACK_ATT = 0x00; // reset
        }
        if ((_printACK_ATT == 0x02) && CN == 0) { // Dis_NAV_ATT 0x06010
          CN = 1;
          Serial.println("- Dis_NAV_ATT, ATT Supported only on protocol v. 19 (only with ADR or UDR products)");
          _printACK_ATT = 0x00; // reset
        }
      } else {
        if ((_printACK_ATT == 0xFF) && CN == 0) {
          CN = 1;
          _printACK_ATT = 0x00;
        } else {
          Serial.print("UBX_ACK_NAK:"), Serial.println(" Message Not-Acknowledged "); //, Serial.println(SERa);
        }
      }
      // _UBX_ACK_NAK_ID = true; // Not implemented
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERa;
      _fpos++;
      break;
    case 0x01:          //  UBX-ACK-ACK (0x05 0x01) Message Acknowledged, len[2]
      _UBX_ACK_ACK_ID = true;
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERa;
      _fpos++;
      break;
    default:
      // Not Supported
      _fpos = 0;
      Serial.println("Unknown packet Class ACK ID:"), Serial.println(SERa, HEX), Serial.println();
      break;
  }
}

/* uBlox NAV - IDs */
void UBLOX::NAV_IDs(uint8_t SERn) {

  switch (SERn) {    // ### Name NAV-Class & IDs Description, Msg Length ###
    case 0x01:          // NAV-POSECEF 0x01 0x01 Position Solution in ECEF, U-Blox7 = len[20]
      Serial.print("UBX_NAV_POSECEF ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x02:          // NAV-POSLLH 0x01 0x02 Geodetic Position Solution, U-Blox7 = len[28]
      _UBX_NAV_POSLLH_ID = true;
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERn; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x03:          // NAV-STATUS 0x01 0x03 Receiver Navigation Status, U-Blox7 = len[16]
      Serial.print("UBX_NAV_STATUS ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x04:         // NAV-DOP 0x01 0x04 Dilution of precision, U-Blox7 = len[18]
      Serial.print("UBX_NAV_DOP ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    //                    NAV-ATT (0x01 0x05) Attitude Solution, U-Blox8 = len[32]
    case 0x05:         // #### NOTE: u-blox 8 only from protocol version 19 up to version 23.01 (only with ADR or UDR products) ###
      _UBX_NAV_ATT_ID = true;
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERn; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x06:         // NAV-SOL 0x01 0x06 Navigation Solution Information, U-Blox7 = len[52]
      Serial.print("UBX_NAV_SOL ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x07:        // NAV-PVT 0x01 0x07 Navigation Position Velocity Time Solution, U-Blox7=len[84], U-Blox8=len[92]
      _UBX_NAV_PVT_ID = true;
      ((unsigned char*)_gpsPayload)[_fpos - 2] = SERn; // grab the payload, need for checksum
      _fpos++;
      break;
    case 0x11:        // NAV-VELECEF 0x01 0x11 Velocity Solution in ECEF, U-Blox7 = len[20]
      Serial.print("UBX_NAV_VELECEF ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x12:        // NAV-VELNED 0x01 0x12 Velocity Solution in NED, U-Blox7 = len[36]
      Serial.print("UBX_NAV_VELNED ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x20:        // NAV-TIMEGPS 0x01 0x20 16 Periodic/Polled GPS Time Solution, U-Blox7 = len[16]
      Serial.print("UBX_NAV_TIMEGPS ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x21:        // NAV-TIMEUTC 0x01 0x21 UTC Time Solution, U-Blox7 = len[20]
      Serial.print("UBX_NAV_TIMEUTC ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x22:       // NAV-CLOCK 0x01 0x22 Clock Solution, U-Blox7 = len[20]
      Serial.print("UBX_NAV_CLOCK ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x30:       // NAV-SVINFO U-Blox7 = len[8 + 12*numCh]
      Serial.print("UBX_NAV_SVINFO ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x31:       // NAV-DGPS 0x01 0x31 DGPS Data Used for NAV, U-Blox7 = len[16 + 12*numCh]
      Serial.print("UBX_NAV_DGPS ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x32:       // NAV-SBAS 0x01 0x32 12 + 12*cnt Periodic/Polled SBAS Status Data
      Serial.print("UBX_NAV_SBAS ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    case 0x60:       // NAV-AOPSTATUS 0x01 0x60 AssistNow Autonomous Status, U-Blox7 = len[20]
      Serial.print("UBX_NAV_AOPSTATUS ID:"), Serial.print(SERn, HEX), Serial.println(" Not implemented"), Serial.println();
      _fpos = 0;
      break;
    default:
      // Not Supported
      _fpos = 0;
      Serial.println("Unknown packet Class NAV ID:"), Serial.println(SERn, HEX), Serial.println();
      break;
  }
}

void UBLOX::DEBUG() {
  // DEBUG PRINT
  Serial.println("HEADER [0]=HEX B5 DEC *181‬");
  Serial.println("HEADER [1]=HEX 62 DEC ** 98‬‬");

  uint8_t MSGpayload = (_gpsPayload[3] << 8) | _gpsPayload[2]; // Combining two uint8_t as uint16_t
  for (int i = 0; i < (MSGpayload) + 4; i++) {
    Serial.print("_gpsPayload["), Serial.print(i, DEC), Serial.print("]=");
    if (_gpsPayload[i] < 16) {
      Serial.print("HEX 0");
      Serial.print(_gpsPayload[i], HEX);
    } else {
      Serial.print("HEX ");
      Serial.print(_gpsPayload[i], HEX);
    }
    if (_gpsPayload[i] < 10) {
      Serial.print(" DEC 0");
      Serial.println(_gpsPayload[i], DEC);
    } else {
      Serial.print(" DEC ");
      Serial.println(_gpsPayload[i], DEC);
    }
  }
}
#endif