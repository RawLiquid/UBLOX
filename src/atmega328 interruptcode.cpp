// Pin change interrupt D8-D13.
ISR (PCINT0_vect) // D8-D13 = PCINT 0-5 = PCIR0 = PB = PCIE0 = pcmsk0
{
  // -------------------SPI-------------------------------------------------------------------------
  /* SS (Slave Select (D10) ), This tells the slave that it should wake up and receive / send data
     and is also used when multiple slaves are present to select the one you’d like to talk to.
     The SS line is normally held high, which disconnects the slave from the SPI bus.
     This type of logic is known as “active low,” and you’ll often see used it for enable and reset lines.
     Just before data is sent to the slave, the line is brought low, which activates the slave.
     When you’re done using the slave, the line is made high again.

     Important point is that we need to add a slight delay on master MCU (something like 30 microseconds).
     Otherwise the slave doesn't have a chance to react to the incoming data and do something with it.
  */

  if (PINB & (1 << PB2)) { //PB2 // true if the pin in question is high  /* Slave Disabled */
    digitalWrite_LOW(PORTD, Debug_led); // debug led on pin 6
  }

  if (LastWrite == 1) {
    ccd_ptr = ccd_ptr_write; // copy pointer
    for (int i = 0; i < ccd_ptr; i ++) {
      ccd_buff[i] = ccd_buff_write[i]; // copy to ccd buff from ccd_buff_write /* SPI SHOW */
    }
    LastWrite = 0;
  }

  //if (digitalRead(10) == LOW) { // Pin 10 <-> SS Pin (Slave Select)
  if (~PINB & (1 << PB2)) { // true when PB2 is low / Pin 10 <-> SS Pin (Slave Select)
    /* Slave Enabled */
    Set_OUTPUT(DDRB, PB4); // Set MISO to OUTPUT --> TEST
    //digitalWrite(Debug_led, HIGH);
    digitalWrite_HIGH(PORTD, Debug_led);

    uint8_t ACK = TransmitStatus;

    // 1--------
    asm volatile("nop");
    while (!(SPSR & (1 << SPIF))); // wait
    TimeToWrite = SPDR; // 1ST READ / receive from the SPI bus, read from master
    SPDR = ACK; // send back ACK: TransmitStatus
    if (TimeToWrite == 0) {
      TransmitStatus = 0; // 0 = was not transmiting
    }

    // 2--------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Len = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_ptr; // send back ccd_ptr

    // 3-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[0] = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_buff[0]; // send back ccd_buff[0]

    // 4-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[1] = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_buff[1]; // send back ccd_buff[1]

    // 5-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[2] = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_buff[2]; // send back ccd_buff[2]

    // 6-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[3] = SPDR; //  receive from the SPI bus, read from master
    SPDR = ccd_buff[3]; // send back ccd_buff[3]

    // 7-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[4] = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_buff[4]; // send back ccd_buff[4]

    // 8-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[5] = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_buff[5]; // send back ccd_buff[5]

    // 9-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[6] = SPDR; //  receive from the SPI bus, read from master
    SPDR = ccd_buff[6]; // send back ccd_buff[6]

    // 10-------
    asm volatile("nop");
    while (!(SPSR & _BV(SPIF))) ; // wait
    CCD_Transmit_Buff[7] = SPDR; // receive from the SPI bus, read from master
    SPDR = ccd_buff[7]; // send back ccd_buff[7]

    Set_INPUT(DDRB, PB4); // Set MISO to INPUT ( 3-state logic )
  }
}