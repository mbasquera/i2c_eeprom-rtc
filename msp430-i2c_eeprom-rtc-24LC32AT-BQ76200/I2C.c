#include <I2C.h>
#include "msp430.h"
    
    unsigned char FlowFlag = 1;

    #define     MAXPAGEWRITE   32

    int PtrTransmit;
    unsigned char I2CBufferArray[128];
    unsigned char I2CBuffer;

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Initialization of the I2C Module
    /*----------------------------------------------------------------------------*/
    void InitI2C()
    {
      I2C_PORT_DIR |= SDA_PIN + SCL_PIN;
      I2C_PORT_SEL |= SDA_PIN + SCL_PIN;        // Assign I2C pins to USCI_B0
      I2C_PORT_OUT |= SDA_PIN + SCL_PIN;        // Assign I2C pins to USCI_B0

      // Recommended initialization steps of I2C module as shown in User Guide:

      UCB2CTL1 |= UCSWRST;                      // Enable SW reset
      UCB2CTL0 = UCMST + UCMODE_3 + UCSYNC;     // I2C Master, synchronous mode
      UCB2CTL1 = UCSSEL_2 + UCTR + UCSWRST;     // Use SMCLK, TX mode, keep SW reset
      UCB2BR0 = SCL_CLOCK_DIV;                  // fSCL = SMCLK/12 = ~100kHz
      UCB2BR1 = 0;

      UCB2IE |= UCTXIE + UCRXIE + UCNACKIE;
      UCB2CTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation

      if (UCB2STAT & UCBBUSY)                   // test if bus to be free
      {                                         // otherwise a manual Clock on is
                                                // generated
        I2C_PORT_SEL &= ~SCL_PIN;               // Select Port function for SCL
        I2C_PORT_OUT &= ~SCL_PIN;               //
        I2C_PORT_DIR |= SCL_PIN;                // drive SCL low
        I2C_PORT_SEL |= SDA_PIN + SCL_PIN;      // select module function for the
                                                // used I2C pins
      };

      __enable_interrupt();
    }

    /*---------------------------------------------------------------------------*/
    // Description:
    //   Initialization of the I2C Module for Write operation.
    /*---------------------------------------------------------------------------*/
    void I2CWriteInit(void)
    {
      UCB2CTL1 |= UCTR;                         // UCTR=1 => Transmit Mode (R/W bit = 0)
      UCB2IFG &= ~UCTXIFG;
      UCB2IE &= ~UCRXIE;                        // disable Receive ready interrupt
      UCB2IE|= UCTXIE;                          // enable Transmit ready interrupt
    }

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Initialization of the I2C Module for Read operation.
    /*----------------------------------------------------------------------------*/
    void I2CReadInit(void)
    {
      UCB2CTL1 &= ~UCTR;                        // UCTR=0 => Receive Mode (R/W bit = 1)
      UCB2IFG &= ~UCRXIFG;
      UCB2IE &= ~UCTXIE;                         // disable Transmit ready interrupt
      UCB2IE |= UCRXIE;                          // enable Receive ready interrupt
    }

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Page Write Operation. The communication via the I2C bus with an I2C
    //   (24xx65) is realized. A data byte is written into a user defined address.
    /*----------------------------------------------------------------------------*/
    void I2C_PageWrite(unsigned int StartAddress, unsigned char * Data, unsigned char Size, unsigned char id)
    {
      volatile unsigned int i = 0;
      volatile unsigned char counterI2cBuffer;
      unsigned char adr_hi;
      unsigned char adr_lo;
      unsigned int  currentAddress = StartAddress;
      unsigned char currentSize = Size;
      unsigned char bufferPtr = 0;
      unsigned char moreDataToRead = 1;

      if(id==1){
          UCB2I2CSA = RTC_ADDRESS;
      }
      else{
          UCB2I2CSA = EEPROM_ADDRESS;
      }

      while (UCB2STAT & UCBUSY);                // wait until I2C module has
                                                // finished all operations.

      // Execute until no more data in Data buffer
      while(moreDataToRead)
      {
        adr_hi = currentAddress >> 8;           // calculate high byte
        adr_lo = currentAddress & 0xFF;         // and low byte of address

        // Chop data down to 64-byte packets to be transmitted at a time
        // Maintain pointer of current startaddress
        if(currentSize > MAXPAGEWRITE)
        {
          bufferPtr = bufferPtr + MAXPAGEWRITE;
          counterI2cBuffer = MAXPAGEWRITE - 1;
          PtrTransmit = MAXPAGEWRITE + 1;       // set I2CBufferArray Pointer
          currentSize = currentSize - MAXPAGEWRITE;
          currentAddress = currentAddress + MAXPAGEWRITE;

          // Get start address
          I2CBufferArray[MAXPAGEWRITE + 1] = adr_hi; // High byte address.
          I2CBufferArray[MAXPAGEWRITE] = adr_lo; // Low byte address.
        }
        else
        {
          bufferPtr = bufferPtr + currentSize;
          counterI2cBuffer = currentSize - 1;
          PtrTransmit = currentSize + 1;        // set I2CBufferArray Pointer.
          moreDataToRead = 0;
          currentAddress = currentAddress + currentSize;

          // Get start address
          I2CBufferArray[currentSize + 1] = adr_hi; // High byte address.
          I2CBufferArray[currentSize] = adr_lo; // Low byte address.
        }

        // Copy data to I2CBufferArray
        unsigned char temp;
        for(i ; i < bufferPtr ; i++)
        {
          temp = Data[i];                       // Required or else IAR throws a
                                                // warning [Pa082]
          I2CBufferArray[counterI2cBuffer] = temp;
          counterI2cBuffer--;
        }

        I2CWriteInit();
        UCB2CTL1 |= UCTXSTT;                    // start condition generation
                                                // => I2C communication is started

        while(FlowFlag);
        FlowFlag = 1;

        while(UCB2CTL1 & UCTXSTP);              // Ensure stop condition got sent

        I2C_AckPolling();                    // Ensure data is written in I2C
      }
    }

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Current Address Read Operation. Data is read from the I2C. The current
    //   address from the I2C is used.
    /*----------------------------------------------------------------------------*/
    unsigned char I2C_CurrentAddressRead(void)
    {
      while(UCB2STAT & UCBUSY);                 // wait until I2C module has
                                                // finished all operations
      I2CReadInit();

      UCB2CTL1 |= UCTXSTT;                      // I2C start condition
      while(UCB2CTL1 & UCTXSTT);                // Start condition sent?
      UCB2CTL1 |= UCTXSTP;                      // I2C stop condition
      while(UCB2CTL1 & UCTXSTP);                // Ensure stop condition got sent
      return I2CBuffer;
    }

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Sequential Read Operation. Data is read from the I2C in a sequential
    //   form from the parameter address as a starting point. Specify the size to
    //   be read and populate to a Data buffer.
    /*----------------------------------------------------------------------------*/
void I2C_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size, unsigned int id)
{
      unsigned char adr_hi;
      unsigned char adr_lo;
      unsigned int counterSize;

      while (UCB2STAT & UCBUSY);              // wait until I2C module has
                                                // finished all operations

      adr_hi = Address >> 8;                    // calculate high byte
      adr_lo = Address & 0xFF;                  // and low byte of address

      I2CBufferArray[1] = adr_hi;               // store single bytes that have to
      I2CBufferArray[0] = adr_lo;               // be sent in the I2CBuffer.

      if(id==1){
      PtrTransmit = 0;                          // set I2CBufferArray Pointer
      UCB2I2CSA = RTC_ADDRESS;
      }
      else{
          PtrTransmit = 1;
          UCB2I2CSA = EEPROM_ADDRESS;
      }

      // Write Address first
      I2CWriteInit();
      UCB2CTL1 |= UCTXSTT;                      // start condition generation
                                                // => I2C communication is started

      while(FlowFlag);
      FlowFlag = 1;

      while(UCB2CTL1 & UCTXSTP);                // Ensure stop condition got sent

      // Read Data byte
      I2CReadInit();

      UCB2CTL1 |= UCTXSTT;                      // I2C start condition

      while(UCB2CTL1 & UCTXSTT);                // Start condition sent?

      for(counterSize = 0 ; counterSize < Size ; counterSize++)
      {
          while(FlowFlag);
          FlowFlag = 1;
        Data[counterSize] = I2CBuffer;
      }

      UCB2CTL1 |= UCTXSTP;                      // I2C stop condition

      while(FlowFlag);
      FlowFlag = 1;

      while(UCB2CTL1 & UCTXSTP);                // Ensure stop condition got sent
}

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Acknowledge Polling. The I2C will not acknowledge if a write cycle is
    //   in progress. It can be used to determine when a write cycle is completed.
    /*----------------------------------------------------------------------------*/
    void I2C_AckPolling(void)
    {
      while (UCB2STAT & UCBUSY);                // wait until I2C module has
                                                // finished all operations
      while(UCNACKIFG & UCB2IFG)
      {
        UCB2STAT = 0x00;                        // clear I2C interrupt flags
        UCB2CTL1 |= UCTR;                       // I2CTRX=1 => Transmit Mode (R/W bit = 0)
        UCB2CTL1 &= ~UCTXSTT;
        UCB2CTL1 |= UCTXSTT;                    // start condition is generated
        while(UCB2CTL1 & UCTXSTT)               // wait till I2CSTT bit was cleared
        {
          if(!(UCNACKIFG & UCB2IFG))           // Break out if ACK received
            break;
        }
        UCB2CTL1 |= UCTXSTP;                    // stop condition is generated after
                                                // slave address was sent => I2C communication is started
        while (UCB2CTL1 & UCTXSTP);             // wait till stop bit is reset

        __delay_cycles(500);                   // Software delay
      };
    }


    /*----------------------------------------------------------------------------*/
    // Description:
    //   Byte Write Operation. The communication via the I2C bus with an I2C
    //   (2465) is realized. A data byte is written into a user defined address.
    /*----------------------------------------------------------------------------*/
    void I2C_ByteWrite(unsigned int Address, unsigned char Data, unsigned int id)
    {

      unsigned char adr_hi;
      unsigned char adr_lo;

      while (UCB2STAT & UCBUSY);                // wait until I2C module has
                                                // finished all operations.

      adr_hi = Address >> 8;                    // calculate high byte
      adr_lo = Address & 0xFF;                  // and low byte of address

      I2CBufferArray[2] = adr_hi;               // Low byte address.
      I2CBufferArray[1] = adr_lo;               // High byte address.
      I2CBufferArray[0] = Data;

      if(id==1){
                 PtrTransmit = 1;                          // set I2CBufferArray Pointer
                 UCB2I2CSA = RTC_ADDRESS;
                 }
      else {
                 PtrTransmit = 2;
                 UCB2I2CSA = EEPROM_ADDRESS;
           }

      I2CWriteInit();
      UCB2CTL1 |= UCTXSTT;                      // start condition generation
                                                // => I2C communication is started

      while(FlowFlag);
      FlowFlag = 1;

      while(UCB2CTL1 & UCTXSTP);                // Ensure stop condition got sent

    }

    /*----------------------------------------------------------------------------*/
    // Description:
    //   Random Read Operation. Data is read from the I2C. The I2C
    //   address is defined with the parameter Address.
    /*----------------------------------------------------------------------------*/
    unsigned char I2C_RandomRead(unsigned int Address, unsigned int id)
    {
      unsigned char adr_hi;
      unsigned char adr_lo;

      while (UCB2STAT & UCBUSY);                // wait until I2C module has
                                                // finished all operations

      adr_hi = Address >> 8;                    // calculate high byte
      adr_lo = Address & 0xFF;                  // and low byte of address

      I2CBufferArray[1] = adr_hi;               // store single bytes that have to
      I2CBufferArray[0] = adr_lo;               // be sent in the I2CBuffer.

      if(id==1){
           PtrTransmit = 0;                          // set I2CBufferArray Pointer
           UCB2I2CSA = RTC_ADDRESS;
           }
           else{
               PtrTransmit = 1;
               UCB2I2CSA = EEPROM_ADDRESS;
      }

      // Write Address first
      I2CWriteInit();
      UCB2CTL1 |= UCTXSTT;                      // start condition generation
                                                // => I2C communication is started

      while(FlowFlag);
      FlowFlag=1;

      while(UCB2CTL1 & UCTXSTP);                // Ensure stop condition got sent

      // Read Data byte
      I2CReadInit();

      UCB2CTL1 |= UCTXSTT;                      // I2C start condition

      while(UCB2CTL1 & UCTXSTT);                // Start condition sent?
      UCB2CTL1 |= UCTXSTP;                      // I2C stop condition

      while(FlowFlag);
      FlowFlag=1;

      while(UCB2CTL1 & UCTXSTP);                // Ensure stop condition got sent
      return I2CBuffer;
    }


#pragma vector = USCI_B2_VECTOR
__interrupt void USCI_B2_ISR(void)
{
  switch(__even_in_range(UCB2IV,12))
  {
  case  0: break;                           // Vector  0: No interrupts
  case  2: break;                           // Vector  2: ALIFG
  case  4: break;                           // Vector  4: NACKIFG
  case  6: break;
  case  8: break;
  case 10:                                  // Vector 10: RXIFG

      I2CBuffer = UCB2RXBUF;                  // store received data in buffer
      FlowFlag = 0;

    break;

  case 12:                                  // Vector 12: TXIFG

      UCB2TXBUF = I2CBufferArray[PtrTransmit];// Load TX buffer
      PtrTransmit--;                          // Decrement TX byte counter

      if(PtrTransmit < 0)
      {
        while(!(UCB2IFG & UCTXIFG));
        UCB2CTL1 |= UCTXSTP;                  // I2C stop condition
        UCB2IE &= ~UCTXIE;                     // disable interrupts.
        UCB2IFG &= ~UCTXIFG;                   // Clear USCI_B0 TX int flag
        FlowFlag = 0;
      }

    break;

  default:break;
  }
}
