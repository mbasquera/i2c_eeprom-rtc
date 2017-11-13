#include <I2C.h>
#include <msp430.h>

unsigned char read_rtc[7];
unsigned char read_eeprom[32];
unsigned char write_val[32];

unsigned char seconds, minutes, hour, day, month, year;

unsigned char timestamp[6];
unsigned char i;

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop watchdog timer

    InitI2C();                // Initialize I2C module

    I2C_ByteWrite(0x0000,0x00,1); //seconds
    I2C_ByteWrite(0x0001,0x42,1); //minutes
    I2C_ByteWrite(0x0002,0x11,1); //hours
    I2C_ByteWrite(0x0003,0x02,1); //day
    I2C_ByteWrite(0x0004,0x01,1); //date
    I2C_ByteWrite(0x0005,0x01,1); //month
    I2C_ByteWrite(0x0006,0x17,1); //year

    __delay_cycles(5000);

    for(i = 0 ; i <= sizeof(write_val) ; i++)
    {
      write_val[i] = i;
    }

    I2C_PageWrite(0x0000, write_val , sizeof(write_val), 2);

    __delay_cycles(5000);

    while(1)
    {
        I2C_SequentialRead(0x0000, read_eeprom, sizeof(read_eeprom), 2);
        I2C_SequentialRead(0x0000, read_rtc, sizeof(read_rtc), 1);

        seconds = (read_rtc[0] & 0x0F) + 10*((read_rtc[0] >> 4) & 0x0F);
        minutes = (read_rtc[1] & 0x0F) + 10*((read_rtc[1] >> 4) & 0x0F);
        hour = (read_rtc[2] & 0x0F) + 10*((read_rtc[2] >> 4) & 0x0F);
        day = (read_rtc[4] & 0x0F) + 10*((read_rtc[4] >> 4) & 0x0F);
        month = (read_rtc[5] & 0x0F) + 10*((read_rtc[5] >> 4) & 0x0F);
        year = (read_rtc[6] & 0x0F) + 10*((read_rtc[6]>> 4) & 0x0F);

        timestamp[0]=hour;
        timestamp[1]=minutes;
        timestamp[2]=seconds;
        timestamp[3]=day;
        timestamp[4]=month;
        timestamp[5]=year;

        __delay_cycles(1000);
    }

}
