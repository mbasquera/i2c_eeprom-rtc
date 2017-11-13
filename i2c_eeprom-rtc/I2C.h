#define I2C_PORT_SEL  P9SEL
#define I2C_PORT_OUT  P9OUT
#define I2C_PORT_REN  P9REN
#define I2C_PORT_DIR  P9DIR
#define SDA_PIN       BIT1                 // UCB0SDA pin
#define SCL_PIN       BIT2                 // UCB0SCL pin
#define SCL_CLOCK_DIV  0x0A                // SCL clock divider

#define EEPROM_ADDRESS 0x50
#define RTC_ADDRESS    0x68

void InitI2C();
void I2C_ByteWrite(unsigned int Address , unsigned char Data, unsigned int id);
void I2C_PageWrite(unsigned int StartAddress , unsigned char * Data , unsigned char Size, unsigned char id);
unsigned char I2C_RandomRead(unsigned int Address, unsigned int id);
unsigned char I2C_CurrentAddressRead(void);
void I2C_SequentialRead(unsigned int Address , unsigned char * Data , unsigned int Size, unsigned int id);
void I2C_AckPolling(void);
