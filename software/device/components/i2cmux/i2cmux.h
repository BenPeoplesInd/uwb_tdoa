#ifndef _I2C_MUX_H_
#define _I2C_MUX_H_

// 1110 000_  <-- address
// 0000 3210 <-- channel selection

#ifndef WRITE_BIT
#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */
#endif

bool i2cmux_init();
bool i2cmux_select(uint8_t which);


#endif
