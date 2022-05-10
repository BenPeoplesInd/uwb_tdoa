#ifndef _DW1000_H_
#define _DW1000_H_

/*
These are the actual meat and potatoes functions.

reg = 6 bit register to access
has_subreg = first byte or two of data is the subreg data, if first byte & 0x80, then two bytes

data, is data.

len is length of data.

*/

void dw_write_reg_real(uint8_t reg, bool has_subreg, uint8_t * data, size_t len);
void dw_read_reg_real(uint8_t reg, bool has_subreg, uint8_t * data, size_t len);

/*
These are the functions that should actually be called.
*/

void dw_write_reg(uint8_t reg, uint8_t * data, size_t len);
void dw_read_reg(uint8_t reg, uint8_t * data, size_t len);

void dw_write_subreg(uint8_t reg, uint8_t subreg, uint8_t * data, size_t len);
void dw_read_subreg(uint8_t reg, uint8_t subreg, uint8_t * data, size_t len);

void dw_write_subsubreg(uint8_t reg, uint16_t subreg, uint8_t * data, size_t len);
void dw_read_subsubreg(uint8_t reg, uint16_t subreg, uint8_t * data, size_t len);

// Initialize DW.
void dw_init();


void tx_packet();
void setup_rx();
void rx_task(void *pvParameter);

#endif
