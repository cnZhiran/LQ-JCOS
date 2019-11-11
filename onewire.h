#ifndef __ONEWIRE_H
#define __ONEWIRE_H

unsigned char rd_temperature(void);  //
void Write_DS18B20(unsigned char dat);
unsigned char Read_DS18B20(void);
bit init_ds18b20(void);

#endif
