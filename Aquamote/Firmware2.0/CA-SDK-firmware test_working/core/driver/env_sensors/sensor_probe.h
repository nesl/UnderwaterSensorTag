/*
 * sensor_probe.h
 *
 *  Created on: Nov 28, 2012
 *      Author: kkatingari
 */

#ifndef SENSOR_PROBE_H_
#define SENSOR_PROBE_H_

#define wai_9250 0x71
#define wai_8963 0x48
#define wai_36682 0x83
#define wai_sht21 0x3A
#define wai_hspad32 0x40
#define wai_hspad32_1 0x44


typedef struct {
	unsigned char address;
	unsigned char reg_option;
	unsigned char reg_value;
	unsigned char length;
	unsigned char *data;
} ic_struct;



int probe_sht21_th_chip(void);
int probe_alpsHS_p_chip(void);
int probe_cm3512_uv_chip(void);
int probe_cm3668_lp_chip(void);
int get_pressure_data(unsigned char *pressure);
int get_ht_data(unsigned char *humidity, unsigned char *temperature);
int get_light_data(unsigned char *light);
int get_uv_data(unsigned char *uv);
int write_data(ic_struct * ic_str);
int read_data(ic_struct *ic_str);
int read_data_uv(ic_struct *ic_str);

#endif /* SENSOR_PROBE_H_ */
