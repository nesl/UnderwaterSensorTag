/*
 * max1708_driver.h
 *
 *  Created on: Apr 19, 2013
 *      Author: Jibran Ahmed
 */

#ifndef MAX1708_DRIVER_H_
#define MAX1708_DRIVER_H_

int max17048_probe(void);
int max17048_get_battery_voltage(unsigned int* result);
int max17048_get_battery_soc(unsigned int* result);
int max17048_get_battery_health(unsigned int* result);
int max17048_get_battery_capacity(unsigned int* result);
int max17048_get_version(unsigned int* result);

#endif /* MAX1708_DRIVER_H_ */
