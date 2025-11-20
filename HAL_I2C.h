#ifndef HAL_DHT11_H_
#define HAL_DHT11_H_

#include <stdbool.h>
#include <stdint.h>

// Variables globales accesibles desde main
extern uint8_t g_dht_humi;
extern uint8_t g_dht_temp;

// Prototipo
bool DHT11_Read_Data(void);

#endif
