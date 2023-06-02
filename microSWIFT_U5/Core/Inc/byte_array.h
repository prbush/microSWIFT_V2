/*
 * byte_array.h
 *
 *  Created on: Apr 3, 2023
 *      Author: Phil
 */

#ifndef INC_BYTE_ARRAY_H_
#define INC_BYTE_ARRAY_H_

#include "stdint.h"

typedef enum {
	AS_LITTLE_ENDIAN = 0,
	AS_BIG_ENDIAN = 1
} endian_t;

uint16_t get_two_bytes(uint8_t* byte_array, uint32_t start_index, endian_t endianess);
uint32_t get_four_bytes(uint8_t* byte_array, uint32_t start_index, endian_t endianess);

#endif /* INC_BYTE_ARRAY_H_ */
