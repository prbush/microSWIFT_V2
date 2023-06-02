/*
 * byte_array.c
 *
 *  Created on: Apr 3, 2023
 *      Author: Phil
 */

#include "byte_array.h"

/**
 * Function that returns two bytes as an uint16_t from a byte array
 *
 * @param byte_array - pointer to byte array
 * @param start_index - the start index of the bytes to extract
 *
 * @return The two bytes as a uint16_t -- can be cast to different formats
 */
uint16_t get_two_bytes(uint8_t* byte_array, uint32_t start_index, endian_t endianess)
{
	uint16_t return_val = 0;

	if (endianess == AS_BIG_ENDIAN) {
		return_val =  ((byte_array[start_index] << 8) +
				(byte_array[start_index + 1]));
	} else {
		return_val =  (byte_array[start_index] +
				(byte_array[start_index + 1] << 8));
	}

	return return_val;
}

/**
 * Function that returns two bytes as an uint16_t from a byte array
 *
 * @param byte_array - pointer to byte array
 * @param start_index - the start index of the bytes to extract
 * @param endianess - either LITTLE_ENDIAN or BIG_ENDIAN
 *
 * @return The four bytes as a uint16_t -- can be cast to different formats
 */
uint32_t get_four_bytes(uint8_t* byte_array, uint32_t start_index, endian_t endianess)
{
	uint32_t return_val = 0;

	if (endianess == AS_BIG_ENDIAN) {
		return_val = ((byte_array[start_index] << 24) +
				(byte_array[start_index + 1] << 16) +
				(byte_array[start_index + 2] << 8) +
				byte_array[start_index + 3]);
	} else {
		return_val = (byte_array[start_index] +
				(byte_array[start_index + 1] << 8) +
				(byte_array[start_index + 2] << 16) +
				(byte_array[start_index + 3] << 24));
	}

	return return_val;
}

