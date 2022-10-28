/*
 * GPS.h
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 */

#ifndef SRC_GPS_H_
#define SRC_GPS_H_
#include <stdint.h>

class GPS {
public:
	GPS();
	virtual ~GPS(UART_HandleTypeDef *uart_handle);
	bool init(void);

	bool sleep(void);

private:
	UART_HandleTypeDef uart_handle;
	const char* search = "$GPGGA";
	bool checksum(uint8_t CK_A, uint8_t CK_B, char* bytes, uint32_t bufferLength);
};

#endif /* SRC_GPS_H_ */
