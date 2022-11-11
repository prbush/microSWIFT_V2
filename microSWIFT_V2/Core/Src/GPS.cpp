/*
 * GPS.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 *
 *`TODO: Make the arrays!!
 *`TODO: Add values to the arrays in processMessage
 *`TODO: Figure out set rtc and turn it into a function
 *`TODO: figure out signaling for CT sensor
 *`TODO: Power pin toggling in init
 *`TODO: Create teardown function
 */

#include <gps.h>

GPS::GPS(UART_HandleTypeDef* uart_handle) {
	// configure, init, toggle power pin, get a few messages, flush the buffer
	gps_uart_handle = uart_handle;
}

GPS::~GPS(void) {
	/* TODO:Power down
	        free any allocated memory
	        Set flags for deep sleep?
	*/
}

int32_t GPS::init(void)
{
	/* TODO:Turn on power pin
	 *      Setup uart?
	 *      Create arrays
	 *      wait until good fix?
	 */
	return false;
}

/**
 * Pull from the UART message buffer and try to process to UBX_NAV_PVT message.
 *
 * @return GPS error code (marcos defined in gps.h)
 */
int32_t GPS::getUBX_NAV_PVT(void){
	// Message + overhead bits = 97 bytes. We'll make enough space for 5 msgs
	uint8_t UART_receive_buf[500];
	memset(UART_receive_buf, 0, sizeof(UART_receive_buf));

    HAL_StatusTypeDef ret = HAL_UART_Receive(gps_uart_handle,
    		&UART_receive_buf[0], 250, 200);
    if (ret != HAL_OK) {
    	// Something went wrong trying to pull from UART buffer
    	++numberCyclesWithoutData;
    	switch(ret){
			case HAL_BUSY: {return GPS_BUSY_ERROR;}
			case HAL_TIMEOUT: {return GPS_TIMEOUT_ERROR;}
			default: {return GPS_UNKNOWN_ERROR;}
    	}
    }
	// Start with a fresh buffer all zero'd out
    memset(UBX_NAV_PVT_message_buf, 0, sizeof(UBX_NAV_PVT_message_buf));
    validMessageProcessed = false;
	// Some fields to keep track of
    int32_t messageClass = 0;
    int32_t messageId = 0;
    const char* UART_buffer_start = (const char*)&(UART_receive_buf[0]);
    const char* UART_buffer_end = UART_buffer_start;
    size_t bufferLength = sizeof(UART_receive_buf);

    // Really hideous for loop
    for (int32_t numBytes = uUbxProtocolDecode(UART_buffer_start,
    			bufferLength, &messageClass, &messageId,
				&(UBX_NAV_PVT_message_buf[0]), sizeof(UBX_NAV_PVT_message_buf),
				&UART_buffer_end);
    	numBytes > 0;
    	numBytes = uUbxProtocolDecode(UART_buffer_start, bufferLength,
				&messageClass, &messageId, &(UBX_NAV_PVT_message_buf[0]),
				sizeof(UBX_NAV_PVT_message_buf), &UART_buffer_end))
    { // start for loop

		if (messageClass != UBX_NAV_PVT_MESSAGE_CLASS ||
				messageId != UBX_NAV_PVT_MESSAGE_ID) {
			// This will get our pointers pointing to the next message in the buf, and the for loop
			// will try processing the next message (if there is one)
			bufferLength -= UART_buffer_end - UART_buffer_start;
			UART_buffer_start = UART_buffer_end;
		}

		if (numBytes == 92) { // Good to go, process message
			// Start by checking time accuracy and set the clock if needed
			// tAcc = time accuracy estimate
			int32_t tAcc = UBX_NAV_PVT_message_buf[12] +
					(UBX_NAV_PVT_message_buf[13]<<8) +
					(UBX_NAV_PVT_message_buf[14]<<16) +
					(UBX_NAV_PVT_message_buf[15]<<24);

			if (tAcc < MAX_ACCEPTABLE_VACC && !clockHasBeenSet) {
				// !!!!! Figure this out and then make it a function
				// set clock
				uint16_t year = UBX_NAV_PVT_message_buf[4];
				uint8_t month = UBX_NAV_PVT_message_buf[6];
				uint8_t day = UBX_NAV_PVT_message_buf[7];
				uint8_t hour = UBX_NAV_PVT_message_buf[8];
				bool isAM = true;
				if (hour > 12) { hour %= 12; isAM = false;}
				uint8_t min = UBX_NAV_PVT_message_buf[9];
				uint8_t sec = UBX_NAV_PVT_message_buf[10];

				RTC_TimeTypeDef time = {hour, min, sec,
						(isAM) ?
						RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM, 0, 0};
			}

			// Check Lat/Long accuracy, assign to class fields if good
			int32_t lon = UBX_NAV_PVT_message_buf[24] +
					(UBX_NAV_PVT_message_buf[25]<<8) +
					(UBX_NAV_PVT_message_buf[26]<<16) +
					(UBX_NAV_PVT_message_buf[27]<<24);
			int32_t lat = UBX_NAV_PVT_message_buf[28] +
					(UBX_NAV_PVT_message_buf[29]<<8) +
					(UBX_NAV_PVT_message_buf[30]<<16) +
					(UBX_NAV_PVT_message_buf[31]<<24);
			int16_t pDOP =  UBX_NAV_PVT_message_buf[76] +
					(UBX_NAV_PVT_message_buf[77]<<8);
			if (pDOP < MAX_ACCEPTABLE_PDOP) {
				currentLatitude = lat;
				currentLongitude = lon;
			}

			// Grab velocities
			int32_t vAcc = UBX_NAV_PVT_message_buf[44] +
					(UBX_NAV_PVT_message_buf[45] << 8) +
					(UBX_NAV_PVT_message_buf[46] << 16) +
					(UBX_NAV_PVT_message_buf[47] << 24);
			if (vAcc > MAX_ACCEPTABLE_VACC) {
				// This message was not within acceptable parameters,
				// but there's probably more in the buffer we can try
				bufferLength -= UART_buffer_end - UART_buffer_start;
				UART_buffer_start = UART_buffer_end;
			} else {
				// We're good to go, vAcc was within acceptable range
				// !!!!! Add values to the arrays
				int32_t vnorth = UBX_NAV_PVT_message_buf[48] +
						(UBX_NAV_PVT_message_buf[49] << 8) +
						(UBX_NAV_PVT_message_buf[50] << 16) +
						(UBX_NAV_PVT_message_buf[51] << 24);
				int32_t veast = UBX_NAV_PVT_message_buf[52] +
						(UBX_NAV_PVT_message_buf[53] << 8) +
						(UBX_NAV_PVT_message_buf[54] << 16) +
						(UBX_NAV_PVT_message_buf[55] << 24);
				int32_t vdown = UBX_NAV_PVT_message_buf[56] +
						(UBX_NAV_PVT_message_buf[57] << 8) +
						(UBX_NAV_PVT_message_buf[58] << 16) +
						(UBX_NAV_PVT_message_buf[59] << 24);

				// add these values to the arrays
				// increment whatever counters, trackers
				numberCyclesWithoutData = 0;
				validMessageProcessed = true;
			}
		} else {
			// UBX_NAV_PVT message is 92 bytes -- if we didn't get all of them, then no valid message was received
			bufferLength -= UART_buffer_end - UART_buffer_start;
			UART_buffer_start = UART_buffer_end;
		}
    }

    if (!validMessageProcessed) {
    	if (++numberCyclesWithoutData > MAX_EMPTY_CYCLES) {
    		// Some sort of bail out condition
    		// We might send a message with a specific payload to indicate
    		// things went wrong.
    	} else {
    		// We'll replace the values with running average
    		float north = 0, east = 0, down = 0;
    		// make sure there are samples to average
    		if (int32_t ret = getRunningAverage(north, east, down)
    				!= GPS_SUCCESS) {
    			return ret;
    		} else {
    			// got the averaged values
    			// !!!! add them to the arrays, increment some flags
    		}
    	}
		return GPS_NO_MESSAGE_RECEIVED;
    }
    return GPS_SUCCESS;
}

/**
 * Get the current lat/long.
 *
 * @param latitude - return parameter for latitude
 * @param longitude - return parameter for longitude
 * @return GPS error code (marcos defined in gps.h)
 */
int32_t GPS::getLocation(int32_t& latitude, int32_t& longitude) {
	if (latLongIsValid) {
		latitude = currentLatitude;
		longitude = currentLongitude;
		return GPS_SUCCESS;
	} else {
		return GPS_LOCATION_INVALID;
	}
}

int32_t GPS::processMessage(void) {

}

/**
 * If a velocity field > MAX_POSSIBLE_VELOCITY, or the velocity accuracy estimate (vAcc) is
 * outside acceptable range, this function will substitute a running average.
 *
 * @param returnNorth - return parameter for the running average North value
 * @param returnEast - return parameter for the running average East value
 * @param returnDown - return parameter for the running average Down value
 * @return GPS error code (marcos defined in gps.h)
 */
int32_t GPS::getRunningAverage(float& returnNorth, float& returnEast, float& returnDown) {
	if (totalSamples > 0) {
		float substituteNorth = vNorthSum / static_cast<float>(totalSamples);
		returnNorth = substituteNorth;
		vNorthSum += substituteNorth;

		float substituteEast = vEastSum / static_cast<float>(totalSamples);
		returnEast = substituteEast;
		vEastSum += substituteEast;

		float substituteDown = vDownSum / static_cast<float>(totalSamples);
		returnDown = substituteDown;
		vDownSum += substituteDown;

		++totalSamples;
		++totalSamplesAveraged;
		return GPS_SUCCESS;
	} else {
		// No valid samples yet, avoid divide by zero error
		return GPS_NO_SAMPLES_ERROR;
	}
}

void GPS::testFunct(void)
{
	    char messagebuf[192];
	    char UBX_NAV_PVT_message_buf[92];
	    HAL_UART_Receive(gps_uart_handle,(uint8_t*) messagebuf, 192, 1000);

	    int32_t ret = uUbxProtocolDecode(messagebuf, 192, nullptr, nullptr, UBX_NAV_PVT_message_buf, 92, nullptr);

	    if (ret == 92) {
		    int32_t lon = messagebuf[24] + (messagebuf[25]<<8) + (messagebuf[26]<<16) + (messagebuf[27]<<24);
		    int32_t lat = messagebuf[28] + (messagebuf[29]<<8) + (messagebuf[30]<<16) + (messagebuf[31]<<24);
		    int32_t vnorth = messagebuf[48] + (messagebuf[49] << 8) + (messagebuf[50] << 16) + (messagebuf[51] << 24);
		    int32_t veast = messagebuf[52] + (messagebuf[53] << 8) + (messagebuf[54] << 16) + (messagebuf[55] << 24);
		    int32_t vdown = messagebuf[56] + (messagebuf[57] << 8) + (messagebuf[58] << 16) + (messagebuf[59] << 24);

		    char txstr[150];
		    snprintf(txstr, 150, "Latitude: %ld\r\n Longitude: %ld\r\n Velocity North: %ld\r\n Velocity East: %ld\r\n Velocity Down: %ld\r\n", lon, lat, vnorth, veast, vdown);
		    HAL_UART_Transmit(gps_uart_handle,(uint8_t*) txstr, sizeof(txstr), 10);
	    }
}
