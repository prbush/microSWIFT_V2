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



#include "gps.h"


void gnss_init(GNSS* self, UART_HandleTypeDef* gnss_uart_handle, uint8_t* uart_buffer,
		int16_t* uGNSSArray, int16_t* vGNSSArray, int16_t* zGNSSArray)
{
	/* TODO:Turn on power pin
	 *      Setup uart?
	 *      Create arrays
	 *      wait until good fix?
	 */
	// initialize everything to 0/false
	self->gnss_uart_handle = gnss_uart_handle;
	self->uart_buffer = uart_buffer;
	self->uGNSSArray = uGNSSArray;
	self->vGNSSArray = vGNSSArray;
	self->zGNSSArray = zGNSSArray;
	self->vNorthSum = 0;
	self->vEastSum = 0;
	self->vDownSum = 0;
	self->currentLatitude = 0;
	self->currentLongitude = 0;
	self->totalSamples = 0;
	self->totalSamplesAveraged = 0;
	self->numberCyclesWithoutData = 0;
	self->latLongIsValid = false;
	self->velocityIsValid = false;
	self->clockHasBeenSet = false;
	self->validMessageProcessed = false;
//	self->init = gnss_init;
	self->get_location = gnss_get_location;
	self->get_running_average_velocities = gnss_get_running_average_velocities;
	self->get_and_process_message = gnss_get_and_process_message;
	self->sleep = gnss_sleep;
}

/**
 * Pull from the UART message buffer and try to process to UBX_NAV_PVT message.
 *
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
	// Message + overhead bits = 97 bytes. We'll make enough space for 5 msgs
gnss_error_code_t gnss_get_and_process_message(GNSS* self)
{
	// Message + overhead bits = 99 bytes. We'll make enough space for 5 msgs
//	uint8_t UART_receive_buf[512];
//	memset(UART_receive_buf, 0, sizeof(UART_receive_buf));
//
//	HAL_StatusTypeDef ret = HAL_UART_Receive_IT(self->gnss_uart_handle, &UART_receive_buf[0], 400);
////    HAL_StatusTypeDef ret = HAL_UART_Receive(self->gnss_uart_handle, &UART_receive_buf[0], 256, 20000);
//    if (!(ret == HAL_OK || ret == HAL_TIMEOUT)) {
//    	// Something went wrong trying to pull from UART buffer
//    	self->numberCyclesWithoutData++;
//    	switch(ret){
//		case HAL_BUSY: {return GNSS_BUSY_ERROR;}
////		case HAL_TIMEOUT: {return GPS_TIMEOUT_ERROR;}
//		default: {return GNSS_UNKNOWN_ERROR;}
//    	}
//    }
	// Hold onto the current UBX_NAV_PVT message
	char UBX_NAV_PVT_message_buf[128];
	// Start with a fresh buffer all zero'd out
    memset(UBX_NAV_PVT_message_buf, 0, sizeof(UBX_NAV_PVT_message_buf));
    self->validMessageProcessed = false;
	// Some fields to keep track of
    int32_t messageClass = 0;
    int32_t messageId = 0;
    const char* UART_buffer_start = (const char*)&(self->uart_buffer[0]);
    const char* UART_buffer_end = UART_buffer_start;
    size_t bufferLength = sizeof(self->uart_buffer);

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
		// If we made it here, we're good to process a message

		// Start by setting the clock if needed
		if (!self->clockHasBeenSet) {
			// Grab time accuracy estimate
			// TODO: check valid time flag first
			int32_t tAcc = UBX_NAV_PVT_message_buf[12] +
					(UBX_NAV_PVT_message_buf[13]<<8) +
					(UBX_NAV_PVT_message_buf[14]<<16) +
					(UBX_NAV_PVT_message_buf[15]<<24);

			if (tAcc < MAX_ACCEPTABLE_TACC) {
				// TODO: Figure this out and then make it a function
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
				// Set RTC time

				self->clockHasBeenSet = true;
			}
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
			self->currentLatitude = lat;
			self->currentLongitude = lon;
		}

		// Grab velocities, start by checking speed accuracy estimate (sAcc)
		int32_t sAcc = UBX_NAV_PVT_message_buf[68] +
				(UBX_NAV_PVT_message_buf[69] << 8) +
				(UBX_NAV_PVT_message_buf[70] << 16) +
				(UBX_NAV_PVT_message_buf[71] << 24);

		if (sAcc > MAX_ACCEPTABLE_SACC) {
			// This message was not within acceptable parameters,
			bufferLength -= UART_buffer_end - UART_buffer_start;
			UART_buffer_start = UART_buffer_end;
			continue;
		}

		// vAcc was within acceptable range, still need to check
		// individual velocities are less than MAX_POSSIBLE_VELOCITY
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

		if (vnorth > MAX_POSSIBLE_VELOCITY ||
			veast > MAX_POSSIBLE_VELOCITY ||
			vdown > MAX_POSSIBLE_VELOCITY) {
			// One or more velocity component was greater than the
			// max possible velocity. Loop around and try again
			bufferLength -= UART_buffer_end - UART_buffer_start;
			UART_buffer_start = UART_buffer_end;
			continue;
		}

		// All velocity values are good to go, convert them to
		// shorts and store them in the arrays
		self->uGNSSArray[self->totalSamples] = (int16_t)veast;
		self->vGNSSArray[self->totalSamples] = (int16_t)vnorth;
		self->zGNSSArray[self->totalSamples] = (int16_t)vdown * -1;

		self->numberCyclesWithoutData = 0;
		self->totalSamples++;
		self->validMessageProcessed = true;
		}
	} // end for loop

    if (!self->validMessageProcessed) {
    	// We weren't able to get a valid message from the buffer, so we'll sub
    	// a running average iff there are more than 0 valid samples so far
    	if (++self->numberCyclesWithoutData > MAX_EMPTY_CYCLES) {
    		// Some sort of bail out condition
    		// We might send a message with a specific payload to indicate
    		// things went wrong.

    		//TODO: figure out what we'll do in this situation. Likely revert
    		//      to taking measurements from the IMU
    	} else {
    		// We'll replace the values with running average
    		float north = 0, east = 0, down = 0;
    		// make sure there are samples to average
    		gnss_error_code_t ret = self->get_running_average_velocities(
    				self, &north, &east, &down);
    		if (ret == GNSS_SUCCESS) {
    			// got the averaged values
    			self->uGNSSArray[self->totalSamples] = (int16_t)north;
    			self->vGNSSArray[self->totalSamples] = (int16_t)east;
    			self->zGNSSArray[self->totalSamples] = (int16_t)down * -1;
    		} else {
    			// Wasn't able to get a running average -- this will return
    			// GNSS_NO_SAMPLES_ERROR
    			return ret;
			}
    	}
    }
    // A valid message was processed
    return GNSS_SUCCESS;
}

/**
 * Get the current lat/long.
 *
 * @param latitude - return parameter for latitude
 * @param longitude - return parameter for longitude
 * @return GPS error code (marcos defined in gps.h)
 */
gnss_error_code_t gnss_get_location(GNSS* self, int32_t* latitude,
		int32_t* longitude)
{
	if (self->latLongIsValid) {
		*latitude = self->currentLatitude;
		*longitude = self->currentLongitude;
		return GNSS_SUCCESS;
	} else {
		return GNSS_LOCATION_INVALID;
	}
}

/**
 * If a velocity field > MAX_POSSIBLE_VELOCITY, or the velocity accuracy estimate (vAcc) is
 * outside acceptable range, this function will substitute a running average.
 *
 * @param returnNorth - return parameter for the running average North value
 * @param returnEast - return parameter for the running average East value
 * @param returnDown - return parameter for the running average Down value
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
gnss_error_code_t gnss_get_running_average_velocities(GNSS* self,
		float* returnNorth, float* returnEast, float* returnDown)
{
	if (self->totalSamples > 0) {
		float substituteNorth = self->vNorthSum /
				((float)self->totalSamples);
		*returnNorth = substituteNorth;
		self->vNorthSum += substituteNorth;

		float substituteEast = self->vEastSum /
				((float)self->totalSamples);
		*returnEast = substituteEast;
		self->vEastSum += substituteEast;

		float substituteDown = self->vDownSum /
				((float)self->totalSamples);
		*returnDown = substituteDown;
		self->vDownSum += substituteDown;

		self->totalSamplesAveraged++;
		return GNSS_SUCCESS;
	} else {
		// No valid samples yet, avoid divide by zero error
		return GNSS_NO_SAMPLES_ERROR;
	}
}

/**
 * If a velocity field > MAX_POSSIBLE_VELOCITY, or the velocity accuracy estimate (vAcc) is
 * outside acceptable range, this function will substitute a running average.
 *
 * @param returnNorth - return parameter for the running average North value
 * @param returnEast - return parameter for the running average East value
 * @param returnDown - return parameter for the running average Down value
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
gnss_error_code_t gnss_sleep(GNSS* self)
{

}
