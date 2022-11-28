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



void gnss_init(GNSS* self, UART_HandleTypeDef* gnss_uart_handle, TX_QUEUE* ubx_queue,
		int16_t* uGNSSArray, int16_t* vGNSSArray, int16_t* zGNSSArray)
{
	/* TODO:Turn on power pin
	 *      Setup uart?
	 *      Create arrays
	 *      wait until good fix?
	 */
	// initialize everything to 0/false
	self->gnss_uart_handle = gnss_uart_handle;
	self->ubx_queue = ubx_queue,
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
	self->gnss_process_message = gnss_process_message;
	self->sleep = gnss_sleep;
}

/**
 * Pull from the UART message buffer and try to process to UBX_NAV_PVT message.
 *
 * @return GPS error code (marcos defined in gps_error_codes.h)
 */
	// Message + overhead bits = 97 bytes. We'll make enough space for 5 msgs
gnss_error_code_t gnss_process_message(GNSS* self)
{
	ULONG num_msgs_enqueued, available_space;
	UINT ret;
	const char message[UBX_NAV_PVT_MESSAGE_LENGTH];
	char payload[UBX_NAV_PVT_PAYLOAD_LENGTH];
    int32_t message_class = 0;
    int32_t message_id = 0;
    int32_t num_payload_bytes = 0;
    // start with 0'd out buffers;
    memset(message, 0, sizeof(message));
    memset(payload, 0, sizeof(payload));
    // Reset the valid message processed flag
    self->validMessageProcessed = false;
	// How many messages are on the queue
	ret = tx_queue_info_get(self->ubx_queue, TX_NULL, &num_msgs_enqueued,
						&available_space, TX_NULL, TX_NULL, TX_NULL);
	// More messages may be added while processing, but we'll leave those
	// for the next time around and only get those on the queue right now.
	while (num_msgs_enqueued > 0) {
		ret = tx_queue_receive(self->ubx_queue, (VOID*)message, TX_NO_WAIT);
		if (ret != TX_SUCCESS) {
			--num_msgs_enqueued;
			continue;
		}
	    // Try to decode message
	    num_payload_bytes = uUbxProtocolDecode(&(message[0]), UBX_NAV_PVT_MESSAGE_LENGTH,
	    		&message_class, &message_id, payload, sizeof(payload), NULL);
	    // UBX_NAV_PVT payload is 92 bytes, if we don't have that many, its
	    // a bad message
		if (num_payload_bytes != UBX_NAV_PVT_PAYLOAD_LENGTH) {
			--num_msgs_enqueued;
			continue;
		}

		// Start by setting the clock if needed
		if (!self->clockHasBeenSet) {
			// Grab time accuracy estimate
			// TODO: check valid time flag first
			int32_t tAcc = payload[12] +
					(payload[13]<<8) +
					(payload[14]<<16) +
					(payload[15]<<24);

			if (tAcc < MAX_ACCEPTABLE_TACC) {
				// TODO: Figure this out and then make it a function
				// set clock
				uint16_t year = payload[4];
				uint8_t month = payload[6];
				uint8_t day = payload[7];
				uint8_t hour = payload[8];
				bool isAM = true;
				if (hour > 12) { hour %= 12; isAM = false;}
				uint8_t min = payload[9];
				uint8_t sec = payload[10];

				RTC_TimeTypeDef time = {hour, min, sec,
						(isAM) ?
						RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM, 0, 0};
				// Set RTC time

				self->clockHasBeenSet = true;
			}

			// Check Lat/Long accuracy, assign to class fields if good
			int32_t lon = payload[24] +
					(payload[25]<<8) +
					(payload[26]<<16) +
					(payload[27]<<24);
			int32_t lat = payload[28] +
					(payload[29]<<8) +
					(payload[30]<<16) +
					(payload[31]<<24);
			int16_t pDOP =  payload[76] +
					(payload[77]<<8);

			if (pDOP < MAX_ACCEPTABLE_PDOP) {
				self->currentLatitude = lat;
				self->currentLongitude = lon;
			}

			// Grab velocities, start by checking speed accuracy estimate (sAcc)
			int32_t sAcc = payload[68] +
					(payload[69] << 8) +
					(payload[70] << 16) +
					(payload[71] << 24);

			if (sAcc > MAX_ACCEPTABLE_SACC) {
				// This message was not within acceptable parameters,
				--num_msgs_enqueued;
				continue;
			}

			// vAcc was within acceptable range, still need to check
			// individual velocities are less than MAX_POSSIBLE_VELOCITY
			int32_t vnorth = payload[48] +
					(payload[49] << 8) +
					(payload[50] << 16) +
					(payload[51] << 24);
			int32_t veast = payload[52] +
					(payload[53] << 8) +
					(payload[54] << 16) +
					(payload[55] << 24);
			int32_t vdown = payload[56] +
					(payload[57] << 8) +
					(payload[58] << 16) +
					(payload[59] << 24);

			if ((vnorth > MAX_POSSIBLE_VELOCITY) ||
				(veast > MAX_POSSIBLE_VELOCITY) ||
				(vdown > MAX_POSSIBLE_VELOCITY)) {
				// One or more velocity component was greater than the
				// max possible velocity. Loop around and try again
				--num_msgs_enqueued;
				continue;
			}

			// All velocity values are good to go, convert them to
			// shorts and store them in the arrays
			self->uGNSSArray[self->totalSamples] = (int16_t)veast;
			self->vGNSSArray[self->totalSamples] = (int16_t)vnorth;
			self->zGNSSArray[self->totalSamples] = (int16_t)(vdown * -1);

			self->numberCyclesWithoutData = 0;
			self->totalSamples++;
			self->validMessageProcessed = true;
		}

	}

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
    		int16_t north = 0, east = 0, down = 0;
    		// make sure there are samples to average
    		gnss_error_code_t ret = self->get_running_average_velocities(
    				self, &north, &east, &down);
    		if (ret == GNSS_SUCCESS) {
    			// got the averaged values
    			self->uGNSSArray[self->totalSamples] = north;
    			self->vGNSSArray[self->totalSamples] = east;
    			self->zGNSSArray[self->totalSamples] = down * -1;
    		} else {
    			// Wasn't able to get a running average -- this will return
    			// GNSS_NO_SAMPLES_ERROR
    			return ret;
			}
    	}
    }
    // Lastly, flush the queue and start fresh
    ret = tx_queue_flush(self->ubx_queue);
    if (ret != TX_SUCCESS) {
    	// Maybe something got weird from the ISR, try again
    	ret = tx_queue_flush(self->ubx_queue);
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
		int16_t* returnNorth, int16_t* returnEast, int16_t* returnDown)
{
	if (self->totalSamples > 0) {
		float substituteNorth = self->vNorthSum /
				((float)self->totalSamples);
		*returnNorth = (int16_t)substituteNorth;
		self->vNorthSum += substituteNorth;

		float substituteEast = self->vEastSum /
				((float)self->totalSamples);
		*returnEast = (int16_t)substituteEast;
		self->vEastSum += substituteEast;

		float substituteDown = self->vDownSum /
				((float)self->totalSamples);
		*returnDown = (int16_t)substituteDown;
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
