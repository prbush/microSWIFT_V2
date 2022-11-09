/*
 * GPS.cpp
 *
 *  Created on: Oct 28, 2022
 *      Author: Phil
 *
 *  We need to figure out if we will use UBX or NMEA messages
 *  	- If we use UBX (preferred), UBX-NAV-PVT (Navigation position velocity time solution)
 *  	  will be the ticket since it is periodic and can be polled.
 */

#include <gps.h>

GPS::GPS(UART_HandleTypeDef* uart_handle) {
	// TODO Auto-generated constructor stub
	// configure, init
	gps_uart_handle = uart_handle;

    // Initialize the APIs we will need
//    uPortInit();
//    uGnssInit();

//    // Open a UART instance
//    transportHandle.uart = uPortUartOpen(uart_handle->instance,
//                                         U_GNSS_UART_BAUD_RATE, NULL,
//										 U_GNSS_UART_BUFFER_LENGTH_BYTES,
//                                         U_CFG_APP_PIN_GNSS_TXD,
//                                         U_CFG_APP_PIN_GNSS_RXD,
//                                         U_CFG_APP_PIN_GNSS_CTS,
//                                         U_CFG_APP_PIN_GNSS_RTS);
//
//    // Add a GNSS instance, giving it the UART handle and
//    // the pin that enables power to the GNSS module; use
//    // -1 if there is no such pin.
//    uGnssAdd(U_GNSS_MODULE_TYPE_MAX_NUM,
//             U_GNSS_TRANSPORT_UART, transportHandle,
//             U_CFG_APP_PIN_GNSS_ENABLE_POWER, false,
//             &gnssHandle);
//
//    // To get prints of the message exchange with the GNSS module
//    uGnssSetUbxMessagePrint(gnssHandle, true);
//
//    uGnssPwrOn(gnssHandle);
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
// I don't think we'll zero out the message buf, we'll retain just in case
//	UBX_NAV_PVTmessageBuf = {0};
	uint8_t UART_receive_buf[320] = {0};
    int32_t messageClass = 0;
    int32_t messageId = 0;

    HAL_StatusTypeDef ret = HAL_UART_Receive(gps_uart_handle, &UART_receive_buf[0], 160, 200);
    if (ret != HAL_OK) {
    	// Something went wrong trying to pull from UART buffer
    	++numberCyclesWithoutData;
    	switch(ret){
			case HAL_BUSY: {return GPS_BUSY_ERROR;}
			case HAL_TIMEOUT: {return GPS_TIMEOUT_ERROR;}
			default: {return GPS_UNKNOWN_ERROR;}
    	}
    }


    int32_t retval = uUbxProtocolDecode(&((char*)UART_receive_buf)[0], 320, &messageClass, &messageId, &UBX_NAV_PVTmessageBuf[0], 92, NULL);
    if (messageClass != UBX_NAV_PVT_MESSAGE_CLASS || messageId != UBX_NAV_PVT_MESSAGE_ID) {
    	// Message class and/or ID did not match that of a UBX_NAV_PVT message
    	++numberCyclesWithoutData;
    	return GPS_NO_MESSAGE_RECEIVED;
    }

    if (retval == 92) { // Good to go, process message
    	// Start by checking time accuracy and set the clock if needed
    	// tAcc = time accuracy estimate
    	int32_t tAcc = UBX_NAV_PVTmessageBuf[12] + (UBX_NAV_PVTmessageBuf[13]<<8) + (UBX_NAV_PVTmessageBuf[14]<<16) + (UBX_NAV_PVTmessageBuf[15]<<24);

    	if (tAcc < MAX_ACCEPTABLE_VACC && !clockHasBeenSet) { // !!!!! UNFINISHED
    		// set clock
    		uint16_t year = UBX_NAV_PVTmessageBuf[4];
    		uint8_t month = UBX_NAV_PVTmessageBuf[6];
    		uint8_t day = UBX_NAV_PVTmessageBuf[7];
    		uint8_t hour = UBX_NAV_PVTmessageBuf[8];
    		bool isAM = true;
    		if (hour > 12) { hour %= 12; isAM = false;}
    		uint8_t min = UBX_NAV_PVTmessageBuf[9];
    		uint8_t sec = UBX_NAV_PVTmessageBuf[10];

    		RTC_TimeTypeDef time = {hour, min, sec, (isAM) ? RTC_HOURFORMAT12_AM : RTC_HOURFORMAT12_PM, 0, 0};
    	}

    	// Check Lat/Long accuracy, assign to class fields if good
	    int32_t lon = UBX_NAV_PVTmessageBuf[24] + (UBX_NAV_PVTmessageBuf[25]<<8) + (UBX_NAV_PVTmessageBuf[26]<<16) + (UBX_NAV_PVTmessageBuf[27]<<24);
	    int32_t lat = UBX_NAV_PVTmessageBuf[28] + (UBX_NAV_PVTmessageBuf[29]<<8) + (UBX_NAV_PVTmessageBuf[30]<<16) + (UBX_NAV_PVTmessageBuf[31]<<24);
    	int16_t pDOP =  UBX_NAV_PVTmessageBuf[76] + (UBX_NAV_PVTmessageBuf[77]<<8);
    	if (pDOP < MAX_ACCEPTABLE_PDOP) {
    		currentLatitude = lat;
    		currentLongitude = lon;
    	}

    	// Grab velocities
    	int32_t vAcc = UBX_NAV_PVTmessageBuf[44] + (UBX_NAV_PVTmessageBuf[45] << 8) + (UBX_NAV_PVTmessageBuf[46] << 16) + (UBX_NAV_PVTmessageBuf[47] << 24);
	    int32_t vnorth = UBX_NAV_PVTmessageBuf[48] + (UBX_NAV_PVTmessageBuf[49] << 8) + (UBX_NAV_PVTmessageBuf[50] << 16) + (UBX_NAV_PVTmessageBuf[51] << 24);
	    int32_t veast = UBX_NAV_PVTmessageBuf[52] + (UBX_NAV_PVTmessageBuf[53] << 8) + (UBX_NAV_PVTmessageBuf[54] << 16) + (UBX_NAV_PVTmessageBuf[55] << 24);
	    int32_t vdown = UBX_NAV_PVTmessageBuf[56] + (UBX_NAV_PVTmessageBuf[57] << 8) + (UBX_NAV_PVTmessageBuf[58] << 16) + (UBX_NAV_PVTmessageBuf[59] << 24);
	    if (vAcc < MAX_ACCEPTABLE_VACC) {
	    	// TRY GRABBING THE NEXT MESSAGE FROM THE BUF!!!!!!!!!!

	    }


	    char txstr[140] = {0};
	    snprintf(txstr, 140, "\r\nLatitude: %ld\r\n Longitude: %ld\r\n Velocity North: %ld\r\n Velocity East: %ld\r\n Velocity Down: %ld\r\n", lon, lat, vnorth, veast, vdown);
	    HAL_UART_Transmit(&huart2,(uint8_t*) txstr, sizeof(txstr), 10);
    } else {
    	// UBX_NAV_PVT message is 92 bytes -- if we didn't get all of them, then no valid message was received
    	++numberCyclesWithoutData;
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
	    char UBX_NAV_PVTmessageBuf[92];
	    HAL_UART_Receive(gps_uart_handle,(uint8_t*) messagebuf, 192, 1000);

	    int32_t ret = uUbxProtocolDecode(messagebuf, 192, nullptr, nullptr, UBX_NAV_PVTmessageBuf, 92, nullptr);

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
