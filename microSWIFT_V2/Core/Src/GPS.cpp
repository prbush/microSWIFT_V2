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

#include "GPS.h"

GPS::GPS() {
	// TODO Auto-generated constructor stub

}

GPS::~GPS() {
	// TODO Auto-generated destructor stub
}

bool GPS::init(void) {

}

bool GPS::checksum(uint8_t CK_A, uint8_t CK_B, char* bytes, uint32_t bufferLength) {
	/* From u-blox Interface Description
	 *  CK_A = 0, CK_B = 0
		For (I = 0; I < N; I++) {
			CK_A = CK_A + Buffer[I]
			CK_B = CK_B + CK_A
		}
	 *
	 * We need two uint8_t's in order to compare them with CK_A, CK_B
	 */
}

