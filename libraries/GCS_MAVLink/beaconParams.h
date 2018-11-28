#pragma once

#include <AP_Common/AP_Common.h>

class BeaconParams {
public:

    AP_Int32        funtionMask;
	AP_Int32		height;
	AP_Int32		width;
	AP_Int32		velocity;
	AP_Int32		flow;
	
	AP_Int32		aPointLatitude;
	AP_Int32		aPointLongitude;

	AP_Int32		bPointLatitude;
	AP_Int32		bPointLongitude;

	AP_Int32		breakPointLatitude;	
	AP_Int32		breakPointLongitude;
	AP_Int32		seqOfNextWayPoint;	// the seq of hte next wayPoint when the break event happend.
	AP_Int8			breakDirection;  	// for AB break point only.

	
	
   BeaconParams()
    {
    	funtionMask = 0;
		height = 1250;
		width = 500;
		velocity = 600;
		flow = 0;

		aPointLatitude = 0;
		aPointLongitude = 0;

		bPointLatitude = 0;
		bPointLongitude = 0;

		breakPointLatitude = 0;
		breakPointLongitude = 0;
		seqOfNextWayPoint = 0;
		breakDirection = 0;
    }
};

