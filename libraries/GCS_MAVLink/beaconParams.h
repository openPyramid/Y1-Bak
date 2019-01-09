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
	AP_Int32 		fixYaw;
	AP_Int8			breakDirection;  	// for AB break point only.
	AP_Int8			breakPointType;		// 1: Auto; 4:AB.
	AP_Int8 		sprayFlag;			// is spray when break point accur

	AP_Int8			needCalcBearingFlag;
	AP_Int8			calcBearingDoneFlag;
	AP_Int8			calcBearingMS;

   BeaconParams()
    {
    	funtionMask = 0;
		height = 1250;
		width = 400;
		velocity = 400;
		flow = 0;

		aPointLatitude = 0;
		aPointLongitude = 0;

		bPointLatitude = 0;
		bPointLongitude = 0;

		breakPointLatitude = 0;
		breakPointLongitude = 0;
		seqOfNextWayPoint = 0;
		breakDirection = 0;
		breakPointType = 0;
		sprayFlag = 0;

		needCalcBearingFlag = 0;
		calcBearingDoneFlag = 0;
		calcBearingMS = 0;
    }
};

