#include "motor_driver.h"

//////////////////////////////////////////////////////////////////////
void lfwd(int speed=255) {
//////////////////////////////////////////////////////////////////////
	digitalWrite( P_LFWD, LOW );
	digitalWrite( P_LREV, HIGH);
	if (EN_BAR) {
		analogWrite( P_LENA, constrain( 255-speed, 0, 255 ) );
	} else {
		analogWrite( P_LENA, constrain(speed, 0, 255 ) );
	}
}

//////////////////////////////////////////////////////////////////////
void lrev(int speed=255) {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_INT("lrev %d", speed);

	digitalWrite( P_LFWD, HIGH);
	digitalWrite( P_LREV, LOW );
	if (EN_BAR) {
		analogWrite( P_LENA, constrain( 255-speed, 0, 255 ) );
	} else {
		analogWrite( P_LENA, constrain(speed, 0, 255 ) );
	}
}

//////////////////////////////////////////////////////////////////////
void lcoast() {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_STR("lcoast");

	digitalWrite( P_LFWD, LOW );
	digitalWrite( P_LREV, LOW );
	digitalWrite( P_LENA, LOW );
}

//////////////////////////////////////////////////////////////////////
void lbrake() {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_STR("lbrake");

	digitalWrite( P_LFWD, HIGH );
	digitalWrite( P_LREV, HIGH );
	digitalWrite( P_LENA, LOW);
}

//////////////////////////////////////////////////////////////////////
void rfwd( int speed=255) {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_INT("rfwd %d", speed);

	digitalWrite( P_RFWD, LOW  );
	digitalWrite( P_RREV, HIGH);
	if (EN_BAR) {
		analogWrite( P_RENA, constrain( 255-speed, 0, 255 ) );
	} else {
		analogWrite( P_RENA, constrain(speed, 0, 255 ) );
	}
}

//////////////////////////////////////////////////////////////////////
void rrev(int speed=255) {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_INT("rrev %d", speed);

	digitalWrite( P_RFWD, HIGH);
	digitalWrite( P_RREV, LOW  );
	if (EN_BAR) {
		analogWrite( P_RENA, constrain( 255-speed, 0, 255 ) );
	} else {
		analogWrite( P_RENA, constrain(speed, 0, 255 ) );
	}
}


//////////////////////////////////////////////////////////////////////
void rcoast() {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_STR("rcoast");

	digitalWrite( P_RFWD, LOW );
	digitalWrite( P_RREV, LOW );
	digitalWrite( P_RENA, LOW );
}

//////////////////////////////////////////////////////////////////////
void rbrake() {
//////////////////////////////////////////////////////////////////////
//	PRINTDEBUG_STR("rbrake");

	digitalWrite( P_RFWD, HIGH );
	digitalWrite( P_RREV, HIGH );
	digitalWrite( P_RENA, LOW);
}



//////////////////////////////////////////////////////////////////////
void initMotorController() {
//////////////////////////////////////////////////////////////////////
	  pinMode(P_LFWD, OUTPUT);
	  pinMode(P_LREV, OUTPUT);
	  pinMode(P_LENA, OUTPUT);

	  pinMode(P_RFWD, OUTPUT);
	  pinMode(P_RREV, OUTPUT);
	  pinMode(P_RENA, OUTPUT);
}
//////////////////////////////////////////////////////////////////////
void setMotorSpeed(int i, int spd){
//////////////////////////////////////////////////////////////////////
	if (spd > 255) spd = 255;
	if (spd < -255) spd = -255;
	if( i==LEFT ) {
		if( spd > 0 )        lfwd( spd );
		else if ( spd < 0 )  lrev( spd );
		else                 lbrake();
	} else {
		if ( spd > 0)        rfwd( spd );
		else if ( spd < 0 )  rrev( spd );
		else                 rbrake();
	}
}
//////////////////////////////////////////////////////////////////////
void setMotorSpeeds(int leftSpeed, int rightSpeed){
//////////////////////////////////////////////////////////////////////
	setMotorSpeed( LEFT, leftSpeed);
	setMotorSpeed( RIGHT, rightSpeed);
}