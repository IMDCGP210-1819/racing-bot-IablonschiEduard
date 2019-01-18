/***************************************************************************

    file                 : robot_base.cpp
    created              : Mon 13 Feb 11:40:23 GMT 2017
    copyright            : (C) 2002 Author

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifdef _WIN32
//#include <windows.h>
#endif

#include <cstdio>
#include <cstdlib> 
#include <cstring> 
#include <cmath>
#include <cstring>
#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
s188800EduardIablonschi(tModInfo *modInfo)
{
    std::memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = strdup("s188800EduardIablonschi");		/* name of the module (short) */
    modInfo->desc    = strdup("");	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 

// initialising the counter
static int stuck = 0;

/* the boolean value represents if the car is stuck */
bool isStuck(tCarElt* car)
{
	float angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
	NORM_PI_PI(angle);
	// checks if the angle is smaller than 30 degrees
	if (fabs(angle) < 30.0 / 180.0*PI) {
		stuck = 0;
		return false;
	}
	if (stuck < 100) {
		stuck++;
		return false;
	}
	else {
		return true;
	}
}


/* Drive during race. */
/* The code written below should work, however, because of the multiple compiling errors I couldn't fix, I wasn't 
really able to test it myself*/
static void
drive(int index, tCarElt* car, tSituation *s)
{
	memset((void *)&car->ctrl, 0, sizeof(tCarCtrl));

	float angle;
	const float SC = 1.0;

	if (isStuck(car)) // if the car is stuck, it will move backwards, and the steering angle will be the opposite of the one which it entered the corner with
	{
		angle = -RtTrackSideTgAngleL(&(car->_trkPos)) + car->_yaw;
		car->ctrl.steer = angle / car->_steerLock; // the steering angle will not change for the current segment of the track
		car->ctrl.gear = -1; // reverse gear
		car->ctrl.accelCmd = 0.25; // 25% acceleration
		car->ctrl.brakeCmd = 0.0; // no brakes
	}
	else
	{
		angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw; // initial steering angle = tangent angle of the track - angle of the car
		NORM_PI_PI(angle);
		angle -= SC * car->_trkPos.toMiddle / car->_trkPos.seg->width;

		car->ctrl.steer = angle / car->_steerLock; // the steering angle will not change for the current segment of the track
		car->ctrl.gear = 1; // first gear
		car->ctrl.accelCmd = 0.25; // 25% acceleration 
		car->ctrl.brakeCmd = 0.0; // no brakes

		/*
		* add the driving code here to modify the
		* car->_steerCmd
		* car->_accelCmd
		* car->_brakeCmd
		* car->_gearCmd
		* car->_clutchCmd
		*/
	}
							  
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

