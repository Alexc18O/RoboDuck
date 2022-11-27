#ifndef MyDuckClass_h
#define MyDuckClass_h
#include "Arduino.h"
#include <ESP32Servo.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>


class MyDuckClass {
    public:
        MyDuckClass(int pinPropellor,int pinPropDirection, int pinHeadingMotor, int pinHeadBobbing);
            //Used to calibrate heading (IMPORTANT)
            String calibrationOfMagnotometer();
            // Turning on DC motor to have Propellor run
            void turnOnProppellor(int propellorSpeed, int propellorDirection);

            void turnOffProppellor(int propellorSpeed, int propellorDirection);
            //Set direction of Propellor
            void movePropHeading(Servo headingServo,double currentlat,double currentlong,double destinationlat,double desitnationlong,TinyGPSPlus gps1,QMC5883LCompass compass1);
            // Spot lock for duck
            void spotLock(float spotLockLocation[2]);
            //Get the current location of the duck
            void currentLocation();
            // Returns angle needed to get to waypoint
            double angleToWaypoint (double waypointLatLonCur[2], double waypointLatLon[2]);
            //Get distance to specified waypoint
            double distanceToWaypoint(double waypointLatLonCur[2], double desiredpointLatLon[2]);
            //get heading from magnotometer
            float GetDuckHeading() ;

      
    private:
        //Pins for Different motors being used
        int _pinPropSpeed,_pinPropDirection, _pinHM, _pinHB;
        //Angle needed for heading
        int _angleOfHeading;
        // Array of Waypoints
        String _wayPoints[10];
        // Spot lock position
        String _spotLockLocation;
        

};
#endif
