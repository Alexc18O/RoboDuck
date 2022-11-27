#include "Arduino.h"
#include <math.h>       /* atan2 */
#include "MyDuckClass.h"
#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <TinyGPSPlus.h>
//TinyGPSPlus gps;
#include <QMC5883LCompass.h>
//QMC5883LCompass compass;



MyDuckClass::MyDuckClass(int pinPropellorSpeed, int pinPropellorDirection, int pinHeadingMotor, int pinHeadBobbing) {
    _pinPropSpeed = pinPropellorSpeed;
    _pinPropDirection = pinPropellorDirection;
    _pinHM = pinHeadingMotor;
    _pinHB = pinHeadBobbing;
    //Remaining Private Variables 
    //_angleOfHeading
    //wayPoints[10]
    //_spotLockLocation
   // Servo headBobbingServo;
   // Servo PropellorServo;
    //headBobbingServo.attach(_pinHB);
   // PropellorServo.attach(_pinHM);


}   

String MyDuckClass::calibrationOfMagnotometer(){
//Used to calibrate heading (IMPORTANT)
    Serial.println("Calibration");
    Servo PropellorServo;
    PropellorServo.attach(_pinHM);
    //Turn heading to face perpendicular to duck decoy
    PropellorServo.write(270);
    digitalWrite(_pinPropDirection, HIGH); // direction = forward
    analogWrite(_pinPropSpeed, -255); // PWM speed = fast max is 255
    delay(3000); // Delay for 3 seconds to allow propper heading of propellor 
    QMC5883LCompass compass;

    int calibrationData[3][2];
    bool changed = false;
    bool done = false;
    int t = 0;
    int c = 0;

    while (done != true) {

        int x, y, z;
        // Read compass values
        compass.read();
        // Return XYZ readings
        x = compass.getX();
        y = compass.getY();
        z = compass.getZ();
        changed = false;

        if (x < calibrationData[0][0]) {
            calibrationData[0][0] = x;
            changed = true;
        }
        if (x > calibrationData[0][1]) {
            calibrationData[0][1] = x;
            changed = true;
        }

        if (y < calibrationData[1][0]) {
            calibrationData[1][0] = y;
            changed = true;
        }
        if (y > calibrationData[1][1]) {
            calibrationData[1][1] = y;
            changed = true;
        }

        if (z < calibrationData[2][0]) {
            calibrationData[2][0] = z;
            changed = true;
        }
        if (z > calibrationData[2][1]) {
            calibrationData[2][1] = z;
            changed = true;
        }

        if (changed && !done) {
 
            c = millis();
        }
        t = millis();


        if ((t - c > 5000) && !done) {
            done = true;

            String result = "compass.setCalibration(";
                        
            result = result + calibrationData[0][0] + "," + calibrationData[0][1] + "," + calibrationData[1][0] + "," + calibrationData[1][1] + "," + calibrationData[2][0] + "," + calibrationData[2][1] + ");";
                return result;
        }

    }

PropellorServo.detach();
}


//turn on propellor
void MyDuckClass::turnOnProppellor(int propellorSpeed, int propellorDirection){
// Turning on DC motor to have Propellor run 
 Serial.println("Turn on prop");
#define MOTOR_A_PWM propellorSpeed // Motor B PWM Speed
#define MOTOR_A_DIR propellorDirection // Motor B Direction


    digitalWrite(propellorDirection, HIGH); // direction = forward
    analogWrite(propellorSpeed, 50); // PWM speed = slow



}

//Turn off propellor
void MyDuckClass::turnOffProppellor(int propellorSpeed, int propellorDirection) {
    // Turning on DC motor to have Propellor run 
 Serial.println("turn off prop");
#define MOTOR_A_PWM propellorSpeed // Motor B PWM Speed
#define MOTOR_A_DIR propellorDirection // Motor B Direction


    digitalWrite(MOTOR_A_DIR, LOW);
    analogWrite(MOTOR_A_PWM, 255);



}


//Change heading of the propellor motor using the servo motor
void MyDuckClass::movePropHeading(Servo headingServo,double currentlat,double currentlong,double destinationlat,double destinationlong,TinyGPSPlus gps1,QMC5883LCompass compass1){
//Set direction of Propellor
 Serial.println("move prop heading");
    double currentLocation[] = {currentlat, currentlong};
    double destinationLocation[] = {destinationlat,destinationlong} ;
    int angleOfHeading = 0;
    int angleMag;

      //while((Serial2.available() > 0)) {
      while(true) {
      //Turn heading to face perpendicular to duck decoy
      Serial.println("Getting GPS");   


        if (gps1.encode(Serial2.read()))
        {
          if (gps1.location.isValid()){
             currentLocation[0] = (gps1.location.lat());
            currentLocation[1] = (gps1.location.lng());
            //Serial.println(currentLocation[0]);
            //Serial.println(currentLocation[1]);
            //Serial.println(distanceToWaypoint(currentLocation,destinationLocation));
              compass1.read();
              //Angle of heading based on Magnotometer
              angleMag = compass1 .getAzimuth();
              
             angleOfHeading = gps1.courseTo(gps1.location.lat(),gps1.location.lng(),44.14774,-93.99575);
              headingServo.write(angleOfHeading-100);
            Serial.println(angleOfHeading-100);
            Serial.println(gps1.distanceBetween( gps1.location.lat(),gps1.location.lng(),44.14774,-93.99575));
            //delay(1000);
            }
          }
          
          //if(distanceToWaypoint(currentLocation,destinationLocation) < 4){
            //break;
        //}
      
    
}
return;
}

void MyDuckClass::spotLock(float spotLockLocation[2]) {
// Spot lock for duck
    float CL;
    CL =  1;

    //if (distanceToWaypoint(CL, spotLockLocation) > 4) {
        //float desiredAngle = angleToWaypoint(CL, spotLockLocation);
        //Fix angle once motor is mounted
      //  movePropHeading(desiredAngle);
    //}


}

void MyDuckClass::currentLocation(){
//Get the current location of the duck

  //  float Cordinates[2];
Serial.print("Getting GPS location");
  //  TinyGPSPlus gps;
//Serial2.print("Getting current location");
//Serial2.begin(9600);
    //if ((Serial2.available() > 0)) {
        //Serial.print("gps available");
        //if ((gps.encode(Serial2.read()))) {
          //Serial.print("gps encode");
            //if (gps.location.isValid()) {
                //Serial.print("Gps location is valid");
                //Cordinates[0] = gps.location.lat();
                
                //Cordinates[1] = gps.location.lng();
               // Serial.print(Cordinates[0]);
              //  Serial.print(Cordinates[1]);
          
                
            //}
            //else {

          //      Serial.print(F("INVALID"));

        //    }
      //  }

    //}
    //return ;
}

float MyDuckClass::GetDuckHeading() {
    int x, y, z, dy, dx, d, azimuth; //Read QMC5883LCompass values
    QMC5883LCompass compass;
    compass.read();

    //Return XYZ readings
    x = compass.getX();
    y = compass.getY();
    z = compass.getZ();
    azimuth = compass.getAzimuth();
    char myArray[3];
    compass.getDirection(myArray, azimuth);
    Serial.print(myArray[0]);
    Serial.print(myArray[1]);
    Serial.print(myArray[2]);
    Serial.println();
    Serial.print(x);
    Serial.print(y);
    Serial.print(z);

    dx = (x * .48828125);
    dy = (y * .48828125);

    d = atan(dy / dx) * (180 / 3.14);
    float result = d;

    return result;
}

double MyDuckClass::angleToWaypoint (double waypointLatLonCur[2], double waypointLatLon[2]){
    // Returns angle needed to get to waypoint
    // cordinates of current location
    float lat1, lon1;
    //cordintes of desitnation
    float lat2, lon2;
    lat1 = waypointLatLonCur[0];
    lon1 = waypointLatLonCur[1];

    lat2 = waypointLatLon[0];
    lon2 = waypointLatLon[1];

    float temp = lon2 - lon1;

    float X = cos(lat2 * 3.14 / 180) * sin(temp * 3.14 / 180);

    float Y1 = (cos(lat1 * 3.14 / 180) * sin(lat2 * 3.14 / 180));
    float Y2 = (sin(lat1 * 3.14 / 180) * cos(lat2 * 3.14 / 180) * cos(temp * 3.14 / 180));
    float Y = Y1 - Y2;

    float result = atan2(X, Y) * 180 / 3.14;
    //bearing North, ex: 0 = N, 90 = E, 180 = S, 270 = W

    return result;
}


double MyDuckClass::distanceToWaypoint(double waypointLatLonCur[2], double desiredpointLatLon[2]) {
    float lat1, lon1;
    //cordintes of desitnation
    float lat2, lon2;
    lat1 = waypointLatLonCur[0];
    lon1 = waypointLatLonCur[1];

    lat2 = desiredpointLatLon[0];
    lon2 = desiredpointLatLon[1];

    float p = 0.017453292519943295;    // Math.PI / 180

    float a = 0.5 - cos((lat2 - lat1) * p) / 2 +
        cos(lat1 * p) * cos(lat2 * p) *
        (1 - cos((lon2 - lon1) * p)) / 2;

    //std::cout << 12742000 * asin(sqrt(a)); // m

    float result = 12742000 * asin(sqrt(a)); // m
    

    return result;
}
