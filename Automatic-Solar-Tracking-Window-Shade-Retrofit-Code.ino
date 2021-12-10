/*******************************************************
March 14, 2014
Nicholas Zibin

Developed for BLDG 6731: Illumination
Center for Zero Energy Building Studies
Department of Building, Civil, and Environmental Engineering
Concordia University
Montreal, Quebec
Canada

This code will control the interaction between a stepper motor and a lux meter using
both open (solar tracking) and closed loop (luminosity sensor) control.
********************************************************/

// Include the relevant libraries
// See my Instructable for links where to download the code.
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561.h>
#include <Time.h> 

/*******************************************************
********************************************************
**    INPUTS: DEFINE VARIABLES
********************************************************
********************************************************/

//Location constants
float LAT  = 45.5;   // Latitude for Montreal
float LON  = 73.6;   // Longitude for Montreal
float LSM  = 75.0;   // Local Standard Meridan
float Elev = 300;    // Elevation (m)
float psi  = -60;    // Outward normal of my office: 60 deg east of south
float x    = 0.45;   // Horizontal distance to from the shade to the workplane (m)

float pi = 3.14159265359; // pi

// Gear constants and variables
float wp     = 0.2;                                  // The workplane is 20cm above the window sill
float h_max  = 1.5 + wp;                             // Max height of the roller shade
float G_R    = 26.0000000 + 103.00000 / 121.000000;  // Gear ratio of the stepper motor
float m_step = 1.800000 / G_R * (pi / 180);          // Motor step size degrees 
float r      = 30.000000 / 1000.00000;               // Gear radius (m)
float n_step = 0;                                    // Number of steps initiliaze
int i        = 0;                                    // Placer
float k_var = 0.150000000;                           // Linear vertical error in shade height

// Select inital shade height. I recommend starting with the shade at its lowest position
float h_i[]  = {0,0};                                // Intial roller shade height (m)

// Luminosity sensor variables
float k_step  = 0.02000;        // Iterate the number of steps to 10 cm.
float oc_lx   = 800;            // Overcast: 800 lx
float max_lx  = 2000;           // Maximum incident lx at 0.5m from the window.
float mid_lx  = 2000;           // Mid lx. 
float t_step  = 10 * 60 * 1000; // 10 min delay
float lx_meas = 0;              // Initialize variable to store the lx measurement.
int motor_dir = 1;              // Motor direction: 1 = forward, 0 = backward
int overshoot = 0;              // Overshoot variable

// Starting Date
int HH = 11;   // Hour
int M  = 50;   // Minute
int S  = 0;    // Sec
int DD = 14;   // Day
int MM = 12;   // Month
int YY = 2013; // Year 

/*******************************************************
********************************************************
**    Initialize Motor and Light Sensor
**    (Majority of Code copied from Adafruit code examples)
********************************************************
********************************************************/

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2); //Motor spot #2

//Initialize the lux sensor
Adafruit_TSL2561 tsl = Adafruit_TSL2561(TSL2561_ADDR_FLOAT, 12345); //12345 is the unique ID for the lux sensor

/**************************************************************************
**  Configures the gain and integration time for the TSL2561
**************************************************************************/

void configureSensor()
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoGain(true);          /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
 // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
}

/**************************************************************************
**  Configures the gain and integration time for the TSL2561
**************************************************************************/

void setup(void) {
  
  //T ime
    setTime(HH,M,SS,DD,MM,YY); // set time (hr,min,sec,day,month,year)
  
  // Stepper Motor
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(200);  // 200 rpm   
  
  // Lux Sensor
  Serial.begin(9600);
  Serial.println("Light Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 
  /* Setup the sensor gain and integration time */
  configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
  
}
 
 
 
 
/************************************************************************* 
**************************************************************************
**  MAIN PROGRAM
**************************************************************************
**************************************************************************/ 
  
void loop(){ 
  
/*******************************************************
**    CALCULATE ALPHA
**    Next position of the sun is calculated in terms of
**    the solar altitude (angle between horizontal and the sun)
**    and the solar surface azimuth (angle between the outward normal of 
**    a surface (e.g., vertical window) and the sun)
********************************************************/


// Time
float n   = (month() - 1) * 30 + day()+5;      //Approximate day number in the year (+5 is for november/december)
float LST = minute() + 60 * (hour());          // minutes for no daylight savings
float EOT = 9.87 * sin(4 * pi * (n - 81) / 364) - 7.53 * cos(2 * pi * (n - 81) / 364) - 1.5 * sin(2 * pi * (n - 81) / 364);  //Equation of Time (EOT) (minute) 
float AST = LST + 4 * (LSM - LON) + EOT;       // Apparent solar time (AST) (minute)

//Angles
float ha = 15 * (AST / 60 - 12); //Hour angle (degrees)
float d = 23.45 * sin(2 * pi * (284 + n)/365);  //Declination angle (degrees)
float a = asin(cos(LAT*pi/180) * cos(d*pi/180) * cos(ha*pi/180) + sin(LAT*pi/180) * sin(d*pi/180))*180/pi;  //Solar altitude (dgerees)
float phi = acos((sin(a*pi/180) * sin(LAT*pi/180) - sin(d*pi/180)) / cos(a*pi/180) / cos(LAT*pi/180))*180/pi * ha / abs(ha);  //Solar azimuth: theta (degrees)
float y = phi - psi; // Surface solar azimuth (degrees)

Serial.print("Solar Altitude: "); Serial.print(a); Serial.println(" (deg)");
Serial.print("Surface Solar Azimuth: "); Serial.print(y); Serial.println(" (deg)"); Serial.println(" ");

/*******************************************************
**    MEASURE LIGHT
********************************************************/
  sensors_event_t event;  
  tsl.getEvent(&event);
  lx_meas = event.light; 
  /* Display the time and results (light is measured in lux) */
  if (event.light){
  digitalClockDisplay(); Serial.print(event.light); Serial.println(" lux"); Serial.println(" ");
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    Serial.println("Sensor overload");
  }

/*******************************************************
**    START HYBRID CONTROL ALGORITHM
********************************************************/


h_i[1] = x * tan(a*pi/180) / cos(abs(y)*pi/180) + wp; // zero is defined 0.8m below the workplane (Tzempelikos & Shen 2013)
   
if (a > 0 && abs(y) < 90 && overshoot == 0){     // The sun is up and is incident on this office.

  h_i[1] = roller_up(h_i[0],h_i[1]); // roller_up is a function for open loop control.
       
}

else if (a > 0 && abs(y) < 90 && overshoot == 1){     // The sun is upbut it is a bright day: open loop overshot 2000 lx. Now closed loop.

  h_i[1] = roller_feed(h_i[0]);         // roller_feed is a function for closed loop control

}

else if(a > 0 && y > 90){ // The sun is up but not incident on this office. Closed loop control

 h_i[1] = roller_feed(h_i[0]);      // Closed loop control
 
}


else{                            // The sun has set.
  h_i[1] = roller_down(h_i[0]);  // Returns the new position of the shade at 0 m. 
}

     h_i[0] = h_i[1];


delay(600000); // wait 10 min


}


/********************************************
*********************************************
Functions
*********************************************
********************************************/



void digitalClockDisplay(){
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(" ");
  Serial.print(month());
  Serial.print(" ");
  Serial.print(year());
  Serial.print(" ");
}

void printDigits(int digits){
  // utility function for clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*********************************************
**   Roller Up
*********************************************/

float roller_up(float h0, float h1){
   n_step  = (h1 - h0) * (1 + k_var) / (r * m_step);  // Calculate the number of steps to move the blind to the new position based on its previous position.
 
   if (n_step < 0){  // Should the motor move backward or forwards?
   n_step = abs(n_step);
   motor_dir = 0; //BACKWARD 
   }
   else{
   motor_dir = 1; // FORWARD 
   }
     
    move_motor(h1,n_step,motor_dir); // function to move the motor n steps
  
  if (lx_meas > max_lx){  // Did the motor overshoot 2000 lx?
    overshoot = 1;
   
     h1 = roller_feed(h1);     //Switch to close loop control
   
  }
  
  
  if (lx_meas < 250 && hour() >= 9){ // Is the day overcast and the lx is very low when using open loop? Is it past 9am?
   overshoot = 1;
   
       h1 = roller_feed(h1);     //Switch to close loop control  
    
  }
  
  
   return h1;
 
}

/*********************************************
**   Roller Down
*********************************************/


float roller_down(float h0){
   n_step  = h0 * (1 + k_var) / (r * m_step);  //Return the blind to the zeroeth position; close the blind fully.
  myMotor->step(n_step, BACKWARD, DOUBLE);
  float h1 = 0;
  return h1;
  overshoot = 0; // the sun has gone down therefore we reset the overshoot
}

/*********************************************
**   Roller Feedback Loop
*********************************************/

float roller_feed(float h1){

      n_step  = k_step * (1 + k_var) / (r * m_step);  // Move the blind 2cm until the light is satisfied.  
      
         if (lx_meas > max_lx){   // move the shade down.
   
     while (lx_meas > max_lx && h1 > wp){    

      motor_dir = 0;      
      move_motor(h1,n_step,motor_dir);
      h1 = h1 - k_step;
  }
  
   }
   else{ // move the shade up.
     
        while (lx_meas < max_lx && h1 < h_max){    

      motor_dir = 1;    
      move_motor(h1, n_step,motor_dir);
      h1 = h1 + k_step;
      
  }
  //This guarantees an overshoot will occur so we subtract one step
      motor_dir = 0;      
      move_motor(h1, n_step,motor_dir);
      h1 = h1 - k_step;
   }      
  
  return h1;
}

/*********************************************
**   Moving motor function
*********************************************/

void move_motor(float h0, float n_step, int motor_dir){    
  
      myMotor->step(n_step, motor_dir, DOUBLE);

  sensors_event_t event;  
  tsl.getEvent(&event);
  lx_meas = event.light; 
  digitalClockDisplay(); Serial.print(event.light); Serial.print(" lux"); Serial.print(" Height above wp: "); Serial.println(h0-wp);
  
}


