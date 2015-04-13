#include <Bridge.h>
#include <YunServer.h>
#include <YunClient.h>
#include <AccelStepper.h>
#include <AFMotor.h>

// Steps per revolution of shaft, this can be greater than
// the motor steps for when gearboxes are used

#define STEPS_PER_REV 12000.0

// two stepper motors one on each port
AF_Stepper motor1(200, 1);
AF_Stepper motor2(200, 2);

// Listening port
#define PORT 4533

YunServer server(PORT);

// Tilt compensation
#include <Wire.h>
#include "BMA250.h"

BMA250 accel;

unsigned long previousMillis = 0;        // will store last time accel update
const long interval = 100;           // interval at which to blink (milliseconds)

float lastUpdates[10];
float averageAccel = 0.0;

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
  motor1.onestep(BACKWARD, DOUBLE);
}
void backwardstep1() {  
  motor1.onestep(FORWARD, DOUBLE);
}
// wrappers for the second motor!
void forwardstep2() {  
  motor2.onestep(FORWARD, DOUBLE);
}
void backwardstep2() {  
  motor2.onestep(BACKWARD, DOUBLE);
}

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
AccelStepper azStepper(forwardstep1, backwardstep1);
AccelStepper elStepper(forwardstep2, backwardstep2);

boolean hadFirstGet = false;

void setup() {
  Wire.begin();
  accel.begin(BMA250_range_2g, BMA250_update_time_64ms);//This sets up the BMA250 accelerometer

  Bridge.begin();
  
  server.noListenOnLocalhost();
  server.begin();

  // Initialize motors, tweak for max performance
  azStepper.setMaxSpeed(200.0);
  azStepper.setAcceleration(100.0);
  azStepper.moveTo(0);
    
  elStepper.setMaxSpeed(200.0);
  elStepper.setAcceleration(100.0);
  elStepper.moveTo(0);
}

void loop() {
	// Start the server 
  YunClient client = server.accept();

  if(client.connected()){
  // A buffer to hold our commands
  char buff[64];
  int commandLen = 0;

  while(client.connected()){
    unsigned long currentMillis = millis();
 
    if(currentMillis - previousMillis >= interval) {
      // save the last time we updated accel
      previousMillis = currentMillis;
      //updateAccel();
    } 

    if(client.available()){
      char received = client.read();
      //  This helps keep the steppers running between commands
      azStepper.run();
      elStepper.run();

      // Keep building string until we get a new line
      if(received != '\n'){
        if(commandLen < 64){
          buff[commandLen] = received;
          commandLen++;
        }
      }
			
      // Got a command, figure out what to do with it 
      if(received == '\n'){						
      // Handle get command
        if(buff[0] == 'p'){
          // Calculate current position for a response to get 	
          float currentAz = 360.0 * (azStepper.currentPosition()  / STEPS_PER_REV);
          float currentEl = 360.0 * (elStepper.currentPosition()  / STEPS_PER_REV);
          // Respond with our current position
          azStepper.run();
          elStepper.run();
          client.print("\r");
          client.println(currentAz);
          client.print("\r");
          client.println(currentEl);
          azStepper.run();
          elStepper.run();

        } 
        // Handle position commands
	if(buff[0] == 'P'){
	  // Split AZ and EL
	  char azStr[9];
	  char elStr[9];

          for(int i = 0; i < 8; i++){
            azStr[i] = buff[i + 1]; // Skip command char
            elStr[i] = buff[i + 9];
          }
							
          // Convert to floats
          float commandedAz = (float)atof(azStr);
          float commandedEl = (float)atof(elStr);
              
          // Command a new position 
          int az = commandedAz * (STEPS_PER_REV / 360.0);
          int el = (commandedEl + averageAccel) * (STEPS_PER_REV / 360.0);
          azStepper.moveTo(az);
          elStepper.moveTo(el);
          azStepper.run();
          elStepper.run();
        }
	
        for( int i = 0; i < 64; i++){
          buff[i] = 0;
        }
        commandLen = 0;
      }
    }
    else {
      // Keep those wheels turnin
      azStepper.run();
      elStepper.run();
    }
  }
  // Client has disconnected, prep for next connection
  commandLen = 0;
  client.stop();
  }
}


void updateAccel()
{
  accel.read();//This function gets new data from the accelerometer
  float x = accel.X / 1024.0;
  float y = accel.Y / 1024.0;
  float angle_z = atan2(-y,-x)*57.2957795;
  
  double sumOf_z = angle_z;
  
  for(int i = 0; i < 9; i++){
    sumOf_z = sumOf_z + lastUpdates[i];
    lastUpdates[i] = lastUpdates[i + 1];
  }
  
  lastUpdates[9] = angle_z;
  
  averageAccel = sumOf_z / 10;
}

