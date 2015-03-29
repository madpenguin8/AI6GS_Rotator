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

// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
// wrappers for the first motor!
void forwardstep1() {  
	motor1.onestep(BACKWARD, SINGLE);
}
void backwardstep1() {  
	motor1.onestep(FORWARD, SINGLE);
}
// wrappers for the second motor!
void forwardstep2() {  
	motor2.onestep(FORWARD, SINGLE);
}
void backwardstep2() {  
	motor2.onestep(BACKWARD, SINGLE);
}

// Motor shield has two motor ports, now we'll wrap them in an AccelStepper object
AccelStepper azStepper(forwardstep1, backwardstep1);
AccelStepper elStepper(forwardstep2, backwardstep2);

boolean hadFirstGet = false;

void setup() {
	Bridge.begin();
  
	server.noListenOnLocalhost();
	server.begin();

	// Initialize motors, tweak for max performance
	azStepper.setMaxSpeed(100.0);
	azStepper.setAcceleration(100.0);
	azStepper.moveTo(0);
    
	elStepper.setMaxSpeed(100.0);
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
				if(client.available()){
					char received = client.read();
					//  This helps keep the steppers running between commands
					azStepper.run();
					elStepper.run();

					// Keep building string until we get a new line
					if(received != '\n'){
						buff[commandLen] = received;
						commandLen++;
					}
			
					// Got a command, figure out what to do with it 
					if(received == '\n'){						
						// Handle get command
						if(buff[0] == 'p'){
							// Calculate current position for a response to get 
							float currentAz = 360.0 * (azStepper.currentPosition()  / STEPS_PER_REV);
							float currentEl = 360.0 * (elStepper.currentPosition()  / STEPS_PER_REV);
							// Respond with our current position
							client.print("\r");
							client.println(currentAz);
							client.print("\r");
							client.println(currentEl);
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
							int el = commandedEl * (STEPS_PER_REV / 360.0);
							azStepper.moveTo(az);
							elStepper.moveTo(el);
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
