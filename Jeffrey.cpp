#include "MyRio.h"
#include "I2C.h"
#include "Motor_Controller.h"
#include "Utils.h"
#include "Ultrasonic.h"
#include "vector"

#include "Jeffrey.h"


Jeffrey::Jeffrey(){	

}
Jeffrey::~Jeffrey(){

}


/*
 * Initialize all of the robot functions
 * Return the fpga status
 */
NiFpga_Status Jeffrey::init(NiFpga_Session* myrio_session){

	NiFpga_Status status = mc.init(myrio_session);
	mc.controllerEnable(DC);
	mc.controllerEnable(SERVO);
	//mc.setMotorInvert(DC, DC_2, 1);

	int volt = mc.readBatteryVoltage(1);
	printf("Battery: %.02fv\n\n", volt/100.0);

	return status;
}

/*
 * Reset all of the robot functions
 */
void Jeffrey::reset(){
	Utils::waitFor(2);
	mc.controllerReset(DC);
	mc.controllerReset(SERVO);
	Utils::waitFor(2);
}


/*
 * Make the robot move a certain amount of cm
 */
void Jeffrey::moveForwardCM(int cm, int speed=100){

	int degree = WHEEL_DEGREE_CM * cm;
	int degree1 = degree+mc.readEncoderDegrees(DC, DC_1);
	int degree2 = -degree+mc.readEncoderDegrees(DC, DC_2);

	mc.resetEncoders(DC);
	Utils::waitFor(2);

	mc.setMotorDegrees(DC, speed, degree1, speed, degree2);

	//TODO maybe make a timeout
	while(mc.readEncoderDegrees(DC, DC_1) != degree1 ){
		Utils::waitForMicro(50000);
		printf("%d\n", degree1);
		printf("%ld\n\n", mc.readEncoderDegrees(DC, DC_1));
	}
	Utils::waitForMicro(50000);
	while(mc.readEncoderDegrees(DC, DC_2) != degree2 ){
		Utils::waitForMicro(50000);
		printf("%d\n", degree2);
		printf("%ld\n\n", mc.readEncoderDegrees(DC, DC_2));
	}

}

/*
 * Make the robot perpendicular to a flat surface
 * TODO: uses an average of the last ten measurement to remove invalid measurement from reflection on adjacent objects
 */
int Jeffrey::alignWithWall(int speed=25, float calib=0.75){
	float leftDistance;
    float rightDistance;

    while(1) {

        leftDistance = ultrasonic.getDistance(Ultrasonic::FRONT_RIGHT);
        rightDistance = ultrasonic.getDistance(Ultrasonic::FRONT_LEFT);

        if(leftDistance >= 400 || leftDistance <= 2 || rightDistance >= 400 || rightDistance <= 2 ){
        	
			mc.setMotorSpeeds(DC, 0, 0);
        	printf("One sensor is Out of range\n");

        }else if(leftDistance - calib > rightDistance){

        	printf("Turning right\n");
			mc.setMotorSpeeds(DC, speed, speed);

        }else  if(leftDistance < rightDistance - calib){

        	printf("Turning left\n");
			mc.setMotorSpeeds(DC, -speed, -speed);

        }else{
			
			mc.setMotorSpeeds(DC, 0, 0);
        	printf("Robot is perpendicular to the surface\n");

        }


		Utils::waitForMicro(100000);
    }
}



/*
 * Make the robot go to forward toward a flat object.
 * The robot must already be perpendicular to the surface.
 * It uses an average of the last ten measurement to remove invalid measurement from reflection on adjacent objects
 */
void Jeffrey::moveToDistanceForward(int speed, float distance, bool verbose){


    float leftDistance;
    float rightDistance;
    std::vector<float> lds;
    std::vector<float> rds;
    float leftAvg = 0;
	float rightAvg = 0;

	mc.setMotorSpeeds(DC, -speed, speed);

    do {

       	do{ leftDistance = ultrasonic.getDistance(Ultrasonic::FRONT_RIGHT); }while(leftDistance < 0);
	    do{ rightDistance = ultrasonic.getDistance(Ultrasonic::FRONT_LEFT); }while(rightDistance < 0);

	    //only keep the last 10 measurement
	    if (lds.size() >= 10 ){
	    	lds.erase(lds.begin());
	    	rds.erase(rds.begin());
	    }
	    lds.push_back(leftDistance);
		rds.push_back(rightDistance);



		leftAvg = leftDistance;
		rightAvg = rightDistance;

		for(unsigned int i=0; i< lds.size(); i++){
			leftAvg += lds[i];
			rightAvg += rds[i];
		} 

		leftAvg/=lds.size()+1;
		rightAvg/=rds.size()+1;

		if(verbose){
			//printf("Distance (L,R): %f, %f\n", leftAvg, rightAvg);
			fflush(stdout);
		}


	}while(leftAvg > distance && rightAvg > distance );

	mc.setMotorSpeeds(DC, 0, 0);
}

/*
 * Make the robot go to backward toward a flat object.
 * The robot must already be perpendicular to the surface.
 * It uses an average of the last ten measurement to remove invalid measurement from reflection on adjacent objects
 */
void Jeffrey::moveToDistanceBackward(int speed, float distance, bool verbose){


    float leftDistance;
    float rightDistance;
    std::vector<float> lds;
    std::vector<float> rds;
    float leftAvg = 0;
	float rightAvg = 0;

	mc.setMotorSpeeds(DC, speed, -speed);

    do {

       	do{ leftDistance = ultrasonic.getDistance(Ultrasonic::FRONT_RIGHT); }while(leftDistance < 0);
	    do{ rightDistance = ultrasonic.getDistance(Ultrasonic::FRONT_LEFT); }while(rightDistance < 0);

	    //only keep the last 10 measurement
	    if (lds.size() >= 10 ){
	    	lds.erase(lds.begin());
	    	rds.erase(rds.begin());
	    }
	    lds.push_back(leftDistance);
		rds.push_back(rightDistance);



		leftAvg = leftDistance;
		rightAvg = rightDistance;

		for(unsigned int i=0; i< lds.size(); i++){
			leftAvg += lds[i];
			rightAvg += rds[i];
		} 

		leftAvg/=lds.size()+1;
		rightAvg/=rds.size()+1;

		if(verbose){
			//printf("Distance (L,R): %f, %f\n", leftAvg, rightAvg);
			fflush(stdout);
		}


	}while(leftDistance < distance && rightDistance < distance );

	mc.setMotorSpeeds(DC, 0, 0);
}



/*
 * Align the hand with the block
 * currently assumes that the robot is centered to the block
 * and uses a precalculated time to center hand
 * TODO: use a distance sensor (ei. infrared)
 */
void Jeffrey::moveHandToBlock(){

	//mc.setServoPosition(SERVO, SERVO_4, HAND_CENTER);
	mc.setCRServoState(SERVO, CR_SERVO_1, 100);
	Utils::waitFor(4);
	mc.setCRServoState(SERVO, CR_SERVO_1, -100);
	Utils::waitFor(1);
	mc.setCRServoState(SERVO, CR_SERVO_1, 0);
}


/*
 * Fully open the hand
 */
void Jeffrey::openHand(){

	printf("Open: %f\n", HAND_OPEN);

	mc.setServoPosition(SERVO, SERVO_2, HAND_OPEN);
	Utils::waitFor(2);
}

/*
 * Fully close the hand
 */
void Jeffrey::closeHand(){
	
	printf("Close: %f\n", HAND_CLOSED);

	mc.setServoPosition(SERVO, SERVO_2,HAND_CLOSED);
	Utils::waitFor(2);

}

/*
 * Move counter weight to the front
 */
void Jeffrey::weightFront(){
	mc.setServoPosition(SERVO,SERVO_4,WEIGHT_FRONT);
	Utils::waitFor(1);
}

/*
 * Move counter weight to the back
 */
void Jeffrey::weightBack(){
	mc.setServoPosition(SERVO,SERVO_4,WEIGHT_BACK);
	Utils::waitFor(1);
}

/*
 * Make the robot turn 90 for n times
 */
void Jeffrey::rotate90dregees(int numberOf90degree){


	printf("start delay");
	fflush(stdout);
	Utils::waitFor(5);

	int speed = 200;
	int delay = 3*numberOf90degree;
	int degrees = -TURN_90_ENCODER*numberOf90degree;

	printf("1");
	fflush(stdout);

	mc.resetEncoders(DC);
	Utils::waitFor(2);

	printf("2");
	fflush(stdout);

	mc.setMotorDegrees(DC, 0, 0, speed, degrees);
	Utils::waitFor(delay);

	printf("3");
	fflush(stdout);
	
	printf("end delay");
	fflush(stdout);
	Utils::waitFor(5);
}



void Jeffrey::test(){

	int numberOf90degree = 2;

	int speed = 200;
	int delay = 3*numberOf90degree;
	int degrees = -TURN_90_ENCODER*numberOf90degree;

	printf("1");
	fflush(stdout);

	mc.resetEncoders(DC);
	Utils::waitFor(2);

	printf("2");
	fflush(stdout);

	mc.setMotorDegrees(DC, 0, 0, speed, degrees);
	Utils::waitFor(delay);

	printf("3");
	fflush(stdout);

	// double HAND_CLOSED = 10;
	// double HAND_OPEN = 170;

	// printf("Close: %f\n", HAND_CLOSED);

	// mc.setServoPosition(SERVO, SERVO_2,HAND_CLOSED);
	// Utils::waitFor(2);

	// Utils::waitFor(2);

	// printf("Open: %f\n", HAND_OPEN);

	// mc.setServoPosition(SERVO, SERVO_2, HAND_OPEN);
	// Utils::waitFor(2);
}