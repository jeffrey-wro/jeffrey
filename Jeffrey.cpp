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

NiFpga_Status Jeffrey::init(NiFpga_Session* myrio_session){

	NiFpga_Status status = mc.init(myrio_session);
	mc.controllerEnable(DC);
	mc.controllerEnable(SERVO);

	int volt = mc.readBatteryVoltage(1);
	printf("Battery: %.02fv\n\n", volt/100.0);

	return status;
}

void Jeffrey::reset(){
	Utils::waitFor(2);
	mc.controllerReset(DC);
}


void Jeffrey::moveForwardCM(int cm, int speed=100){

	int degree = WHEEL_DEGREE_CM * cm;
	int degree1 = degree+mc.readEncoderDegrees(DC, DC_1);
	int degree2 = -degree+mc.readEncoderDegrees(DC, DC_2);

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

int Jeffrey::servo(){

		mc.setServoSpeeds(SERVO, 250, 250, 250, 250, 250, 250);
	Utils::waitFor(1);

	while(1){

		printf("%d\n", mc.readServoPosition(SERVO, SERVO_1));
		//mc.setServoPositions(SERVO, 180, 180, 180, 180, 180, 180);
		//Utils::waitForMicro(1000000);
		//mc.setServoPositions(SERVO, 0, 0, 0, 0, 0, 0);
		//Utils::waitForMicro(1000000);

		mc.setServoPosition(SERVO, SERVO_1, 180);
    
		Utils::waitForMicro(1000000);

		/*int servo = 0;
		int move = -255;
		printf("QUIT = 0\n");
		printf("Hand = 1\n");
		printf("Elbow = 2\n");
		printf("Enter which servo to move:\n");
		scanf("%d", &servo);
		if(servo == 0)
			break;
		int pos = mc.readServoPosition(SERVO, servo);
		printf("Curent servo pos: %d \n", pos);
		printf("Enter how much to move (%d,%d)\n", -1*pos, 180-pos);
		scanf("%d", &move);
		while(move < -1*pos || move > 180-pos ){
			printf("Invalid input (%d,%d)\n", -1*pos, 180-pos);
			scanf("%d", &move);
		}
		printf("Moving to %d\n", move+pos);
		mc.setServoPosition(SERVO, servo, move+pos);*/

	}




}



//make the robot go to the block when already aligned 
void Jeffrey::moveToDistanceForward(int speed, float distance){


    float leftDistance;
    float rightDistance;
    float lavg = 0;
	float ravg = 0;
    std::vector<float> lds;
    std::vector<float> rds;

	mc.setMotorSpeeds(DC, -speed, speed);

    do {

       	do{
	       	leftDistance = ultrasonic.getDistance(Ultrasonic::FRONT_RIGHT);
	    }while(leftDistance < 0);
	    do{
       		rightDistance = ultrasonic.getDistance(Ultrasonic::FRONT_LEFT);
	    }while(rightDistance < 0);

	    if (lds.size() >= 10 ){
	    	lds.erase(lds.begin());
	    	rds.erase(rds.begin());
	    }

	    lds.push_back(leftDistance);
		rds.push_back(rightDistance);

		lavg = leftDistance;
		ravg = rightDistance;

		for(unsigned int i=0; i< lds.size(); i++){
			lavg += lds[i];
			ravg += rds[i];

		} 
		lavg/=lds.size()+1;
		ravg/=rds.size()+1;


printf("%f %f\n", lavg, ravg);
fflush(stdout);

	}while(lavg > distance && ravg > distance );

	mc.setMotorSpeeds(DC, 0, 0);
}

//make the robot go to the block when already aligned 
void Jeffrey::moveToDistanceBackward(int speed, float distance){


    float leftDistance;
    float rightDistance;

	mc.setMotorSpeeds(DC, speed, -speed);

    do {

       	do{
	       	leftDistance = ultrasonic.getDistance(Ultrasonic::FRONT_RIGHT);
       		rightDistance = ultrasonic.getDistance(Ultrasonic::FRONT_LEFT);
	    }while(leftDistance < 0 || rightDistance < 0);

printf("%f %f\n", leftDistance, rightDistance);
fflush(stdout);

	}while(leftDistance < distance && rightDistance < distance );

	mc.setMotorSpeeds(DC, 0, 0);
}

//align the hane with the block
// currently assums that the robot is centered to the block
void Jeffrey::moveHandToBlock(){

	//mc.setServoPosition(SERVO, SERVO_4, HAND_CENTER);
	mc.setCRServoState(SERVO, CR_SERVO_1, 100);
	Utils::waitFor(4);
	mc.setCRServoState(SERVO, CR_SERVO_1, -100);
	Utils::waitFor(1);
	mc.setCRServoState(SERVO, CR_SERVO_1, 0);
}

//open the hand
void Jeffrey::openHand(){

	mc.setServoPosition(SERVO, SERVO_2, HAND_OPEN);
	Utils::waitFor(1);
}

//open the hand
void Jeffrey::closeHand(){
	
	mc.setServoPosition(SERVO, SERVO_2,HAND_CLOSED);
	Utils::waitFor(1);

}

void Jeffrey::weightFront(){
	mc.setServoPosition(SERVO,SERVO_4,WEIGHT_FRONT);
	Utils::waitFor(1);
}

void Jeffrey::weightBack(){
	mc.setServoPosition(SERVO,SERVO_4,WEIGHT_BACK);
	Utils::waitFor(1);
}

void Jeffrey::rotate180dregees(){
	int speed = 200;

	int rigthCount = 0;

	mc.resetEncoders(DC);

	for(int i=0; i<2; i++){

		rigthCount -= 485;
		mc.setMotorDegrees(DC, speed, 0, speed, rigthCount);

	}

	Utils::waitFor(1);
	
}