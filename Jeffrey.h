#ifndef JEFFREY_H_
#define JEFFREY_H_

class Jeffrey {
private:
	/*
	 * The number of degree to turn the (4" / 31.9cm) wheel forward one centimenter
	 *  Formula used : 360/31.918581360472
	 */
	static constexpr double WHEEL_DEGREE_CM = 11.2786968;
	/*
	 *Formula to turn 90 degrees:
	 * 2pi(w)/4 ∗ 360/(d(pi))​
	 * w = width of robot base​
	 * d = diameter of wheel
	 */
	static constexpr double TURN_90_ENCODER = 485;


	static constexpr double HAND_LEFT = 0;
	static constexpr double HAND_CENTER = 0;
	static constexpr double HAND_RIGHT = 0;

	static constexpr double HAND_OPEN = 180;
	static constexpr double HAND_CLOSED = 0;

	static constexpr double WEIGHT_FRONT = 0;
	static constexpr double WEIGHT_BACK = 170;


	Motor_Controller mc;
    Ultrasonic ultrasonic;


public:
    Jeffrey();
    ~Jeffrey();

    NiFpga_Status init(NiFpga_Session* myrio_session);
    void reset();

	void moveForwardCM(int cm, int speed);
	int alignWithWall(int speed, float calib);

	void moveToDistanceForward(int speed, float distance, bool verbose);
	void moveToDistanceBackward(int speed, float distance, bool verbose);

	void moveHandToBlock();
	void openHand();
	void closeHand();

	void weightFront();
	void weightBack();

	void rotate90dregees(int numberOf90degree);

};

#endif