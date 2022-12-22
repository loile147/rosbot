
/////////////// Luan Van Tot Nghiep /////////////////
//*************************************************//

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
//#include <std_msgs/Int16.h>

#include <PID_v1.h>
	

//////////// Motor Controller Variables and Constants ///////////////
// char cmd = 's';
// int i = 0;

#define LOOPTIME 	100	
unsigned long lastMillis = 0;

//Encoder output to Arduino Interupt pin.
#define ENC_MOTOR_LEFT_A 2
#define ENC_MOTOR_RIGHT_A 3

//Other encoder output to Arduino to keep track of wheel direction.
#define ENC_MOTOR_LEFT_B 6
#define ENC_MOTOR_RIGHT_B 11

//Motor left connected to LM298
#define in1   4  
#define in2   5
#define enA   9

//Motor right connected to LM298
#define in3   7   
#define in4   8
#define enB   10

///////////////// Robot specific constants ////////////////////
const double WHEEL_RADIUS = 0.033;			//Wheel radius in meters
const double WHEEL_BASE = 0.2;				//Distance between 2 wheels in meters
const double ENCODER_PER_ROTATION = 980;		//Encoder ticks or counts per rotation
const double SPEED_MAX = 0.18;				//Max speed in m/s
const double SPEED_MIN = 0.08;				//Min speed in m/s
const int LINEAR_PARAM_LEFT[] = {340, 21}; // PWM = K*v + b. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function)
const int LINEAR_PARAM_RIGHT[] = {330, 27}; // PWM = K*v + b. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function)

double linear_speed_req = 0;				//Desired linear speed for the robot, in m/s
double angular_speed_req = 0;				//Desired angular speed for the robot, in rad/s

double speed_left_req = 0;					//Desired speed for left wheel in m/s
double speed_left_act = 0;					//Actual speed for left wheel in m/s
double speed_left_cmd =0;					//Command speed for left wheel in m/s

double speed_right_req = 0;					//Desired speed for right wheel in m/s
double speed_right_act = 0;					//Actual speed for right wheel in m/s
double speed_right_cmd =0;					//Command speed for right wheel in m/s

int pwm_leftMotor = 0;
int pwm_rightMotor = 0;
volatile float pos_left = 0;
volatile float pos_right = 0 ;

/////////////////PID Parameters //////////////////
const double PID_LEFT_PARAM[] = {2, 20, 0};		//Kp, Ki and Kd for left motor PID
const double PID_RIGHT_PARAM[] = {2, 20, 0};		//Kp, Ki and Kd for right motor PID

PID PID_leftMotor(&speed_left_act, &speed_left_cmd, &speed_left_req, PID_LEFT_PARAM[0], PID_LEFT_PARAM[1], PID_LEFT_PARAM[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_right_act, &speed_right_cmd, &speed_right_req, PID_RIGHT_PARAM[0],PID_RIGHT_PARAM[1], PID_RIGHT_PARAM[2], DIRECT);   //Setting up the PID for right motor

ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd(const geometry_msgs::Twist& vel){
	//Handle velocity and angular value received from host
	linear_speed_req = vel.linear.x;
	angular_speed_req = vel.angular.z;
	// if (abs(linear_speed_req) < SPEED_MIN)
	// 	linear_speed_req = 0;
	// else
	linear_speed_req = constrain(linear_speed_req, -SPEED_MAX, SPEED_MAX);
	angular_speed_req = constrain(angular_speed_req, -3, 3);

  	if(abs(linear_speed_req) < 0.02 && abs(angular_speed_req) < 0.8){
		speed_left_req = -sgn(angular_speed_req)*SPEED_MIN;
		speed_right_req = sgn(angular_speed_req)*SPEED_MIN;
  	} 
	else{
		speed_left_req = linear_speed_req - angular_speed_req*(WHEEL_BASE/2);
		if(abs(speed_left_req) < SPEED_MIN)
			speed_left_req = sgn(speed_left_req)*SPEED_MIN;
		else
			speed_left_req = constrain(speed_left_req, -SPEED_MAX, SPEED_MAX); 

		speed_right_req = linear_speed_req + angular_speed_req*(WHEEL_BASE/2);
		if(abs(speed_right_req) < SPEED_MIN)
			speed_right_req = sgn(speed_right_req)*SPEED_MIN;
		else
			speed_right_req = constrain(speed_right_req, -SPEED_MAX, SPEED_MAX);
	}
}

ros::Subscriber<geometry_msgs::Twist> vel_sub("cmd_vel", handle_cmd);   	//create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);

void setup(){
	//setup ros node
	nh.initNode();
	// nh.getHardware()->setBaud(115200);
	nh.subscribe(vel_sub); 
	nh.advertise(speed_pub);
	//----------------//
	
	//Setting PID parameters
	PID_leftMotor.SetSampleTime(95);
	PID_rightMotor.SetSampleTime(95);
	PID_leftMotor.SetOutputLimits(-SPEED_MAX, SPEED_MAX);
	PID_rightMotor.SetOutputLimits(-SPEED_MAX, SPEED_MAX);
	PID_leftMotor.SetMode(AUTOMATIC);
	PID_rightMotor.SetMode(AUTOMATIC);
	
	//Define the rotary encoder for left motor
	pinMode(ENC_MOTOR_LEFT_A, INPUT_PULLUP);
	pinMode(ENC_MOTOR_LEFT_B, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_LEFT_A), ISRLeftMotor, RISING);
	
	//Define the rotary encoder for right motor
	pinMode(ENC_MOTOR_RIGHT_A, INPUT_PULLUP);
	pinMode(ENC_MOTOR_RIGHT_B, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(ENC_MOTOR_RIGHT_A), ISRRightMotor, RISING);
	
	//Set motor speed to zero
	pinMode(enA,OUTPUT);  
	pinMode(in1,OUTPUT);
	pinMode(in2,OUTPUT);
	  
	pinMode(enB,OUTPUT);
	pinMode(in3,OUTPUT);
	pinMode(in4,OUTPUT);
	
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
	
	analogWrite(enA, 0); 
	analogWrite(enB, 0);
  
	//Serial.begin(115200);
	
}

void loop(){
	nh.spinOnce();
	//--------------------
	/* if (Serial.available()>0){
		control(cmd);
		print_vel_ang(linear_speed_req, angular_speed_req);
		delay(200);
		handle_cmd();			
	} */
	
	//-------------------

	if((millis()-lastMillis) >= LOOPTIME){
		lastMillis = millis();
		
		
		if(abs(pos_left) < 5){
			speed_left_act = 0;
		}
		else {
			speed_left_act = ((pos_left/ENCODER_PER_ROTATION)*2*PI)*(1000/LOOPTIME)*WHEEL_RADIUS;
		}
		
		if(abs(pos_right) < 5){
			speed_right_act = 0;
		}
		else {
			speed_right_act = ((pos_right/ENCODER_PER_ROTATION)*2*PI)*(1000/LOOPTIME)*WHEEL_RADIUS;
		}
		
		/* Serial.print("pos_left: ");
		Serial.print(pos_left);
		Serial.print("	pos_right: ");
		Serial.println(pos_right); */

		/* Serial.print("vel_x: ");
		Serial.print(linear_speed_req);
		Serial.print("	vel_z: ");
		Serial.println(angular_speed_req); */

		pos_left = 0;
		pos_right = 0;
		
		////////////////-LEFT-////////////////
		
		PID_leftMotor.Compute();
		speed_left_cmd = constrain(speed_left_cmd, -SPEED_MAX, SPEED_MAX);
		
		pwm_leftMotor = constrain((LINEAR_PARAM_LEFT[0]*speed_left_req + sgn(speed_left_req)*LINEAR_PARAM_LEFT[1]), -255, 255);
    
		if (speed_left_req == 0){
			//set speed to 0, stop
			digitalWrite(in1, LOW);
			digitalWrite(in2, LOW);
		}
		else if (pwm_leftMotor > 0){
			//go forward
			digitalWrite(in1, LOW);
			digitalWrite(in2, HIGH);
			//set speed abs(pwm_leftMotor)
			analogWrite(enA, abs(pwm_leftMotor));
		}
		else {
			//go backward
			digitalWrite(in1, HIGH);
			digitalWrite(in2, LOW);
			//set speed abs(pwm_leftMotor)
			analogWrite(enA, abs(pwm_leftMotor));
		}
		
		////////////////-RIGHT-////////////////
		
		PID_rightMotor.Compute();
		speed_right_cmd = constrain(speed_right_cmd, -SPEED_MAX, SPEED_MAX);
		
		pwm_rightMotor = constrain((LINEAR_PARAM_RIGHT[0]*speed_right_req + sgn(speed_right_req)*LINEAR_PARAM_RIGHT[1]), -255, 255);
    
		if (speed_right_req == 0){
			//set speed to 0, stop
			digitalWrite(in3, LOW);
			digitalWrite(in4, LOW);
		}
		else if (pwm_rightMotor > 0){
			//go forward
			digitalWrite(in3, HIGH);
			digitalWrite(in4, LOW);
			//set speed abs(pwm_rightMotor)
			analogWrite(enB, abs(pwm_rightMotor));
		}
		else {
			//go backward
			digitalWrite(in3, LOW);
			digitalWrite(in4, HIGH);
			//set speed abs(pwm_rightMotor)
			analogWrite(enB, abs(pwm_rightMotor));
		}
		/* if (i == 10){
			Serial.print("vel_left: ");
			Serial.print(speed_left_act);
			Serial.print("	vel_right: ");
			Serial.println(speed_right_act);
			i = 0;
		}
		else i++; */

		publishSpeed(LOOPTIME);
	}
}

// Publish speed
void publishSpeed(double time) {
	speed_msg.header.stamp = nh.now();
	speed_msg.vector.x = speed_left_act;
	speed_msg.vector.y = speed_right_act;
	speed_msg.vector.z = time/1000;
	speed_pub.publish(&speed_msg);
	nh.spinOnce();
	nh.loginfo("Publishing odometry");
}

// Left motor encoder counter
void ISRLeftMotor(){
	if (digitalRead(ENC_MOTOR_LEFT_B) == LOW)
		pos_left++;
	else pos_left--;
}

// Right motor encoder counter
void ISRRightMotor() {
	if (digitalRead(ENC_MOTOR_RIGHT_B) == LOW)
		pos_right--;
	else pos_right++;
}

template <class T> 
int sgn(T val){
	return ((T(0) < val) - (val < T(0)));
}



/////////////////////////////////////////////////
/////////////////////////////////////////////////

/* void control (char cmd){

	cmd = Serial.read();
	switch (cmd){
		case 'f': move(); break;
		case 's': stop(); break;
		case 'w': vel_increase(linear_speed_req,SPEED_MAX); break;
		case 'x': vel_decrease(linear_speed_req,SPEED_MAX); break;
		case 'd': ang_decrease(angular_speed_req,PI); break;
		case 'a': ang_increase(linear_speed_req,PI); break;
		default: stop();
	}
} */

/* void move(){
	analogWrite(enA,pwm_rightMotor);
	analogWrite(enB,pwm_leftMotor);
}

void stop(){
	analogWrite(enA,0);
	analogWrite(enB,0);
	linear_speed_req =0;
	angular_speed_req =0;
}
void vel_increase(double &vel_cmd, const double &vel_max){
	Serial.print("tang vel: ");
	vel_cmd += 0.010;
	vel_cmd = (vel_cmd > SPEED_MAX)? SPEED_MAX : vel_cmd;
	Serial.println(vel_cmd);
}

void vel_decrease(double &vel_cmd, const double &vel_max){
	vel_cmd -= 0.010;
	vel_cmd = (vel_cmd < (- vel_max))? (- vel_max) : vel_cmd;
}

void ang_increase(double &ang_cmd, const double &ang_max){
	ang_cmd += PI/4;
	ang_cmd = (ang_cmd > ang_max)? ang_max : ang_cmd;
}

void ang_decrease(double &ang_cmd, const double &ang_max){
	ang_cmd -= PI/4;
	ang_cmd = (ang_cmd < (-ang_max))? (-ang_max) : ang_cmd;
}

void print_vel_ang(double &vel_cmd, double &ang_cmd){
	Serial.print("Velocity: ");
	Serial.print(vel_cmd,3);
	Serial.print(", Angular: ");
	Serial.println(ang_cmd,3);
} */
