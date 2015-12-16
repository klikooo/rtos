#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define ENGINE_OFF HIGH


/**
CTC = Clear timer on compare mode
PWM = pulse witdh modulation mode

TCCR = Timer counter control register
  CS = Clock Set Bits (for prescalar)
  WGM = Wave Generation mode
  COM = Compare Output mode for Compare Unit A/B
  FOC = Force Output Compare for Compare Unit A/B (only in non-PWM mode)
TCNT = Timer Count register (high and low)
TIMSK = Timer Interrupt Mask register
  TOIE = Timer Overflow Interrupt Enable flag (interrupt when overflow)
  OCIE = Output Compare Interrupt Enable flag (intertupt when TCNT >= OCR, in CTC mode)
TIFR = Timer Interrupt Flag register
  OCF = Output Compare A/B Match Flag (set to 1 when TCNT >= OCR, in CTC mode)
  TOV = Timer Overflow flag (set to 1 when timer overflows)
OCR = Output Compare Register (reset when timer hits value of OCR)


*/

const int pin_fwd_left = 6; //Timer 4
const int pin_fwd_right = 2; //Timer 3
const int pin_rev_left = 7;  //Timer 4
const int pin_rev_right = 3; //Timer 3

volatile int speed_fwd_left = 0;
volatile int speed_fwd_right = 0;
volatile int speed_rev_left = 0;
volatile int speed_rev_right = 0;


void messageTwist(const geometry_msgs::Twist& msg);

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageTwist);

// speed : 0 is top, 255 is min?
void messageTwist(const geometry_msgs::Twist& msg) {
        speed_fwd_left = msg.linear.x;
        speed_fwd_right = msg.linear.y;
        
        analogWrite(pin_fwd_left, speed_fwd_left);
        analogWrite(pin_fwd_right, speed_fwd_right);
/*	//determine if we want to go backwards or forwards
	if(msg.angular.x == 1) {
		//forward
   		controlEngine(pin_fwd_left, msg.linear.x);
   		controlEngine(pin_fwd_right, msg.linear.y);
   		digitalWrite(pin_rev_left, ENGINE_OFF);
   		digitalWrite(pin_rev_right, ENGINE_OFF);
	} else if(msg.angular.y == 1) {
		//backward
   		controlEngine(pin_rev_left, msg.linear.x);
   		controlEngine(pin_rev_right, msg.linear.y); 
   		digitalWrite(pin_fwd_left, ENGINE_OFF);
   		digitalWrite(pin_fwd_right, ENGINE_OFF);
   	} else if(msg.angular.z == 0) {
                //stop
                digitalWrite(pin_fwd_left, ENGINE_OFF);
                digitalWrite(pin_fwd_right, ENGINE_OFF);
                digitalWrite(pin_rev_left, ENGINE_OFF);
                digitalWrite(pin_rev_right, ENGINE_OFF);
        } else {
                analogWrite(pin_fwd_left, msg.linear.x);
                analogWrite(pin_fwd_right, msg.linear.y);
                digitalWrite(pin_rev_left, ENGINE_OFF);
                digitalWrite(pin_rev_right, ENGINE_OFF);
        }               */
}

//inline?? for speed?
inline void controlEngine(const int pin, const int velocity) {
  //add timers so we can control velocity(using pwm) also make interrupt based
  analogWrite(pin, velocity);
}

void initPinsMotor() {
  //init all pins for the engine
  //do we need pinMode?
  pinMode(pin_fwd_left, OUTPUT);
  pinMode(pin_fwd_right, OUTPUT);
  pinMode(pin_rev_left, OUTPUT);
  pinMode(pin_rev_right, OUTPUT);
  digitalWrite(pin_fwd_left,ENGINE_OFF);
  digitalWrite(pin_fwd_right, ENGINE_OFF);
  digitalWrite(pin_rev_left, ENGINE_OFF);
  digitalWrite(pin_rev_right, ENGINE_OFF);  
}




void setup() {
  
  noInterrupts();
  
  initPinsMotor();
  nh.initNode();
  nh.subscribe(sub);
  
  interrupts();
  
}


void loop()
{
  nh.spinOnce();
  delay(1000);
}
