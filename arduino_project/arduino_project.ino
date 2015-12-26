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

const int pin_fwd_left = 6; //Timer 4  OCR4A
const int pin_fwd_right = 2; //Timer 3 OCR3B
const int pin_rev_left = 7;  //Timer 4 OCR4B
const int pin_rev_right = 3; //Timer 3 OCR3C

volatile int speed_fwd_left = 255;
volatile int speed_fwd_right = 255;
volatile int speed_rev_left = 255;
volatile int speed_rev_right = 255;
volatile float speed_engine  = 0;


void messageTwist(const geometry_msgs::Twist& msg);

class NewHardware: public ArduinoHardware {
  public: NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

ros::NodeHandle_<NewHardware> nh;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageTwist);

// speed : 0 is top, 255 is min?
void messageTwist(const geometry_msgs::Twist& msg) {
        //SET TIMER COUNT TO 0
        TCNT5 = 0;  
  
        if(msg.linear.z < 0 || msg.linear.z > 1) {
           speed_engine = 1;
        } else {
           speed_engine = msg.linear.z;
        }
        
        //backwards
        if(msg.linear.x < 0) {
            digitalWrite(pin_fwd_left, ENGINE_OFF);
            analogWrite(pin_rev_left, speed_engine * -1 * msg.linear.x);          
        } else {
            digitalWrite(pin_rev_left, ENGINE_OFF);
            analogWrite(pin_fwd_left, speed_engine * msg.linear.x);
        } 
        if(msg.linear.y < 0) {
            digitalWrite(pin_fwd_right, ENGINE_OFF);
            analogWrite(pin_rev_right, speed_engine * -1 * msg.linear.y);
        } else {
            digitalWrite(pin_rev_right, ENGINE_OFF);
            analogWrite(pin_fwd_right, speed_engine * msg.linear.y);
        }
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
  
  //timer
  //prescaler =256, time = 1s, 16MHz; 62500 ticks needed
  TCNT5 =0;
  TCCR5A = 0;
  TCCR5B = 0;
  OCR5A = 62500;
 // OCR5B = 100; use for pwm?
  TCCR5A |=  (1 << WGM12); //ctc mode
  TCCR5B |=  (1 << CS12); //256-prescalar
  TIMSK5 |= (1 << OCIE5A); //enable interrupts
   
  
  interrupts();
  
}

ISR(TIMER5_COMPA_vect) {
  speed_engine = 0;
}


void loop() {

  nh.spinOnce();
  delay(500);
}
