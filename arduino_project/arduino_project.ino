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

const int pin_en_on_left = 24;
const int pin_en_on_right = 25;

const int pin_sensor_trigger = 23;
const int pin_sensor_echo    = 22;

volatile int speed_fwd_left = 255;
volatile int speed_fwd_right = 255;
volatile int speed_rev_left = 255;
volatile int speed_rev_right = 255;
volatile float speed_engine  = 0;
volatile int stop = 1;
volatile int fwd_stop = 0;

unsigned long sensorDelay = 0;


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
        stop = 0;
        
        
        if(msg.linear.z < 0 || msg.linear.z > 1) {
           speed_engine = 1;
        } else {
           speed_engine = msg.linear.z;
        }
        
        //backwards
        if(msg.linear.x < 0) {
            speed_rev_left = msg.linear.x * -1;
            speed_fwd_left = 0;
        } else {
            speed_fwd_left = msg.linear.x;
            speed_rev_left = 0;
        } 
        if(msg.linear.y < 0) {
            speed_rev_right = msg.linear.y * -1;
            speed_fwd_right = 0;
        } else {
            speed_rev_right = 0;
            speed_fwd_right = msg.linear.y;
        }
}


void initPinsMotor() {
  //init all pins for the engine
  //do we need pinMode?
  pinMode(pin_fwd_left, OUTPUT);
  pinMode(pin_fwd_right, OUTPUT);
  pinMode(pin_rev_left, OUTPUT);
  pinMode(pin_rev_right, OUTPUT);
  pinMode(pin_sensor_trigger, OUTPUT);
  pinMode(pin_sensor_echo, INPUT);
  pinMode(pin_en_on_left, OUTPUT);
  pinMode(pin_en_on_right, OUTPUT);
  
  digitalWrite(pin_en_on_left, HIGH);
  digitalWrite(pin_en_on_right, HIGH);
  
  
  
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
  stop = 1;
}


void loop() {

  nh.spinOnce();
  if (millis() > sensorDelay ) {
    digitalWrite(pin_sensor_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_sensor_trigger, LOW);
    long distance = pulseIn(pin_sensor_echo, HIGH);    
    if (distance <= 1200) {
      fwd_stop = 1;
    }
    
    sensorDelay = millis() + 50;
    
    if(stop) {
      digitalWrite(pin_rev_left, ENGINE_OFF);
      digitalWrite(pin_fwd_right, ENGINE_OFF);
      digitalWrite(pin_fwd_left, ENGINE_OFF);
      digitalWrite(pin_rev_right, ENGINE_OFF);
    } else {
      if(!fwd_stop) {          
        analogWrite(pin_fwd_left, speed_engine * speed_fwd_left);    
        analogWrite(pin_fwd_right, speed_engine * speed_fwd_right);
      } else {
        digitalWrite(pin_fwd_right, ENGINE_OFF);
        digitalWrite(pin_fwd_left, ENGINE_OFF);      
      }
      analogWrite(pin_rev_left, speed_engine * speed_rev_left);
      analogWrite(pin_rev_right, speed_engine * speed_rev_right);
      fwd_stop = 0;
    }
  }
  
  
}
