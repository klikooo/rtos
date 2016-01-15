/**
∗ Group number : 33
∗ Student1 : Rico Tubbing
∗ Rico Tubbing, 4254104
∗ Student2 : 4374657
* Stefan Breetveld, 
*/

#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>

#define ENGINE_OFF LOW


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

const int pin_fwd_right = 6; //Timer 4  OCR4A
const int pin_fwd_left = 2; //Timer 3 OCR3B
const int pin_rev_right = 7;  //Timer 4 OCR4B
const int pin_rev_left = 3; //Timer 3 OCR3C

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


//callback function prototype
void messageTwist(const geometry_msgs::Twist& msg);

//For connection over bluetooth
class NewHardware: public ArduinoHardware {
  public: NewHardware() : ArduinoHardware(&Serial1, 57600) {};
};

ros::NodeHandle_<NewHardware> nh;
//Subscribe to Twist messages on cmd_vel with the callback function
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageTwist);

//The callback function when we receive a message
void messageTwist(const geometry_msgs::Twist& msg) {
        //Set timer count to 0 and set stop to 0
        TCNT5 = 0;  
        stop = 0;
        
        //set the speed
        if(msg.linear.z < 0 || msg.linear.z > 1) {
           speed_engine = 1;
        } else {
           speed_engine = msg.linear.z;
        }
        
        //left backwards
        if(msg.linear.x < 0) {
            speed_rev_left = msg.linear.x * -1;
            speed_fwd_left = 0;
        //left forwards
        } else {  
            speed_fwd_left = msg.linear.x;
            speed_rev_left = 0;
        } 
        //right backwards
        if(msg.linear.y < 0) {
            speed_rev_right = msg.linear.y * -1;
            speed_fwd_right = 0;
        //right forwards 
        } else { 
            speed_rev_right = 0;
            speed_fwd_right = msg.linear.y;
        }
}


void initPins() {
  //set pinmode for the engine control
  pinMode(pin_fwd_left, OUTPUT);
  pinMode(pin_fwd_right, OUTPUT);
  pinMode(pin_rev_left, OUTPUT);
  pinMode(pin_rev_right, OUTPUT);
  //set pinmode for the sonser
  pinMode(pin_sensor_trigger, OUTPUT);
  pinMode(pin_sensor_echo, INPUT);
  
  
  //senable engine
  pinMode(pin_en_on_left, OUTPUT);
  pinMode(pin_en_on_right, OUTPUT);
  digitalWrite(pin_en_on_left, HIGH);
  digitalWrite(pin_en_on_right, HIGH);
}




void setup() {

  noInterrupts();
  //init all pins  
  initPins();
  //init out node and subscribe
  nh.initNode();
  nh.subscribe(sub);
  
  //timer
  //prescaler =256, time = 1s, 16MHz; 62500 ticks needed
  TCNT5 =0; //count to zero
  //clear control registers
  TCCR5A = 0;
  TCCR5B = 0;
  //set time needed to fire an interrupt
  OCR5A = 62500;
  //set control registers
  TCCR5A |=  (1 << WGM12); //ctc mode
  TCCR5B |=  (1 << CS12); //256-prescalar
  TIMSK5 |= (1 << OCIE5A); //enable interrupts
   
  
  interrupts();
  
}

//interrupt for timer 5, when the interrupt is called the robot stops
ISR(TIMER5_COMPA_vect) {
  stop = 1;
}


void loop() {
  //when we get a message, make the callback function
  nh.spinOnce();
  
  //make sure that we only do things with the ultrasonic sensor after the specified minimal delay
  if (millis() > sensorDelay ) {
    fwd_stop = 0; //reset fwd_stop
    //specifications note that the sensor should be enabled for 10 μs
    digitalWrite(pin_sensor_trigger, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_sensor_trigger, LOW);
    //measure the time it takes for the pin to receive a high signal, times out after 1 second.
    long duration = pulseIn(pin_sensor_echo, HIGH); 
    //a duration of 1200 μs makes for a distance of (1200/58) approximately 21 centimeters
    if (duration <= 1200) {
      fwd_stop = 1; //set fwd_stop
    }
    //set the next time the sensor operation can be run
    sensorDelay = millis() + 50;
  }
  //stop if we have not received any msgs (in the last second)
  if(stop) {
    digitalWrite(pin_rev_left, ENGINE_OFF);
    digitalWrite(pin_fwd_right, ENGINE_OFF);
    digitalWrite(pin_fwd_left, ENGINE_OFF);
    digitalWrite(pin_rev_right, ENGINE_OFF);
  } else {
    //we can go forward if there is a no object in our way
    if(!fwd_stop) {          
      analogWrite(pin_fwd_left, speed_engine * speed_fwd_left);    
      analogWrite(pin_fwd_right, speed_engine * speed_fwd_right);
    //now we cant go forward, so stop the forward engines
    } else {
      digitalWrite(pin_fwd_right, ENGINE_OFF);
      digitalWrite(pin_fwd_left, ENGINE_OFF);      
    }
    analogWrite(pin_rev_left, speed_engine * speed_rev_left);
    analogWrite(pin_rev_right, speed_engine * speed_rev_right);
  
  }
  
  
}
