#include <ros/ros.h>
#include "geometry_msgs/Twist.h"


#include <signal.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>

#define KEYCODE_FWD 0x77
#define KEYCODE_REV 0x73
#define KEYCODE_LEFT 0x61
#define KEYCODE_RIGHT 0x64
#define KEYCODE_INCREASE_SPEED 0x72 //r
#define KEYCODE_DECREASE_SPEED 0x66 //f




class Controller {
	public:
		Controller();
		void keyLoop();
	private:
		ros::NodeHandle nh_;
		ros::Publisher vel_pub_;
		double x,y,z;
};

Controller::Controller() {
	 x = y = z = 0;
	 vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}


int kfd =0;
struct termios cooked, raw;

void Controller::keyLoop() {
	char c;
	bool dirty = false;

                                                              
   tcgetattr(kfd, &cooked);
   memcpy(&raw, &cooked, sizeof(struct termios));
   raw.c_lflag &=~ (ICANON | ECHO);
   // Setting a new line, then end of file                         
   raw.c_cc[VEOL] = 1;
   raw.c_cc[VEOF] = 2;
   tcsetattr(kfd, TCSANOW, &raw);
 
   puts("Reading from keyboard");
   puts("---------------------------");
   puts("Use wasd to move the robot. Use r to increase and f to decrease the speed");
 
   //linear.x = left
   //linear.y = right
   int left  = 0;
   int right = 0;
   double speed = 1;
   for(;;) {
     // get the next event from the keyboard  
     if(read(kfd, &c, 1) < 0) {
       perror("read():");
       exit(-1);
     }
 
     left=right=0;
     ROS_DEBUG("value: 0x%02X\n", c);
   
     switch(c) {
       case KEYCODE_FWD:
         ROS_DEBUG("FORWARD");
         left = right = 255;
         dirty = true;
         break;
       case KEYCODE_LEFT:
         ROS_DEBUG("LEFT");
         left = 255;
         dirty = true;
         break;
       case KEYCODE_RIGHT:
         ROS_DEBUG("RIGHT");
         right = 255;
         dirty = true;
         break;
       case KEYCODE_REV:
         ROS_DEBUG("DOWN");
         left = -255;
         right = -255;
         dirty = true;
         break;
       case KEYCODE_INCREASE_SPEED:
	       	ROS_DEBUG("INCREASE SPEED");
       		speed = (speed + 0.1) > 1 ? 1 : (speed + 0.1);
       		dirty = true;
       		break;
       	case KEYCODE_DECREASE_SPEED:
       		ROS_DEBUG("DECREASE SPEED");
       		speed = (speed - 0.1) < 0 ? 0 : (speed - 0.1);
       		dirty = true;
     }
     geometry_msgs::Twist vel;
     vel.linear.x = left;
     vel.linear.y = right;
     vel.linear.z = speed;
     if(dirty ==true) {
     	printf("Publishing: left: %d right: %d speed: %f\n", left, right, speed);
     	vel_pub_.publish(vel);    
     	dirty=false;
     }
   }
   return;		
	
}




void quit(int sig) {
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "controller");
	
	Controller control;
	
	signal(SIGINT,quit);
	
	control.keyLoop();
	
	return 0;
}



