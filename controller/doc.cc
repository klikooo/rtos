#include <ros/ros.h>
#include "geometry_msgs/Twist.h"


#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_FWD 0x77
#define KEYCODE_REV 0x3D
#define KEYCODE_LEFT 0x61
#define KEYCODE_RIGHT 0x72



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

Controller::keyLoop() {
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
   puts("Use arrow keys to move the turtle.");
 
 
   for(;;) {
     // get the next event from the keyboard  
     if(read(kfd, &c, 1) < 0) {
       perror("read():");
       exit(-1);
     }
 
     linear_=angular_=0;
     ROS_DEBUG("value: 0x%02X\n", c);
   
     switch(c) {
       case KEYCODE_FWD:
         ROS_DEBUG("FORWARD");
         angular_ = 1.0;
         dirty = true;
         break;
       case KEYCODE_LEFT:
         ROS_DEBUG("LEFT");
         angular_ = -1.0;
         dirty = true;
         break;
       case KEYCODE_RIGHT:
         ROS_DEBUG("RIGHT");
         linear_ = 1.0;
         dirty = true;
         break;
       case KEYCODE_REV:
         ROS_DEBUG("DOWN");
         linear_ = -1.0;
         dirty = true;
         break;
     }
     geometry::Twist vel;
     vel.angular.x = a_scale_*angular_;
     vel.linear.x = l_scale_*linear_;
     if(dirty ==true) {
     	printf("Publishing\n");
     	vel_pub_.publish(vel);    
     	dirty=false;
     }
   }
   return;		
	
}


void quit(int sig) {
	tcserattr(kfd, TCSANOW, &cooked);
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



