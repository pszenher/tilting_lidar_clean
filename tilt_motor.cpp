#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<dynamixel_msgs/JointState.h>
#include <cmath>

/*This code allows the motor to move back and forth to a maximum and minimum number of degrees defined in parameters.*/

using namespace std;

float error;

//obtains error from message
void obtainValues(const dynamixel_msgs::JointState &msg) {
    error = msg.error;
}

//creates all commands for each motor
class Dynamixel {
    private:
    ros::NodeHandle nh;
    ros::Publisher pub_n;
    public:
    Dynamixel();
    void checkError();
    void moveMotor(double position);
};

//creates publisher
Dynamixel::Dynamixel() {
    pub_n = nh.advertise<std_msgs::Float64>("/tilt_controller/command", 10);
}

//creates message and publishes -> degree to radian to publish
void Dynamixel::moveMotor(double position) {
    double convert = (position * 3.14/180);
    std_msgs::Float64 aux;
    aux.data = convert;
    pub_n.publish(aux);
    ROS_INFO_STREAM(aux);
}

//ensures proper alignment
void Dynamixel::checkError() {
    ros::spinOnce();
    ROS_ERROR_STREAM(error);
    while((abs (error))>0.02) {
    ros::Duration(.1).sleep();
//  ros::Subscriber sub=nh.subscribe("/tilt_controller/state", 5, &obtainValues); //checks error
    ros::spinOnce();
    }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "Motor_Tilt");
    ros::NodeHandle nh;

    Dynamixel motor;

    //variables
    int max;
    int min;
    double pause;

    //initialize subscription
    ros::Subscriber sub=nh.subscribe("/tilt_controller/state", 5, &obtainValues); //checks error

    //intitialize parameters
    nh.param("maximum", max, 90);
    nh.param("minimum", min, -90);
    nh.param("pause", pause, 0.1);

    while(ros::ok()) {

        motor.moveMotor(max);
	ros::Duration(pause).sleep();
        motor.checkError();
        ros::Duration(pause).sleep();
    
        motor.moveMotor(min);
	ros::Duration(pause).sleep();
        motor.checkError();
        ros::Duration(pause).sleep();
  

    }
}
