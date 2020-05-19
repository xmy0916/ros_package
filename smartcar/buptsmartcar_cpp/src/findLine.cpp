#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Twist.h"
#include <opencv-3.3.1-dev/opencv2/core.hpp>
#include <opencv-3.3.1-dev/opencv/highgui.h>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/JointTrajectory.h>
//用于将四元数转换成欧拉角的文件
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
using namespace cv;

typedef struct ImuData{
	float orientation[4];//四元数
	float angular[3];//角速度
	float linear[3];//线速度
	double quat[3];//欧拉角  横滚,俯仰,偏航->quat[0],quat[1],quat[2]
}ImuData,* ImuDataPtr;

class Follower{

    public:
	ros::NodeHandle node;
	static ros::Publisher cmdpub;
	static ros::Subscriber imageSub;
	static ros::Subscriber imuSub;
	static ros::Publisher pitchpub;
	static ImuDataPtr imuPtr;

	static void image_callback(const sensor_msgs::ImageConstPtr& msg);
	static void speed_contrl(float speed_car,float angluar_car);
	static void imu_callback(const sensor_msgs::Imu& imu_msg);
	Follower();
};


ros::Publisher Follower::cmdpub;
ros::Subscriber Follower::imageSub;
ros::Subscriber Follower::imuSub;
ros::Publisher Follower::pitchpub;
ImuDataPtr Follower::imuPtr = (ImuDataPtr)malloc(sizeof(ImuData));

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"follower_cpp");
	Follower f;//调用构造函数
	while(ros::ok())
	{
		ros::Rate loop_rate(0.2);
		//这里是程序的循环处，可以写你的代码
		ros::spinOnce();
	}

}

void Follower::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	
        try
        {
            cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        }
        catch(cv_bridge::Exception& e)  
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
	}
	Mat hsv = cv_ptr->image.clone();
	Mat mask = cv_ptr->image.clone();
	cvtColor(cv_ptr->image, hsv, COLOR_BGR2HSV);
        double low_H = 0;
        double low_S = 0;
        double low_V = 221;
        double high_H = 180;
        double high_S = 30;
        double high_V = 255;
	inRange(hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask);

	//speed_contrl(0.5,0.5);

	imshow("mask",mask);
	waitKey(3);
}

void Follower::imu_callback(const sensor_msgs::Imu& imu_msg)
{

	Follower::imuPtr->orientation[0] = imu_msg.orientation.x;
	Follower::imuPtr->orientation[1] = imu_msg.orientation.y;
	Follower::imuPtr->orientation[2] = imu_msg.orientation.z;
	Follower::imuPtr->orientation[3] = imu_msg.orientation.w;

	//四元数转换成欧拉角
	tf::Quaternion quat;
	tf::quaternionMsgToTF(imu_msg.orientation, quat);
	tf::Matrix3x3(quat).getRPY(		\
		Follower::imuPtr->quat[0],      \
		Follower::imuPtr->quat[1],      \
		Follower::imuPtr->quat[2]);

	Follower::imuPtr->angular[0] = imu_msg.angular_velocity.x;
	Follower::imuPtr->angular[1] = imu_msg.angular_velocity.y;
	Follower::imuPtr->angular[2] = imu_msg.angular_velocity.z;

	Follower::imuPtr->linear[0] = imu_msg.linear_acceleration.x;
	Follower::imuPtr->linear[1] = imu_msg.linear_acceleration.y;
	Follower::imuPtr->linear[2] = imu_msg.linear_acceleration.z;

	std::cout<<"横滚角 = "<<Follower::imuPtr->quat[0]<<std::endl;
	std::cout<<"俯仰角 = "<<Follower::imuPtr->quat[1]<<std::endl;
	std::cout<<"偏航角 = "<<Follower::imuPtr->quat[2]<<std::endl;

	std_msgs::Float64 msg;
	msg.data = Follower::imuPtr->quat[1];//rqt_plot打印俯仰角
	pitchpub.publish(msg);
}


void Follower::speed_contrl(float speed_car,float angluar_car)
{
	geometry_msgs::Twist twist;
        geometry_msgs::Vector3 linear;
        linear.x=speed_car;
        linear.y=0;
        linear.z=0;
        geometry_msgs::Vector3 angular;
        angular.x=0;
        angular.y=0;
        angular.z=angluar_car;
        twist.linear=linear;
        twist.angular=angular;
        cmdpub.publish(twist);
}


Follower::Follower()
{
	cmdpub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10, true);
	imageSub = node.subscribe("camera/image_raw",10,image_callback);
	imuSub = node.subscribe("/imu",10,imu_callback);
	pitchpub = node.advertise<std_msgs::Float64>("/pitch", 10, true);
}

