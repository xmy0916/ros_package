#ifndef GAZEBOINTERFACE_H
#define GAZEBOINTERFACE_H

#include <gazebo_interface/RobotController.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <r2_msgs/JointControl.h>
#include <r2_msgs/JointStatus.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
    class GazeboInterface : public ModelPlugin
    {
    public:
        GazeboInterface();
        ~GazeboInterface();

        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

        // Called by the world update start event
        void update();

    private:
        // handle joint commands
        void commandJoints(const sensor_msgs::JointState::ConstPtr& msg);

        // handle joint control messages
        void controlJoints(const r2_msgs::JointControl::ConstPtr& msg);

        // traverse through yaml structure looking for the mapped values
        // param: structure to traverseB
        // valMap: returned map
        // searchKey: named value to find
        // ns: namespace to prepend
        // name: name of param
        void traverseParams(XmlRpc::XmlRpcValue param, std::map<std::string, XmlRpc::XmlRpcValue>& valMap, std::string searchKey = "",
                            std::string ns = "", std::string name = "");

        // get a double from double or int XmlRpcValue
        // returns false if type is neither double or int
        bool getDoubleVal(XmlRpc::XmlRpcValue& val, double& doubleVal);

        // Pointer to the model
        physics::ModelPtr modelPtr;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnectionPtr;

        std::auto_ptr<RobotController> robotControllerPtr;

        // ros stuff
        std::auto_ptr<ros::NodeHandle> rosNodePtr;
        std::auto_ptr<ros::NodeHandle> paramsNodePtr;

        // joint commands
        std::string jointCommandsTopic;
        ros::Subscriber jointCommandsSub;

        // joint states
        std::string jointStatesTopic;
        ros::Publisher jointStatePub;
        double jointStatesStepTime;

        // joint status
        bool advancedMode;
        std::string jointControlTopic;
        ros::Subscriber jointControlSub;
        std::string jointStatusTopic;
        ros::Publisher jointStatusPub;
        double jointStatusStepTime;

        common::Time prevStatesUpdateTime;
        common::Time prevStatusUpdateTime;
    };
}

#endif
