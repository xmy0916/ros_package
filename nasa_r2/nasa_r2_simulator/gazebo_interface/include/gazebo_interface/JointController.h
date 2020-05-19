#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include <map>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <boost/thread.hpp>

#include <r2_msgs/JointStatus.h>
#include <r2_msgs/JointControl.h>

namespace gazebo
{
    class JointController
    {
    public:
        JointController(physics::JointPtr _jointPtr, bool _advancedMode = false);
        ~JointController();

        void setPosPid(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                    double _imax = 0.0, double _imin = 0.0,
                    double _cmdMax = 0.0, double _cmdMin = 0.0);
        void setVelPid(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                    double _imax = 0.0, double _imin = 0.0,
                    double _cmdMax = 0.0, double _cmdMin = 0.0);

        // set targets
        void setPosTarget(double target);
        void setVelTarget(double target);
        void setEffortTarget(double target);

        // update PIDs and send forces to joints
        // stepTime is elapsed time since last update
        void update(common::Time& stepTime);

        // joint state management
        void setJointControl(const r2_msgs::JointControl::ConstPtr& msg);
        const r2_msgs::JointStatus& getJointStatus() const;

        void releaseBrake(bool release = true);
        void clearFaults();

    private:
        physics::JointPtr jointPtr;

        common::PID posPid;
        common::PID velPid;

        double position;
        double velocity;
        double effort;

        boost::mutex controllerMutex;

        bool advancedMode; // use state management stuff below

        // joint state management
        enum JointControlMode {POS_COM = 0, TORQ_COM = 1, IMP_COM = 2, VEL_COM = 3};
        enum JointFault {OK = 0};

        r2_msgs::JointStatusPtr currStatusPtr;
        JointControlMode controlMode;
        JointFault fault;

        double jointLowLimit;
        double jointHighLimit;
    };

    typedef boost::shared_ptr<JointController> JointControllerPtr;
}
#endif
