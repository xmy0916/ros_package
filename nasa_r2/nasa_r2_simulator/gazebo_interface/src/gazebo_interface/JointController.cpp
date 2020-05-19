#include <gazebo_interface/JointController.h>

#include <ros/console.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>

using namespace gazebo;

JointController::JointController(physics::JointPtr _jointPtr, bool _advancedMode)
: jointPtr(_jointPtr)
, position(0.)
, velocity(0.)
, effort(0.)
, advancedMode(_advancedMode)
, controlMode(POS_COM)
, fault(OK)
{
    currStatusPtr.reset(new r2_msgs::JointStatus);
    currStatusPtr->publisher = "GazeboSim";
    currStatusPtr->joint = _jointPtr->GetName();

    // initialize some parameters for sim only
    // this approach may change as robot control develops
    currStatusPtr->bridgeEnabled = true; // bridge enable status ignored, but set to true to start
    currStatusPtr->motorPowerDetected = true; // always true when sim running
    currStatusPtr->embeddedMotCom = true; // default to true on startup

    ROS_DEBUG("GetLimits for %s", _jointPtr->GetName().c_str());
    jointLowLimit = _jointPtr->GetLowStop(0).Radian();
    jointHighLimit = _jointPtr->GetHighStop(0).Radian();

    // set brake based on mode
    releaseBrake(!_advancedMode);
}

JointController::~JointController()
{
}

void JointController::setPosPid(double _p, double _i, double _d,
                                double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    boost::mutex::scoped_lock lock(controllerMutex);
    posPid.Init(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin);
}

void JointController::setVelPid(double _p, double _i, double _d,
                                double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    boost::mutex::scoped_lock lock(controllerMutex);
    velPid.Init(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin);
}

void JointController::setPosTarget(double target)
{
    boost::mutex::scoped_lock lock(controllerMutex);
    position = target;

    // if not using advanced mode, target calls determine current mode
    if (!advancedMode)
    {
        controlMode = POS_COM;
    }
}

void JointController::setVelTarget(double target)
{
    boost::mutex::scoped_lock lock(controllerMutex);
    velocity = target;

    // if not using advanced mode, target calls determine current mode
    if (!advancedMode)
    {
        controlMode = VEL_COM;
    }
}

void JointController::setEffortTarget(double target)
{
    boost::mutex::scoped_lock lock(controllerMutex);
    effort = target;

    // if not using advanced mode, target calls determine current mode
    if (!advancedMode)
    {
        controlMode = TORQ_COM;
    }
}

void JointController::update(common::Time& stepTime)
{
    boost::mutex::scoped_lock lock(controllerMutex);

    if (advancedMode && (!currStatusPtr->embeddedMotCom || !currStatusPtr->motorEnabled))
    {
        // not ready for command
        return;
    }

    double cmd = 0;
    switch (controlMode)
    {
    case TORQ_COM:
        cmd = effort;
        break;
    case POS_COM:
        cmd = posPid.Update(jointPtr->GetAngle(0).Radian() - position, stepTime);
        break;
    case VEL_COM:
        cmd = velPid.Update(jointPtr->GetVelocity(0) - velocity, stepTime);
        break;
    case IMP_COM:
    {
        static bool warnReported = false;
        if (!warnReported)
        {
            ROS_WARN("GazeboInterface JointController received impedence command which is unsupported. This message will only be shown once.");
            warnReported = true;
        }
        break;
    }
    default:
    {
        static bool warnReported = false;
        if (!warnReported)
        {
            ROS_WARN("GazeboInterface JointController received unsupported joint command. This message will only be shown once.");
            warnReported = true;
        }
    }
    }

    jointPtr->SetForce(0, cmd);
}

void JointController::setJointControl(const r2_msgs::JointControl::ConstPtr& msg)
{
    if (msg->joint != currStatusPtr->joint)
    {
        ROS_WARN("GazeboInterface setJointControl %s recieved control command for the wrong joint (%s), ignored",
                 currStatusPtr->joint.c_str(), msg->joint.c_str());
        return;
    }
    else
    {
        boost::mutex::scoped_lock lock(controllerMutex);
        // ignore publisher string
        // ignore registerValue uint32
        currStatusPtr->bridgeEnabled = msg->enableBridge;
        currStatusPtr->motorEnabled = msg->enableMotor;
        currStatusPtr->embeddedMotCom = msg->embeddedMotCom;
        controlMode = (JointControlMode)msg->controlMode;
        lock.unlock();
        if (msg->clearFaults)
        {
            clearFaults();
        }
        releaseBrake(msg->releaseBrake);
    }
}

const r2_msgs::JointStatus& JointController::getJointStatus() const
{
    return *currStatusPtr;
}

void JointController::clearFaults()
{
    boost::mutex::scoped_lock lock(controllerMutex);
    fault = OK;
    currStatusPtr->jointFaulted = false;
}

void JointController::releaseBrake(bool release)
{
    boost::mutex::scoped_lock lock(controllerMutex);
    ROS_DEBUG("%s brake for %s", release ? "release" : "set", jointPtr->GetName().c_str());
    if (release)
    {
        jointPtr->SetHighStop(0, jointHighLimit);
        jointPtr->SetLowStop(0, jointLowLimit);
    }
    else
    {
        math::Angle currAngle = jointPtr->GetAngle(0);
        jointPtr->SetHighStop(0, currAngle);
        jointPtr->SetLowStop(0, currAngle);
    }
    currStatusPtr->brakeReleased = release;
}
