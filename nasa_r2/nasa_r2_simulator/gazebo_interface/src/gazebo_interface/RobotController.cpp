#include <gazebo_interface/RobotController.h>

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>

#include <ros/ros.h>

#include <r2_msgs/JointStatusArray.h>

using namespace gazebo;

RobotController::RobotController(physics::ModelPtr _modelPtr)
: modelPtr(_modelPtr)
{
    prevUpdateTime = _modelPtr->GetWorld()->GetSimTime();
}

RobotController::~RobotController()
{
}

void RobotController::addJoint(physics::JointPtr _jointPtr, bool advancedMode)
{
    boost::mutex::scoped_lock(pidMutex);
    JointControllerPtr jointPtr(new JointController(_jointPtr, advancedMode));
    joints[_jointPtr->GetName()] = jointPtr;
}

void RobotController::addJointDependency(std::string _childName, std::string _parentName, double factor)
{
    if (joints.find(_parentName) == joints.end())
    {
        ROS_WARN("GazeboInterface joint dependency not set because parent (%s) not found", _parentName.c_str());
        return;
    }
    if (joints.find(_childName) == joints.end())
    {
        ROS_WARN("GazeboInterface joint dependency not set because child (%s) not found", _childName.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    dependencies.insert(std::make_pair(_parentName, std::make_pair(_childName, factor)));
}

void RobotController::setPosPid(const std::string& name, double _p, double _i, double _d,
                                  double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface PosPID not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setPosPid(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin);
}

void RobotController::setVelPid(const std::string& name, double _p, double _i, double _d,
                                  double _imax, double _imin, double _cmdMax, double _cmdMin)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface PosPID not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setVelPid(_p, _i, _d, _imax, _imin, _cmdMax, _cmdMin);
}

// move the robot to the specified positions while taking dependencies into account
void RobotController::setJointPositions(std::map<std::string, double> posMap)
{
    // pause gazebo
    bool is_paused = modelPtr->GetWorld()->IsPaused();
    if (!is_paused) modelPtr->GetWorld()->SetPaused(true);

    // release brakes on all joints
    std::map<std::string, bool> brakeStates;
    for (std::map<std::string, JointControllerPtr>::iterator it = joints.begin(); it != joints.end(); ++it)
    {
        brakeStates[it->first] = it->second->getJointStatus().brakeReleased;
        it->second->releaseBrake();
    }

    // check for dependencies
    for (std::map<std::string, double>::iterator posIt = posMap.begin(); posIt != posMap.end(); ++posIt)
    {
        std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(posIt->first);
        for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
        {
            posMap[depIt->second.first] = depIt->second.second * posIt->second;
        }
    }

    // set positions
    modelPtr->SetJointPositions(posMap);

    // reset brakes on all joints
    for (std::map<std::string, bool>::iterator it = brakeStates.begin(); it != brakeStates.end(); ++it)
    {
        joints[it->first]->releaseBrake(it->second);
    }

    // resume paused state
    modelPtr->GetWorld()->SetPaused(is_paused);
}

void RobotController::setJointPosTarget(const std::string& name, double target)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointPosTarget not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setPosTarget(target);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(name);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        joints[depIt->second.first]->setPosTarget(depIt->second.second * target);
    }
}

void RobotController::setJointVelTarget(const std::string& name, double target)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointVelTarget not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setVelTarget(target);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(name);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        joints[depIt->second.first]->setVelTarget(depIt->second.second * target);
    }
}

void RobotController::setJointEffortTarget(const std::string& name, double target)
{
    if (joints.find(name) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointEffortTarget not set because joint (%s) not found", name.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[name]->setEffortTarget(target);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(name);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        joints[depIt->second.first]->setEffortTarget(depIt->second.second * target);
    }
}

// joint state management
void RobotController::setJointControl(const r2_msgs::JointControl::ConstPtr& msg)
{
    if (joints.find(msg->joint) == joints.end())
    {
        ROS_WARN("GazeboInterface setJointControl not set because joint (%s) not found", msg->joint.c_str());
        return;
    }
    boost::mutex::scoped_lock(pidMutex);
    joints[msg->joint]->setJointControl(msg);

    // check for dependencies
    std::pair<dependenciesType::iterator, dependenciesType::iterator> deps = dependencies.equal_range(msg->joint);
    for (dependenciesType::iterator depIt = deps.first; depIt != deps.second; ++depIt)
    {
        r2_msgs::JointControl* jcPtr = new r2_msgs::JointControl(*msg);
        jcPtr->joint = depIt->second.first;
        r2_msgs::JointControl::ConstPtr depJointControlPtr(jcPtr);
        joints[depIt->second.first]->setJointControl(depJointControlPtr);
    }
}

const r2_msgs::JointStatus& RobotController::getJointStatus(const std::string& name) const
{
    std::map<std::string, JointControllerPtr>::const_iterator cit = joints.find(name);
    if (cit == joints.end())
    {
        ROS_WARN("GazeboInterface getJointStatus failed because joint (%s) not found", name.c_str());
    }
    return cit->second->getJointStatus();
}

void RobotController::update()
{
    common::Time currTime = modelPtr->GetWorld()->GetSimTime();
    common::Time stepTime = currTime - prevUpdateTime;
    prevUpdateTime = currTime;

    for (std::map<std::string, JointControllerPtr>::iterator it = joints.begin(); it != joints.end(); ++it)
    {
        it->second->update(stepTime);
    }
}

void RobotController::publishJointStatuses(ros::Publisher& rosPub) const
{
    r2_msgs::JointStatusArray statusArray;
    statusArray.header.stamp = ros::Time::now();
    for (std::map<std::string, JointControllerPtr>::const_iterator it = joints.begin(); it != joints.end(); ++it)
    {
        statusArray.status.push_back(it->second->getJointStatus());
    }
    rosPub.publish(statusArray);
}
