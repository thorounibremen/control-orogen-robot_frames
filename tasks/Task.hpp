/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROBOT_FRAME_TRANSFORMATIONS_TASK_TASK_HPP
#define ROBOT_FRAME_TRANSFORMATIONS_TASK_TASK_HPP

#include "robot_frames/TaskBase.hpp"
#include "robot_frames/RobotFrames.hpp"

namespace robot_frames {
typedef std::map<std::string, RTT::OutputPort<base::samples::RigidBodyState>*> OutputPortMap;

class Task : public TaskBase
{
    friend class TaskBase;
protected:
    OutputPortMap out_ports;
    TransformationCalculator calc;
    bool output_static_transforms_;
    std::vector<base::samples::RigidBodyState> transforms_;
    std::vector<std::string> joint_names_;


public:

    Task(std::string const& name = "robot_frames::Task");

    Task(std::string const& name, RTT::ExecutionEngine* engine);
     ~Task();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

