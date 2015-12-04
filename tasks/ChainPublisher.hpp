/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROBOT_FRAMES_CHAINPUBLISHER_TASK_HPP
#define ROBOT_FRAMES_CHAINPUBLISHER_TASK_HPP

#include "robot_frames/ChainPublisherBase.hpp"

namespace robot_frames {


class ChainPublisher : public ChainPublisherBase
{
    friend class ChainPublisherBase;
protected:

public:
    ChainPublisher(std::string const& name = "robot_frames::ChainPublisher");

    ChainPublisher(std::string const& name, RTT::ExecutionEngine* engine);

    ~ChainPublisher();

    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

