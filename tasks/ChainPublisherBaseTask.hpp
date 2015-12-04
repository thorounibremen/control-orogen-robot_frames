/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef ROBOT_FRAMES_CHAINPUBLISHERBASETASK_TASK_HPP
#define ROBOT_FRAMES_CHAINPUBLISHERBASETASK_TASK_HPP

#include "robot_frames/ChainPublisherBaseTaskBase.hpp"
#include "robot_framesTypes.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <base/Logging.hpp>

#include <robot_frames/RobotFrames.hpp>

namespace robot_frames {
class ChainPublisherBaseTask : public ChainPublisherBaseTaskBase
{
    friend class ChainPublisherBaseTaskBase;
protected:
    std::vector<robot_frames::Chain> chain_definitions_;
    std::vector<KDL::Chain> chains_;
    std::vector<std::string> chain_names_;
    std::vector<KDL::ChainFkSolverPos_recursive*> pos_solvers_;
    std::vector<KDL::Frame> kdl_frames_;
    std::vector<base::samples::RigidBodyState> bt_frames_;
    std::vector<KDL::JntArray> joint_arrays_;
    std::vector<std::vector<std::string> >involved_active_joints_;
    base::samples::Joints joint_state_;
    KDL::Tree tree_;
    int n_defined_chains_;
    std::vector<RTT::OutputPort<base::samples::RigidBodyState>*> out_ports_;

    void clear_and_resize_vectors();
    bool unpack_joints(const base::samples::Joints& joint_state,
                       const std::vector<std::string>& involved_joints,
                       KDL::JntArray& joint_array);


public:
    ChainPublisherBaseTask(std::string const& name = "robot_frames::ChainPublisherBaseTask");
    ChainPublisherBaseTask(std::string const& name, RTT::ExecutionEngine* engine);
    ~ChainPublisherBaseTask();
    bool configureHook();
    bool startHook();
    void updateHook();
    void errorHook();
    void stopHook();
    void cleanupHook();
};
}

#endif

