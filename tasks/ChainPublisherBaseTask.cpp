/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ChainPublisherBaseTask.hpp"

using namespace robot_frames;

ChainPublisherBaseTask::ChainPublisherBaseTask(std::string const& name)
    : ChainPublisherBaseTaskBase(name)
{
}

ChainPublisherBaseTask::ChainPublisherBaseTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ChainPublisherBaseTaskBase(name, engine)
{
}

ChainPublisherBaseTask::~ChainPublisherBaseTask()
{
}

void ChainPublisherBaseTask::clear_and_resize_vectors()
{
    chains_.clear();
    chains_.resize(n_defined_chains_);

    chain_names_.clear();
    chain_names_.resize(n_defined_chains_);

    for(size_t i=0; i<pos_solvers_.size(); i++){
        delete pos_solvers_[i];
    }
    pos_solvers_.clear();
    pos_solvers_.resize(n_defined_chains_);

    kdl_frames_.clear();
    kdl_frames_.resize(n_defined_chains_);

    bt_frames_.clear();
    bt_frames_.resize(n_defined_chains_);

    joint_arrays_.clear();
    joint_arrays_.resize(n_defined_chains_);

    involved_active_joints_.clear();
    involved_active_joints_.resize(n_defined_chains_);

    for(size_t i=0; i<out_ports_.size(); i++){
        ports()->removePort(out_ports_[i]->getName());
        delete out_ports_[i];
    }
    out_ports_.clear();
    out_ports_.resize(n_defined_chains_);
}

bool ChainPublisherBaseTask::unpack_joints(const base::samples::Joints& joint_state,
                                   const std::vector<std::string>& involved_joints,
                                   KDL::JntArray& joint_array)
{
    assert(involved_joints.size() == joint_array.rows());

    bool ok = true;
    for(size_t i = 0; i<involved_joints.size(); i++){
        try{
            base::JointState js = joint_state.getElementByName(involved_joints[i]);
            joint_array(i) = js.position;
        }
        catch(base::samples::Joints::InvalidName ex){
            LOG_ERROR("Could not find joint %s in joint state vector.", involved_joints[i].c_str());
            ok = false;
        }
    }

    return ok;
}

bool ChainPublisherBaseTask::configureHook()
{
    if (! ChainPublisherBaseTaskBase::configureHook())
        return false;

    n_defined_chains_ = chain_definitions_.size();
    if(!n_defined_chains_){
        LOG_WARN_S << "No chains are defined! This component will do nothing..." << std::endl;
    }

    std::string urdf_file_path = _urdf_file.get();

    bool st;
    st = kdl_parser::treeFromFile(urdf_file_path, tree_);
    if(!st){
        LOG_ERROR_S << "Error parsing urdf file: " << urdf_file_path << std::endl;
        return false;
    }

    //Resize stuff
    clear_and_resize_vectors();

    //Receive Chains from urdf
    for(size_t i=0; i<n_defined_chains_; i++){
        std::string root = chain_definitions_[i].root_link;
        std::string tip = chain_definitions_[i].tip_link;
        std::string name = chain_definitions_[i].name;

        st = tree_.getChain(root, tip, chains_[i]);
        if(!st){
            LOG_ERROR("Error extracting chain '%s' with root '%s' and tip '%s'. The urdf_file is: %s",
                      name.c_str(), root.c_str(), tip.c_str(), urdf_file_path.c_str());
            return false;
        }
        chain_names_[i] = name;
    }

    //Determine invloved joits, prepare frames storage and solvers
    for(size_t i=0; i<n_defined_chains_; i++){
        KDL::Chain chain = chains_[i];
        //Determine involved joints for each chain
        for(uint s=0; s<chain.segments.size(); s++){
            KDL::Segment segment = chain.segments[s];
            std::string jname = segment.getJoint().getName();

            if(segment.getJoint().getType() == KDL::Joint::None){
                LOG_DEBUG("Skipping joint %s of chain %s. Joint is fixed.",
                          jname.c_str(),
                          chain_names_[i].c_str());
                continue;
            }

            involved_active_joints_[i].push_back(jname);
        }
        //Resize joint arrays
        joint_arrays_[i].resize(involved_active_joints_[i].size());
        joint_arrays_[i].data.setZero();

        //Prepare frames storage
        base::samples::RigidBodyState rbs;
        rbs.sourceFrame = chain_definitions_[i].tip_link;
        rbs.targetFrame = chain_definitions_[i].root_link;
        rbs.invalidate();
        bt_frames_[i] = rbs;

        //Prepare solver
        pos_solvers_[i] = new KDL::ChainFkSolverPos_recursive(chains_[i]);
    }

    return true;
}
bool ChainPublisherBaseTask::startHook()
{
    if (! ChainPublisherBaseTaskBase::startHook())
        return false;
    return true;
}
void ChainPublisherBaseTask::updateHook()
{
    ChainPublisherBaseTaskBase::updateHook();

    while (_input.read(joint_state_, false) == RTT::NewData){
        int st;
        //Go thourgh chains
        for(size_t i=0; i<n_defined_chains_; i++){
            KDL::ChainFkSolverPos_recursive* solver = pos_solvers_[i];

            //Extract joint array for chain
            if(!unpack_joints(joint_state_, involved_active_joints_[i], joint_arrays_[i]))
                throw std::runtime_error("Unable to correctly unpack joint values on kinematic chain");

            //Calculate
            st = solver->JntToCart(joint_arrays_[i], kdl_frames_[i]);
            if(st < 0){
                LOG_ERROR_S << "Something went wrong solving forward kineamtics for chain " << chain_names_[i] << std::endl;
            }

            //Convert and write to port
            convert(kdl_frames_[i], bt_frames_[i]);
            bt_frames_[i].time = joint_state_.time;
            out_ports_[i]->write(bt_frames_[i]);
        }
    }
}
void ChainPublisherBaseTask::errorHook()
{
    ChainPublisherBaseTaskBase::errorHook();
}
void ChainPublisherBaseTask::stopHook()
{
    ChainPublisherBaseTaskBase::stopHook();
}
void ChainPublisherBaseTask::cleanupHook()
{
    ChainPublisherBaseTaskBase::cleanupHook();
}
