/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ChainPublisher.hpp"

using namespace robot_frames;

ChainPublisher::ChainPublisher(std::string const& name)
    : ChainPublisherBase(name)
{
}

ChainPublisher::ChainPublisher(std::string const& name, RTT::ExecutionEngine* engine)
    : ChainPublisherBase(name, engine)
{
}

ChainPublisher::~ChainPublisher()
{
}

bool ChainPublisher::configureHook()
{
    chain_definitions_ = _chains.get();

    if (! ChainPublisherBase::configureHook())
        return false;

    //Create dynamic output per chain
    for(int i=0; i<n_defined_chains_; i++){
        std::string port_name = chain_names_[i];
        LOG_INFO("Adding port %s to component", port_name.c_str());
        RTT::OutputPort<base::samples::RigidBodyState>* output_port =
                new RTT::OutputPort<base::samples::RigidBodyState>(port_name);
        ports()->addPort(port_name, *output_port);
        out_ports_[i] = output_port;
    }

    return true;
}
bool ChainPublisher::startHook()
{
    if (! ChainPublisherBase::startHook())
        return false;
    return true;
}
void ChainPublisher::updateHook()
{
    ChainPublisherBase::updateHook();

}
void ChainPublisher::errorHook()
{
    ChainPublisherBase::errorHook();
}
void ChainPublisher::stopHook()
{
    ChainPublisherBase::stopHook();
}
void ChainPublisher::cleanupHook()
{
    ChainPublisherBase::cleanupHook();
}
