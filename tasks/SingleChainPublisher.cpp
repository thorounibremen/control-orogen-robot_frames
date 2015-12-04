/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SingleChainPublisher.hpp"

using namespace robot_frames;

SingleChainPublisher::SingleChainPublisher(std::string const& name)
    : SingleChainPublisherBase(name)
{
}

SingleChainPublisher::SingleChainPublisher(std::string const& name, RTT::ExecutionEngine* engine)
    : SingleChainPublisherBase(name, engine)
{
}

SingleChainPublisher::~SingleChainPublisher()
{
}

bool SingleChainPublisher::configureHook()
{
    chain_definitions_.push_back(_chain.get());

    if (! SingleChainPublisherBase::configureHook())
        return false;

    out_ports_.push_back(&_tip_frame);

    return true;
}

bool SingleChainPublisher::startHook()
{
    if (! SingleChainPublisherBase::startHook())
        return false;
    return true;
}
void SingleChainPublisher::updateHook()
{
    SingleChainPublisherBase::updateHook();
}
void SingleChainPublisher::errorHook()
{
    SingleChainPublisherBase::errorHook();
}
void SingleChainPublisher::stopHook()
{
    SingleChainPublisherBase::stopHook();
}
void SingleChainPublisher::cleanupHook()
{
    SingleChainPublisherBase::cleanupHook();
}
