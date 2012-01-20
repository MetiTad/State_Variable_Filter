#ifndef OROCOS_STATE_VARIABLE_FILTER_COMPONENT_HPP
#define OROCOS_STATE_VARIABLE_FILTER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <iostream>

class State_Variable_Filter
    : public RTT::TaskContext
{
 public:
    State_Variable_Filter(string const& name)
        : TaskContext(name)
    {
        std::cout << "State_Variable_Filter constructed !" <<std::endl;
    }

    bool configureHook() {
        std::cout << "State_Variable_Filter configured !" <<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "State_Variable_Filter started !" <<std::endl;
        return true;
    }

    void updateHook() {
        std::cout << "State_Variable_Filter executes updateHook !" <<std::endl;
    }

    void stopHook() {
        std::cout << "State_Variable_Filter executes stopping !" <<std::endl;
    }

    void cleanupHook() {
        std::cout << "State_Variable_Filter cleaning up !" <<std::endl;
    }
};

#endif
