#ifndef OROCOS_STATE_VARIABLE_FILTER_COMPONENT_HPP
#define OROCOS_STATE_VARIABLE_FILTER_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/Time.hpp>
#include <iostream>
#include <rtt/Activity.hpp>
#include <math.h>

using namespace RTT;
using namespace RTT::os;

class State_Variable_Filter
    : public RTT::TaskContext
{
 public:
    double x1_n,x1_c,x2_n,x2_c;
    double BW;
    double step;
    double Ad1,Ad2,Ad3,Ad4;
    double Bd1,Bd2;
    double u,y;
    InputPort<double> inPort;
    OutputPort<double> outPort;
    OutputPort<double> outPortVelocity;
    OutputPort<double> outPortInput;

    TimeService::ticks timestamp;

    State_Variable_Filter(string const& name)
        : TaskContext(name)
    {
        x1_n=x1_c=x2_n=x2_c=0;
        BW = 70.0;
        this->addProperty("BW",BW).doc("bandwidth of the filter");
        this->ports()->addPort("inPort",inPort).doc("Position input port");
        this->ports()->addPort("outPort",outPort).doc("Position output port");
        this->ports()->addPort("outPortVelocity",outPortVelocity).doc("Velocity output port");
        this->ports()->addPort("outPortInput",outPortInput).doc("Velocity output port");
        std::cout << "SVF constructed !" <<std::endl;
    }

    bool configureHook() {
        this->setActivity(new Activity(5,0.001));
        std::cout << "SVF configured ! "<<std::endl;
        return true;
    }

    bool startHook() {
        std::cout << "SVF started !" <<std::endl;
        step = this->getPeriod();
        timestamp = TimeService::Instance()->getTicks();
        return true;
    }

    void updateHook() {
//        double val;

//        if ( inPort.read(val) == RTT::NewData ) e{
//            val = u;
//        }

        Seconds elapsed = TimeService::Instance()->secondsSince( timestamp );
        u = sin(2*elapsed);

//        Ad1 = (4*sqrt(2)*pow(step,3)*pow(BW,3) - 8*pow(step,2)*pow(BW,2) + 32)/(pow(step,4)*pow(BW,4) + 16)-1;
//        Ad2 = (4*step*(pow(step,2)*pow(BW,2) + 4) - 8*sqrt(2)*pow(step,2)*BW)/(pow(step,4)*pow(BW,4) + 16);
//        Ad3 = -(4*step*pow(BW,2)*(pow(step,2)*pow(BW,2) - 2*sqrt(2)*step*BW  + 4))/(pow(step,4)*pow(BW,4) + 16);
//        Ad4 = (8*pow(step,2)*pow(BW,2) - 16*sqrt(2)*step*BW+32)/(pow(step,4)*pow(BW,4) + 16)-1;
//        Bd1 = 2 - ((4*sqrt(2)*pow(step,3)*pow(BW,3) -8*pow(step,2)*pow(BW,2)+32)/(pow(step,4)*pow(BW,4) + 16));
//        Bd2 = (4*step*pow(BW,2)*(pow(step,2)*pow(BW,2) - 2*sqrt(2)*step*BW  +  4))/(pow(step,4)*pow(BW,4) + 16);

          Ad1 = 1;
          Ad2 = step;
          Ad3 = -(pow(BW,2)*step);
          Ad4 = 1 -(sqrt(2)*step*BW);
          Bd1 = 0;
          Bd2 = step*pow(BW,2);

        x1_n = (Ad1*x1_c + Ad2*x2_c) + (Bd1*u);
        x2_n = (Ad3*x1_c + Ad4*x2_c) + (Bd2*u);

       std::cout << "SVF executes updateHook - " << x1_c <<std::endl;
       std::cout << "SVF executes updateHook - " << x2_c <<std::endl;

        outPort.write(x1_c);
        outPortVelocity.write(x2_c);
        outPortInput.write(u);

        x1_c = x1_n;
        x2_c = x2_n;

    }

    void stopHook() {
        std::cout << "SVF executes stopping ! " << " Ad1 " << Ad1 << " Ad2 " << Ad2 << " Ad3 " << Ad3 << " Ad4 " << Ad4 <<std::endl;
        std::cout << "SVF executes stopping ! " << " Bd1 " << Bd1 << " Bd2 " << Bd2 <<std::endl;
    }

    void cleanupHook() {
        std::cout << "SVF cleaning up !" <<std::endl;
    }
};

#endif
