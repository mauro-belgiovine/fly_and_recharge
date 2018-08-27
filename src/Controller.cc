#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <map>

#include <Suav.h>
#include "suav_m.h"

using namespace omnetpp;

#define PI 3.141592653589793

class Controller : public cSimpleModule
{
  protected:
    double tot_area = 0;
    double k = 0.5; // Coverage threshold limit. TODO use as parameter
    std::vector<Suav *> nodes;

    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    cMessage *check_evt = new cMessage("check");  // pointer to the event object which we'll use for timing
    virtual double C();
    virtual bool anyDead();

};

Define_Module(Controller);

void Controller::initialize(){

     cModule *network = getParentModule();
     uint n_nodes = 0;
     for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
         cModule *submodule = *it;
         // consider all the Suav objects
         if (strcmp(submodule->getClassName(), "Suav") == 0 ){

             nodes.push_back((Suav *) submodule);
             EV << nodes.at(n_nodes)->getFullName() << " active " << nodes.at(n_nodes)->state << " coverage " << nodes.at(n_nodes)->cov_h << "\n";
             n_nodes++;
         }
     }

     tot_area = C();

     EV << "fraction of area covered " << C()/tot_area << "\n";

     scheduleAt(1.0, check_evt);

}

double Controller::C(){ //total area covered by the active nodes

    double area_covered = 0;

    for(int i = 0; i < nodes.size(); i++){
        area_covered += nodes.at(i)->cov_h * nodes.at(i)->state;
    }

    return area_covered;
}

bool Controller::anyDead(){ //total area covered by the active nodes

    bool any_dead = false;

    for(int i = 0; (i < nodes.size() && !any_dead); i++){
        if(nodes.at(i)->battery_lvl <= 0) any_dead = true;
    }

    return any_dead;
}

void Controller::handleMessage(cMessage *msg)
{
    if (msg == check_evt) { //my own message (that's why we use pointers)

        //check that the fraction of area covered is above the threshold
        if((C()/tot_area) < k){
            EV << "Coverage " << C()/tot_area << " below " << k << "\n";
            endSimulation();
        }



        if(anyDead()){
            EV << "One or more Suavs are dead.\n " ;
            endSimulation();
        }

        scheduleAt(simTime()+1.0, check_evt);

    }/*else {
        SuavMsg *suav_msg = (SuavMsg *) msg;
        EV << "Message from " << msg->getSenderModule()->getFullName() << " arrived.\n";
        EV << "Status " << suav_msg->getActive() << "\n";
    }*/

}

