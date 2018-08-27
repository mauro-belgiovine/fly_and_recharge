/*
 * Suav.h
 *
 *  Created on: 02/mar/2017
 *      Author: mauro
 */

#ifndef SUAV_H_
#define SUAV_H_

#include <stdio.h>
#include <string.h>
#include <omnetpp.h>
#include <stdlib.h>
#include <Station.h>

#include "suav_m.h"


#define RANDOM_TRAFFIC 0
#define NEIGHBORS_UPDT 1
#define CONTROLLER_UPDT 2
#define LEAVE 3

#define SUAV_FROM_GATE(gate) ((Suav *) gate->getPathEndGate()->getOwnerModule())

#define BATTERY_LOW 0.2
#define CHARGE_THRESHOLD 0.5
#define INIT_BATTERY intuniform(85,100)*0.01

#define MIN_CHARGE_TIME 20

using namespace omnetpp;

#define PI 3.141592653589793


/* DIFFERENT STRATEGIES */
/*//BETTER RESULTS:
#define FIXED_CHARGE_T
#ifdef FIXED_CHARGE_T
    //#define CHARGE_THRESHOLD (1300.0f/(n_suav+1))/1300.0f
    //#define INIT_BATTERY 1
#endif

#define DYNAMIC_THRESHOLD
#define LOCAL_MIN_CHARGES
//#define PROPORTION_NO_FULL
#define LOWER_THRESHOLD
#define USE_BATTERY_LIMIT
//#define RANDOM_PROBABILITY
*/

#define FIXED_CHARGE_T
#ifdef FIXED_CHARGE_T
    //#define CHARGE_THRESHOLD (1300.0f/(n_suav+1))/1300.0f
    //#define INIT_BATTERY 1
#endif

#define DYNAMIC_THRESHOLD
#define LOCAL_MIN_CHARGES
//#define PROPORTION_NO_FULL
#define LOWER_THRESHOLD
#define USE_BATTERY_LIMIT
//#define RANDOM_PROBABILITY




class Suav : public cSimpleModule
{

  protected:
    //double h = par("h"); // network height (meters)
    #define TIMESLOT 1.0
    //#define MY_CONFIG

#ifdef MY_CONFIG
    double h = 10;
    double alpha = 0.003; // discharge factor per Tslot
    double beta =  0.008; //charge factor per Tslot
    double angle = 45; // (degrees)
    double theta = angle*(PI/180);
    double gamma = 0.002; // energy consumption factor during descending/ascending operation (final cost will depend also on h)
#else
    double h;
    double alpha; // discharge factor per Tslot
    double beta; //charge factor per Tslot
    double angle; // (degrees)
    double theta;
    double gamma; // energy consumption factor during descending/ascending operation (final cost will depend also on h)
    /*
        double h = 30.0; // (m)
        double battery_cap = 130000; // battery capacity in j
        double alpha = 100; // (W) discharge factor per Tslot (NOTE: 1W = 1J/s)
        double beta = 25; //(W) charge factor per Tslot
        double theta = PI/3.0; // angle for coverage computation (radians)
        double gamma = 5; // (J/m) energy consumption factor during descending/ascending operation (final cost will depend on h)
         */
#endif

    cMessage *update_ctrl_evt = new cMessage("update");  // pointer to the event object which we'll use for timing
    cMessage *wake_up = new cMessage("wakeup");
    cMessage *charge = new cMessage("charge");
    int station_i = -1;
    int charges = 0;

    int n_suav = 0;

    std::vector<SuavMsg> update_msgs;
    std::vector<Suav *> neighbors;
    std::vector<Station *> ch_stations;

    //stats
    simsignal_t stateChange;
    simsignal_t batteryLevel;


    virtual void batteryUpdateRoutine();
    virtual void handleCharge();
    virtual void handleWakeup();

    virtual void randForwardMessage(cMessage *msg);
    virtual void initialize() override;
    virtual void handleMessage(cMessage *msg) override;
    virtual void generateMessage();
    virtual double E(int i, simtime_t j); //Energy update function
    virtual double OP(double h);
    virtual double p_charge();
    virtual int chargeLength();
    virtual int dischargeLength(double energy, double h);
    virtual double energyProportion(std::vector<SuavMsg> msgs);
    virtual int minDischargeLength(std::vector<SuavMsg> updt_m);
    virtual bool isLocalMinimumCharges(std::vector<SuavMsg> updt_m);
    virtual int countActive();

  public:
      double cov_h = PI * pow(h*tan(theta/2), 2);
      short int state = 1; // 1 = active, 0 = charging
      short int prev_state = state; //at each update, this has to be updated with the previous state


      // PERCENTAGE BATTERY MODEL
      double battery_lvl=INIT_BATTERY; //battery level at start

      // KJ BATTERY MODEL
      //double battery_lvl=battery_cap; //battery level at start


};


#endif /* SUAV_H_ */
