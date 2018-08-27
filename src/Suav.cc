#include "Suav.h"

Define_Module(Suav);

void Suav::initialize()
{
    //assign all parameters
    h = par("h");
    alpha = par("alpha"); // discharge factor per Tslot
    beta =  par("beta"); //charge factor per Tslot
    angle = par("angle"); // (degrees)
    theta = angle*(PI/180);
    gamma = par("gamma");

    EV << "node " << getIndex() << " battery= " << battery_lvl << " h " << h << " cov " << cov_h << "\n" ;

    /*if (getIndex() == 0) {
        generateMessage();
    }*/

    scheduleAt(TIMESLOT, update_ctrl_evt);

    /*
    cModule *targetModule = getParentModule()->getSubmodule("control");
    SuavMsg *status_msg = new SuavMsg("active", CONTROLLER_UPDT);
    status_msg->setActive(state );
    status_msg->setHeight(h);
    status_msg->setAngle(angle);

    sendDirect(status_msg, targetModule, "air_gate");
    */

    //setup active gate map: in the beginning, every suav is active
    //NOTE: in this scenario, the position of the nodes is static, so they're always connected to the same ones.
    //      In case of a dynamic scenario, this list has to be updated every time connection changes
    int n = gateSize("gate");
    for(int i = 0; i < n; i++){
        cGate *dest_o = gate("gate$o", i);
        neighbors.push_back(SUAV_FROM_GATE(dest_o));
    }

    // get info on charging stations
    cModule *network = getParentModule();
    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
         cModule *submodule = *it;
         // consider all the Station objects
         if (strcmp(submodule->getClassName(), "Station") == 0 ){

             ch_stations.push_back((Station *) submodule);

         }
    }

    WATCH(battery_lvl); // omnetpp macro for displaying info

    stateChange = registerSignal("stateChange");  //used for statistics
    batteryLevel = registerSignal("batteryLevel");

    // send a signal to notify initial state
    emit(stateChange, state);


    // get total number of suavs
    for (cModule::SubmoduleIterator it(network); !it.end(); ++it) {
         cModule *submodule = *it;
         // consider all the Station objects
         if (strcmp(submodule->getClassName(), "Suav") == 0 ){

             n_suav++;

         }
    }

}

void Suav::handleMessage(cMessage *msg)
{

    if (msg == update_ctrl_evt) { //my own message (that's why we use pointers)

        batteryUpdateRoutine();

    }else if (msg == wake_up){ //my own message

        handleWakeup();

    }else if (msg == charge){

        handleCharge();
        //empty the queue
        update_msgs.clear();

    } else if (msg->getKind() == NEIGHBORS_UPDT){



        SuavMsg *got = (SuavMsg *) msg;
        //EV << "got " << got->getSenderModuleId() << "\n";
        //if we already had some previous message from the same SUAV, remove the old msg and add a new one
        int remove = -1;
        for (int m = 0; m < update_msgs.size(); m++){
            //EV << "in list " << update_msgs.at(m).getSenderModuleId() ;
            if (got->getSenderModuleId() == update_msgs.at(m).getSenderModuleId() ){
                //EV << "EQUAL";
                remove = m;
            }
            //EV << "\n";
        }
        if(remove > -1){
            update_msgs.erase(update_msgs.begin()+remove);
        }
        update_msgs.push_back(*got);
        delete got;

        //EV << "update_msgs.size = " << update_msgs.size() << " active " << countActive() << "\n";

        if (update_msgs.size() >= countActive()){ // CHECK >= INSTEAD OF ==

            double chance = uniform(0,1);

            //compute probability to go in charge state
            EV << " E ratio is " << energyProportion(update_msgs) << ", charge time"<< chargeLength() << "\n";
            EV << "E is " << battery_lvl << "\n";
            EV << " P(X) charge = " <<  p_charge() << " rolled " << chance << "\n";

            if ((chance <= p_charge()) && (state == 1)){

                //wait a random part of a second (to avoid nodes being checked in the same order all the time, leading less priority to modules with higher ID)
                //then go in charge state for min(chargeLength(), minDischargeLength(NE))
                cancelEvent(charge);
                scheduleAt(simTime()+uniform(0,0.5), charge);
                //EV << " SCHEDULED CHARGE \n";
            }else{
                //empty the queue
                update_msgs.clear();
            }


        }


    } else if (msg->getKind() == LEAVE){
        //check if we have any message in the update_msgs queue from the SUAV that left
        //if we already had some previous message from the same SUAV, remove the old msg and add a new one

        int remove = -1;
        for (int m = 0; m < update_msgs.size(); m++){
            EV << "in list " << update_msgs.at(m).getSenderModuleId() ;
            if (msg->getSenderModuleId() == update_msgs.at(m).getSenderModuleId() ){
                remove = m;
            }
        }
        if(remove > -1){
            update_msgs.erase(update_msgs.begin()+remove);
        }

        delete msg;

    }/*else if(msg->getKind() == RANDOM_TRAFFIC){



        if (getIndex() == atoi((const char *) msg->getName())) {

            // Message arrived.
            EV << "Message " << atoi((const char *) msg) << " arrived.\n";
            delete msg;
            // generate new message
            generateMessage();

        }
        else {
            // We need to forward the message.
            randForwardMessage(msg);
        }
    }*/

    /*for(int i = 0; i < ch_stations.size(); i++){
        EV << "station " << i << " is " << ch_stations.at(i)->busy << "\n";
       }*/
}

double Suav::OP(double h){
    return gamma * h;
}

double Suav::E(int i, simtime_t j){
    //change battery level
    battery_lvl = battery_lvl - (alpha*TIMESLOT*state) + (beta*TIMESLOT*(1-state)) - (OP(h) * (prev_state) * (1-state)) - (OP(h) * (state) * (1-prev_state));
    return battery_lvl;
}

int Suav::chargeLength(){
    return floor((1.0 - (battery_lvl - 2*OP(h)))/beta);
}

int Suav::dischargeLength(double energy, double h){
    //int disch_T = floor((energy - BATTERY_THRESHOLD - 2*OP(h))/alpha);
    int disch_T = floor((energy - BATTERY_LOW - OP(h))/alpha);
    if (disch_T < 0) disch_T = 0;
    return disch_T;

}

bool Suav::isLocalMinimumCharges(std::vector<SuavMsg> updt_m){

    int min_charges = 999;

    for(int i = 0; i < updt_m.size(); i++){
        int n_charges = updt_m.at(i).getCharges();
        if ( n_charges < min_charges){
            min_charges = n_charges;
        }

    }

    if(charges <= min_charges) return true; else return false;
}

void Suav::handleCharge(){

    int charge_slots = 0;
#ifdef FIXED_CHARGE_T
    int chargeTime = ((1.0/alpha)/(n_suav+1));
    charge_slots = chargeTime;
#else
    int chargeTime = chargeLength()/n_suav;
#endif

#ifdef USE_BATTERY_LIMIT
    if(dischargeLength(battery_lvl, h) == 0)
        charge_slots = chargeTime;
    else

        charge_slots = std::min(chargeTime, minDischargeLength(update_msgs)); //check also the other modules
#endif
    //charge_slots *= p_charge(); //the time is proportional to the probability (this is to avoid that suavs go in charge with a high level of energy by chance)


#ifdef LOCAL_MIN_CHARGES
    if((charge_slots > MIN_CHARGE_TIME) && isLocalMinimumCharges(update_msgs)){
#else
    if((charge_slots > MIN_CHARGE_TIME)){
#endif

        bool one_available = false;
        station_i = -1;
        // first, check if any charging station is available
        for(int i = 0; i < ch_stations.size() && !one_available; i++){

            EV << "station " << i << " is " << ch_stations.at(i)->busy << "\n";
            if (!ch_stations.at(i)->busy){
                ch_stations.at(i)->busy = true;
                station_i = i;
                one_available = true;
            }
        }

        if(station_i > -1){

            EV << "NAP TIME! = " <<  charge_slots << " Tslot\n";
            //change suav's state
            state = 0;
            charges++;
            getDisplayString().setTagArg("i",0,"block/star"); //charge state
            scheduleAt(simTime()+charge_slots, wake_up);

            int n = gateSize("gate");
            SuavMsg *leave = new SuavMsg("leave", LEAVE);
            for(int i = 0; i < n; i++){
                cGate *dest_o = gate("gate$o", i);
                if(((Suav *) dest_o->getPathEndGate()->getOwner())->state){
                    send(leave->dup(), dest_o);
                }
            }
            delete leave;

            // send a signal to notify state change
            emit(stateChange, state);


        }else{
            EV << "no station is available\n";
        }
    }

}

void Suav::handleWakeup(){
    EV << "WAKEUP TIME!\n";
    //change suav's state
    state = 1;
    //free the charging station
    ch_stations.at(station_i)->busy = false;
    station_i = -1;

    getDisplayString().setTagArg("i",0,"misc/drone"); //active state

    //emit signal for state change (statistics)
    emit(stateChange, state);


}


#ifndef RANDOM_PROBABILITY
double Suav::p_charge(){

    // STIMULUS: chargeLength() --> the longer the suav has to charge, the higher the stimulus
    // THETA (RESPONSE THRESHOLD): depends on the battery limit threshold

#ifdef LOWER_THRESHOLD
    double resp_thr = ((CHARGE_THRESHOLD-BATTERY_LOW)/beta);
#else
    double resp_thr = ((CHARGE_THRESHOLD)/beta);
#endif

#ifdef DYNAMIC_THRESHOLD

    resp_thr += (resp_thr)*(energyProportion(update_msgs)-0.5);
            // the higher the proportion of energy held in the group, the higher the threshold (thus, a lower probability)
#endif
    int n = 15; // this helps to make the curve steepest
    return (pow(chargeLength(),n))/(pow(resp_thr,n)+(pow(chargeLength(),n)));

}
#else
double Suav::p_charge(){
    return uniform(0,1);
}
#endif


int Suav::minDischargeLength(std::vector<SuavMsg> updt_m){

    int min_dl = 999;

    for(int i = 0; i < updt_m.size(); i++){
        int dl = dischargeLength(updt_m.at(i).getEnergy(), updt_m.at(i).getHeight());
        //EV << " dl " << dl << " |";
        if ( dl < min_dl){
            min_dl = dl;
        }
    }

    return min_dl;
}


void Suav::batteryUpdateRoutine(){
    // This function is used to update the Suav energy and notify actual neighbors
    //EV << "Energy " << E(getIndex(),simTime()) << "\n";

    E(getIndex(),simTime());
    char text[32];
    sprintf(text, "n. charges %d\n", charges);
    sprintf(text, "energy: %lf\n", battery_lvl, dischargeLength(battery_lvl, h), chargeLength());
    //sprintf(text, "station %d\n", station_i);
    getDisplayString().setTagArg("t", 0, text);

    //EV << "Charge length " << chargeLength() << " Discharge length " << dischargeLength(battery_lvl, h);
    //update the previous state
    prev_state = state;
    scheduleAt(simTime()+1.0, update_ctrl_evt);
    if(state == 1){
        int n = gateSize("gate");
        char msgname[20];
        sprintf(msgname, "%d", getIndex());
        SuavMsg *status_msg = new SuavMsg(msgname, NEIGHBORS_UPDT);
        status_msg->setActive(state );
        status_msg->setHeight(h);
        status_msg->setAngle(theta);
        status_msg->setEnergy(battery_lvl);
        status_msg->setCharges(charges);
        for(int i = 0; i < n; i++) {
            cGate *out = gate("gate$o", i);
            if(SUAV_FROM_GATE(out)->state){
                send(status_msg->dup(), "gate$o", i);
            }
        }
        delete status_msg;
    }

    emit(batteryLevel, battery_lvl); //statistics

    // Handle what to do in case the suav is isolated (doesn't receive any update msg)
    // we need also an "independent" behavior for suavs
    if ((battery_lvl <= BATTERY_LOW) && (countActive() == 0) && (state == 1)){
        //wait a random part of a second (to avoid being checked in the same order all the time)
        //go in charge state for min(chargeLength(), minDischargeLength(NE))
        scheduleAt(simTime()+uniform(0,0.5), charge);
    }
}

double Suav::energyProportion(std::vector<SuavMsg> msgs){

#ifdef PROPORTION_NO_FULL
    double sum = 0;
    for(int i = 0; i < msgs.size(); i++){
        sum += msgs.at(i).getEnergy(); //get the sum of energies of all neighbors
    }
    //add mine
    sum += battery_lvl;
    //return the proportion of my energy level with respect to total amount of energy in my neighborhood
    return battery_lvl/sum;
#else
    return battery_lvl/(msgs.size()+1); // proportion with respect if they were all full (all the proportion will be lower)
#endif
}

void Suav::randForwardMessage(cMessage *msg)
{

    bool gateActive = false;

    while(!gateActive){
        // In this example, we just pick a random gate to send it on.
        // We draw a random number between 0 and the size of gate `gate[]'.
        int n = gateSize("gate");
        int k = intuniform(0, n-1);

        cGate *out = gate("gate$o", k);
        if(SUAV_FROM_GATE(out)->state){ // this is the actual state (cause it's a pointer to the suav)
            gateActive = true;
            EV << "Forwarding message " << msg << " on gate[" << k << "]\n";
            char text[32];
            sprintf(text, "Sending on gate %d", k);
            bubble(text);
            // $o and $i suffix is used to identify the input/output part of a two way gate
            send(msg, "gate$o", k);
        }
    }
}

void Suav::generateMessage(){
    int n = 9;
    int dest = intuniform(0, n-1);
    // Boot the process scheduling the initial message as a self-message.
    char msgname[20];
    sprintf(msgname, "%d", dest);
    cMessage *msg = new cMessage(msgname, RANDOM_TRAFFIC);
    scheduleAt(simTime()+0.5, msg);
}

int Suav::countActive(){
    int count_active = 0;
    for(int i=0; i < neighbors.size(); i++){
        if (neighbors.at(i)->state == 1) count_active++;
    }
    return count_active;
}

