package fly_and_recharge.simulations;

import fly_and_recharge.Suav;
import fly_and_recharge.Controller;
import fly_and_recharge.Station;

// using two way connections to further simplify the network definition
network suav_fixed_net
{
    types:
        channel Channel extends ned.DelayChannel {
            delay = 100ms;
        }
    submodules:
        suav[15]: Suav;
        control: Controller;
        station[3]: Station; 
        //station[4]: Station;
        //station[5]: Station;  
        //station[6]: Station;
        
    connections:
        
        //SCENARIO 1
        
        suav[0].gate++ <--> Channel <--> suav[1].gate++;
        suav[1].gate++ <--> Channel <--> suav[2].gate++;
        suav[1].gate++ <--> Channel <--> suav[4].gate++;
        suav[3].gate++ <--> Channel <--> suav[4].gate++;
        suav[4].gate++ <--> Channel <--> suav[5].gate++;
        suav[0].gate++ <--> Channel <--> suav[6].gate++;
        suav[4].gate++ <--> Channel <--> suav[7].gate++;
        suav[3].gate++ <--> Channel <--> suav[8].gate++;
        
        suav[9].gate++ <--> Channel <--> suav[2].gate++;
        suav[9].gate++ <--> Channel <--> suav[1].gate++;
        suav[11].gate++ <--> Channel <--> suav[1].gate++;
        suav[10].gate++ <--> Channel <--> suav[7].gate++;
        suav[12].gate++ <--> Channel <--> suav[4].gate++;
        suav[12].gate++ <--> Channel <--> suav[7].gate++;
        suav[13].gate++ <--> Channel <--> suav[0].gate++;
        suav[14].gate++ <--> Channel <--> suav[9].gate++;
        
        suav[12].gate++ <--> Channel <--> suav[0].gate++;
        suav[11].gate++ <--> Channel <--> suav[3].gate++;
       
        
}

