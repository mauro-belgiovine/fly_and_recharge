//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package fly_and_recharge;

simple Suav
{
    parameters:
        @signal[stateChange](type="long");
        @statistic[stateChange](title="status change"; source="stateChange"; record=vector,stats; interpolationmode=none);
        @signal[batteryLevel](type="double");
        @statistic[batteryLevel](title="battery level"; source="batteryLevel"; record=vector,stats; interpolationmode=none);
        
        @display("i=misc/drone");
        double h = default(30); // network height (meters)
    	double alpha = default(0.00076);
    	double beta = default(0.00019);
    	double angle = default(60); // (degrees)
    	double gamma = default(0.000038);
    	bool active = default(true);
        
    gates:
        inout gate[];  // declare two way connections
}

