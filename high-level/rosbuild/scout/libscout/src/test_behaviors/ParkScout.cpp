/**
 * Copyright (c) 2011 Colony Project
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "ParkScout.h"
#define LEFT_EMITTER_ID 1
#define RIGHT_EMITTER_ID 0


#define LEFT 1
#define RIGHT 2
#define NEITHER 0

using namespace std;

ParkScout::ParkScout(string scoutname, Sensors* sensors) 
    : Behavior(scoutname, "Park Scout", sensors) {
    name = scoutname;
    scout_pos = new pos;
}

void ParkScout::run() {
    
    update_readings();
    start_side = get_side();

    while (ok() && ((start_side == LEFT && checkBOM(3, RIGHT_EMITTER_ID)) ||
           (start_side == RIGHT && checkBOM(6, LEFT_EMITTER_ID)))) {
       motors->set_sides(25, 25, MOTOR_PERCENT);
       update_readings();
	
     }

    motors->set_sides(0, 0, MOTOR_PERCENT);
           
    
    while(ok()) {  
	update_readings(); 
        reverse_park(); 
    }



}

bool ParkScout::reverse_park() {
    if(checkBOM(5, LEFT_EMITTER_ID) ||
       checkBOM(4, RIGHT_EMITTER_ID)) { //move forward
        motors->set_sides(10, 10, MOTOR_PERCENT);
    }
    else if(checkBOM(4, LEFT_EMITTER_ID) && 
            checkBOM(5, RIGHT_EMITTER_ID)) {
        motors->set_sides(-20, -20, MOTOR_PERCENT);
	return true;
    }
    else {   
        if (start_side == RIGHT) { //reverseright
            motors->set_sides(-40, -10, MOTOR_PERCENT);
        }
        else { //reverseleft
            motors->set_sides(-10, -40, MOTOR_PERCENT);
        
        }
    }
    return false;    
}

int ParkScout::get_side() {
    
    if ((readings[1] > 0 || readings[2] > 0|| readings[3] > 0) &&
	(readings[6] == 0 && readings[7] == 0 && readings[8] == 0)) {
	return LEFT;
    }
    else if ((readings[6] > 0 || readings[7] > 0|| readings[8] > 0) &&
	(readings[1] == 0 && readings[2] == 0 && readings[3] == 0)) {
	return RIGHT;
    }
    else {
    	return NEITHER;
    }
}

bool ParkScout::checkBOM(int bNum, unsigned int ID) {    
    if(readings[bNum] == true) {
        //ROS_INFO("Emitter ID: %d", senders[bNum]);
        return senders[bNum] == ID;
    }    
    return false;
    
}

void ParkScout::update_readings() {
    BomReadings bomR = bom->query();
    readings = bomR.readings;
    senders = bomR.senders;

}
