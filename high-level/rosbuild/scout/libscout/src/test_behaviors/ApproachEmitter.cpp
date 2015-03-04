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

#include "ApproachEmitter.h"
#define TOP_BOM 0
#define BOT_BOM 1
#define LEFT_EMITTER_ID 1
#define RIGHT_EMITTER_ID 0


#define LEFTSIDE 1
#define RIGHTSIDE 2
#define REAR 3
#define NEITHER 0

#define CENTRED 2
#define SEARCH 0
#define ADJUSTED 1
#define SIGNAL_ONLY 3

#define LOST_SIGNAL_THRESH 100

using namespace std;

ApproachEmitter::ApproachEmitter(string scoutname, Sensors* sensors)
    : Behavior(scoutname, "Approach Emitter", sensors) {
    name = scoutname;
    scout_pos = new pos;
    lost_signal_time = 0;
    wheel_vel_diff = 60;

    // initallize the side index values :D UGLY!!! But best solution so far
    FRONT.push_back(0);
    LEFT.push_back(1);
    LEFT.push_back(2);
    LEFT.push_back(3);
    BACK.push_back(4);
    BACK.push_back(5);
    RIGHT.push_back(6);
    RIGHT.push_back(7);
    RIGHT.push_back(8);
    FRONT.push_back(9);

}

int ApproachEmitter::identifyPosition(
        vector<uint32_t> readings, vector<uint32_t> senders) {
    int identified = SEARCH;
    int top_em_count = 0;
    int bot_em_count = 0;
    if (accumulate(&(readings[RIGHT.front()]),
                   &(readings[RIGHT.back()])+1, 0) >= 2) {
        for (int i=RIGHT.front(); i<RIGHT.back(); i++) {
            top_em_count += (senders[i] == TOP_BOM);
            bot_em_count += (senders[i] == BOT_BOM);
        }
        if (top_em_count * bot_em_count != 0) {
            return CENTRED;
        } else if (top_em_count > 0) {
            motors->set_sides(-50, -50, MOTOR_PERCENT);
        } else {
            motors->set_sides(50, 50, MOTOR_PERCENT);
        }
        identified = ADJUSTED;
    } else if (accumulate(&(readings[LEFT.front()]),
                   &(readings[LEFT.back()])+1, 0) >= 2) {

        for (int i=LEFT.front(); i<LEFT.back(); i++) {
            top_em_count += (senders[i] == TOP_BOM);
            bot_em_count += (senders[i] == BOT_BOM);
        }
        if (top_em_count * bot_em_count != 0) {
            return CENTRED;
        } else if (top_em_count > 0) {
            motors->set_sides(50, 50, MOTOR_PERCENT);
        } else {
            motors->set_sides(-50, -50, MOTOR_PERCENT);
        }
        identified = ADJUSTED;
    }
    return identified;
}

void ApproachEmitter::searchForSignal() {
    lost_signal_time++;
    int mid = 50;
    if ((lost_signal_time) > LOST_SIGNAL_THRESH) {
        lost_signal_time = 0;
        wheel_vel_diff= (wheel_vel_diff>=0)?wheel_vel_diff-10:60;
        ROS_INFO("CHANGING SPEED! %d", wheel_vel_diff);
    }
    motors->set_sides(mid-wheel_vel_diff/2,
                      mid+wheel_vel_diff/2, MOTOR_PERCENT);
}

void ApproachEmitter::run() {
    ROS_INFO("BOM Main Running");
    BomReadings bomR;
    vector<uint32_t> r;
    vector<uint32_t> s;
    int identified = 0;
    while (ok()) {
        bomR = bom->query();
        r = bomR.readings;
        s = bomR.senders;
	identified = identifyPosition(r, s);
        if (identified == CENTRED) {
            break;
        } else if (identified == ADJUSTED) {
            lost_signal_time = 0;
        } else if (identified == SEARCH) {
            searchForSignal();
        } else {
            ROS_INFO("WTF?");
        }
    }

    update_readings();
    start_side = get_side();

    while (ok() && ((start_side == LEFTSIDE && checkBOM(3, RIGHT_EMITTER_ID)) ||
           (start_side == RIGHTSIDE && checkBOM(6, LEFT_EMITTER_ID)))) {
       motors->set_sides(25, 25, MOTOR_PERCENT);
       update_readings();

     }

    motors->set_sides(0, 0, MOTOR_PERCENT);


    while(ok() && reverse_park()) {
        update_readings();
    }


    //motors->set_sides(0,0,MOTOR_PERCENT);
    ROS_INFO("done");
/*
    while(ok()) {
	update_readings();
	if (at_destination()) {
    	  motors->set_sides(0, 0, MOTOR_PERCENT);
	  ROS_INFO("done");
	  break;
	}
	else {
	  motors->set_sides(-20,-20,MOTOR_PERCENT);

	}
    }
*/
}

bool ApproachEmitter::reverse_park() {
    if(checkBOM(5, LEFT_EMITTER_ID) ||
       checkBOM(4, RIGHT_EMITTER_ID)) { //move forward

        motors->set_sides(10, 10, MOTOR_PERCENT);
    }
    else if(checkBOM(4, LEFT_EMITTER_ID) &&
            checkBOM(5, RIGHT_EMITTER_ID)) {

        motors->set_sides(-40, -40, MOTOR_PERCENT);
    }
    else if (get_side() == REAR) {
	if (checkBOM(4,LEFT_EMITTER_ID) && 
	    !checkBOM(4, RIGHT_EMITTER_ID)) { //reverse right
	    //ROS_INFO("correct left");
	    motors->set_sides(-20, -10, MOTOR_PERCENT);
	}
	else if (!checkBOM(4, LEFT_EMITTER_ID) && 
		  checkBOM(5, RIGHT_EMITTER_ID)) { //reverse left
	    //ROS_INFO("correct right");
	    motors->set_sides(-10, -20, MOTOR_PERCENT);
	}
	else {
	    motors->set_sides(-40, -40, MOTOR_PERCENT);
	}
    }
    else {
        if (start_side == RIGHTSIDE) { //reverseright
            motors->set_sides(-40, -10, MOTOR_PERCENT);
        }
        else { //reverseleft
            motors->set_sides(-10, -40, MOTOR_PERCENT);

        }
    }

    if (at_destination()) {
      return false;
    }
    return true;
}

int ApproachEmitter::get_side() {

    if ((readings[1] > 0 || readings[2] > 0|| readings[3] > 0) &&
        (readings[6] == 0 && readings[7] == 0 && readings[8] == 0)) {
        return LEFTSIDE;
    }
    else if ((readings[6] > 0 || readings[7] > 0|| readings[8] > 0) &&
        (readings[1] == 0 && readings[2] == 0 && readings[3] == 0)) {
        return RIGHTSIDE;
    }
    else if ((readings[4] > 0) || (readings[5] > 0)) {
	return REAR;
    }
    else {
        return NEITHER;
    }
}

bool ApproachEmitter::checkBOM(int bNum, unsigned int ID) {
    if(readings[bNum] == true) {
        //ROS_INFO("Emitter ID: %d", senders[bNum]);
        return senders[bNum] == ID;
    }
    return false;

}

void ApproachEmitter::update_readings() {
    BomReadings bomR = bom->query();
    readings = bomR.readings;
    senders = bomR.senders;

}

bool ApproachEmitter::at_destination()
{
    vector<uint32_t> readings = linesensor->query();

/*   ROS_INFO("%d %d %d %d %d %d %d %d", readings[0], readings[1],
	readings[2],readings[3],readings[4],readings[5],readings[6],readings[7]);
*/
    //changed values so it detects solution
    if ( readings[0] > 75 ||
         readings[1] > 75 ||
	 readings[2] > 75 ||
	 readings[3] > 75 ||
         readings[4] > 75 ||
	 readings[5] > 75 ||
         readings[6] > 75 ||
         readings[7] > 75 )
    {
        return true;
    }
    return false;
}
