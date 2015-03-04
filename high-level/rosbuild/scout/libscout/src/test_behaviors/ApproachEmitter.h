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

#ifndef _APPR_EMITTER_H_
#define _APPR_EMITTER_H_
#include "../Behavior.h"
#include "../Sensors.h"
#include "../helper_classes/ScoutPosition.h"
/* Details about map:
 * 1 meter = 200 pixels
 * 1 block is a 13x13 pixel section
 * actual meter x meter map has 15 blocks in one row/column
 * # blocks per robot map row = 2*(block per map's row)-1
 */

// number of blocks in one row/column of robot's map
#define MAP_LENGTH 29

class ApproachEmitter: public Behavior
{
    public:
    ApproachEmitter(std::string scoutname, Sensors* sensors);
    void run();
    private:
    int identifyPosition(std::vector<uint32_t> readings,
                     std::vector<uint32_t> senders);
    void searchForSignal();

    bool reverse_park();
    int get_side();
    bool checkBOM(int bNum, unsigned int ID);
    void update_readings();
    bool at_destination();

    std::string name;
    pos* scout_pos;
    std::vector<uint8_t> LEFT;
    std::vector<uint8_t> RIGHT;
    std::vector<uint8_t> FRONT;
    std::vector<uint8_t> BACK;

    int start_side;
    std::vector<uint32_t> readings;
    std::vector<uint32_t> senders;

    int lost_signal_time;
    int wheel_vel_diff;



};
#endif
