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

#ifndef _SMART_RUNAROUND_H_
#define _SMART_RUNAROUND_H_

#include "../behaviors/line_follow.h"
/* Details about map:
 * 1 meter = 200 pixels
 * 1 block is a 13x13 pixel section
 * actual meter x meter map has 15 blocks in one row/column
 * # blocks per robot map row = 2*(block per map's row)-1
 */

// number of blocks in one row/column of robot's map
#define MAP_LENGTH 29

class smart_runaround: public line_follow
{
    public:
        smart_runaround(std::string scoutname, Sensors* sensors):
            line_follow(scoutname, "smart_runaround", sensors) {};
        void run();
    private:
	int choose_direc(int row, int col, int info);
        bool solve(int row, int col, int dir);
        void turn_from_to(int current_dir, int intended_dir);
        bool look_around(int row, int col, int dir);
        bool at_destination();
	// functions for changing direction or moving
        void turn_straight(int dir);
        void turn_right();
        void spot_turn();
	void turn_left();

	/* Initialize map so that boundaries MAP_LENGTH away
	 * from the center. Then car won't go outside of
	 * map even if it starts at a boundary and goes to
	 * the other side.
         * Rows top to bottom, and columns left to right
	*/
        int map[MAP_LENGTH][MAP_LENGTH];
};
#endif

