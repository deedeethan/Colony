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

#include "smart_runaround.h"

using namespace std;


// want to have a minimal working thing, use a big enough 
// static array and start in the middle
// we assume we are facing right, that affects where we store
// wall information
// -1 for wall, 0 for unseen, 1 for traveled, 2 for critical
#define WALL -1
#define UNSEEN 0
#define SEEN 1
#define CRITICAL 2
// facings on map, NOT on robot
#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3
// number of pixels on one row/column of map block
#define BLOCK_LENGTH 13

// robot control variables
#define BASESPEED 50

// robots row and column on map, starting at center
int row = MAP_LENGTH/2+1; // add 1 because we know MAP_LENGTH is odd
int col = MAP_LENGTH/2+1;

// utility functions
// pixels to meters
float pix_to_m(int pixels)
{
    return pixels/200.0;
}

// millimeters to pixels
int mm_to_idx(float meters)
{
    // 200 pixels per meter, and 1000 millimeters per meter
    float pixels = meters*0.2;
    float idx = pixels/BLOCK_LENGTH;
    return floor(idx+0.5);
}

float *matrix_mult(float inputs[2], float matrix[2][2])
{
    float newX = matrix[0][0]*inputs[0]+matrix[0][1]*inputs[1];
    float newY = matrix[1][0]*inputs[0]+matrix[1][1]*inputs[1];
    float output[2] = {newX, newY};
    return output;
}

// @todo This is bad! It's defined globally across all behaviors. Please fix this. -Alex
Duration sonar_update_time2(1.5);

// THIS VERSION MODIFIED BY ZANE
void smart_runaround::run(){
    
    ROS_INFO("Starting to solve the maze");
    // Go up to the first line.
    //follow_line();
    // Turn the sonar on.
    sonar->set_on();
    sonar->set_range(0, 23);
    // initialize map to all UNSEEN
    for(int r = 0; r < MAP_LENGTH; r++) {
	for(int c = 0; c < MAP_LENGTH; c++)
	    map[r][c] = UNSEEN;
    }

    // Wait for the sonar to initialize.
    while(!look_around(25, 25, RIGHT) && ok())
    {
      spinOnce();
    }

    /* Assumptions:
     * Grid has a fixed size
     * Start location unknown
     * When robot moves forward, it moves to exact center
     * of the block in front.
     */


    int dir = RIGHT; // current direction
    //int new_dir = RIGHT; // direction in which to turn after a scan
    bool success = false; // true when maze solved
    while(ok())
    {
	look_around(row, col, dir);
	// Try moving in each direction
	/*new_dir = choose_direc(row, col, UNSEEN);
	if(new_dir < 0)
	    new_dir = choose_direc(row, col, SEEN);
	if(new_dir >= 0) {
	    turn_from_to(dir, new_dir);
	    dir = new_dir;
	}*/
    }

    // Check and report final condition.
    if (success)
        ROS_INFO("YAY! I have solved the maze");
    else
        ROS_INFO("NO! The maze is unsolvable");
}

/* return a direction (if any) where adjacent block
 * is labeled "info" on map. Searches clockwise
 * starting at up. Returns -1 if no direction valid.
 */
int smart_runaround::choose_direc(int row, int col, int info)
{
    if (map[row-1][col] == info)
	return UP;
    else if (map[row][col+1] == info)
	return RIGHT;
    else if (map[row+1][col] == info)
	return DOWN;
    else if (map[row][col-1] == info)
	return LEFT;
    return -1;
}

/* this function takes in the current direction,
 * and turns the scout to the intended direction
 */
void smart_runaround::turn_from_to(int current_dir, int intended_dir) {
    switch ((4 + intended_dir - current_dir) % 4) // @todo: Try without "4 +" at start
    {
        case 0:
	    turn_straight(intended_dir);
            break;
        case 1:
            turn_right();
            break;
        case 2:
            spot_turn();
            break;
        case 3:
            turn_left();
            break;
    }
}

/* Purpose: look front, left, and right using sonar, and update
 * map accordingly. Returns true if and only if sonar is initialized.
 */
bool smart_runaround::look_around(int row, int col, int dir)
{
    int* readings = sonar->get_sonar_readings();
    spinOnce();

    // Assumption: readings are given in millimeters - Zane
    // distances with respect to robot, NOT map
    // Look to the left.
    float left_distance = readings[0]/1000.0;
    int left_idx = mm_to_idx(left_distance);
    // Look to the front.
    float front_distance = readings[36]/1000.0;
    int front_idx = mm_to_idx(front_distance);
    // Look to the right.
    float right_distance = readings[24]/1000.0;
    int right_idx = mm_to_idx(right_distance);

    ROS_INFO("front: %lf  left: %lf  right: %lf",
        front_distance, left_distance, right_distance);
    if (right_distance == 0 || front_distance == 0 || left_distance == 0)
      return false;

    // determine relative distances on map, based on robot position
    int up_d = 0;
    int right_d = 0;
    int down_d = 0;
    int left_d = 0;

    // determine upward distance
    switch (dir)
    {
	case UP:
	    up_d = front_idx;
	    right_d = right_idx;
	    down_d = 0; // unknown
	    left_d = left_idx;
	    break;
	case RIGHT:
	    up_d = left_idx;
	    right_d = front_idx;
	    down_d = right_idx;
	    left_d = 0; // unknown
	    break;
	case DOWN:
	    up_d = 0; // unknown
	    right_d = left_idx;
	    down_d = front_idx;
	    left_d = right_idx;
	    break;
	case LEFT:
	    up_d = right_idx;
	    right_d = 0; // unknown
	    down_d = left_idx;
	    left_d = front_idx;
	    break;
    }

    // change map until wall index, or until reading < 500
    // reading < 500 <=> left_idx < 8 (approx.)

    // map blocks above robot (on map)
    for(int u = 0; u < 8; u++)
    {
        if(u == up_d) {
            map[row-u][col] = (up_d)?WALL:SEEN;
            break;
        }
        map[row-u][col] = SEEN;
    }

    // map blocks to right of robot
    for(int r = 0; r < 8; r++)
    {
        if(r == right_d) {
            map[row][col+r] = (right_d)?WALL:SEEN;
                break;
        }
        map[row][col+r] = SEEN;
    }

    // map blocks under robot (on map)
    for(int d = 0; d < 8; d++)
    {
        if(d == down_d) {
            map[row+d][col] = (down_d)?WALL:SEEN;
                break;
        }
        map[row+d][col] = SEEN;
    }

    // map blocks to left of robot
    for(int l = 0; l < 8; l++)
    {
        if(l == left_d) {
            map[row][col-l] = (left_d)?WALL:SEEN;
                break;
        }
        map[row][col-l] = SEEN;
    }
    return true;
}

// @todo: test this function
// return true iff the map has been completely explored
// and only true if it "returned" to home destination after solving
bool smart_runaround::at_destination() 
{
    // check whether there is a square portion with
    // only SEEN or WALL blocks, of the size of the map.
    //int start_row = -1;       // Unused
    //int start_col = -1;       // Unused
    int seen_width = 0;
    int seen_height = 0;
    for(int r = 0; r < MAP_LENGTH; r++) {
        for(int c = 0; c < MAP_LENGTH; c++) {
            if(map[r][c] == UNSEEN) {
            //start_row = -1;
            //start_col = -1;
            seen_width = 0;
            seen_height = 0;
            }
            else {
            //start_row = r;
            //start_col = c;
            seen_width++;
            seen_height++;
            }
            if(seen_width >= 15)
            return true;
        }
    }
    return false;
}

// @todo: test these functions to make sure robot moves well
// move forward one block in direction "dir"
void smart_runaround::turn_straight(int dir)
{
    // @todo: try to use motor encoder values to move forward enough
    motors->set_sides(BASESPEED, BASESPEED, MOTOR_ABSOLUTE);
    spinOnce();
    Duration moveTime(1.0);
    moveTime.sleep();

    // update robot's position on map
    switch(dir)
    {
	case UP:
	  row--;
	  break;
	case RIGHT:
	  col++;
	  break;
	case DOWN:
	  row++;
	  break;
	case LEFT:
	  col--;
	  break;
    }
}

// turn clockwise (right)
void smart_runaround::turn_right()
{
    motors->set_sides(BASESPEED, -BASESPEED, MOTOR_ABSOLUTE);
    spinOnce();
    Duration moveTime(3.3);
    moveTime.sleep();
}

// do a 180 deg turn, NOT a barrel roll
void smart_runaround::spot_turn()
{
    motors->set_sides(BASESPEED, -BASESPEED, MOTOR_ABSOLUTE);
    spinOnce();
    Duration moveTime(6.6);
    moveTime.sleep();
}

// turn counter-clockwise (left)
void smart_runaround::turn_left()
{
    motors->set_sides(-BASESPEED, BASESPEED, MOTOR_ABSOLUTE);
    spinOnce();
    Duration moveTime(3.3);
    moveTime.sleep();
}
