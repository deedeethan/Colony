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

#include "maze_solve.h"

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
// facings
#define UP 0
#define RIGHT 1
#define DOWN 2
#define LEFT 3
#define WALL_LIMIT 500

// @todo This is bad! It's defined globally across all files. Please put it inside a good scope. -Alex
Duration sonar_update_time(1.5);

void maze_solve::run()
{    
    // @todo:first initialize map to all 0's
    ROS_INFO("Starting to solve the maze");
    // Go up to the first line.
    follow_line();
    // Turn the sonar on.
    sonar->set_on();
    sonar->set_range(0, 23);

    // Wait for the sonar to initialize.
    while(!look_around(25, 25, RIGHT) && ok())
    {
      spinOnce();      
    }
    ROS_INFO("sonar initialized");

    //spot_turn();
    // Solve the maze
    bool finished = solve(25,25, RIGHT, true);

   
    // Check and report final condition.
    if (finished)
        ROS_INFO("YAY! I have solved the maze");
    else
        ROS_INFO("NO! The maze is unsolvable");
}


bool maze_solve::solve(int row, int col, int dir, bool root)
{
    int initial_dir = dir;

    ROS_INFO("I am at direction %d", dir);

    // use backtracking to solve the maze
    if (at_destination())
        return true;

    // Wait for sonar to update.
    sonar_update_time.sleep();

    // this function should fill the adjacent cells around me with
    // wall's or paths
    while(!look_around(row, col, dir) && ok())
    {
        spinOnce();
    }

    print_board(map,row,col);

    /* try go up */
    if (map[row-1][col] != WALL && (root || initial_dir != UP))
    {
    ROS_INFO("GOING UP!");
        // Turn up.
        turn_from_to(dir, UP);
        follow_line();
        // Solve recursively.
        bool solved = solve(row-2, col, DOWN, false);
        if (solved)
        {
            return solved;
        }
        else
        {
            //Update where we are.
            dir = UP;
        }
    }
    /* try down */
    if (map[row+1][col] != WALL && (root || initial_dir != DOWN))
    {
    ROS_INFO("GOING DOWN!");
        // Turn down.
        turn_from_to(dir, DOWN);
        follow_line();
        // Solve recursively.
        bool solved = solve(row+2, col, UP, false);
        if (solved)
        {
            return solved;
        }
        else
        {
            //Update where we are.
            dir = DOWN;
        }
    }
    /* try right */
    if (map[row][col+1] != WALL && (root || initial_dir != RIGHT))
    {
    ROS_INFO("GOING RIGHT!");
        // Turn right.
        turn_from_to(dir, RIGHT);
        follow_line();
        // Solve recursively.
        bool solved = solve(row, col+2, LEFT, false);
        if (solved)
        {
            return solved;
        }
        else
        {
            //Update where we are.
            dir = RIGHT;
        }
    }
    /* try left */
    if (map[row][col-1] != WALL && (root || initial_dir != LEFT))
    {
    ROS_INFO("GOING LEFT!");
        // Turn down.
        turn_from_to(dir, LEFT);
        follow_line();
        // Solve recursively.
        bool solved = solve(row, col-2, RIGHT, false);
        if (solved)
        {
            return solved;
        }
        else
        {
            //Update where we are.
            dir = LEFT;
        }
    }

    ROS_INFO("DEAD END FOUND, TURNING BACK.");
    // we have exhausted all the options. This path is clearly a
    // dead end. go back to where we come from and return false.
    ROS_INFO("%d %d", dir, initial_dir);
    turn_from_to(dir, initial_dir);
    follow_line();
    return false;
}

// this function takes in the current direction and turns the scout
// into it intended direction
void maze_solve::turn_from_to(int current_dir, int intended_dir) {
    switch ((4 + intended_dir - current_dir) % 4) 
    {
        case 0:
            spot_turn();
            break;
        case 1:
            turn_left();
            break;
        case 2:
            turn_straight();
            break;
        case 3:
            turn_right();
            break;
    }
}

bool maze_solve::look_around(int row, int col, int dir)
{
    // look around current place using sonar
    // store whether or not
    // there is a wall into the map
    // stores at row col 2 if point is critical, 1 otherwise
    
    wait(2);    
    //ros::Duration(1).sleep(); //need to wait until sonar completely refreshes
    int* readings = sonar->get_sonar_readings();

    //spinOnce();

/*
    // Look to the left.
    int left_distance = (readings[1] + readings[0] + readings[47])/3;
    // Look to the front.
    int front_distance = (readings[35] + readings[36] + readings[37])/3;
    // Look to the right.
    int right_distance = (readings[23] + readings[24] + readings[25])/3;
*/
    // Look to the left.
    int left_distance = readings[0];
    // Look to the front.
    int front_distance = readings[36];
    // Look to the right.
    int right_distance = readings[24];

    ROS_INFO("front: %d  left: %d  right: %d", front_distance, left_distance, right_distance);

    if (right_distance == 0 || front_distance == 0 || left_distance == 0)
      return false;

    switch (dir)
    {
        case UP:
            // If the distance is less than 500, mark the area as a wall otherwise
            // mark it as seen.
            map[row][col+1] = (left_distance < WALL_LIMIT)?WALL:SEEN;
            map[row+1][col] = (front_distance < WALL_LIMIT)?WALL:SEEN;
            map[row][col-1] = (right_distance < WALL_LIMIT)?WALL:SEEN;
            break;
        case RIGHT:
            // If the distance is less than 500, mark the area as a wall otherwise
            // mark it as seen.
            map[row+1][col] = (left_distance < WALL_LIMIT)?WALL:SEEN;
            map[row][col-1] = (front_distance < WALL_LIMIT)?WALL:SEEN;
            map[row-1][col] = (right_distance < WALL_LIMIT)?WALL:SEEN;
            break;
        case DOWN:
            // If the distance is less than 500, mark the area as a wall otherwise
            // mark it as seen.
            map[row][col-1] = (left_distance < WALL_LIMIT)?WALL:SEEN;
            map[row-1][col] = (front_distance < WALL_LIMIT)?WALL:SEEN;
            map[row][col+1] = (right_distance < WALL_LIMIT)?WALL:SEEN;
            break;
        case LEFT:
            // If the distance is less than 500, mark the area as a wall otherwise
            // mark it as seen.
            map[row-1][col] = (left_distance < WALL_LIMIT)?WALL:SEEN;
            map[row][col+1] = (front_distance < WALL_LIMIT)?WALL:SEEN;
            map[row+1][col] = (right_distance < WALL_LIMIT)?WALL:SEEN;
            break;
    }

    return true;
}

//prints the current map
void maze_solve::print_board(int board[60][60], int c_row, int c_col)
{
    int lower_limit_r = 60;
    int upper_limit_r = 0;
    int lower_limit_c = 60;
    int upper_limit_c = 0;

   for(int row = 0; row<60; row++)
    {
        for(int col = 0; col<60; col++)
        {
                if (board[row][col] != UNSEEN)
                {
                   if (row < lower_limit_r)
                    {
                        lower_limit_r = row;
                    }
                   if (col < lower_limit_c)
                    {
                        lower_limit_c = col;
                    }
                   if (row > upper_limit_r)
                    {
                        upper_limit_r = row;
                    }
                   if (col > upper_limit_c)
                    {
                        upper_limit_c = col;
                    }
                }
        }
    }

    for(int row = lower_limit_r; row<=upper_limit_r; row++)
    {
        for(int col = lower_limit_c; col<=upper_limit_c; col++)
        {
                if (row == c_row && col == c_col)
                {
                    cout << "S";
                }
                else if (board[row][col] == UNSEEN)
                {
                    cout << " ";
                }
                else if (board[row][col] == WALL)
                {
                    cout << "#";
                }
                else
                {
                    cout << "O";
                }
        }
        cout << endl;
    }
}


bool maze_solve::at_destination() 
{
    vector<uint32_t> readings = linesensor->query();

    //changed values so it detects solution
    if ( readings[0] > 75 &&
         readings[1] > 75 &&
         readings[2] < 55 &&
         readings[3] > 200 &&
         readings[4] > 200 &&
         readings[5] < 55 &&
         readings[6] > 75 &&
         readings[7] > 75 )
    {
        return true;
    }
    return false;
}
