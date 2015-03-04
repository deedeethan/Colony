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

/**
 * @file navigationMap.h
 * @brief Contains navigation map Behavior declarations and definitions
 * 
 * Contains functions and definitions for the use of
 * navigation map Behavior 
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priya Deo
 * @author Lalitha
 * @author James
 * @author Leon
 **/

#ifndef _NAVIGATION_MAP_
#define _NAVIGATION_MAP_

#include <cstdlib>
#include <queue>
#include "../Behavior.h"
//#include "lineDrive.h" // Get turn Macros

/** Turn defintions */
#define ISTRAIGHT	0
#define ILEFT		  1
#define IRIGHT		2
#define IUTURN		3
#define ISPOTTURN 4

#define START_STATE 3

#define DEADEND 255
#define ARRAY_SIZE 3
#define MAX_NODES 16

#define SPEED 10 //distance_unit per s
#define DROPOFF_TIME 10 //seconds
#define TURN_TIME 3 //Seconds
#define NUM_ROBOTS 2 
#define WAIT_TIME (TURN_TIME * NUM_ROBOTS) //Seconds
 
/** used to extract information from an encoded Edge */
#define GET_EDGE_DIR(edge) ((edge)&0xF)
#define GET_EDGE_STATE(edge) (((edge)>>4)&0xFF)
#define GET_EDGE_DIST(edge) (((edge)>>12)&0xFFFFF)

/** used to change or build an Edge's information */
#define SET_EDGE_DIR(dir) ((dir)&0xF)
#define SET_EDGE_STATE(state) (((state)&0xFF)<<4)
#define SET_EDGE_DIST(dist) (((dist)&0xFFFFF)<<12)

#define MAKE_EDGE(dir, state, dist) \
  SET_EDGE_DIR(dir)+SET_EDGE_STATE(state)+SET_EDGE_DIST(dist)

/** an integer with a direction, an associated state, and distance
 *  encoded into its bits*/
typedef unsigned int Edge;

/** a simple number representing the number of a node*/
typedef unsigned int State;

/** a number representing a type of turn, as defined above*/
typedef unsigned int Turn;

/** a list of turns to follow a path */
typedef struct{
  int len;
  Turn* path;
} Path;

class navigationMap : Behavior
{
  /** ASCII Representation of the map
   *
   *   1           2          3         4
   *  ----|-----------|----------|---------|---------->
   *  <---|--5--------|--6-------|--7------|--8-------
   *      |           |          |         |
   *     9|         10|        11|       12|
   *      |           |          |         |
   *     ---13       ---14      ---15     ---16
   */

  public:
    /** Initializes the navigation map */
    navigationMap(std::string scoutname, Sensors* sensors);
    /** Goes through and frees all allocated memory */
    ~navigationMap();

    /** FSM implementation */
    void run();
    
    /** sets the current state to the state associated with the turn made */
    State update_state(Turn turn_made);

    /** returns the predicted time of arrival for our current task */
    Time get_eta();
    /** returns the predicted amount of time it will take to finish our task */
    Duration get_time_remaining();

    /** returns the Edges connecting from a given State */
    Edge* get_outbound_edges(State state);  

    /** returns the current state of the scout in the map*/
    State get_state();
    /** uses BFS to find the shortest path to a target State node */
    Path shortest_path(State start_state, State target_state);
    Path shortest_path(State target_state);

    /** returns the predicted worst case time it will take to travel from src to dest nodes */
    Duration get_worst_case_time(State start_state, State target_state);

  private:
    /** the dynamic array of edge arrays representing individual State nodes */
    std::vector <Edge*> map;
    /** the current State node */
    State curr_state;
    /** the predicted time of arrival for our current task */
    Time arrival_time;
};
#endif
