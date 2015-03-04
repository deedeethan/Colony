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
 * @file navigationMap.cpp
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

#include "navigationMap.h"

using namespace std;

/**
 * @brief Initializes the navigation map
 *
 * Initialize the navigation map. 
 * The map itself is represented as a dynamic array of edge arrays
 * @param the string name of the scout
 */
navigationMap::navigationMap(string scoutname, Sensors* sensors) : 
            Behavior(scoutname, "navigationMap", sensors)
{
  /** Initialize Map 
   *                     _____
   *   1           2     |    | 3         4
   *  ----|-----------|--|----|--|---------|---------->
   *  <---|--5--------|--6-------|--7------|--8-------
   *      |           |          |         |
   *     9|         10|        11|       12|
   *      |           |          |         |
   *     ---13       ---14      ---15     ---16
   */
  
    Edge* a1 = new Edge[ARRAY_SIZE];
    a1[0] = MAKE_EDGE(ISTRAIGHT, 2, 10);
    a1[1] = MAKE_EDGE(IRIGHT, 13, 40);
    a1[2] = MAKE_EDGE(IUTURN, DEADEND, 0);

    Edge* a2 = new Edge[ARRAY_SIZE]; 
    a2[0] = MAKE_EDGE(ISTRAIGHT, 3, 10);
    a2[1] = MAKE_EDGE(IRIGHT, 14, 40);
    a2[2] = MAKE_EDGE(IUTURN, 5, 10);

    Edge* a3 = new Edge[ARRAY_SIZE]; 
    a3[0] = MAKE_EDGE(ISTRAIGHT, 4, 10);
    a3[1] = MAKE_EDGE(IRIGHT, 15, 40);
    a3[2] = MAKE_EDGE(IUTURN, 6, 10);

    Edge* a4 = new Edge[ARRAY_SIZE]; 
    a4[0] = MAKE_EDGE(ISTRAIGHT, DEADEND, 0);
    a4[1] = MAKE_EDGE(IRIGHT, 16, 40);
    a4[2] = MAKE_EDGE(IUTURN, 7, 10);

    Edge* a5 = new Edge[ARRAY_SIZE];
    a5[0] = MAKE_EDGE(ISTRAIGHT, DEADEND, 0);
    a5[1] = MAKE_EDGE(ILEFT, 13, 40);
    a5[2] = MAKE_EDGE(IUTURN, 2, 10);

    Edge* a6 = new Edge[ARRAY_SIZE];
    a6[0] = MAKE_EDGE(ISTRAIGHT, 5, 0);
    a6[1] = MAKE_EDGE(ILEFT, 14, 40);
    a6[2] = MAKE_EDGE(IUTURN, 3, 10);

    Edge* a7 = new Edge[ARRAY_SIZE];
    a7[0] = MAKE_EDGE(ISTRAIGHT, 6, 0);
    a7[1] = MAKE_EDGE(ILEFT, 15, 40);
    a7[2] = MAKE_EDGE(IUTURN, 4, 10);

    Edge* a8 = new Edge[ARRAY_SIZE];
    a8[0] = MAKE_EDGE(ISTRAIGHT, 7, 0);
    a8[1] = MAKE_EDGE(ILEFT, 16, 40);
    a8[2] = MAKE_EDGE(IUTURN, DEADEND, 10);

    Edge* a9 = new Edge[ARRAY_SIZE];
    a9[0] = MAKE_EDGE(IRIGHT, 2, 10);
    a9[1] = MAKE_EDGE(ILEFT, DEADEND, 0);
    a9[2] = MAKE_EDGE(ISPOTTURN, 13, 40);

    Edge* a10 = new Edge[ARRAY_SIZE];
    a10[0] = MAKE_EDGE(IRIGHT, 3, 10);
    a10[1] = MAKE_EDGE(ILEFT, 5, 10);
    a10[2] = MAKE_EDGE(ISPOTTURN, 14, 40);

    Edge* a11 = new Edge[ARRAY_SIZE];
    a11[0] = MAKE_EDGE(IRIGHT, 4, 10);
    a11[1] = MAKE_EDGE(ILEFT, 6, 10);
    a11[2] = MAKE_EDGE(ISPOTTURN, 15, 40);

    Edge* a12 = new Edge[ARRAY_SIZE];
    a12[0] = MAKE_EDGE(IRIGHT, DEADEND, 0);
    a12[1] = MAKE_EDGE(ILEFT, 7, 10);
    a12[2] = MAKE_EDGE(ISPOTTURN, 16, 40);

    Edge* a13 = new Edge[ARRAY_SIZE];
    a13[0] = MAKE_EDGE(IRIGHT, DEADEND, 0);
    a13[1] = MAKE_EDGE(ILEFT, DEADEND, 0);
    a13[2] = MAKE_EDGE(ISPOTTURN, 9, 40);

    Edge* a14 = new Edge[ARRAY_SIZE];
    a14[0] = MAKE_EDGE(IRIGHT, DEADEND, 0);
    a14[1] = MAKE_EDGE(ILEFT, DEADEND, 0);
    a14[2] = MAKE_EDGE(ISPOTTURN, 10, 40);

    Edge* a15 = new Edge[ARRAY_SIZE];
    a15[0] = MAKE_EDGE(IRIGHT, DEADEND, 0);
    a15[1] = MAKE_EDGE(ILEFT, DEADEND, 0);
    a15[2] = MAKE_EDGE(ISPOTTURN, 11, 40);

    Edge* a16 = new Edge[ARRAY_SIZE];
    a16[0] = MAKE_EDGE(IRIGHT, DEADEND, 0);
    a16[1] = MAKE_EDGE(ILEFT, DEADEND, 0);
    a16[2] = MAKE_EDGE(ISPOTTURN, 12, 40);


    map.push_back(a1);
    map.push_back(a2);
    map.push_back(a3);
    map.push_back(a4);
    map.push_back(a5);
    map.push_back(a6);
    map.push_back(a7);
    map.push_back(a8);
    map.push_back(a9);
    map.push_back(a10);
    map.push_back(a11);
    map.push_back(a12);
    map.push_back(a13);
    map.push_back(a14);
    map.push_back(a15);
    map.push_back(a16);

    curr_state = START_STATE;
    arrival_time = ros::TIME_MAX;
}

/** @brief Goes through and frees all allocated memory */
navigationMap::~navigationMap()
{
  while(!map.empty())
  {
    Edge* temp = map.back();
    map.pop_back();
    delete temp;
  }
  return;
}

/** @brief FSM implementation*/
void navigationMap::run()
{
  Duration t;

  ROS_INFO("Going to state 16\n");
  Path path = shortest_path(16);
  if(path.path == NULL)
  {
    ROS_WARN("There is no path to state 16");
    return;
  }

  ROS_INFO("Worst case time to 16 is %d", get_worst_case_time(curr_state, 6).sec);

  for(int i=0; i<path.len && ok(); i++)
  {
    update_state(path.path[i]);
    ROS_INFO("Made turn %d, at state %d\n", path.path[i], curr_state);
    t = get_time_remaining();
    while(t.sec > 0)
      t = get_time_remaining();
    ROS_INFO("Now at state %d\n", curr_state);
  }

  ROS_INFO("Traveled route!\n");

  while(ok())
    continue;
}

/**@brief sets the current state to the state associated with the turn made
 * @param the Turn that we made from our state
 * @return our new State after making the turn 
 */
State navigationMap::update_state(Turn turn_made)
{
  Edge* possible_edges = get_outbound_edges(curr_state);
  for(int i=0;i<ARRAY_SIZE;i++)
  {
    //sets the current state to the state associated with the turn made
    if(GET_EDGE_DIR(possible_edges[i]) == turn_made)
    {
      //@todo: get actual speed
      int speed = 10;
      curr_state = GET_EDGE_STATE(possible_edges[i]);
      Duration travel_time(GET_EDGE_DIST(possible_edges[i])/speed);
      arrival_time = Time::now() + travel_time;
      return curr_state;
    }
  }
  return -1;//failure to succeed
}

/**@brief returns the predicted time of arrival for our current task
 * @return the predicted time of arrival for our current task
 */
Time navigationMap::get_eta()
{
  return arrival_time;
}

/**@brief returns the predicted amount of time it will take to finish our task
 * @return the predicted amount of time it will take to finish our task
 */
Duration navigationMap::get_time_remaining()
{
  return (arrival_time - Time::now());
}

/**@brief returns the current state of the scout in the map
 * @return the current State (ie: int) of the scout in the map
 */
State navigationMap::get_state()
{
  return curr_state;
}

/**@brief returns the Edges connecting from a given State
 * @param the State whose edges we want to get
 * @return the Edges connecting from the given State
 */
Edge* navigationMap::get_outbound_edges(State state)
{
  return map.at(state-1); 
}

/**@brief uses BFS to find the shortest path to a target State node
 * @param target_state the State that we want to get to
 * @return a Path struct containing the Turn* to take to get to the target state
 */
Path navigationMap::shortest_path(State target_state)
{
  return shortest_path(curr_state, target_state);
}

/**@brief uses BFS to find the shortest path to a target State node
 * @param start_state the State that we start from
 * @param target_state the State that we want to get to
 * @return a Path struct containing the Turn* to take to get to the target state
 */
Path navigationMap::shortest_path(State start_state, State target_state)
{
  // BFS algorithm
  State curr_state = start_state;
  int visited[MAX_NODES+1] = {0};

  queue<State> q;
  q.push(curr_state);
  // not zero = visited, zero = unvisited, negative = start state
  visited[curr_state] = -1;

  while (!q.empty())
  {
    State state = q.front();
    //actually dequeue it
    q.pop();
    if (state == target_state)
    {  
      Path path;
      path.path = (Turn*)calloc(sizeof(Turn), MAX_NODES);
      int j = 0; // counter
      for(State child = state; visited[child] >= 0; 
          child = visited[child]) //while not start state
      {
        State parent = visited[child];
        Edge* edges = get_outbound_edges(parent);
        for (int i = 0; i < ARRAY_SIZE; i++)
        {
          if (GET_EDGE_STATE(edges[i]) == child)
          {
            path.path[j] = GET_EDGE_DIR(edges[i]);
            j++;
            break;
          }
        }
      }
      /** Reverse moves list */
      for (int i = 0; i < j/2; i++)
      {
        path.path[i] ^= path.path[j-i-1];
        path.path[j-i-1] ^= path.path[i];
        path.path[i] ^= path.path[j-i-1];
      }
      path.len = j;
      return path;
    }
    Edge* edges = get_outbound_edges(state);

    for (int i = 0; i < ARRAY_SIZE; i++)
    {
      State new_state = GET_EDGE_STATE(edges[i]);
      if (new_state != DEADEND && !visited[new_state]) 
      {
        // set this state in visited as the parent of the new_state
        visited[new_state] = state;
        q.push(new_state);
      }
    }
  }
  //oops, no way to get to target from state
  Path path;
  path.len = 0;
  path.path = NULL;
  return path;
}

/** @brief returns worst case time to travel to dest
 *
 *  Takes into account speed of robot and interactions with other robots
 *
 *  @param start_state Node that we start at
 *  @param target_state Node that we end up at
 *  @return the worst case time to travel to target node
 */
Duration navigationMap::get_worst_case_time(State start_state, State target_state)
{
  Path path = shortest_path(start_state, target_state);
  Duration worst_case_time(0);

  State curr_state = start_state;
  //keep iterating over path while there are turns
  for(int i=0; i<path.len; i++)
  {
    Edge* edges = get_outbound_edges(curr_state); 
    for(int j=0; j<ARRAY_SIZE; j++)
    {
      if(GET_EDGE_DIR(edges[j]) == path.path[i])
      {
        Duration turn_time(TURN_TIME + (GET_EDGE_DIST(edges[j])/SPEED));
        worst_case_time += turn_time;
      }
    }
  }
  Duration additional_time(DROPOFF_TIME + WAIT_TIME);
  worst_case_time += additional_time; 

  return worst_case_time;
}
