#include "WH_Robot.h"
#include "../helper_classes/Order.h"

/** @Brief: warehouse robot constructor **/
WH_Robot::WH_Robot(std::string scoutname, Sensors* sensors):
            line_follow(scoutname, sensors) 
{
  nav_map = new navigationMap(scoutname, sensors);
  curr_task = DEFAULT_TASK;
  name = scoutname;
  reg_failed = 1;
  srand(0xDEADBEEF);
}

WH_Robot::~WH_Robot()
{
  delete(nav_map);
}

Duration WH_Robot::get_worst_case_time(State start_state, State target_state)
{
  return nav_map->get_worst_case_time(start_state, target_state);
}

/** @brief Drives from state 2 to the robot home base and goes to sleep for
 *  x seconds
 */
void WH_Robot::go_home(int x)
{
  unsigned int curr_state = nav_map->get_state();
  if(curr_state != 2)
  {
    ROS_WARN("Tried to go home from incorrect state %d", curr_state);
    return;
  }
  turn_straight();
  follow_line();
  turn_left();
  follow_line();
  Duration sleep_time(x);
  sleep_time.sleep();
  return;
}

void WH_Robot::leave_home()
{
  unsigned int curr_state = nav_map->get_state();
  if(curr_state != 2)
  {
    ROS_WARN("Tried to go home from incorrect state %d", curr_state);
    return;
  }
  turn_straight();
  follow_line();
  nav_map->update_state(ISTRAIGHT);
  turn_left();
  follow_line();
}

/** @brief Based on the given path, make
 *         the series of turns to follow this path,
 *         updating the current state in the navigation map as we do so   
 *  @param the Path to follow
 */
void WH_Robot::follow_path(Path path_to_follow){
  for(int i=0; i<path_to_follow.len; i++)
  {
    Turn t = path_to_follow.path[i];
    unsigned int curr_state = nav_map->get_state();

    nav_map->update_state(t);
    switch(t)
    {
      case ISTRAIGHT:
        turn_straight();
        follow_line();
        if(curr_state == 2)
        {
          turn_straight();
          follow_line();
          turn_straight();
          follow_line();
        }
        break;
      case ILEFT:
        turn_left();
        follow_line();
        break;
      case IRIGHT:
        if(9 <= curr_state && curr_state <= 12)
        {
          turn_straight();
          follow_line();
        }
        turn_right();
        follow_line();
        if(curr_state <= 4)
        {
          turn_straight();
          follow_line();
        }
        break;
      case IUTURN:
        u_turn();
        follow_line();
        break;
      case ISPOTTURN:
        spot_turn();
        follow_line();
        break;
    }
  }  
}

int WH_Robot::exec_task()
{
  assert(curr_task != DEFAULT_TASK);
  
  // remember where you started doing this task so we know where to return
  // State home_state = nav_map->get_state();
  // For now, use state 2.
  Path new_path = nav_map->shortest_path(curr_task->get_dest());
  curr_task->set_path(new_path);

  follow_path(new_path);
   
  //@todo: forklift yaaaay

  int did_task_complete;

  int error = rand() % 12;
  if(error < 9) //Fail with 1/4 probability
  {
    ROS_INFO("WH_robot: TASK COMPLETE!");
    did_task_complete = TASK_COMPLETED;
  }
  else
  {
    ROS_INFO("WH_robot: TASK FAILED!");
    did_task_complete = TASK_FAILED;
  }
  
  // For now use state 2 as home state
  // Path return_path = nav_map->shortest_path(home_state);
  Path return_path = nav_map->shortest_path(2);
  curr_task->set_path(return_path);
  
  follow_path(return_path);

  return did_task_complete;
}

void WH_Robot::robot_callback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data.compare(0, 11, "REG_SUCCESS") == 0)
  {
    id = atoi(msg->data.substr(12).c_str());
    reg_failed = 0;
  }
  else if(msg->data.compare(0, 8, "SET_TASK") == 0)
  {
    char* string = (char*)msg->data.c_str();
    char* data;
    data = strtok(string, " ");

    data = strtok(NULL, " ");
    int order_id = atoi(data);

    data = strtok(NULL, " ");
    Address source = atoi(data);

    data = strtok(NULL, " ");
    Address dest = atoi(data);

    Time start_time = ros::Time::now();
    Path path;
    path.len = 0;
    path.path = NULL;
    Duration est_time(0);

    curr_task = new Order(order_id, source, dest, start_time, path, est_time);
  }
  else
  {
    ROS_INFO("I got a bad message: %s", msg->data.c_str());
  }
}

void WH_Robot::run ()
{
  ROS_INFO("A WH_Robot has been created");

  robot_to_sched = node.advertise<std_msgs::String>("robot_to_sched", 1000);
  sched_to_robot = node.subscribe(name + "_topic", 1000, &WH_Robot::robot_callback, this);

  ROS_INFO("I am a robot. Registering with scheduler...");
  while(reg_failed && ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "REGISTER "<<name;
    msg.data = ss.str();
    robot_to_sched.publish(msg);

    spinOnce();
  }

  follow_line();

  while(ok())
  {
    ROS_INFO("WH_ROBOT: Running main loop");


      std_msgs::String msg;
      std::stringstream ss;
      ss << "GET_TASK "<<id;
      msg.data = ss.str();
      robot_to_sched.publish(msg);

    while(curr_task == DEFAULT_TASK && ok()){
      spinOnce();
    }

    int error = exec_task();

    /** Go to robot home base */
    go_home(2);
    leave_home();

    if(error == TASK_COMPLETED)
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "SUCCESS "<<curr_task->getid();
      msg.data = ss.str();
      robot_to_sched.publish(msg);
    }
    else //error == TASK_FAILED
    {
      std_msgs::String msg;
      std::stringstream ss;
      ss << "FAILED "<<curr_task->getid();
      msg.data = ss.str();
      robot_to_sched.publish(msg);
    }
    delete curr_task;
    curr_task = DEFAULT_TASK;

  }
}

void WH_Robot::set_task(Order order)
{
  curr_task = new Order(order.getid(), order.get_source(), order.get_dest(), 
      order.get_start_time(), order.get_path(), order.get_est_time()); 
  return;
}

