#include "Scheduler.h"
#include "../helper_classes/Order.h"

using namespace std;

/** @Brief: Initialize data structures for the scheduler. */
Scheduler::Scheduler(std::string scoutname, Sensors* sensors):
            Behavior(scoutname, "Scheduler", sensors)
{
  unassignedOrders = new PQWrapper(NUM_TASKS);

	create_orders();
}

/** @Brief: Free allocatetd memory. */
Scheduler::~Scheduler()
{
  delete unassignedOrders;
}

/** @Brief: Add robot to the waiting queue. 
 *  A robot calls this function with itself
 *  and gets pushed on a list of waiting robots.
 *  When a task is availaible the scheduler, removes
 *  the robot of the waiting queue, and gives it a
 *  task.
 */
void Scheduler::get_task(int robot)
{
  Robot my_robot = robots[robot-1];
  if(my_robot.sched_status != WAITING_ROBOT)
  {
	  waitingRobots.push(my_robot);
    my_robot.sched_status = WAITING_ROBOT;
  }
}

/** @Brief: Statically add orders to the Priority Queue Wrapper.*/
void Scheduler::create_orders()
{
  Path p;
  p.len = 0;
  p.path = NULL;
  Time t = ros::Time::now();
  Duration end(0);
  Duration five(5);
  Duration three(3);
  Duration ten(10);
	Order a(1,0,13,t+ten,p,end);
  Order b(2,0,14,t+five,p,end);
  Order c(3,0,15,t+three,p,end);
  Order d(4,0,16,t,p,end);
	
	unassignedOrders->insert(a);
	unassignedOrders->insert(b);
	unassignedOrders->insert(c);
	unassignedOrders->insert(d);
}

/** @Brief: This is a confirmation that the task is complete. 
    This function removes the order from assignedOrders. */
void Scheduler::task_complete(Order o)
{
  ROS_INFO("Scheduler: Task id %d was completed", o.getid());
  for (unsigned int i=0; i<assignedOrders.size(); i++)
  {
    if (assignedOrders[i].getid()==o.getid())
    {
      assignedOrders.erase(assignedOrders.begin()+i);
    }
  }
}

/** @Brief: This is a confirmation that the task failed. 
    This function places the order back on the PQWrapper. */
void Scheduler::task_failed(Order o)
{
  ROS_INFO("Scheduler: Task id %d was failed", o.getid());
	task_complete(o);
	unassignedOrders->insert(o);
}

/** @Brief: Do a waiting dance. */
void Scheduler::waiting_dance()
{ 
    //ROS_INFO("Scheduler: TEEHEE i do a dance!");
}

/** @Brief: The scheduling algorithm. Picks a task and pops from the PQWrapper. */
Order Scheduler::get_next_item()
{
  Time t = ros::TIME_MAX; 
  double time = t.toSec();

  //Order* best;        // Unused; restore when needed
  int best_index = 0;
  for(unsigned int i=0; i<unassignedOrders->arraySize(); i++)
  {
    Order order = unassignedOrders->peek(i);

    Time end_time = order.get_start_time() - order.get_est_time();

    if(end_time.toSec() + MAX_WAIT_TIME < time) //@todo: use another function
    {
      //best = &order;
      best_index = i;
      time = end_time.toSec() + MAX_WAIT_TIME;
    }
  }

  Order ret;
  ret = unassignedOrders->remove(best_index);
  assignedOrders.push_back(ret);
  return ret;
}

void Scheduler::msg_callback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data.compare(0, 6, "FAILED") == 0)
  {
    int order_id = atoi(msg->data.substr(7).c_str());
    for(unsigned int i=0; i<assignedOrders.size(); i++)
    {
      if(assignedOrders[i].getid() == order_id)
      {
        task_failed(assignedOrders[i]);
        break;
      }
    }
  }
  else if(msg->data.compare(0, 7, "SUCCESS") == 0)
  {
    int order_id = atoi(msg->data.substr(8).c_str());
    for(unsigned int i=0; i<assignedOrders.size(); i++)
    {
      if(assignedOrders[i].getid() == order_id)
      {
        task_complete(assignedOrders[i]);
        break;
      }
    }
  }
  else if(msg->data.compare(0, 8, "GET_TASK") == 0)
  {
    ROS_INFO("SCHEDULER: got get task message");
    int robot = atoi(msg->data.substr(9).c_str());
    get_task(robot);
  }
  else if(msg->data.compare(0, 8, "REGISTER") == 0)
  {
    string robot_name = msg->data.substr(9);
    for(unsigned int i=0; i<robots.size(); i++)
    {
      if(robots[i].name.compare(robot_name) == 0)
      { 
        return;
      }
    }

    int id = robots.size() +1;
    Robot new_robot;
    new_robot.name = robot_name;
    new_robot.topic = node.advertise<std_msgs::String>(new_robot.name + "_topic", 1000);
    new_robot.sched_status = NEW_ROBOT;
    robots.push_back(new_robot);

    std_msgs::String msg;
    std::stringstream ss;
    ss<<"REG_SUCCESS "<<id;
    msg.data = ss.str();
    new_robot.topic.publish(msg);
    spinOnce();

    ROS_INFO("Registration a success");

    assert(robots.size() > 0);
  }
  else
  {
    ROS_INFO("I got a bad message: %s", msg->data.c_str());
  }

  return;
}

/** @Brief: Continuously checks for waiting robots. If no robots are waiting,
    this function calls the waiting_dance() function. */
void Scheduler::run()
{
  robot_to_sched = node.subscribe("robot_to_sched", 1000, &Scheduler::msg_callback, this);
  ROS_INFO("I am a scheduler!. Now waiting for robots");
  while(robots.size() < 1 && ok())
  {
    spinOnce();
  }
  ROS_INFO("SCHEDULER: main loop");
	while (ok())
	{
    ROS_INFO("Scheduler running");
    spinOnce();
		while((waitingRobots.empty() || unassignedOrders->arraySize()==0) && ok())
    {
      waiting_dance();
      spinOnce();
    }
  
    ROS_INFO("SCHEDULER: Setting task");
    Order next = get_next_item();
    Robot r = waitingRobots.front();

    ROS_INFO("SCHEDULER: Setting task %d to robot %s", next.getid(), r.name.c_str());
    std_msgs::String msg;
    std::stringstream ss;
    ss<<"SET_TASK "<<next.getid()<<" "<<next.get_source()<<" "<<next.get_dest();
    std::cout<<"msg: "<<ss.str()<<endl;
    msg.data = ss.str();
    r.topic.publish(msg);
  
    spinOnce();

    waitingRobots.front().sched_status = ORDERED_ROBOT;
		waitingRobots.pop();
	}
}




