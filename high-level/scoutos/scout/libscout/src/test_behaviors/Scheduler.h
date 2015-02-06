#ifndef _SCHEDULER_
#define _SCHEDULER_

#include "../helper_classes/PQWrapper.h"
#include "../helper_classes/Order.h"
#include "../Behavior.h"

#define NUM_TASKS 5
#define WAITING_ROBOT 1
#define NEW_ROBOT 2
#define ORDERED_ROBOT 3

typedef struct{
  std::string name;
  ros::Publisher topic;
  int sched_status;
} Robot;

class Scheduler : Behavior {
  std::vector<Robot> robots;
	std::queue<Robot> waitingRobots;

  PQWrapper* unassignedOrders;
	std::vector<Order> assignedOrders;

	void create_orders();

	void waiting_dance();

  void msg_callback(const std_msgs::String::ConstPtr& msg);

public:
  Scheduler(std::string scoutname, Sensors* sensors);
	~Scheduler();
	
	void get_task(int robot);
	
	void task_complete(Order o);
	void task_failed(Order o);
	
	
	Order get_next_item();
	
	void run();

  ros::Subscriber robot_to_sched;
    
};
#endif
