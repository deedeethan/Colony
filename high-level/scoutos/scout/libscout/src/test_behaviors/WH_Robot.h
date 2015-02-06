#ifndef _WH_ROBOT_
#define _WH_ROBOT_

#define DEFAULT_TASK NULL
#define TASK_COMPLETED 0
#define TASK_FAILED -1

#include "../behaviors/line_follow.h"
#include "../Behavior.h"
#include "../behaviors/navigationMap.h"
#include "../helper_classes/Order.h"
#include <assert.h>
#include <stdlib.h>

class WH_Robot : line_follow{
        std::string name;
        int id;

        int reg_failed;

        Order* curr_task;
        navigationMap* nav_map;

        Duration get_worst_case_time(State start_state, State target_state);
        int exec_task();

        void robot_callback(const std_msgs::String::ConstPtr& msg);

        void go_home(int x);
        void leave_home();

    public:
        WH_Robot(std::string scoutname, Sensors* sensors);
        ~WH_Robot();
        void run();

        void set_task(Order order);
        void follow_path(Path path_to_follow);

        ros::Publisher robot_to_sched;
        ros::Subscriber sched_to_robot;
};

#endif
