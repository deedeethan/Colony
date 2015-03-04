#ifndef _ORDER_
#define _ORDER_

#include "../Behavior.h"
#include "../behaviors/navigationMap.h"

#define MAX_WAIT_TIME 120 //2 minutes in seconds

typedef unsigned int Address;

class Order {
    int id;
    Address source, dest;
    Time start_time;
    Path path;
    Duration est_time;
public:
    Order(int order_id, Address order_source, Address order_dest, 
        Time order_start_time, Path order_path, Duration order_est_time);
  
    Order() {};

    int getid() const;
    Address get_source() const;
    Address get_dest() const;
    Time get_start_time() const;
    Path get_path() const;
    Duration get_est_time() const;
    void set_path(Path order_path);
    int get_priority() const;

    bool operator==(Order& order);
    
};

class CompareOrder {
  public:
    bool operator()(Order& o1, Order& o2);   
};

#endif
