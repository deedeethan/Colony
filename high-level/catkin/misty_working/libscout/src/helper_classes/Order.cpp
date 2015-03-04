#include "Order.h"

using namespace std;

/** @Brief: Regular order constructor */
Order::Order(int order_id, Address order_source, Address order_dest, 
    Time order_start_time, Path order_path, Duration order_est_time) 
{ 
    id = order_id;
    source = order_source;
    dest = order_dest;
    start_time = order_start_time;
    path = order_path;
    est_time = order_est_time;
}

/** @Brief: Get order ID */
int Order::getid() const 
{
    return id;
}  

Address Order::get_source() const
{
    return source;
}

Address Order::get_dest() const
{
    return dest;
}

Time Order::get_start_time() const
{
    return start_time;
}

Path Order::get_path() const
{
    return path;
}

Duration Order::get_est_time() const
{
    return est_time;
}

int Order::get_priority() const
{
    return start_time.toSec() + MAX_WAIT_TIME - est_time.toSec();
}

bool Order::operator==(Order& order)
{
  return this->id == order.id;
}

void Order::set_path(Path order_path)
{
    path = order_path;
    return;
}
/** @Brief: Order comparison function for PQWrapper 
 *  NOTE: In order to have a min priority queue, using c++'s pq
 *  implementation, the compare function must return true if
 *  o1 is greater than o2.
 */
bool CompareOrder::operator()(Order& o1, Order& o2) 
{
  return o1.get_priority() > o2.get_priority();
}

