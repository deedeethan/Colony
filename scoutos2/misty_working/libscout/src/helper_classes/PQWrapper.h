#ifndef _PQ_WRAPPER_
#define _PQ_WRAPPER_

#include <cstdlib>
#include <queue>
#include <iostream>
#include "Order.h"

class PQWrapper
{
    public:
        PQWrapper(unsigned int numElems);
        ~PQWrapper();
        
        void insert(Order o);
        Order remove(unsigned int index);
        Order peek(unsigned int index) const;
        unsigned int arraySize() const;

    private:
        Order* minElems;
        unsigned int arrCapacity, arrContents;
        std::priority_queue<Order,std::vector<Order>, CompareOrder> pq;
};
#endif
