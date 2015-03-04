#include "PQWrapper.h"

using namespace std;

/** @Brief: Initialize PQWrapper data structures. */
PQWrapper::PQWrapper(unsigned int numElems)
{
    minElems = new Order[numElems];
    arrCapacity = numElems;
    arrContents = 0;
}

/** @Brief: Free allocatetd memory. */
PQWrapper::~PQWrapper()
{
    while (!pq.empty()) pq.pop();
}

/** @Brief: Insert an order into the PQWrapper. */
void PQWrapper::insert(Order o)
{

    unsigned int position=0;
    bool foundPosition = false;
    while (!foundPosition && position < arrContents)
    {
        if (o.get_priority() < minElems[position].get_priority()) 
        {
            foundPosition=true;
        } 
        else 
        {
            position++;
        }
    }
    if (foundPosition)
    {
        if (arrContents<arrCapacity)
        {
            for (unsigned int i = arrContents; i >=position+1; i--) 
            {
                minElems[i] = minElems[i-1];
            }            
            minElems[position] = o;
            arrContents++;
        } 
        else 
        {
            pq.push(minElems[arrCapacity-1]);
            for (unsigned int i = arrContents-1; i >= position+1; i--) 
            {
                minElems[i] = minElems[i-1];
            }            
            minElems[position] = o;
        }
    } 
    else 
    {     
        if (arrContents<arrCapacity) 
        {
            minElems[arrContents] = o;
            arrContents++;
        } else {
            pq.push(o);
        }
    }
}

/** @Brief: Remove and return an Order from the array if 0 <= index < arrContents. */
Order PQWrapper::remove(unsigned int index)
{
    if (index >= arrContents) return minElems[-1];// @todo: can I return null?
    Order toDelete = minElems[index];
    for (unsigned int i = index; i < arrContents - 1; i++) 
    {
        minElems[i] = minElems[i+1];
    }
    if (!pq.empty()) 
    {
        minElems[arrContents-1] = pq.top();
        pq.pop();
    } 
    else 
    {
        arrContents--;
    }
    return toDelete;
}

/** @Brief: Return an Order from the array if 0 <= index < arrContents */
Order PQWrapper::peek(unsigned int index) const
{
    if (index >= arrContents)
    {
        return minElems[-1];// @todo: can I return null?
    } 
    else 
    {
        return minElems[index];
    }
}

/** @Brief: Return the size of the minElems array */
unsigned int PQWrapper::arraySize() const
{
    return arrContents;
}


/*
//For testing purposes only
int main() {
    Order a(1,0,0,101,0,0,0);
    Order b(2,0,0,11,0,0,0);
    Order c(3,0,0,91,0,0,0);
    Order d(4,0,0,21,0,0,0);
    Order e(5,0,0,41,0,0,0);
    Order f(6,0,0,81,0,0,0);
    Order g(7,0,0,31,0,0,0);
    Order h(8,0,0,71,0,0,0);
    Order i(9,0,0,51,0,0,0);
    Order j(10,0,0,61,0,0,0);

    PQWrapper pqw(5);

    pqw.insert(a);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 


    pqw.insert(b);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(c);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(d);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(e);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(f);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(g);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(h);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(i);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    pqw.insert(j);
    for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
    cout << endl; 

    for (int i=0;i<10;i++) 
    {
        pqw.remove(0);
        for (int i=0;i<pqw.arraySize();i++) cout << pqw.peek(i).getid() << " ";
        cout << endl; 
    }

    return 0;
}
*/
