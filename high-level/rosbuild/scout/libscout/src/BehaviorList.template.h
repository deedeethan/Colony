#ifndef _BEHAVIOR_LIST_H_
#define _BEHAVIOR_LIST_H_

#include "Behavior.h"
#include "Sensors.h"
#include "AUTOGEN_BEHAVIOR_LAST_PATH"

template<typename T> Behavior* behavior(std::string scoutname, Sensors* sensors){ return (Behavior*)new T(scoutname, sensors); } 

typedef Behavior* (* behavior_func)(std::string, Sensors*);

class BehaviorList
{
  public:
  std::vector<behavior_func> behavior_list;

  //Constructor. Initializes behavior_list
  BehaviorList();
  //Destructor. Frees behavior_list;
  ~BehaviorList();
};

#endif
