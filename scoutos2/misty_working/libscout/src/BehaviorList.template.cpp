#include "BehaviorList.h"

BehaviorList::BehaviorList()
{
  behavior_list.push_back(behavior<AUTOGEN_BEHAVIOR_BASE_NAME>);
  return;
}

BehaviorList::~BehaviorList()
{
  while(!behavior_list.empty())
    behavior_list.pop_back();
}
