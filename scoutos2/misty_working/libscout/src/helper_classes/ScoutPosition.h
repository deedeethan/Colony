#ifndef _SCOUT_POSITION_
#define _SCOUT_POSITION_

#include <string>

struct pos
{
  float x;
  float y;
  float theta;
  std::string name;

  public:
  void init();
  void set_name(std::string new_name);
  void update(float new_x, float new_y, float new_t);
};

typedef struct pos pos;



#endif
