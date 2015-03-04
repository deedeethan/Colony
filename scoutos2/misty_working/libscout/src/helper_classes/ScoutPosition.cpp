#include "ScoutPosition.h"

using namespace std;

void pos::init()
{
  x = 0;
  y = 0;
  theta = 0;
  name = std::string("");
}

void pos::set_name(string new_name)
{
  name = string(new_name);
}

void pos::update(float new_x, float new_y, float new_t)
{
  x = new_x;
  y = new_y;
  theta = new_t;
}
