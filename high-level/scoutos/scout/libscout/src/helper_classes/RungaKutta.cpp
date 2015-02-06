#include "RungaKutta.h"
#include <math.h>

using namespace std;

vector<double> RungaKutta::integrate(vector<double> a, vector<double> b, double t)
{
  //Copy over the a vector.
  vector<double> result(a);
  //Add half a timestep to the starting conditions.
  for(unsigned int i=0; i<a.size(); i++)
  {
    result[i] += t*b[i];
  }
  //Return the integrated result.
  return result;
}

vector<double> RungaKutta::diff_drive_robot(vector<double> x)
{
  vector<double> dx(5,0);
  dx[0] = x[3] * cos(x[2]);
  dx[1] = x[3] * sin(x[2]);
  dx[2] = x[4];
  dx[3] = x[3];
  dx[4] = x[4];
  return dx;
}

vector<double> RungaKutta::rk4(vector<double> start,
    vector<double> (*func)(vector<double>),
    double loop_time)
{
  vector<double> k0 = func(start);
  vector<double> temp_k0 = integrate(start, k0, loop_time/2);
  vector<double> k1 = func(temp_k0);
  vector<double> temp_k1 = integrate(start, k1, loop_time/2);
  vector<double> k2 = func(temp_k1);
  vector<double> temp_k2 = integrate(start, k2, loop_time);
  vector<double> k3 = func(temp_k2);
  
  // Average out each loop's result.
  vector<double> result(start);
  for(unsigned int i=0; i<result.size(); i++)
  {
    result[i] = loop_time/6*(k0[i]+2*k1[i]+2*k2[i]+k3[i]);
  }
  return result;
}
