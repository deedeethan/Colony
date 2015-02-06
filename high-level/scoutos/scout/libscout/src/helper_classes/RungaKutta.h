#ifndef _RUNGA_KUTTA_H_
#define _RUNGA_KUTTA_H_

#include <vector>

class RungaKutta
{
  public:
    static std::vector<double> rk4(
        std::vector<double> start,
        std::vector<double> (*func)(std::vector<double>),
        double loop_time);
    static std::vector<double> diff_drive_robot(std::vector<double> x);
  private:
    static std::vector<double> integrate(
        std::vector<double> a,
        std::vector<double> b,
        double t);
};

#endif
