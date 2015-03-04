/**
 * Copyright (c) 2011 Colony Project
 * 
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 **/

/**
 * @file noise_gen.c
 * @brief Contains functions that generate noise for scoutsim sensor data
 *
 * @ingroup scoutsim
 *
 * Uses central limit theorem to generate gaussian noise.
 *
 * @author Colony Project, CMU Robotics Club
 * @author Priyanka Deo
 * @{
 **/

#include "noise_gen.h"
#include <stdio.h>

int main()
{
  printf("%f \n", add_gaussian_noise(5, 2));
  printf("%f \n", add_gaussian_noise(5, 2));
  printf("%f \n", add_gaussian_noise(5, 2));
  printf("%f \n", add_gaussian_noise(5, 2));
  printf("%f \n", add_gaussian_noise(5, 2));
}

float add_gaussian_noise(float reading, float max_dev)
{
  float unit_normal = get_unit_normal_random();
  return reading + max_dev*unit_normal;
}

float get_unit_normal_random()
{
  int n=5;
  float sum;

  for(int i=0; i<n; i++)
  {
    sum += rand()/double(RAND_MAX);
    printf("%f \t", sum);
  }

  sum -= (float)n/2.0;
  sum *= 2.0/(float)n;
  return sum;
}

/** @} */
