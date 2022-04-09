#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>


#include <nist_gear/Model.h>

namespace Utility 
{
  bool is_same_part(const nist_gear::Model& part1, const nist_gear::Model& part2); 

  void print_part_pose(const nist_gear::Model& part); 
}

#endif 


