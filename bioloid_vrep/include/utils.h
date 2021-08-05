#ifndef BIOLOID_VREP_UTILS_H
#define BIOLOID_VREP_UTILS_H

#include <string>
#include <cstring>
#include <sstream>
#include <map>

using namespace std;

extern string int_to_str(int input);
extern string get_member_str(int no);
extern void map_plus(map<string, float> &para1, map<string, float> para2);

#endif //BIOLOID_VREP_UTILS_H
