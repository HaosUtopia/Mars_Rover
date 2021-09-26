#include <robot_utils/robot_utils.h>
#include <iostream>

using namespace std;
using namespace robot_utils;

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "test_robot_utils");
  
  cout << "normalize_angle(1.8) = " << normalize_angle(1.8) << endl;
  cout << "normalize_angle(-1.8) = " << normalize_angle(-1.8) << endl;
  cout << "normalize_angle(-1.35) = " << normalize_angle(-1.35) << endl;

  return 0;
}
