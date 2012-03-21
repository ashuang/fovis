#include "stereo-odometry.hpp"

int main(int argc, char** argv)
{
  fovis::StereoOdometry stereo_odom;
  int status = stereo_odom.initialize(argc, argv);
  if (status) { return status; }
  stereo_odom.go();
  return 0;
}
