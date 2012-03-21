#ifndef __fovis_calibration_hpp__
#define __fovis_calibration_hpp__

#include <Eigen/Geometry>

namespace fovis
{

/**
 * \ingroup FovisCore
 * \brief Intrinsic projection parameters for a camera.
 *
 * TODO
 */
struct CameraIntrinsicsParameters
{
  CameraIntrinsicsParameters() :
    width(0), height(0), fx(0), fy(0), cx(0), cy(0),
    k1(0), k2(0), k3(0), p1(0), p2(0)
  {}

  Eigen::Matrix<double, 3, 4> toProjectionMatrix() const
  {
    Eigen::Matrix<double, 3, 4> result;
    result <<
      fx,  0, cx, 0,
       0, fy, cy, 0,
       0,  0,  1, 0;
    return result;
  }

  int width;
  int height;
  double fx;
  double fy;
  double cx;
  double cy;

  // plumb-bob distortion model.  TODO support other distortion models
  double k1;
  double k2;
  double k3;
  double p1;
  double p2;
};

}

#endif
