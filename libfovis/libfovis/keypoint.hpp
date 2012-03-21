#ifndef __fovis_keypoint_hpp__
#define __fovis_keypoint_hpp__

#include <inttypes.h>

#include <Eigen/Core>

namespace fovis
{

    
class KeyPoint
{
  public:
    float u;
    float v;
    float score;
    KeyPoint() : u(0), v(0), score(0) {}
    KeyPoint(float u_, float v_, float score_) : u(u_), v(v_), score(score_) {}
};

/**
 * \ingroup FovisCore
 * \brief Image feature used for motion estimation
 *
 * TODO
 */
class KeypointData
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:

    /**
     * Pixel coordinates of keypoint in its pyramid level
     */
    KeyPoint kp;

    /**
     * Homogeneous coordinates of the feature in the camera frame
     */
    Eigen::Vector4d xyzw;

    /**
     * Indicates whether or not depth data is available for this keypoint.
     */
    bool has_depth;

    /**
     * Inhomogeneous coordinates of the feature in the camera frame.
     * Convenience variable, computed from xyzw.  These coordinates
     * may be infinite.
     */
    Eigen::Vector3d xyz;

    /**
     * Keypoint pixel coordinates in the base pyramid level
     */
    Eigen::Vector2d base_uv;

    /**
     * rectified keypoint pixel coordinates in the base pyramid level
     */
    Eigen::Vector2d rect_base_uv;

    /**
     * Pixel disparity, if applicable.  If not applicable (e.g., the
     * depth source is not a stereo camera), this value is NAN
     *
     * Note that the meaning of this value depends on the DepthSource used to
     * compute it.  For StereoDepth, this corresponds directly to stereo
     * disparity.  For PrimeSenseDepth, this corresponds to the "disparity"
     * values reported by the PrimeSense sensor, which are not the same as
     * stereo disparity.  See the PrimeSenseDepth class documentation for more
     * details.
     */
    float disparity;

    /**
     * Which pyramid level was this feature computed on
     */
    uint8_t pyramid_level;

    int keypoint_index;

    /**
     * to identify unique feature track externally
     */
    int track_id;

    KeypointData() :
      kp(0, 0, 0),
      xyzw(0, 0, 0, 1),
      has_depth (false),
      xyz(0, 0, 0),
      base_uv(0, 0),
      rect_base_uv(0, 0),
      disparity(NAN),
      pyramid_level(0),
      keypoint_index(0),
      track_id(0)
    {}

    void copyFrom(const KeypointData& src) {
      kp.u = src.kp.u;
      kp.v = src.kp.v;
      kp.score = src.kp.score;
      xyzw = src.xyzw;
      xyz = src.xyz;
      has_depth = src.has_depth;
      base_uv = src.base_uv;
      rect_base_uv = src.rect_base_uv;
      disparity = NAN;
      pyramid_level = src.pyramid_level;
      keypoint_index = src.keypoint_index;
      track_id = src.track_id;
    }
};

}
#endif
