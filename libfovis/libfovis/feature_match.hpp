#ifndef __fovis_feature_match_hpp__
#define __fovis_feature_match_hpp__

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "keypoint.hpp"

namespace fovis
{

class PyramidLevel;

enum MatchStatusCode {
  MATCH_NEEDS_DEPTH_REFINEMENT,
  MATCH_REFINEMENT_FAILED,
  MATCH_OK
};

/**
 * \ingroup FovisCore
 * \brief Represents a single image feature matched between two camera images taken at
 * different times.
 *
 * The two frames are referred to as the reference and target frames.
 *
 * TODO
 */
class FeatureMatch
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeatureMatch() :
      target_keypoint(NULL),
      ref_keypoint(NULL),
      compatibility_degree(0),
      in_maximal_clique(false),
      inlier(false),
      reprojection_error(0),
      track_id(-1)
    {
    }

    FeatureMatch(KeypointData* target_keypoint, KeypointData* ref_keypoint) :
      target_keypoint(target_keypoint),
      ref_keypoint(ref_keypoint),
      compatibility_degree(0),
      in_maximal_clique(false),
      inlier(false),
      reprojection_error(0),
      track_id(-1)
    {
      refined_target_keypoint.copyFrom(*target_keypoint);
    }
    KeypointData* target_keypoint;
    KeypointData* ref_keypoint;

    /**
     * The target keypoint, after subpixel refinement of the feature match in
     * image space.
     */
    KeypointData refined_target_keypoint;

    // binary vector, one entry for every feature match.  Each entry is 1 if
    // the motion according to this match is compatible with the motion
    // according to the other match.
    std::vector<int> consistency_vec;

    // number of 1s in consistency_vec
    int compatibility_degree;

    // is this feature match in the maximal consistency clique
    bool in_maximal_clique;

    // is this feature an inlier, used for motion estimation
    bool inlier;

    // to identify the match during outlier rejection
    int id;

    double reprojection_error;

    // to identify the feature track externally
    int track_id;

    MatchStatusCode status;
};

} /*  */

#endif
