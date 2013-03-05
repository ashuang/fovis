#ifndef __fovis_stereo_disparity_hpp__
#define __fovis_stereo_disparity_hpp__

#include <inttypes.h>

#include "camera_intrinsics.hpp"
#include "depth_source.hpp"
#include "frame.hpp"
#include "stereo_frame.hpp"
#include "feature_match.hpp"
#include "options.hpp"
#include "motion_estimation.hpp"

// required for StereoCalibration
#include "stereo_depth.hpp"

namespace fovis
{

/**
 * \ingroup DepthSources
 * \brief Stores image data for a stereo camera pair.
 *
 * Supports VO where input is left and disparity. No right
 * image required. Disparity values are floats with no
 * disparity values given by disparity==0
 * Similar in concept to DepthImage and PrimesenseDepth
 */
class StereoDisparity : public DepthSource
{
  public:
    StereoDisparity(const StereoCalibration* calib,
                const VisualOdometryOptions& options);

    ~StereoDisparity();

    void setDisparityData(const float * disparity_data);

    virtual bool haveXyz(int u, int v);

    virtual void getXyz(OdometryFrame * frame);

    virtual void refineXyz(FeatureMatch * matches,
                           int num_matches,
                           OdometryFrame * frame);

    virtual double getBaseline() const { return _calib->getBaseline(); }

  private:
    typedef std::vector<std::pair<double, double> > Points2d;

    /** 
     * \ingroup DepthSources
     * \brief Interpolate feature position of a non-unit uvd using the 4 neighbouring pixels
     * 
     * Mirrors the interpolation method in depth_image.cpp
     */
    bool getXyzInterp(KeypointData* kpdata);    
    
    /**
     * \ingroup DepthSources
     * \brief Given a uvd, determine the xyz reprojection position
     * 
     * Equivalent to cv::reprojectImageTo3D()
     */
    Eigen::Vector3d getXyzValues(int u, int v, float disparity);
    
    const StereoCalibration* _calib;

    int _width;
    int _height;

    int _feature_window_size;

    int _num_pyramid_levels;

    // params for adaptive feature detector threshold
    int _fast_threshold;
    int _fast_threshold_min;
    int _fast_threshold_max;
    int _target_pixels_per_feature;
    float _fast_threshold_adaptive_gain;

    bool _use_adaptive_threshold;
    bool _require_mutual_match;
    double _max_dist_epipolar_line;
    double _max_refinement_displacement;

    FeatureMatcher _matcher;
    std::vector<Points2d> _matched_right_keypoints_per_level;
    std::vector<std::vector<int> > _legal_matches;

    Eigen::Matrix4d *_uvd1_to_xyz;
    
    float* _disparity_data;
    
    int _max_disparity;

    const VisualOdometryOptions _options;

    // matches buffer.
    FeatureMatch* _matches;
    int _num_matches;
    int _matches_capacity;
};

}

#endif
