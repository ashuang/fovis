#ifndef __fovis_visual_odometry_hpp__
#define __fovis_visual_odometry_hpp__

#include <stdint.h>

#include <Eigen/Geometry>

#include "keypoint.hpp"
#include "camera_intrinsics.hpp"
#include "frame.hpp"
#include "depth_source.hpp"
#include "motion_estimation.hpp"
#include "options.hpp"

namespace fovis
{

/**
 * Utility class so that the VisualOdometry class not need
 * EIGEN_MAKE_ALIGNED_OPERATOR_NEW.
 */
class VisualOdometryPriv
{
  private:
    friend class VisualOdometry;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // best estimate for current position and orientation
    Eigen::Isometry3d pose;

    // transformation relating reference frame and most recent frame
    Eigen::Isometry3d ref_to_prev_frame;

    // best estimate of motion from current to previous frame
    Eigen::Isometry3d motion_estimate;
    // the 6x6 estimate of the covriance [x-y-z, roll-pitch-yaw];
    Eigen::MatrixXd motion_estimate_covariance;

    Eigen::Matrix3d initial_homography_est;
    Eigen::Isometry3d initial_motion_estimate;
    Eigen::MatrixXd initial_motion_cov;
};

/**
 * \ingroup FovisCore
 * \brief Main visual odometry class.
 * \include: fovis/fovis.hpp
 */
class VisualOdometry
{
  public:
    VisualOdometry(const Rectification* rectification,
                   const VisualOdometryOptions& options);

    ~VisualOdometry();

    void processFrame(const uint8_t* gray, DepthSource* depth_source);

    const Eigen::Isometry3d& getPose() {
      return _p->pose;
    }

    const OdometryFrame* getReferenceFrame() const {
      return _ref_frame;
    }

    const OdometryFrame* getTargetFrame() const {
      return _cur_frame;
    }

    // If this is true, _cur_frame will be _ref_frame
    // on next call of processFrame.
    // TODO better name.
    bool getChangeReferenceFrames() const {
      return _change_reference_frames;
    }

    MotionEstimateStatusCode getMotionEstimateStatus() const {
      return _estimator->getMotionEstimateStatus();
    }

    const Eigen::Isometry3d& getMotionEstimate() const {
      return _p->motion_estimate;
    }

    const Eigen::MatrixXd& getMotionEstimateCov() const {
      return _p->motion_estimate_covariance;
    }

    const MotionEstimator* getMotionEstimator() const {
      return _estimator;
    }

    int getFastThreshold() const {
      return _fast_threshold;
    }

    // for drawing
    const Eigen::Matrix3d & getInitialHomography() const {
      return _p->initial_homography_est;
    }

    const VisualOdometryOptions& getOptions() const {
      return _options;
    }

    static VisualOdometryOptions getDefaultOptions();

    void sanityCheck() const;

  private:
    void prepareFrame(OdometryFrame* frame);

    Eigen::Quaterniond estimateInitialRotation(const OdometryFrame* prev,
                                               const OdometryFrame* cur,
                                               const Eigen::Isometry3d
                                               &init_motion_estimate =
                                               Eigen::Isometry3d::Identity());

    const Rectification* _rectification;

    OdometryFrame* _ref_frame;
    OdometryFrame* _prev_frame;
    OdometryFrame* _cur_frame;

    MotionEstimator* _estimator;

    VisualOdometryPriv* _p;

    bool _change_reference_frames;

    long _frame_count;

    // === tuning parameters ===

    int _feature_window_size;

    int _num_pyramid_levels;

    // initial feature detector threshold
    int _fast_threshold;

    // params for adaptive feature detector threshold
    int _fast_threshold_min;
    int _fast_threshold_max;
    int _target_pixels_per_feature;
    float _fast_threshold_adaptive_gain;

    bool _use_adaptive_threshold;
    bool _use_homography_initialization;

    // if there are least this many inliers in the previous motion estimate,
    // don't change reference frames.
    int _ref_frame_change_threshold;

    // Which level of the image pyramid to use for initial rotation estimation
    int _initial_rotation_pyramid_level;

    VisualOdometryOptions _options;
};

}
#endif
