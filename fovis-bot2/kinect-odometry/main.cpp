#include <stdio.h>
#include <getopt.h>
#include <signal.h>

#include <zlib.h>
#include <lcm/lcm.h>

#include <lcmtypes/kinect_frame_msg_t.h>
#include <lcmtypes/bot_core.h>
#include <lcmtypes/fovis_update_t.h>
#include <lcmtypes/fovis_stats_t.h>

#ifdef USE_LCMGL
#include <GL/gl.h>
#include <bot_lcmgl_client/lcmgl.h>
#endif

#include <fovis/fovis.hpp>

#include "jpeg-utils.h"

using namespace std;
using namespace fovis;

typedef struct _State
{
  lcm_t* subscribe_lcm;
  lcm_t* publish_lcm;
  string kinect_frame_channel;
  VisualOdometry* odom;
  PrimeSenseCalibrationParameters kinect_params;
  PrimeSenseCalibration* kinect_calib;
  PrimeSenseDepth* depth_producer;

  string input_log_fname;
  string odom_channel;
  string pose_channel;
  bool publish_pose;

  string frame_update_channel;
  string fovis_stats_channel;
  bool publish_frame_update;
  bool publish_fovis_stats;

  int width;
  int height;

  int64_t utime_cur;
  int64_t utime_prev;

  uint8_t* rgb_buf;
  int rgb_buf_size;

  uint8_t* gray_buf;
  int gray_buf_size;

  uint16_t* disparity_buf;
  int disparity_buf_size;

#ifdef USE_LCMGL
  bool draw_lcmgl;
  bot_lcmgl_t* lcmgl;
#endif

} State;


sig_atomic_t shutdown_flag = 0;
static void
sig_action(int signal, siginfo_t *s, void *user)
{
  fprintf(stderr,"Shutting Down!\n");
  shutdown_flag = 1;
}

static int
_pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
  int i, j;
  for (i=0; i<height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j=0; j<width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] +
        0.7154 * srow[j*3+1] +
        0.0721 * srow[j*3+2];
    }
  }
  return 0;
}

#ifdef USE_LCMGL
static void
draw_pyramid_level(State* state, int level_num)
{
#if 0
  // draw point cloud
  Eigen::Isometry3d rotation;
  rotation.setIdentity();
  rotation.rotate(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitZ()));
  rotation.rotate(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitX()));

  double depth_to_depth_xyz_data[16];
  kinect_calib_get_depth_uvd_to_depth_xyz_4x4(_calib, depth_to_depth_xyz_data);
  Eigen::Matrix4d depth_to_depth_xyz(Eigen::Matrix4d::Map(depth_to_depth_xyz_data).transpose());
  int max_disparity_index = width * height - 1;

  bot_lcmgl_point_size(_lcmgl, 3.0f);
  bot_lcmgl_begin(_lcmgl, GL_POINTS);
  for(int u_rgb=0; u_rgb<width; u_rgb++) {
      for(int v_rgb=0; v_rgb<height; v_rgb++) {
          int rgb_index = v_rgb * width + u_rgb;
          int disparity_index = _cur_frame->rgb_to_disparity_map[rgb_index];

          // ignore pixels with unknown disparity
          if(disparity_index > max_disparity_index)
              continue;

          uint16_t disparity = _cur_frame->disparity[disparity_index];
          float d = 0.125 * (_calib->shift_offset - disparity);
          int u_disp = disparity_index % width;
          int v_disp = disparity_index / width;

          // compute homogeneous coordinates of matching disparity pixel
          Eigen::Vector4d uvd_depth(u_disp, v_disp, d, 1);
          Eigen::Vector4d xyzw_depth = depth_to_depth_xyz * uvd_depth;
          Eigen::Vector3d xyz_depth(xyzw_depth[0] / xyzw_depth[3],
                  xyzw_depth[1] / xyzw_depth[3],
                  xyzw_depth[2] / xyzw_depth[3]);

          xyz_depth = rotation * xyz_depth;

          float c = _cur_frame->rectified_gray[v_rgb * _cur_frame->rectified_gray_stride + u_rgb] / 255.;
          bot_lcmgl_color3f(_lcmgl, c, c, c);
          bot_lcmgl_vertex3f(_lcmgl, xyz_depth(0), xyz_depth(1), xyz_depth(2));
      }
  }
  bot_lcmgl_end(_lcmgl);
  double cxyz[3] = { 0, 0, 0 };
  bot_lcmgl_circle(_lcmgl, cxyz, 0.1);
#endif

#if 0
  // draw feature matches in Cartesian space
  bot_lcmgl_point_size(_lcmgl, 3.0f);
  for(int i=0, nmatches=matches.size(); i<nmatches; i++) {
    FeatureMatch& match = matches[i];
    if(!match.inlier)
        continue;
    const Eigen::Vector3d& ref_point = ref_frame->keypoints_xyz[match.ref_feature_index];
    const Eigen::Vector3d& target_point = _cur_frame->keypoints_xyz[match.target_feature_index];
    bot_lcmgl_begin(_lcmgl, GL_POINTS);
    bot_lcmgl_color3f(_lcmgl, 1, 0, 1);
    bot_lcmgl_vertex3d(_lcmgl, ref_point(0), ref_point(1), ref_point(2));
//    bot_lcmgl_color3f(_lcmgl, 0, 1, 0);
//    bot_lcmgl_vertex3d(_lcmgl, target_point(0), target_point(1), target_point(2));
    bot_lcmgl_end(_lcmgl);
    bot_lcmgl_begin(_lcmgl, GL_LINES);
    bot_lcmgl_color3f(_lcmgl, 0, 0, 1);
    bot_lcmgl_vertex3d(_lcmgl, ref_point(0), ref_point(1), ref_point(2));
    bot_lcmgl_vertex3d(_lcmgl, target_point(0), target_point(1), target_point(2));
    bot_lcmgl_end(_lcmgl);

//    double ref_xyz[3] = { ref_point(0), ref_point(1), ref_point(2) };
//    char txt[500];
//    bot_lcmgl_color3f(_lcmgl, 1, 1, 1);
//    const KeyPoint& kp = ref_frame->keypoints[match.ref_feature_index];
//    int rgb_index = kp.v * width + kp.u;
//    uint32_t disparity_index = _cur_frame->rgb_to_disparity_map[rgb_index];
//    uint16_t disparity = _cur_frame->disparity[disparity_index];
//    float d = 0.125 * (_calib->shift_offset - disparity);
//    int u_disp = disparity_index % width;
//    int v_disp = disparity_index / width;
//    snprintf(txt, 80, "%d / %d - (%d, %d) - (%d, %d, %f) - %.3f, %.3f, %.3f)", i, nmatches,
//        kp.u, kp.v,
//        u_disp, v_disp, d,
//        ref_point(0), ref_point(1), ref_point(2));
//    bot_lcmgl_text(_lcmgl, ref_xyz, txt);
  }
#endif

#if 0
    // debugging... draw the consistency graph
    bot_bgl_point_size(self->bgl, 4);
    bot_bgl_enable(self->bgl, GL_BLEND);
    bot_bgl_blend_func(self->bgl, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    bot_bgl_color4f(self->bgl, 0, 0, 1, 0.01);
    bot_bgl_begin(self->bgl, GL_LINES);
    foreach(vo_feature_t &af, self->cur_features) {
        for(int i=0,n=af.consistency_vec.size(); i<n; i++) {
            if(af.consistency_vec[i]) {
                vo_feature_t &cf = self->cur_features[i];
                bot_bgl_vertex2d(self->bgl, af.col, af.row);
                bot_bgl_vertex2d(self->bgl, cf.col, cf.row);
            }
        }
    }
    bot_bgl_end(self->bgl);
    bot_bgl_disable(self->bgl, GL_BLEND);
#endif

  bot_lcmgl_t* lcmgl = state->lcmgl;

  const OdometryFrame* ref_frame = state->odom->getReferenceFrame();
  const OdometryFrame* target_frame = state->odom->getTargetFrame();
  const PyramidLevel* ref_level = ref_frame->getLevel(level_num);
  const PyramidLevel* target_level = target_frame->getLevel(level_num);

  int width = ref_level->getWidth();
  int height = ref_level->getHeight();

  const MotionEstimator* estimator = state->odom->getMotionEstimator();
  const FeatureMatch* matches = estimator->getMatches();
  int num_matches = estimator->getNumMatches();

  int x_offset = 0;
  int y_offset = 0;
  for(int i=0; i<level_num; i++) {
      x_offset += (state->kinect_params.width >> i) + 10;
  }

  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_rotated(lcmgl, -90, 0, 0, 1);
  bot_lcmgl_scalef(lcmgl, 10.0 / state->kinect_params.width, -10.0 / state->kinect_params.width, 1);
  bot_lcmgl_translated(lcmgl, x_offset, 0, 0);

  // previous image
  bot_lcmgl_color3f(lcmgl, 1,1,1);
  const uint8_t* ref_gray = ref_level->getGrayscaleImage();
  int ref_gray_stride = ref_level->getGrayscaleImageStride();
  int prev_gray_texid = bot_lcmgl_texture2d(lcmgl, ref_gray,
      width, height, ref_gray_stride,
      BOT_LCMGL_LUMINANCE, BOT_LCMGL_UNSIGNED_BYTE, BOT_LCMGL_COMPRESS_NONE);

  bot_lcmgl_push_matrix(lcmgl);
  bot_lcmgl_translated(lcmgl, 0, height + 10, 0);
  bot_lcmgl_texture_draw_quad(lcmgl, prev_gray_texid,
      0, 0, 0,
      0, height, 0,
      width, height, 0,
      width, 0, 0);

  // draw features in reference frame
  bot_lcmgl_color3f(lcmgl, 1, 0, 1);
  bot_lcmgl_point_size(lcmgl, 1.5f);
  bot_lcmgl_begin(lcmgl, GL_POINTS);
  for(int i=0, nfeatures=ref_level->getNumKeypoints(); i<nfeatures; i++) {
    const KeyPoint& kp = ref_level->getKeypoint(i);
    bot_lcmgl_vertex2f(lcmgl, kp.u, kp.v);
  }
  bot_lcmgl_end(lcmgl);
  bot_lcmgl_pop_matrix(lcmgl);

  // current image
  bot_lcmgl_color3f(lcmgl, 1,1,1);
  const uint8_t* target_gray = target_level->getGrayscaleImage();
  int target_gray_stride = target_level->getGrayscaleImageStride();
  int gray_texid = bot_lcmgl_texture2d(lcmgl, target_gray,
      width, height, target_gray_stride,
      BOT_LCMGL_LUMINANCE, BOT_LCMGL_UNSIGNED_BYTE,
      BOT_LCMGL_COMPRESS_NONE);
  bot_lcmgl_texture_draw_quad(lcmgl, gray_texid,
      0, 0, 0,
      0, height, 0,
      width, height, 0,
      width, 0, 0);

  // draw features
  bot_lcmgl_color3f(lcmgl, 0, 1, 0);
  bot_lcmgl_point_size(lcmgl, 3.0f);
  bot_lcmgl_begin(lcmgl, GL_POINTS);
  for(int i=0, nfeatures=target_level->getNumKeypoints(); i<nfeatures; i++) {
    const KeyPoint& kp = target_level->getKeypoint(i);
    bot_lcmgl_vertex2f(lcmgl, kp.u, kp.v);
  }
  bot_lcmgl_end(lcmgl);

  // draw matches that are not in the maximal clique
  bot_lcmgl_color3f(lcmgl, 0.3, 0, 0);
  bot_lcmgl_begin(lcmgl, GL_LINES);
  for(int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if(match.inlier || match.in_maximal_clique || match.target_keypoint->pyramid_level != level_num)
        continue;
    int cur_x = match.target_keypoint->kp.u;
    int cur_y = match.target_keypoint->kp.v;
    int prev_x = match.ref_keypoint->kp.u;
    int prev_y = match.ref_keypoint->kp.v;
    bot_lcmgl_vertex2f(lcmgl, cur_x, cur_y);
    bot_lcmgl_vertex2f(lcmgl, prev_x, prev_y + height + 10);
  }
  bot_lcmgl_end(lcmgl);

  // draw inliers
  bot_lcmgl_color3f(lcmgl, 0, 0, 1);
  bot_lcmgl_line_width(lcmgl, 2.0);
  bot_lcmgl_begin(lcmgl, GL_LINES);
  for(int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if(!match.inlier || match.target_keypoint->pyramid_level != level_num)
        continue;
    int cur_x = match.target_keypoint->kp.u;
    int cur_y = match.target_keypoint->kp.v;
    int prev_x = match.ref_keypoint->kp.u;
    int prev_y = match.ref_keypoint->kp.v;
    bot_lcmgl_vertex2f(lcmgl, cur_x, cur_y);
    bot_lcmgl_vertex2f(lcmgl, prev_x, prev_y + height + 10);
  }
  bot_lcmgl_end(lcmgl);

  // draw matches that are in the maximal clique but failed the projection test
  bot_lcmgl_line_width(lcmgl, 1.0);
  for(int i=0; i<num_matches; i++) {
    const FeatureMatch& match = matches[i];
    if(match.in_maximal_clique && !match.inlier && match.target_keypoint->pyramid_level == level_num) {
      int cur_x = match.target_keypoint->kp.u;
      int cur_y = match.target_keypoint->kp.v;
      int prev_x = match.ref_keypoint->kp.u;
      int prev_y = match.ref_keypoint->kp.v;
      bot_lcmgl_color3f(lcmgl, 1, 0, 0);
      bot_lcmgl_begin(lcmgl, GL_LINES);
      bot_lcmgl_vertex2f(lcmgl, cur_x, cur_y);
      bot_lcmgl_vertex2f(lcmgl, prev_x, prev_y + height + 10);
      bot_lcmgl_end(lcmgl);

      bot_lcmgl_color3f(lcmgl, 1, 1, 1);
      double cur_xyz[] = { cur_x, cur_y + 10, 0 };
      char txt[500];
      snprintf(txt, 80, "%.3f", match.reprojection_error);
      bot_lcmgl_text(lcmgl, cur_xyz, txt);
      double prev_xyz[] = { prev_x, prev_y + height + 15, 0 };
      bot_lcmgl_text(lcmgl, prev_xyz, txt);
    }
  }


  if (level_num ==0){
    //draw the ESM homography estimate
    bot_lcmgl_line_width(lcmgl, 2.0);
    bot_lcmgl_color3f(lcmgl,1,1,0);
    bot_lcmgl_begin(lcmgl,GL_LINE_STRIP);
    const Eigen::Matrix3d & H = state->odom->getInitialHomography();
    Eigen::MatrixXd vertices(5,3);
    vertices << 0,0,1,
        width,0,1,
        width,height,1,
        0,height,1,
        0,0,1;

    Eigen::MatrixXd warpedPoints = H*vertices.transpose();
    warpedPoints.row(0) = warpedPoints.row(0).array()/warpedPoints.row(2).array();
    warpedPoints.row(1) = warpedPoints.row(1).array()/warpedPoints.row(2).array();
    for (int i=0;i<warpedPoints.cols();i++){
      bot_lcmgl_vertex2f(lcmgl,warpedPoints(0,i),warpedPoints(1,i));
    }
    bot_lcmgl_end(lcmgl);
  }


  bot_lcmgl_pop_matrix(lcmgl);
}

static void
draw_lcmgl(State* state)
{
  const OdometryFrame* target_frame = state->odom->getTargetFrame();
  int num_levels = target_frame->getNumLevels();

  for(int i=0; i<num_levels; i++) {
      draw_pyramid_level(state, i);
  }

  bot_lcmgl_switch_buffer(state->lcmgl);
}
#endif

void on_kinect_frame(const lcm_recv_buf_t *rbuf, const char *channel,
    const kinect_frame_msg_t *msg, void *user_data)
{
  State* state = static_cast<State*>(user_data);

  state->utime_prev = state->utime_cur;
  state->utime_cur = msg->timestamp;

  // extract image data
  const uint8_t* gray = state->gray_buf;
  if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB) {
      // image came in as raw RGB buffer.  convert to grayscale
      _pixel_convert_8u_rgb_to_8u_gray(state->gray_buf, state->kinect_params.width,
              state->kinect_params.width, state->kinect_params.height, msg->image.image_data, state->kinect_params.width*3);
  } else if(msg->image.image_data_format == KINECT_IMAGE_MSG_T_VIDEO_RGB_JPEG) {
      // image came in as JPEG-compressed data.  Decompress straight to grayscale
      jpeg_decompress_8u_gray(msg->image.image_data, msg->image.image_data_nbytes,
              state->gray_buf, state->kinect_params.width, state->kinect_params.height, state->kinect_params.width);
  }


  // extract depth data
  const uint16_t* disparity = NULL;
  if(msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_NONE) {
      disparity = (uint16_t*) msg->depth.depth_data;
  } else if (msg->depth.compression == KINECT_DEPTH_MSG_T_COMPRESSION_ZLIB) {
      unsigned long dlen = msg->depth.uncompressed_size;
      uncompress((uint8_t*)state->disparity_buf, &dlen, msg->depth.depth_data, msg->depth.depth_data_nbytes);
      disparity = state->disparity_buf;
  }

  state->depth_producer->setDisparityData(disparity);

  state->odom->processFrame(gray, state->depth_producer);

  Eigen::Isometry3d cam_to_local = state->odom->getPose();

  // rotate coordinate frame so that look vector is +X, and up is +Z
  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;
  cam_to_local = M * cam_to_local;
  Eigen::Vector3d translation(cam_to_local.translation());
  Eigen::Quaterniond rotation(cam_to_local.rotation());
  rotation = rotation * M.transpose();

  Eigen::Isometry3d motion_estimate = state->odom->getMotionEstimate();
  fovis_update_t update_msg;
  update_msg.timestamp = state->utime_cur;
  update_msg.prev_timestamp = state->utime_prev;
  Eigen::Vector3d motion_T = motion_estimate.translation();
  update_msg.translation[0] = motion_T(0);
  update_msg.translation[1] = motion_T(1);
  update_msg.translation[2] = motion_T(2);
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(motion_estimate.rotation());
  update_msg.rotation[0] = motion_R.w();
  update_msg.rotation[1] = motion_R.x();
  update_msg.rotation[2] = motion_R.y();
  update_msg.rotation[3] = motion_R.z();

  //get the cov
  const Eigen::MatrixXd & motion_cov = state->odom->getMotionEstimateCov();
  for (int i=0;i<6;i++)
    for (int j=0;j<6;j++)
      update_msg.covariance[i][j] =motion_cov(i,j);

  const MotionEstimator* me = state->odom->getMotionEstimator();
  MotionEstimateStatusCode estim_status = me->getMotionEstimateStatus();
  switch(estim_status) {
    case NO_DATA:
      break;
    case SUCCESS:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_VALID;
      printf("Inliers: %4d  Rep. fail: %4d Matches: %4d Feats: %4d Mean err: %5.2f\n",
          me->getNumInliers(),
          me->getNumReprojectionFailures(),
          me->getNumMatches(),
          (int) state->odom->getTargetFrame()->getNumKeypoints(),
          me->getMeanInlierReprojectionError());
      break;
    case INSUFFICIENT_INLIERS:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_INSUFFICIENT_FEATURES;
      printf("Insufficient inliers\n");
      break;
    case OPTIMIZATION_FAILURE:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_DEGENERATE;
      printf("Unable to solve for rigid body transform\n");
      break;
    case REPROJECTION_ERROR:
      update_msg.estimate_status = FOVIS_UPDATE_T_ESTIMATE_REPROJECTION_ERROR;
      printf("Excessive reprojection error (%f).\n", me->getMeanInlierReprojectionError());
      break;
    default:
      printf("Unknown error (this should never happen)\n");
      break;
  }

  if (estim_status != NO_DATA) {
    fovis_update_t_publish(state->publish_lcm, state->odom_channel.c_str(), &update_msg);
  }

  if (estim_status != NO_DATA && state->publish_fovis_stats) {
    fovis_stats_t stats_msg;
    stats_msg.timestamp = update_msg.timestamp;
    stats_msg.num_matches = me->getNumMatches();
    stats_msg.num_inliers = me->getNumInliers();
    stats_msg.mean_reprojection_error = me->getMeanInlierReprojectionError();
    stats_msg.num_reprojection_failures = me->getNumReprojectionFailures();
    const OdometryFrame * tf(state->odom->getTargetFrame());
    stats_msg.num_detected_keypoints = tf->getNumDetectedKeypoints();
    stats_msg.num_keypoints = tf->getNumKeypoints();
    stats_msg.fast_threshold = state->odom->getFastThreshold();
    fovis_stats_t_publish(state->publish_lcm, state->fovis_stats_channel.c_str(), &stats_msg);
  }

  // publish current pose
  if (state->publish_pose) {
      bot_core_pose_t pose_msg;
      memset(&pose_msg, 0, sizeof(pose_msg));
      pose_msg.utime = msg->timestamp;
      pose_msg.pos[0] = translation[0];
      pose_msg.pos[1] = translation[1];
      pose_msg.pos[2] = translation[2];
      pose_msg.orientation[0] = rotation.w();
      pose_msg.orientation[1] = rotation.x();
      pose_msg.orientation[2] = rotation.y();
      pose_msg.orientation[3] = rotation.z();
      bot_core_pose_t_publish(state->publish_lcm, state->pose_channel.c_str(), &pose_msg);
      //  printf("[%6.2f %6.2f %6.2f]\n", translation[0], translation[1], translation[2]);
  }

  if (state->publish_frame_update) {
    //publish the frame update message as well
    bot_core_rigid_transform_t iso_msg;
    iso_msg.utime = msg->timestamp;
    for (int i = 0; i < 3; i++)
      iso_msg.trans[i] = translation[i];
    iso_msg.quat[0] = rotation.w();
    iso_msg.quat[1] = rotation.x();
    iso_msg.quat[2] = rotation.y();
    iso_msg.quat[3] = rotation.z();
    bot_core_rigid_transform_t_publish(state->publish_lcm,state->frame_update_channel.c_str(), &iso_msg);

  }

#ifdef USE_LCMGL
  if(state->draw_lcmgl) {
      draw_lcmgl(state);
  }
#endif
}

static void
usage(const char* progname)
{
  fprintf(stderr, "Usage: %s [options]\n"
      "\n"
      "Options:\n"
      "  -c, --channel CHAN           Kinect channel to subscribe\n"
      "  -p, --pose [CHAN]            Publish pose messages on specified channel.\n"
      "  -u, --update-channel [CHAN]  Publish frame updates on specified channel.\n"
      "                                 Default channel is BODY_TO_LOCAL.\n"
      "  -o, --odometry-channel CHAN  Publish relative odometry messages on specified channel.\n"
      "                                 Default channel is KINECT_REL_ODOMETRY  \n"
      "  -s, --stats [CHAN]           Publish odometry statistics.\n"
      "                                 Default channel is FOVIS_STATS\n"
      "  -f, --file FILE              Process a log file.\n"
#ifdef USE_LCMGL
      "  -l, --lcmgl                  Render debugging information with LCMGL\n"
#endif
      "  -h, --help                   Shows this help text and exits\n",
      progname);
  exit(1);
}

int main(int argc, char** argv)
{
  State* state = new State();
  state->publish_lcm = lcm_create(NULL);
  if(!state->publish_lcm) {
      fprintf(stderr, "Unable to initialize LCM\n");
      return 1;
  }
  state->kinect_frame_channel = "KINECT_FRAME";
  state->utime_prev = 0;
  state->utime_cur = 0;
  state->odom_channel = "KINECT_REL_ODOMETRY";
  state->publish_pose = false;
  state->pose_channel = "POSE";
  state->publish_frame_update = false;
  state->frame_update_channel = "BODY_TO_LOCAL";
  state->fovis_stats_channel = "FOVIS_STATS";
  state->publish_fovis_stats = false;

#ifdef USE_LCMGL
  state->draw_lcmgl = false;
  state->lcmgl = bot_lcmgl_init(state->publish_lcm, "kinect-odometry");
#endif

  // make up an initial calibration
  state->kinect_params.width = 640;
  state->kinect_params.height = 480;

  memset(&state->kinect_params.depth_params, 0, sizeof(CameraIntrinsicsParameters));
  state->kinect_params.depth_params.width = state->kinect_params.width;
  state->kinect_params.depth_params.height = state->kinect_params.height;
  state->kinect_params.depth_params.fx = 576.09757860;
  state->kinect_params.depth_params.fy = state->kinect_params.depth_params.fx;
  state->kinect_params.depth_params.cx = 321.06398107;
  state->kinect_params.depth_params.cy = 242.97676897;

  memset(&state->kinect_params.rgb_params, 0, sizeof(CameraIntrinsicsParameters));
  state->kinect_params.rgb_params.width = state->kinect_params.width;
  state->kinect_params.rgb_params.height = state->kinect_params.height;
  state->kinect_params.rgb_params.fx = 528.49404721;
  state->kinect_params.rgb_params.fy = state->kinect_params.rgb_params.fx;
  state->kinect_params.rgb_params.cx = 319.50000000;
  state->kinect_params.rgb_params.cy = 239.50000000;

  state->kinect_params.shift_offset = 1093.4753;
  state->kinect_params.projector_depth_baseline = 0.07214;

  Eigen::Matrix3d R;
  R << 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970;
  state->kinect_params.depth_to_rgb_translation[0] = -0.015756;
  state->kinect_params.depth_to_rgb_translation[1] = -0.000923;
  state->kinect_params.depth_to_rgb_translation[2] =  0.002316;
  Eigen::Quaterniond Q(R);
  state->kinect_params.depth_to_rgb_quaternion[0] = Q.w();
  state->kinect_params.depth_to_rgb_quaternion[1] = Q.x();
  state->kinect_params.depth_to_rgb_quaternion[2] = Q.y();
  state->kinect_params.depth_to_rgb_quaternion[3] = Q.z();

  // allocate temporary buffers
  state->rgb_buf_size = state->kinect_params.width * state->kinect_params.height * 3;
  state->rgb_buf = (uint8_t*) malloc(state->rgb_buf_size);
  state->gray_buf_size = state->kinect_params.width * state->kinect_params.height;
  state->gray_buf = (uint8_t*) malloc(state->gray_buf_size);
  state->disparity_buf_size = state->kinect_params.width * state->kinect_params.height * sizeof(uint16_t);
  state->disparity_buf = (uint16_t*) malloc(state->disparity_buf_size);

  // TODO parse options
  const char *optstring = "hp::u::o:lf:s::c:";
  int c;
  struct option long_opts[] = {
    { "help", no_argument, 0, 'h' },
    { "pose", optional_argument, 0, 'p' },
    { "update-channel", optional_argument, 0, 'u' },
    { "odometry-channel", required_argument, 0, 'o' },
    { "file", required_argument, 0, 'f' },
    { "lcmgl", no_argument, 0, 'l' },
    { "stats", optional_argument, 0, 's' },
    { "channel", required_argument, 0, 'c'},
    {0, 0, 0, 0}
  };
  while ((c = getopt_long (argc, argv, optstring, long_opts, 0)) >= 0) {
    switch(c) {
      case 'p':
        state->publish_pose = true;
        if(optarg)
          state->pose_channel = optarg;
        printf("Publishing pose messages on channel %s\n", state->pose_channel.c_str(), optarg);
        break;
      case 'u':
        state->publish_frame_update = true;
        if (optarg)
          state->frame_update_channel = optarg;
        break;
      case 'o':
        state->odom_channel = optarg;
        break;
      case 'f':
        state->input_log_fname = optarg;
        break;
#if USE_LCMGL
      case 'l':
        state->draw_lcmgl = true;
        break;
#endif
      case 's':
        state->publish_fovis_stats = true;
        if (optarg)
          state->fovis_stats_channel = optarg;
        break;
      case 'c' :
	state->kinect_frame_channel = optarg;
	break;
      case 'h':
      default:
        usage(argv[0]);
        break;
    }
  }

  if(state->input_log_fname.size()) {
    string lcmurl = string("file://") + state->input_log_fname + string("?speed=0");
    state->subscribe_lcm = lcm_create(lcmurl.c_str());
    if(!state->subscribe_lcm) {
      fprintf(stderr, "Unable to initialize LCM\n");
      return 1;
    }
  } else {
    state->subscribe_lcm = state->publish_lcm;
  }

  // subscribe to raw kinect data
  kinect_frame_msg_t_subscribe(state->subscribe_lcm, state->kinect_frame_channel.c_str(), on_kinect_frame, state);

  state->kinect_calib = new PrimeSenseCalibration(state->kinect_params);
  const Rectification* rectification = state->kinect_calib->getRgbRectification();
//  const CameraIntrinsics * rgb_intrinsics = state->kinect_calib->getRgbIntrinsics();
  state->odom = new VisualOdometry(rectification, VisualOdometry::getDefaultOptions());
  state->depth_producer = new PrimeSenseDepth(state->kinect_calib);

  // Register signal handlers so that we can
  // exit cleanly when interrupted
  struct sigaction new_action;
  new_action.sa_sigaction = sig_action;
  sigemptyset(&new_action.sa_mask);
  new_action.sa_flags = 0;
  sigaction(SIGINT, &new_action, NULL);
  sigaction(SIGTERM, &new_action, NULL);
  sigaction(SIGHUP, &new_action, NULL);

  // go!
  while(0 == lcm_handle(state->subscribe_lcm) && !shutdown_flag);

//  std::cerr << "_num_frames = " << _num_frames << std::endl;
//  std::cerr << "_mean_inliers = " << _mean_inliers/_num_frames << std::endl;

  free(state->rgb_buf);
  free(state->gray_buf);
  free(state->disparity_buf);

#ifdef USE_LCMGL
  bot_lcmgl_destroy(state->lcmgl);
#endif

  lcm_destroy(state->subscribe_lcm);
  if(state->subscribe_lcm != state->publish_lcm)
    lcm_destroy(state->publish_lcm);
  delete state->depth_producer;
  delete state->odom; //TODO: make destructor for state that does all this?
  delete state;

  return 0;
}
