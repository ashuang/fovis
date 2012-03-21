""" Compare stuff.
"""

import os
import sys
import argparse
import pdb
import pprint

import numpy as np
import matplotlib.pyplot as pl

from lcm import EventLog

from fovis.update_t import update_t
from fovis.stats_t import stats_t
from fovis.tictoc_t import tictoc_t
from bot_core.pose_t import pose_t
from bot_core.rigid_transform_t import rigid_transform_t

import fa.common
from fa.rigid_transform import RigidTransform

DEFAULT_VEL_NORM_OUTLIER_THRESHOLD = 1.
COL_WIDTH = 30

def get_options():
    parser = argparse.ArgumentParser(description=
            'Compare velocity estimations in LCM logfiles.')
    parser.add_argument('GROUND', action='store',
            help="'Ground truth' poses as logfile:channel pair.\
            If no channel is given, use GROUND_TRUTH_POSE.")
    parser.add_argument('ODOMETRY', action='store',
            help='Odometry estimates as logfile:channel pair.\
            If no channel is given, use STEREO_REL_ODOMETRY.')
    parser.add_argument('--plot', action="store_true",
            default=False,
            help="Display some plots.")
    parser.add_argument('--vel-outlier-thresh', action="store", type=float,
            default=DEFAULT_VEL_NORM_OUTLIER_THRESHOLD,
            help="Outlier threshold on norm of error of velocity.")
    parser.add_argument('--single-line', action="store_true",
            default=False,
            help="Print metrics as a single line.")
    parser.add_argument('--header', action="store_true",
            default=False,
            help="Print header and exit.")
    return parser.parse_args()

def plot_vels(gnd_vels, camera_vels):
    # scale and offset time to something intelligible
    start_time = min((np.min(gnd_vels['utime']), np.min(camera_vels['utime'])))
    gnd_vels['utime'] = (gnd_vels['utime'] - start_time)*1e-6
    camera_vels['utime'] = (camera_vels['utime'] - start_time)*1e-6

    pl.figure()
    xyz = ['dx/dt', 'dy/dt', 'dz/dt']
    for i in xrange(3):
        pl.subplot(3,1,i+1)
        #pl.title('%s vs time'%xyz[i])
        pl.plot(gnd_vels['utime'], gnd_vels['trans_vel'][:,i], 'r-', label='gnd')
        pl.plot(camera_vels['utime'], camera_vels['trans_vel'][:,i], 'g-', label='vo')
        pl.xlabel('t (s)')
        pl.ylabel(xyz[i])
        if i==0:
            pl.legend() # avoid redundancy

def plot_error_stats(vel_error_norm, vo_stats):
    start_time = vo_stats['timestamp'][0]
    vo_stats['timestamp'] = (vo_stats['timestamp'] - start_time)*1e-6
    vel_error_norm['timestamp'] = (vel_error_norm['timestamp'] - start_time)*1e-6

    pl.figure()
    #pl.plot(vel_error_norm['timestamp'], vel_error_norm['vel_error_norm'], 'r-', label='vel_error')

    pl.subplot(4, 1, 1)
    pl.plot(vo_stats['timestamp'], vo_stats['num_detected_keypoints'], 'g-', label='num_detected_keypoints')
    pl.plot(vo_stats['timestamp'], vo_stats['num_keypoints'], 'b-', label='num_keypoints')
    pl.legend()

    pl.subplot(4, 1, 2)
    pl.plot(vo_stats['timestamp'], vo_stats['fast_threshold'], 'm-', label='fast_threshold')
    pl.legend()

    pl.subplot(4, 1, 3)
    pl.plot(vo_stats['timestamp'], vo_stats['mean_reprojection_error'], 'k-', label='mean_reprojection_error')
    pl.legend()

    pl.subplot(4, 1, 4)
    pl.plot(vo_stats['timestamp'], vo_stats['num_matches'], 'c-', label='num_matches')
    pl.plot(vo_stats['timestamp'], vo_stats['num_inliers'], 'b-', label='num_inliers')
    pl.plot(vo_stats['timestamp'], vo_stats['num_reprojection_failures'], 'y-', label='num_reprojection_failures ')
    pl.legend()

def load_data(opts):
    elts = opts.GROUND.strip().split(':')
    gnd_lcmlog_fname, gnd_channel = elts[0], 'GROUND_TRUTH_POSE' if len(elts)==1 else elts[1]
    elts = opts.ODOMETRY.strip().split(':')
    vo_lcmlog_fname, vo_channel = elts[0], 'STEREO_REL_ODOMETRY' if len(elts)==1 else elts[1]

    gnd_lcmlog = EventLog(gnd_lcmlog_fname, 'r')

    vo_lcmlog = gnd_lcmlog if (gnd_lcmlog_fname==vo_lcmlog_fname)\
            else EventLog(vo_lcmlog_fname,'r')


    gnd_poses = fa.common.log_to_recarray(gnd_lcmlog, gnd_channel, pose_t)
    vo_updates = fa.common.log_to_recarray(vo_lcmlog, vo_channel, update_t)

    vo_stats = []
    try:
        vo_stats = fa.common.log_to_recarray(vo_lcmlog, 'FOVIS_STATS', stats_t)
    except ValueError, ex:
        print ex

    camera_to_body_msg = [rigid_transform_t.decode(ev.data) for ev in gnd_lcmlog
            if ev.channel=='CAMERA_TO_BODY'][0]
    camera_to_body = RigidTransform(camera_to_body_msg.quat, camera_to_body_msg.trans)

    return gnd_poses, vo_updates, vo_stats, camera_to_body

def get_vels(gnd_poses, vo_updates, camera_to_body):

    # match and interpolate observations
    gnd_poses, vo_updates = fa.common.matched_observations(gnd_poses, vo_updates)

    gnd_vels = fa.common.poses_to_velocities(gnd_poses)
    camera_vels = fa.common.transformed_velocities(camera_to_body, vo_updates)

    return gnd_vels, camera_vels

def main():
    opts = get_options()

    metrics_keys = [\
        'fname',
        'mean_error',
        'std_error',
        'num_failures',
        'num_outliers',
        'num_failures_plus_num_outliers',
        'percent_failures_or_outliers',
        'mean_match_inliers',
        'mean_mean_reproj_error',
        ]


    if opts.header:
        print ', '.join([k.rjust(COL_WIDTH) for k in metrics_keys])
        return

    gnd_poses, vo_updates, vo_stats, camera_to_body = load_data(opts)

    # find and remove failures
    failures = (vo_updates['estimate_status'] != 0)
    num_failures = failures.sum()
    # remember original number of vo_updates before selection, for statistics
    num_vo_updates = len(vo_updates)
    vo_updates = vo_updates[~failures]

    gnd_vels, camera_vels = get_vels(gnd_poses, vo_updates, camera_to_body)

    vel_errors = gnd_vels['trans_vel']-camera_vels['trans_vel']
    vel_error_norm = np.sqrt(np.sum(vel_errors**2, 1))
    orig_vel_error_norm = vel_error_norm.copy()

    outliers = vel_error_norm > opts.vel_outlier_thresh
    num_outliers = outliers.sum()
    # exclude outliers from error statistics
    vel_error_norm = vel_error_norm[~outliers]
    percent_failures_or_outliers = (float(num_failures+num_outliers)/num_vo_updates)*100.

    mean_error = vel_error_norm.mean()
    std_error = vel_error_norm.std()/np.sqrt(len(vel_error_norm))
    conf_width = std_error * 1.96
    conf_intervals = np.array((mean_error-conf_width, mean_error+conf_width))

    metrics = {\
        'fname' : opts.ODOMETRY.split(':')[0],
        'mean_error' : mean_error,
        'std_error': std_error,
        'num_failures': num_failures,
        'num_outliers': num_outliers,
        'num_failures_plus_num_outliers': num_failures+num_outliers,
        'percent_failures_or_outliers': percent_failures_or_outliers,
    }

    if len(vo_stats) > 0:
        metrics['mean_match_inliers'] = vo_stats['num_inliers'].mean(),
        metrics['mean_mean_reproj_error'] = vo_stats['mean_reprojection_error'].mean(),
    else:
        metrics['mean_match_inliers'] = 0
        metrics['mean_mean_reproj_error'] = 0

    # TODO not repeat myself
    assert (set(metrics.keys()) == set(metrics_keys))

    if opts.single_line:
        str_values = [str(metrics[k]).rjust(COL_WIDTH) for k in metrics_keys]
        print ', '.join(str_values)
    else:
        pprint.pprint(metrics)

    if opts.plot:
        plot_vels(gnd_vels, camera_vels)
        if len(vo_stats) > 0:
            plot_error_stats(np.rec.fromarrays([gnd_vels['utime'],
                orig_vel_error_norm], names='timestamp, vel_error_norm'), vo_stats)
        pl.show()

if __name__ == '__main__':
    main()
