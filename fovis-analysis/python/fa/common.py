import numpy as np

from rigid_transform import RigidTransform

VEL_DTYPE = [('utime', float), ('trans_vel', float, 3)]

def quat_slerp(q0, q1, w):
    """ spherical interpolation.
    q0, q1: arrays of Nx4, each row is a rotation quaternion.
    w: array of N weights.
    output: array Nx4 with spherically interpolated quaternions.
    """
    # TODO is it worthwhile to index more carefully?
    w1 = 1. - w
    q0 = q0.T
    q1 = np.copy(q1.T)
    cos_omega = (q0 * q1).sum(0)
    # find and flip quats to ensure shortest path
    nix = cos_omega < 0.
    cos_omega[nix] *= -1.
    q1[:,nix] *= -1.
    cos_omega.clip(0., 1., out=cos_omega)
    omega = np.arccos(cos_omega)
    lerp = q0*w + q1*w1
    slerp = q0*np.sin(w1*omega) + q1*np.sin(w*omega)
    # find inputs that are too close, use linearly interpolation
    # to avoid numerical instability
    close_ix = np.fabs(1.-cos_omega) < 1e-3
    out = np.choose(close_ix, (slerp, lerp))
    # normalize
    out /= np.sqrt((out**2).sum(0))
    return out.T

def lerp(v0, v1, w):
    """ linear interpolation.
    v0, v1: arrays of NxD, each row is a vector
    w: array of N weights.
    output: array NxD with linearly interpolated vectors.
    """
    w = np.expand_dims(w, 1)
    return v0*w + v1*(1. - w)

def create_dtype(msg):
    """ Given a message, create a dtype to describe it.
    Only handles scalars and array-like data.
    """
    dtype = []
    for slot in msg.__slots__:
	attr = getattr(msg, slot)
	if type(attr) in (int, long, float):
	    dtype.append((slot, float))
	elif type(attr) in (list, tuple):
	    arr = np.array(attr)
	    dtype.append((slot, arr.dtype, arr.shape))
	elif type(attr) is bool:
	    dtype.append((slot, bool))
	else:
	    print 'Ignoring type', type(attr)
    dtype.append(('log_timestamp', float))
    return dtype

def log_to_recarray(log, channel_name, lcm_type):
    msgs_ts = [(lcm_type.decode(event.data), event.timestamp) for event in log
	    if event.channel == channel_name]
    if len(msgs_ts)==0:
        raise ValueError("No messages from channel %s"%channel_name)
    dtype = create_dtype(msgs_ts[0][0])
    data = [tuple((getattr(msg, slot) for slot in lcm_type.__slots__))+(ts,)
	        for msg,ts in msgs_ts]
    array = np.array(data, dtype=dtype)
    return array

def matched_observations(gnd_poses, vo_updates):
    ix = np.searchsorted(gnd_poses['utime'], vo_updates['timestamp'])
    # discard vo_updates observations outside of gnd_poses bounds
    in_borders = (ix > 0) & (ix < len(gnd_poses))
    vo_updates = vo_updates[in_borders]
    ix = ix[in_borders]
    assert(len(ix) > 0)
    new_gnd_poses = np.empty(len(vo_updates), dtype=gnd_poses.dtype)
    new_gnd_poses['utime'] = vo_updates['timestamp']
    t0 = gnd_poses['utime'][ix-1]
    t1 = gnd_poses['utime'][ix]
    # interpolation weights
    w = (vo_updates['timestamp'] - t0)/(t1 - t0)
    # for position interpolate linearly
    pos0 = gnd_poses['pos'][ix-1]
    pos1 = gnd_poses['pos'][ix]
    new_gnd_poses['pos'] = lerp(pos0, pos1, w)
    # for rotations, spherical interpolation
    q0 = gnd_poses['orientation'][ix-1]
    q1 = gnd_poses['orientation'][ix]
    new_gnd_poses['orientation'] = quat_slerp(q0, q1, w)
    return new_gnd_poses, vo_updates

def poses_to_velocities(poses):
    # compute vicon velocities from absolute poses and timestamps
    vicon_vels = np.zeros(len(poses), dtype=VEL_DTYPE)
    vicon_vels['utime'] = poses['utime']
    body_to_globals = [RigidTransform(orientation, pos) for orientation, pos
            in zip(poses['orientation'], poses['pos'])]
    for i in xrange(1, len(vicon_vels)):
        prev_to_curr = body_to_globals[i-1].inverse() * body_to_globals[i]
        vicon_vels['trans_vel'][i] = prev_to_curr.translation()
    dt = (poses['utime'][1:]-poses['utime'][:-1])*1e-6
    vicon_vels['trans_vel'][1:] /= np.atleast_2d(dt).T
    return vicon_vels

def transformed_velocities(camera_to_body, vo_updates):
    body_to_camera = camera_to_body.inverse()
    camera_body_vels = np.zeros(len(vo_updates), dtype=VEL_DTYPE)
    camera_body_vels['utime'] = vo_updates['timestamp']
    for i in xrange(1, len(camera_body_vels)):
        prev_to_curr_camera = RigidTransform(vo_updates['rotation'][i], vo_updates['translation'][i])
        prev_to_curr_body = body_to_camera * prev_to_curr_camera * camera_to_body
        camera_body_vels['trans_vel'][i] = prev_to_curr_body.translation()
    dt = (vo_updates['timestamp'][1:]-vo_updates['timestamp'][:-1])*1e-6
    camera_body_vels['trans_vel'][1:] /= np.atleast_2d(dt).T
    return camera_body_vels

