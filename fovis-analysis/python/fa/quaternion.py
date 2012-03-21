import math
import numpy

class Quaternion:
    def __init__ (self, *args):
        if len(args) == 4:
            self.q = numpy.array(args[:])
        elif len(args) == 1:
            if isinstance(args[0], Quaternion):
                self.q = args[0].q.copy()
            elif len(args[0]) == 4:
                self.q = numpy.array(args[0][:])
        else:
            raise TypeError ("invalid initializer")
        norm = numpy.sqrt(numpy.dot(self.q, self.q))
        assert abs(norm-1) < 1e-12
        if abs(norm - 1) > 1e-12:
            self.q /= norm

    def __mul__ (self, other):
        a = self.q
        b = other.q
        return Quaternion (a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                           a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                           a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                           a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0])

    def __getitem__ (self, i):
        return self.q[i]

    def __repr__ (self):
        return repr (self.q)

    def rotate(self, v):
        ab  =  self.q[0]*self.q[1]
        ac  =  self.q[0]*self.q[2]
        ad  =  self.q[0]*self.q[3]
        nbb = -self.q[1]*self.q[1]
        bc  =  self.q[1]*self.q[2]
        bd  =  self.q[1]*self.q[3]
        ncc = -self.q[2]*self.q[2]
        cd  =  self.q[2]*self.q[3]
        ndd = -self.q[3]*self.q[3]
        return numpy.array((2*( (ncc + ndd)*v[0] + (bc -  ad)*v[1] + (ac + bd)*v[2] ) + v[0],
                         2*( (ad +  bc)*v[0] + (nbb + ndd)*v[1] + (cd - ab)*v[2] ) + v[1],
                         2*( (bd -  ac)*v[0] + (ab +  cd)*v[1] + (nbb + ncc)*v[2] ) + v[2]))

    def rotate_rev (self, vector):
        b = numpy.array((0, v[0], v[1], v[2]))
        a = numpy.array((b[0]*self.q[0] - b[1]*self.q[1] - b[2]*self.q[2] - b[3]*self.q[3],
             b[0]*self.q[1] + b[1]*self.q[0] + b[2]*self.q[3] - b[3]*self.q[2],
             b[0]*self.q[2] - b[1]*self.q[3] + b[2]*self.q[0] + b[3]*self.q[1],
             b[0]*self.q[3] + b[1]*self.q[2] - b[2]*self.q[1] + b[3]*self.q[0]))
        b[0] = q[0]
        b[1:] = -q[1:]
        return numpy.array((b[0]*a[1] + b[1]*a[0] + b[2]*a[3] - b[3]*a[2],
                         b[0]*a[2] - b[1]*a[3] + b[2]*a[0] + b[3]*a[1],
                         b[0]*a[3] + b[1]*a[2] - b[2]*a[1] + b[3]*a[0]))

    def inverse(self):
        return Quaternion(self.q[0], -self.q[1], -self.q[2], -self.q[3])

    @staticmethod
    def from_roll_pitch_yaw (roll, pitch, yaw):
	half_rpy = numpy.array((roll, pitch, yaw))/2.
        sin_r2, sin_p2, sin_y2 = numpy.sin(half_rpy)
        cos_r2, cos_p2, cos_y2 = numpy.cos(half_rpy)
        return Quaternion((cos_r2 * cos_p2 * cos_y2 + sin_r2 * sin_p2 * sin_y2,
                           sin_r2 * cos_p2 * cos_y2 - cos_r2 * sin_p2 * sin_y2,
                           cos_r2 * sin_p2 * cos_y2 + sin_r2 * cos_p2 * sin_y2,
                           cos_r2 * cos_p2 * sin_y2 - sin_r2 * sin_p2 * cos_y2))

    @staticmethod
    def from_angle_axis(theta, axis):
        x, y, z = axis
        norm = math.sqrt(x*x + y*y + z*z)
        if 0 == norm:
            return Quaternion(1, 0, 0, 0)
        t = math.sin(theta/2) / norm;
        return Quaternion(math.cos(theta/2), x*t, y*t, z*t)

    def to_roll_pitch_yaw (self):
        roll_a = 2 * (self.q[0]*self.q[1] + self.q[2]*self.q[3])
        roll_b = 1 - 2 * (self.q[1]*self.q[1] + self.q[2]*self.q[2])
        roll = math.atan2(roll_a, roll_b)
        pitch_sin = 2 * (self.q[0]*self.q[2] - self.q[3]*self.q[1])
        pitch = math.asin(pitch_sin)
        yaw_a = 2 * (self.q[0]*self.q[3] + self.q[1]*self.q[2])
        yaw_b = 1 - 2 * (self.q[2]*self.q[2] + self.q[3]*self.q[3])
        yaw = math.atan2(yaw_a, yaw_b)
        return roll, pitch, yaw

    def to_angle_axis(self):
        halftheta = math.acos(self.q[0])
        if abs(halftheta) < 1e-12:
            return 0, numpy.array((0, 0, 1))
        else:
            theta = halftheta * 2
            axis = numpy.array(self.q[1:4]) / math.sin(halftheta)
            return theta, axis

    def to_matrix(self):
        norm_sq = numpy.dot(self.q, self.q)
        if norm_sq < 1e-12: return numpy.eye(3)
        w, x, y, z = self.q/norm_sq
        x2 = x*x
        y2 = y*y
        z2 = z*z
        w2 = w*w
        xy = 2*x*y
        xz = 2*x*z
        yz = 2*y*z
        wx = 2*w*x
        wy = 2*w*y
        wz = 2*w*z
        return numpy.matrix(((w2+x2-y2-z2, xy-wz,       xz+wy),
                            (xy+wz,       w2-x2+y2-z2, yz-wx),
                            (xz-wy,       yz+wx,       w2-x2-y2+z2)))


    def to_matrix_homogeneous(self):
        result = numpy.eye(4)
        result[:3, :3] = self.to_matrix()
        return numpy.matrix(result)

    def interpolate(self, other, this_weight):
        q0, q1 = self.q, other.q
        u = 1 - this_weight
        assert(u >= 0 and u <= 1)
        cos_omega = numpy.dot(q0, q1)

        if cos_omega < 0:
            result = -q0[:]
            cos_omega = -cos_omega
        else:
            result = q0[:]

        cos_omega = min(cos_omega, 1)

        omega = math.acos(cos_omega)
        sin_omega = math.sin(omega)
        a = math.sin((1-u) * omega)/ sin_omega
        b = math.sin(u * omega) / sin_omega

        if abs(sin_omega) < 1e-6:
            # direct linear interpolation for numerically unstable regions
            result = result * this_weight + q1 * u
            result /= math.sqrt(numpy.dot(result, result))
        else:
            result = result*a + q1*b
        return Quaternion(result)

if __name__ == "__main__":
    import random
    q = Quaternion.from_roll_pitch_yaw (0, 0, 2 * math.pi / 16)
    v = [ 1, 0, 0 ]
    print v
    for i in range (16):
        v = q.rotate (v)
        print v

    q2 = Quaternion.from_roll_pitch_yaw(0, 0, 0)
    rpy_start = numpy.array(q.to_roll_pitch_yaw())
    rpy_goal = numpy.array(q2.to_roll_pitch_yaw())
    print "interpolate from ", q2.to_roll_pitch_yaw(), " to ", q.to_roll_pitch_yaw()
    for i in range(101):
        alpha = i / 100.
        qinterp = q2.interpolate(q, alpha)
        rpy_interp = numpy.array(qinterp.to_roll_pitch_yaw())
        rpy_expected = (rpy_goal * alpha + rpy_start * (1 - alpha))
        err = rpy_expected - rpy_interp
        for k in [ 0, 1, 2 ]:
            assert abs(err[k]) < 1e-12

    def mod2pi_positive(vin):
        q = vin / (2*numpy.pi) + 0.5
        qi = int(q)
        return vin - qi*2*numpy.pi

    def mod2pi(vin):
        if (vin < 0):
            return -mod2pi_positive(-vin)
        return mod2pi_positive(vin)

    def mod2pi_ref(ref, vin):
        return ref + mod2pi(vin - ref)

    print "testing angle-axis conversion"
    for unused in range(100):
        theta = random.uniform(-numpy.pi, numpy.pi)
        axis = numpy.array([ random.random(), random.random(), random.random() ])
        axis /= numpy.linalg.norm(axis)
        q = Quaternion.from_angle_axis(theta, axis)
        theta_check, axis_check = q.to_angle_axis()
        if numpy.dot(axis, axis_check) < 0:
            theta_check *= -1
            axis_check *= -1
        theta_check = mod2pi_ref(theta, theta_check)
        dtheta = theta_check - theta
        daxis = axis - axis_check
        assert abs(dtheta) < 1e-12
        assert numpy.linalg.norm(daxis) < 1e-9
    print "OK"

