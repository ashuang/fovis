import math
import numpy
from quaternion import Quaternion

class RigidTransform(object):
    def __init__(self, rotation_quat, translation_vec):
        self.quat = Quaternion(rotation_quat)
        self.tvec = numpy.array(translation_vec)

    def inverse(self):
        """ returns a new RigidTransform that corresponds to the inverse of this one """
        qinv = self.quat.inverse()
        return RigidTransform(qinv, qinv.rotate(- self.tvec))

    def interpolate(self, other_transform, this_weight):
        assert this_weight >= 0 and this_weight <= 1
        t = self.tvec * this_weight + other_transform.tvec * (1 - this_weight)
        r = self.quat.interpolate(other_transform.quat, this_weight)
        return RigidTransform(r, t)

    def __mul__(self, other):
        if isinstance(other, RigidTransform):
            t = self.quat.rotate(other.tvec) + self.tvec
            r = self.quat * other.quat
            return RigidTransform(r, t)
        else:
            olen = len(other)
            if olen == 3:
                r = numpy.array(self.quat.rotate(other))
                return r + self.tvec
            elif olen == 4:
                return np.dot(self.to_homogeneous_matrix(), other)
            else:
                raise ValueError()

    def to_homogeneous_matrix(self):
        result = self.quat.to_matrix_homogeneous()
	result.A[:3, 3] = self.tvec
        return result

    def to_roll_pitch_yaw_x_y_z(self):
        r, p, y = self.quat.to_roll_pitch_yaw()
        return numpy.array((r, p, y, self.tvec[0], self.tvec[1], self.tvec[2]))

    @staticmethod
    def from_roll_pitch_yaw_x_y_z(r, p, yaw, x, y, z):
        q = Quaternion.from_roll_pitch_yaw(r, p, yaw)
        return RigidTransform(q, (x, y, z))

    def quaternion(self):
        return self.quat

    def translation(self):
        return self.tvec

    @staticmethod
    def identity():
        return RigidTransform((1, 0, 0, 0), (0, 0, 0))

if __name__ == "__main__":
    import random
    q = Quaternion([1, 0, 0, 0])
    t = [ 1, 2, 3 ]
    m = RigidTransform(q, t)
    print "m"
    print m.to_homogeneous_matrix()

    q2 = Quaternion.from_roll_pitch_yaw(numpy.pi / 4, 0, 0)
    t2 = [ 0, 0, 0 ]
    m2 = RigidTransform(q2, t2)
    print "m2"
    print m2.to_homogeneous_matrix()

    m3 = m * m2
    print "m * m2"
    print m3.to_homogeneous_matrix()

    m4 = m2 * m
    print "m * m2"
    print m4

    def make_random_transform():
        q_wxyz = [ random.random(), random.random(), random.random(), random.random() ]
        qmag = math.sqrt(sum([x*x for x in q_wxyz]))
        q_wxyz = [ x / qmag for x in q_wxyz ]
        translation = [ random.uniform(-100, 100), random.uniform(-100, 100), random.uniform(-100, 100) ]
        return RigidTransform(q_wxyz, translation)

    print "Testing inverse"
    identity = numpy.identity(4)
    for unused in range(100):
        # generate a bunch of random rigid body transforms, then compose them and apply their inverses
        # the result should be the identity transform
        num_transforms = random.randint(0, 10)
        ms = [ make_random_transform() for unused in range(num_transforms) ]
        inverses = [ m.inverse() for m in ms ]
        inverses.reverse()
        r = RigidTransform.identity()
        for m in ms + inverses:
            r *= m
        errs = (identity - r.to_homogeneous_matrix()).flatten().tolist()[0]
        sse = numpy.dot(errs, errs)
        assert sse < 1e-10
    print "OK"
#        print sse

    print "Testing composition"
    t = RigidTransform.identity()
    m = numpy.identity(4)
    for unused in range(1000):
#        print "===="
#        print t.to_homogeneous_matrix()
#        print m

        n = make_random_transform()
#        n.quat = Quaternion(1, 0, 0, 0)
#        print "applying "
#        print n.to_homogeneous_matrix()

        t = t * n
        m = m * n.to_homogeneous_matrix()

        errs = (t.to_homogeneous_matrix() - m).flatten().tolist()[0]
        sse = numpy.dot(errs, errs)
#        print "--"
#        print t.to_homogeneous_matrix()
#        print m
        assert sse < 1e-10

    for unused in range(1000):
        t1 = make_random_transform()
        t2 = make_random_transform()
        t1.to_homogeneous_matrix

    print "OK"
