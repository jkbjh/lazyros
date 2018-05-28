import numpy
import geometry_msgs.msg


def quat_from_numbers(quat_array):
    return geometry_msgs.msg.Quaternion(*quat_array)


def quat_to_numbers(quat):
    return numpy.asarray([quat.x, quat.y, quat.z, quat.w])


def pose_from_numbers(position, orientation):
    pose = geometry_msgs.msg.Pose()
    p = pose.position
    p.x, p.y, p.z = position
    o = pose.orientation
    o.x, o.y, o.z, o.w = orientation
    return pose


def pose_to_array(pose):
    p = pose.position
    o = pose.orientation
    return numpy.array([p.x, p.y, p.z, o.x, o.y, o.z, o.w])


def pose_from_array(position_and_orientation):
    pose = geometry_msgs.msg.Pose()
    p = pose.position
    o = pose.orientation
    p.x, p.y, p.z, o.x, o.y, o.z, o.w = position_and_orientation
    return pose


def point_from_array(position_array):
    p = geometry_msgs.msg.Point()
    p.x, p.y, p.z = position_array
    return p


def point_to_array(point):
    return numpy.asarray([point.x, point.y, point.z])


def transform_to_array(transform):
    t = transform.translation
    r = transform.rotation
    return numpy.array([t.x, t.y, t.z, r.x, r.y, r.z, r.w])


def unpack_position(position):
    return numpy.asarray((position.x, position.y, position.z))


def to_homogeneous(point_array):
    homo_point = numpy.zeros((4,))
    homo_point[0:3] = point_array
    return homo_point


def from_homogeneous(point_array):
    return point_array[0:3]
