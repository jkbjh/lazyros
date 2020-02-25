from __future__ import print_function

import threading
import atexit
import time
import signal
import sys
import weakref

import tf2_msgs.msg
import tf2_ros

import rospy
import rospy.rostime
import rospy.exceptions
import rostopic
import lazyros.numbers

class _NOP(object):
    """This is like a mock-object with all the methods that you can possible imagine."""
    def __init__(self):
        pass

    def __call__(self, *a, **kw):
        return None

    def __getattr__(self, name):
        return self


class FailSilentWeakref(object):
    def __init__(self, reference):
        self._wref = weakref.ref(reference)

    def __call__(self):
        dereferenced = self._wref()
        if dereferenced is None:
            return _NOP()
        return dereferenced


class BufferingSubscriber(object):
    def __init__(self, topic, msg_type=None, data_mangler=None):
        if msg_type is None:
            msg_type = get_topic_class(topic)
        wself = FailSilentWeakref(self)
        self.subscriber = rospy.Subscriber(topic,
                                           msg_type,
                                           lambda data: wself()._callback(data))

        self.data = None
        self.time = rospy.rostime.Time()
        self._data_mangler = data_mangler
        self.event = threading.Event()

    def _callback(self, data):
        self.time = rospy.get_rostime()
        self.data = data if self._data_mangler is None else self._data_mangler(data)
        self.event.set()


class TFSubscriber(BufferingSubscriber):
    def __init__(self, transforms):
        self._latest_received_time = rospy.get_rostime()
        self._time_on_callback = rospy.get_rostime()
        timeout = rospy.Duration(1.0)  # I just choose this value without any reason
        self._transformer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._transformer)
        self._transforms = transforms
        self.transformed = ()
        super(TFSubscriber, self).__init__("/tf", tf2_msgs.msg.TFMessage)

    def update_transform(self):
        time = rospy.Time(0)
        if all((self._transformer.can_transform(target, source, time) for target, source in self._transforms)):
            self.transformed = tuple((
                ((source, target),
                 lazyros.numbers.transform_to_array(
                     self._transformer.lookup_transform(target, source, time).transform))
                for target, source in self._transforms
            ))
            self.event.set()

    def _callback(self, data):
        # in TF static transformations are future dated to
        # hack around not having static transformations.  but
        # we want them to be available at the same time.  so
        # we use the latest non-future timestamp as the
        # current one, and re-date all future ones to the
        # latest one.  this takes the implicit assumption,
        # that the robot_state_publisher posts the non-static
        # messages first and with the same timestamp!
        current_rostime = rospy.get_rostime()
        if current_rostime < self._time_on_callback:  # jump back in time!
            self._transformer.clear()
            print("DEBUG: wolololo clearing buffer.")
        self._time_on_callback = current_rostime
        for transform in data.transforms:
            # date future dated transformations back to current
            # date.
            if transform.header.stamp > current_rostime:
                transform.header.stamp = self._latest_received_time  # when no separate tf_static topic is used timestamps are future dated.
            elif transform.header.stamp > self._latest_received_time:
                self._latest_received_time = transform.header.stamp
        self.update_transform()


class ResponseException(rospy.exceptions.ROSException):
    def __init__(self, response, *a, **kw):
        self.response_msg = response
        super(ResponseException, self).__init__(*a, **kw)

    def __repr__(self):
        return "ResponseException carrying the response: %r" % (self.response_msg,)

    def __str__(self):
        return repr(self)


def start_ros_background():
    """spin ros in the background"""
    my_thread = threading.Thread(target=rospy.spin)
    my_thread.daemon = True
    my_thread.start()

    def kill_ros():
        print("killing ros")
        rospy.signal_shutdown("shutdown!")
    atexit.register(kill_ros)
    # end of background ros


def get_topic_class(topic):
    msg_class, _, _ = rostopic.get_topic_class(topic)
    return msg_class


def receive_one_message(topic, timeout=None):
    msg_class = get_topic_class(topic)
    msg = rospy.wait_for_message(topic, msg_class, timeout=timeout)
    return msg


def receive_n_messages(topic, count):
    """
    wait to receive count messages. Requires rospy.spin() running in a different thread.
    """
    msg_class = get_topic_class(topic)
    buffer = [0]  # first element holds the count, required because of closure
    event = threading.Event()

    def call_me_back(data):
        buffer.append(data)
        buffer[0] = buffer[0] + 1
        if buffer[0] >= count:
            event.set()
    subscriber = rospy.Subscriber(topic, msg_class, call_me_back)
    try:
        event.wait()
    finally:
        subscriber.unregister()
    return buffer[1:]


def send_one_message(topic, msg):
    msg_class = get_topic_class(topic)
    pub = rospy.Publisher(topic, msg_class, queue_size=1, latch=True)
    if type(msg) != msg_class:
        classed_msg = msg_class(msg)
    else:
        classed_msg = msg
    pub.publish(classed_msg)
    time.sleep(1)  # ... no way to figure out whether the message has actually been sent....
    pub.unregister()


def setup_ctrl_c_exit():
    # wait for ctrl-c
    def signal_handler(signal, frame):
            print("SIGINT received.")
            sys.stdout.flush()
            sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)


def wait_for_ctrl_c():
    setup_ctrl_c_exit()
    print("Press Ctrl+C")
    signal.pause()
