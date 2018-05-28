import sys
import re

import rosservice
import rospy
import rosmsg
import threading


def get_service_names():
    return rosservice.get_service_list()


class ServiceDiscoverer(object):
    def __init__(self, name="/", proxy_collector=None):
        self._name = name
        self._proxy_collector = proxy_collector

    @staticmethod
    def _get_service_names(prefix, names):
        prefix_len = len(prefix)
        names = [name[prefix_len:] for name in names if name.startswith(prefix)]
        def find(string, substr):
            pos = string.find(substr)
            if pos == -1:
                return len(string)
            return pos + 1
        limited_names = sorted({name[:find(name, "/")] for name in names})
        return limited_names

    def __dir__(self):
        return [name.rstrip("/") for name in self._get_service_names(self._name, get_service_names())]

    def __getattr__(self, name):
        service_names = set(self._get_service_names(self._name, get_service_names()))
        if name in service_names:
            proxy_name = "%s%s" % (self._name, name)
            if self._proxy_collector is not None:
                self._proxy_collector.add(proxy_name)
            return ServiceProxy(proxy_name)
        elif name + "/" in service_names:
            return ServiceDiscoverer("%s%s/" % (self._name, name), proxy_collector=self._proxy_collector)
        raise AttributeError("%s%s does not exist!" % (self._name, name))


class ServiceProxy(object):
    def __init__(self, name, service_class=None):
        self._name = name
        self._srv_class = service_class
        self._srv_proxy = None
        self._args = None
        self._proxy_lock = threading.Lock()

    @property
    def srv_class(self):
        if self._srv_class is None:
            self._srv_class = rosservice.get_service_class_by_name(self._name)
        return self._srv_class

    @property
    def __doc__(self):
        return ("%s is of type %s,\n"
                "and has the following structure:\n"
                "%s\n") % (
                    self._name,
                    self.srv_class._type,
                    rosmsg.get_srv_text(self.srv_class._type))
    @property
    def args(self):
        if self._args is None:
            self._args = rosservice.get_service_args(self._name)
        return self._args

    @property
    def srv_proxy(self):
        if self._srv_proxy is None:

            self._srv_proxy = rospy.ServiceProxy(self._name, self.srv_class)
        return self._srv_proxy

    def __call__(self, *a, **kw):
        with self._proxy_lock:
            return self.srv_proxy(*a, **kw)

    def code(self):
        return "lazyros_services.ServiceProxy(%r, %s)" % (self._name, self.__get_class_name(self.srv_class))

    @staticmethod
    def __get_class_name(ros_service_class):
        module = ros_service_class.__module__
        name = ros_service_class.__name__
        index = module.find("_" + name)
        if index > 0:
            simplified_module = module[:index -1]
            try:
                getattr(sys.modules[simplified_module], name)
                return "%s.%s" % (simplified_module, name)
            except:
                pass
        return "%s.%s" % (module, name)


def debug_create_code_for_proxies(proxies):
    output = []
    for proxy in proxies:
        sp = ServiceProxy(proxy)
        output.append("%s = %s" % (sp.srv_class.__name__, sp.code(),))
    return output
