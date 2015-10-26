import rospy
import roslib.message

from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest

_default_map_agent_name = '/lama_map_agent'


def map_agent_name():
    return rospy.get_param('map_agent', _default_map_agent_name)


def clean_tuple(lama_objects):
    """Transform references from tuples to list in a list of LamaObject

    rospy deserializes arrays as tuples, transform them back in place to lists.
    """
    for lama_object in lama_objects:
        lama_object.references = list(lama_object.references)


class MapAgent(object):
    """Class to interact with the map through service calls

    The service calls must be done through MapAgent.proxy.
    Example of database request through service call:
        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_DESCRIPTOR_LINKS
        map_action.object.id = request_object_id
        map_action.interface_name = request_interface_name
        response = MapAgent().proxy(map_action)
    A few function members are provided to facilitate such requests
    (get_vertex_list, get_edge_list).
    """
    def __init__(self, timeout=None):
        """
        Parameters
        ----------
        - timeout: None or double. Timeout for contacting service. If None,
            the service is supposed to be up and wait_for_service will not be
            called. If 0, wait_for_service() is called, i.e. wait undefinitely.
            Otherwise, wait_for_service(timeout) is called.
        """
        self._timeout = timeout
        action_srv_type = 'lama_interfaces/ActOnMap'
        srv_action_class = roslib.message.get_service_class(action_srv_type)
        # Action class.
        self.action_service_name = map_agent_name()
        self.action_service_class = srv_action_class
        self.proxy = rospy.ServiceProxy(self.action_service_name, ActOnMap)

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        self._timeout = value

    def wait_for_service(self):
        if self.timeout is not None:
            if self.timeout == 0:
                rospy.logdebug('Waiting for service {}'.format(
                    self.action_service_name))
                self.proxy.wait_for_service()
                rospy.logdebug('Service {} replied'.format(
                    self.action_service_name))
                return True
            else:
                try:
                    self.proxy.wait_for_service(self.timeout)
                except rospy.ROSException:
                    rospy.logerr('Service {} not available'.format(
                        self.action_service_name))
                    return False
        return True

    def get_lama_object_list(self, criteria=None):
        """Return the list of vertices and edges as LamaObject"""
        if not self.wait_for_service():
            return
        vertices = self.get_vertex_list(criteria)
        if vertices is None:
            return
        edges = self.get_edge_list(criteria)
        if edges is None:
            return
        return vertices + edges

    def get_edge_list(self, criteria=None):
        """Return the list of edges as LamaObject that match the search criteria

        Retrieve all edges that match the search criteria.

        Search criteria are attributes of criteria with non-default values
        (defaults are 0 or '').
        Return a list of LamaObject, an empty list of no LamaObject matches, or
        None on service error.

        Parameters
        ----------
        - criteria: an instance of LamaObject, supposed of type EDGE.
        """
        if not self.wait_for_service():
            return
        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_EDGE_LIST
        if criteria is not None:
            lama_object_attr = ['id', 'id_in_world', 'name', 'emitter_id',
                                'emitter_name', 'type', 'references']
            for attr in lama_object_attr:
                setattr(map_action.object, attr, getattr(criteria, attr))
        response = self.proxy(map_action)
        clean_tuple(response.objects)
        return response.objects

    def get_vertex_list(self, criteria=None):
        """Return the list of vertices as LamaObject that match the criteria

        Retrieve all vertices that match the search criteria.

        Search criteria are attributes of criteria with non-default values
        (defaults are 0 or '').
        Return a list of LamaObject, an empty list of no LamaObject matches, or
        None on service error.

        Parameters
        ----------
        - criteria: an instance of LamaObject, supposed of type VERTEX.
        """
        if not self.wait_for_service():
            return

        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_VERTEX_LIST
        if criteria is not None:
            lama_object_attr = ['id', 'id_in_world', 'name', 'emitter_id',
                                'emitter_name', 'type', 'references']
            for attr in lama_object_attr:
                setattr(map_action.object, attr, getattr(criteria, attr))
        response = self.proxy(map_action)
        clean_tuple(response.objects)
        return response.objects

    def get_descriptor_links(self, id_, interface_name=None):
        """Retrieve the list of DescriptorLink associated with a Lama object

        Return a list of DescriptorLink. If interface_name is given, return
        all DescriptorLink corresponding to this interface_name, otherwise, and
        if interface_name is '' or '*', return all DescriptorLink.

        Parameters
        ----------
        - id_: int, lama object id in the database.
        - interface_name: string, default to None.
            If None, '', or '*', all DescriptorLink are returned.
            Otherwise, only DescriptorLink from this interface are returned.
        """
        if not self.wait_for_service():
            return

        map_action = ActOnMapRequest()
        map_action.action = map_action.GET_DESCRIPTOR_LINKS
        map_action.object.id = id_
        if interface_name and interface_name != '*':
            map_action.interface_name = interface_name
        response = self.proxy(map_action)
        if not response:
            return
        return response.descriptor_links
