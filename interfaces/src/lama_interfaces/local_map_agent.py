import copy

import rospy
import roslib.message

from lama_msgs.msg import LamaObject
from lama_msgs.srv import GetInterfaceInfo
from lama_msgs.srv import GetInterfaceInfoResponse

from lama_interfaces.core_interface import CoreDBInterface
from lama_interfaces.map_agent import map_agent_name
from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest
from lama_interfaces.srv import ActOnMapResponse


class LocalMapAgent(object):
    """Define callbacks for ActOnMap and possibly start the map agent service

    The class directly accesses the database.
    """
    def __init__(self, start=False):
        action_srv_type = 'lama_interfaces/ActOnMap'
        srv_action_class = roslib.message.get_service_class(action_srv_type)
        # Action class.
        self.action_service_name = map_agent_name()
        self.action_service_class = srv_action_class
        if start:
            self.map_agent = rospy.Service(self.action_service_name,
                                           self.action_service_class,
                                           self.action_callback)
            # Start the service for GetInterfaceInfo.
            rospy.Service('get_interface_info',
                          GetInterfaceInfo,
                          self._get_interface_info_callback)
        else:
            self.map_agent = None
        self.proxy = rospy.ServiceProxy(self.action_service_name, ActOnMap)

        self.core_iface = CoreDBInterface(start=start)

    def get_lama_object_list(self, lama_object):
        """Retrieve all elements that match the search criteria

        Search criteria are attributes of lama_object with non-default values
        (defaults are 0 or '').
        Return a list of LamaObject, or an empty list of no LamaObject matches.

        Parameters
        ----------
        - lama_object: an instance of LamaObject
        """
        # Reset references, because it happens to be a tuple.
        if lama_object.type == LamaObject.VERTEX:
            lama_object.references = [0, 0]

        coretable = self.core_iface.core_table
        query = coretable.select()
        # We exclude the undefined node.
        query = query.where(coretable.c['id'] != 0)
        for attr in self.core_iface.direct_attributes:
            v = getattr(lama_object, attr)
            if v:
                # Only add a "where" clause, where the attribute is non-default
                # (i.e. neither 0 nor '').
                query = query.where(coretable.c[attr] == v)
        if lama_object.references[0]:
            query = query.where(coretable.c['v0'] == lama_object.references[0])
        if lama_object.references[1]:
            query = query.where(coretable.c['v1'] == lama_object.references[1])
        rospy.logdebug('SQL query: {}'.format(query))

        connection = self.core_iface.engine.connect()
        with connection.begin():
            results = connection.execute(query).fetchall()
        connection.close()
        if not results:
            return []
        objects = []
        for result in results:
            lama_object = self.core_iface._lama_object_from_query_result(result)
            objects.append(lama_object)
        return objects

    def action_callback(self, msg):
        """Callback of ActOnMap service

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # Raise an error if pushing the wrong type.
        if ((msg.action == msg.PUSH_VERTEX) and
            (msg.object.type not in [0, msg.object.VERTEX])):
            raise rospy.ServiceException(
                'Action PUSH_VERTEX, LamaObject is not a vertex')
        if ((msg.action == msg.PUSH_EDGE) and
            (msg.object.type not in [0, msg.object.EDGE])):
            raise rospy.ServiceException(
                'Action PUSH_EDGE, LamaObject is not an edge')

        # Force object type for PUSH_VERTEX and PUSH_EDGE.
        if msg.action == msg.PUSH_EDGE:
            msg.object.type = msg.object.EDGE
        if msg.action == msg.PUSH_VERTEX:
            msg.object.type = msg.object.VERTEX

        callbacks = {
            msg.PUSH_VERTEX: self.push_lama_object,
            msg.PULL_VERTEX: self.pull_lama_object,
            msg.ASSIGN_DESCRIPTOR_VERTEX: (
                self.assign_descriptor_to_lama_object),
            msg.PUSH_EDGE: self.push_lama_object,
            msg.PULL_EDGE: self.pull_lama_object,
            msg.ASSIGN_DESCRIPTOR_EDGE: (
                self.assign_descriptor_to_lama_object),
            msg.GET_VERTEX_LIST: self.get_vertex_list,
            msg.GET_EDGE_LIST: self.get_edge_list,
            msg.GET_DESCRIPTOR_LINKS: self.get_descriptor_links,
            msg.GET_NEIGHBOR_VERTICES: self.get_neighbor_vertices,
            msg.GET_OUTGOING_EDGES: self.get_outgoing_edges,
        }
        if msg.action not in callbacks:
            raise rospy.ServiceException('Action {} not implemented'.format(
                msg.action))
        callback = callbacks[msg.action]
        response = callback(msg)
        return response

    def _get_interface_info_callback(self, msg):
        response = GetInterfaceInfoResponse()
        response.interface_info = self.core_iface.get_interface_info(
            msg.interface_name)
        if response.interface_info is None:
            return
        return response

    def push_lama_object(self, msg):
        """Add a LaMa object to the database

        Callback for PUSH_VERTEX and PUSH_EDGE.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        lama_object = copy.copy(msg.object)
        lama_object.id = self.core_iface.set_lama_object(lama_object)
        response = ActOnMapResponse()
        response.objects.append(lama_object)
        return response

    def pull_lama_object(self, msg):
        """Retrieve a LaMa object from the database

        Callback for PULL_VERTEX and PULL_EDGE.
        The object id is given in msg.object.id.
        Return an instance of ActOnMap response. The field descriptor_links will
        be filled with all DescriptorLink associated with this LamaObject.
        An error is raised if more LaMa objects correspond to the search
        criteria.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        response = ActOnMapResponse()
        id_ = msg.object.id
        response.objects.append(self.core_iface.get_lama_object(id_))
        get_desc_links = self.core_iface.get_descriptor_links
        response.descriptor_links = get_desc_links(id_)
        return response

    def assign_descriptor_to_lama_object(self, msg):
        """Add a descriptor to a vertex

        Callback for ASSIGN_DESCRIPTOR_VERTEX and ASSIGN_DESCRIPTOR_EDGE.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # Ensure that the lama object exists in the core table.
        object_id = msg.object.id
        core_table = self.core_iface.core_table
        query = core_table.select(
            whereclause=(core_table.c.id == object_id))
        connection = self.core_iface.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            err = 'No lama object with id {} in database table {}'.format(
                object_id, core_table.name)
            raise rospy.ServiceException(err)

        # Ensure that the descriptor exists in the database.
        if not msg.interface_name:
            raise rospy.ServiceException('Missing interface name')
        table_name = msg.interface_name
        if not self.core_iface.has_table(table_name):
            err = 'No interface "{}" in the database'.format(
                msg.interface_name)
            raise rospy.ServiceException(err)
        table = self.core_iface.metadata.tables[table_name]
        desc_id = msg.descriptor_id
        query = table.select(
            whereclause=(table.c.id == desc_id))
        connection = self.core_iface.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            err = 'No descriptor with id {} in database table {}'.format(
                desc_id, table.name)
            raise rospy.ServiceException(err)

        # Add the descriptor to the descriptor table.
        time = rospy.Time.now()
        insert_args = {
            'object_id': object_id,
            'descriptor_id': desc_id,
            'interface_name': table_name,
            'timestamp_secs': time.secs,
            'timestamp_nsecs': time.nsecs,
        }
        connection = self.core_iface.engine.connect()
        with connection.begin():
            connection.execute(self.core_iface.descriptor_table.insert(),
                               insert_args)
        connection.close()

        response = ActOnMapResponse()
        return response

    def get_vertex_list(self, msg):
        """Retrieve all vertices from the database

        Callback for GET_VERTEX_LIST.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        msg.object.type = LamaObject.VERTEX
        response = ActOnMapResponse()
        response.objects = self.get_lama_object_list(msg.object)
        return response

    def get_edge_list(self, msg):
        """Retrieve edges from the database

        Callback for GET_EDGE_LIST.
        Retrieved all edges from the database that correspond to the search
        criteria given in msg.object.
        Search criteria are attributes with non-default values (0 or '').
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        msg.object.type = LamaObject.EDGE
        response = ActOnMapResponse()
        response.objects = self.get_lama_object_list(msg.object)
        return response

    def get_descriptor_links(self, msg):
        """Retrieve DescriptorLink associated with a LamaObject and an interface

        Callback for GET_DESCRIPTOR_LINKS.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        response = ActOnMapResponse()
        response.objects.append(msg.object)
        get_desc_links = self.core_iface.get_descriptor_links
        response.descriptor_links = get_desc_links(msg.object.id,
                                                   msg.interface_name)
        return response

    def get_neighbor_vertices(self, msg):
        """Retrieve all neighbor vertices from the database

        Callback for GET_NEIGHBOR_VERTICES.
        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # TODO: Discuss with Karel what this action does.
        rospy.logerr('GET_NEIGHBOR_VERTICES not implemented')
        response = ActOnMapResponse()
        return response

    def get_outgoing_edges(self, msg):
        """Retrieve edges starting at a given vertex

        Callback for GET_OUTGOING_EDGES.
        Return an instance of ActOnMap response containing edges (instances
        of LamaObject) starting at the given vertex.
        This is syntactic sugar because the functionality can be obtained with
        GET_EDGE_LIST.

        Parameters
        ----------
        - msg: an instance of ActOnMapRequest.
        """
        msg_get_edges = ActOnMapRequest()
        msg_get_edges.object.type = msg_get_edges.object.EDGE
        msg_get_edges.object.references[0] = msg.object.id
        return self.get_edge_list(msg_get_edges)
