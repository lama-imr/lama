# Class for management of the map core functionalities.
# This includes management of vertices, edges, and descriptor links.

import copy

from sqlalchemy import ForeignKey
from sqlalchemy import Table
from sqlalchemy import Column
from sqlalchemy import types

import rospy
import roslib.msgs
import roslib.message

from lama_interfaces.msg import LamaObject
from lama_interfaces.msg import DescriptorLink
from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest
from lama_interfaces.srv import ActOnMapResponse

from abstract_db_interface import AbstractDBInterface

_default_core_table_name = 'lama_object'
_default_descriptor_table_name = 'lama_descriptor_link'
_default_map_agent_name = '/lama_map_agent'


class CoreDBInterface(AbstractDBInterface):
    def __init__(self, interface_name=None, descriptor_table_name=None,
                 start=False):
        """Build the map interface for LamaObject

        Parameters
        ----------
        - interface_name: string, name of the table containing LamaObject
            messages (vertices and edges). Defaults to 'lama_objects'.
        - descriptor_table_name: string, name of the table for DescriptorLink
            messages. Defaults to 'lama_descriptor_links'.
        """
        self._check_md5sum()
        self.direct_attributes = ['id', 'id_in_world', 'name', 'emitter_id',
                                  'emitter_name', 'type']
        if not interface_name:
            interface_name = _default_core_table_name
        if not descriptor_table_name:
            descriptor_table_name = _default_descriptor_table_name
        self.descriptor_table_name = descriptor_table_name

        get_srv_type = 'lama_interfaces/GetLamaObject'
        set_srv_type = 'lama_interfaces/SetLamaObject'
        super(CoreDBInterface, self).__init__(interface_name,
                                              get_srv_type, set_srv_type,
                                              start=start)

        # Add the "unvisited" vertex. Edge for which the outoing vertex is not
        # visited yet have reference[1] == unvisited_vertex.id.
        unvisited_vertex = LamaObject()
        unvisited_vertex.id = -1
        unvisited_vertex.name = 'unvisited'
        unvisited_vertex.type = LamaObject.VERTEX
        self.set_lama_object(unvisited_vertex)

        # Add the "undefined" vertex. This is to ensure that automatically
        # generated ids (primary keys) are greater than 0.
        undefined_vertex = LamaObject()
        undefined_vertex.id = 0
        undefined_vertex.name = 'undefined'
        undefined_vertex.type = LamaObject.VERTEX
        self.set_lama_object(undefined_vertex)

    @property
    def interface_type(self):
        return 'core'

    def _check_md5sum(self):
        """Check that current implementation is compatible with LamaObject"""
        lama_object = LamaObject()
        if lama_object._md5sum != 'e2747a1741c10b06140b9673d9018102':
            raise rospy.ROSException('CoreDBInterface incompatible ' +
                                     'with current LamaObject implementation')

    def _generate_schema(self):
        """Create the SQL tables"""
        self._add_interface_description()
        self._generate_core_table()
        self._generate_descriptor_table()
        self.metadata.create_all()

    def _generate_core_table(self):
        """Create the SQL tables for LamaObject messages"""
        # The table format is hard-coded.
        table = Table(self.interface_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True),
                      extend_existing=True)
        table.append_column(Column('id_in_world',
                                   types.Integer))
        table.append_column(Column('name', types.String))
        table.append_column(Column('emitter_id', types.Integer))
        table.append_column(Column('emitter_name', types.String))
        table.append_column(Column('type', types.Integer))
        table.append_column(Column('v0', types.Integer))
        table.append_column(Column('v1', types.Integer))
        self.core_table = table

    def _generate_descriptor_table(self):
        """Create the SQL tables for descriptor_links"""
        table = Table(self.descriptor_table_name,
                      self.metadata,
                      Column('id', types.Integer, primary_key=True),
                      extend_existing=True)
        table.append_column(Column('object_id',
                                   types.Integer,
                                   ForeignKey(self.interface_name + '.id')))
        table.append_column(Column('descriptor_id', types.Integer))
        table.append_column(Column('interface_name', types.String))
        table.append_column(Column('timestamp_secs', types.Integer))
        table.append_column(Column('timestamp_nsecs', types.Integer))
        # TODO: add a uniqueness constraint on (object_id, descriptor_id,
        # interface_name)
        self.descriptor_table = table

    def _lama_object_from_result(self, result):
        lama_object = LamaObject()
        for attr in self.direct_attributes:
            setattr(lama_object, attr, result[attr])
        lama_object.references[0] = result['v0']
        lama_object.references[1] = result['v1']
        return lama_object

    def getter_callback(self, msg):
        """Get a LamaObject from the database

        Get a LamaObject from the database, from its id.
        Return an instance of GetLamaObject.srv response.

        Parameters
        ----------
        - msg: an instance of GetLamaObject.srv request.
        """
        id_ = msg.id
        lama_object = self.get_lama_object(id_)
        # Create an instance of getter response.
        response = self.getter_service_class._response_class()
        response.object = lama_object
        return response

    def setter_callback(self, msg):
        """Add a LamaObject message to the database

        Return an instance of SetLamaObject.srv response.

        Parameters
        ----------
        - msg: an instance of SetLamaObject.srv request.
        """
        # Create an instance of setter response.
        response = self.setter_service_class._response_class()
        response.id = self.set_lama_object(msg.object)
        return response

    def get_lama_object(self, id_):
        """Get a vertex or an edge from its unique id_

        Return an instance of LamaObject.

        Parameters
        ----------
        - id_: int, lama object id (id in the database).
        """
        # Make the transaction for the core table.
        query = self.core_table.select(
            whereclause=(self.core_table.c.id == id_))
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            err = 'No element with id {} in database table {}'.format(
                id_, self.core_table.name)
            raise rospy.ServiceException(err)

        return self._lama_object_from_result(result)

    def set_lama_object(self, lama_object):
        """Add/modify a lama object to the database

        Return the lama object's id.

        Parameter
        ---------
        - lama_object: an instance of LamaObject.
        """
        if len(lama_object.references) != 2:
                raise rospy.ServiceException(
                    'malformed references, length = {}'.format(
                        len(lama_object.references)))
        if lama_object.type == LamaObject.EDGE and 0 in lama_object.references:
                # 0 is undefined and not allowed.
                raise rospy.ServiceException('edge references cannot be 0')

        is_undefined_vertex = (lama_object.name == 'undefined')
        is_special_vertex = lama_object.id < 0 or is_undefined_vertex
        is_new_vertex = lama_object.id == 0 and not is_undefined_vertex

        # Check for id existence.
        query = self.core_table.select(
            whereclause=(self.core_table.c.id == lama_object.id))
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()
        connection.close()

        if result is not None and is_special_vertex:
            # Exit if lama_object is an already-existing special object.
            return lama_object.id

        # Possibly delete any node with the same id.
        if result is not None and not is_new_vertex:
            self.del_lama_object(lama_object.id)

        # Insert data into the core table.
        insert_args = {
            'id_in_world': lama_object.id_in_world,
            'name': lama_object.name,
            'emitter_id': lama_object.emitter_id,
            'emitter_name': lama_object.emitter_name,
            'type': lama_object.type,
            'v0': lama_object.references[0],
            'v1': lama_object.references[1],
        }
        if not is_new_vertex:
            # If id is not given, i.e. if it is 0, the database will give it
            # an id automatically (primary key).
            insert_args['id'] = lama_object.id
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(self.core_table.insert(),
                                        insert_args)
        connection.close()
        object_id = result.inserted_primary_key[0]

        self._set_timestamp(rospy.Time.now())
        return object_id

    def del_lama_object(self, id_):
        """Remove a LamaObject from the database"""
        delete = self.core_table.delete(
            whereclause=(self.core_table.c.id == id_))
        connection = self.engine.connect()
        with connection.begin():
            connection.execute(delete)
        connection.close()

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
        desc_links = []
        # Make the transaction from the descriptor table.
        table = self.descriptor_table
        query = table.select()
        query = query.where(table.c.object_id == id_)
        if interface_name and interface_name != '*':
            query = query.where(table.c.interface_name == interface_name)
        connection = self.engine.connect()
        with connection.begin():
            results = connection.execute(query).fetchall()
        connection.close()
        if not results:
            return []
        for result in results:
            desc_link = DescriptorLink()
            desc_link.object_id = id_
            desc_link.descriptor_id = result['descriptor_id']
            desc_link.interface_name = result['interface_name']
            desc_links.append(desc_link)
        return desc_links


class MapAgentInterface(object):
    """Define callbacks for ActOnMap and start the map agent service"""
    def __init__(self, start=False):
        action_srv_type = 'lama_interfaces/ActOnMap'
        srv_action_class = roslib.message.get_service_class(action_srv_type)
        # Action class.
        map_agent_name = rospy.get_param('map_agent', _default_map_agent_name)
        self.action_service_name = map_agent_name
        self.action_service_class = srv_action_class
        if start:
            self.map_agent = rospy.Service(self.action_service_name,
                                           self.action_service_class,
                                           self.action_callback)
            self.map_agent_proxy = rospy.ServiceProxy(self.action_service_name,
                                                      ActOnMap)
        else:
            self.map_agent = None
            self.map_agent_proxy = None

        self.core_iface = CoreDBInterface(start=False)

    def action_callback(self, msg):
        """Callback of ActOnMap service

        Return an instance of ActOnMap response.

        Parameters
        ----------
        - msg: an instance of ActOnMap request.
        """
        # Raise an error if pushing the wrong type.
        if (msg.action == msg.PUSH_VERTEX and
            msg.object.type not in [0, msg.object.VERTEX]):
            raise rospy.ServiceException(
                'Action PUSH_VERTEX, LamaObject is not a vertex')
        if (msg.action == msg.PUSH_EDGE and
            msg.object.type not in [0, msg.object.EDGE]):
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

    def get_lama_object_list(self, lama_object):
        """Retrieve all elements that match the search criteria

        Search criteria are attributes of lama_object with non-default values
        (0 or '').
        Return a list of LamaObject, or an empty list of no LamaObject matches.

        Parameters
        ----------
        - lama_object: an instance of LamaObject
        """
        if lama_object.type == LamaObject.VERTEX:
            lama_object.references = [0, 0]

        coretable = self.core_iface.core_table
        query = coretable.select()
        query = query.where(coretable.c['id'] != 0)
        for attr in self.core_iface.direct_attributes:
            v = getattr(lama_object, attr)
            if v:
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
            lama_object = self.core_iface._lama_object_from_result(result)
            objects.append(lama_object)
        return objects

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


def core_interface():
    """Return an interface and a map agent classes and run associated services

    Generate an interface class and run the getter, setter, and action services.
    Service definition must be in the form
    GetLamaObject.srv:
      int32 id
      ---
      LamaObject object

    and
    SetLamaObject.srv:
      LamaObject object
      ---
      int32 id

    This function should be called only once with each parameter set because
    it starts ROS services and an error is raised if services are started
    twice.
    """
    iface = CoreDBInterface(start=True)
    map_agent_iface = MapAgentInterface(start=True)
    return iface, map_agent_iface
