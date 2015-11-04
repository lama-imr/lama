# Class for management of the map core functionalities.
# This includes management of vertices, edges, and descriptor links.

from sqlalchemy import ForeignKey
from sqlalchemy import Table
from sqlalchemy import Column
from sqlalchemy import types

import rospy

from lama_msgs.msg import DescriptorLink
from lama_msgs.msg import InterfaceInfo
from lama_msgs.msg import LamaObject

from abstract_db_interface import AbstractDBInterface

_default_core_table_name = 'lama_object'
_default_descriptor_table_name = 'lama_descriptor_link'
_expected_lama_object_md5sum = 'e2747a1741c10b06140b9673d9018102'


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

        get_srv_type = 'lama_msgs/GetLamaObject'
        set_srv_type = 'lama_msgs/SetLamaObject'
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

        # Add the "unknown" edge. This is used to indicate that the robot
        # position is unknown because we didn't recognized any vertex yet.
        unknown_edge = LamaObject()
        unknown_edge.id = -2
        unknown_edge.name = 'unknown'
        unknown_edge.type = LamaObject.EDGE
        unknown_edge.references[0] = unvisited_vertex.id
        unknown_edge.references[1] = unvisited_vertex.id
        self.set_lama_object(unknown_edge)

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
        if lama_object._md5sum != _expected_lama_object_md5sum:
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
        # The table format is hard-coded but the compatibility was checked in
        # __init__.
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

    def _lama_object_from_query_result(self, result):
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

        return self._lama_object_from_query_result(result)

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
        if ((lama_object.id == 0) and
            (lama_object.type == LamaObject.EDGE) and
            (0 in lama_object.references)):
                # 0 is undefined and not allowed.
                raise rospy.ServiceException('edge references cannot be 0')

        is_undefined = (lama_object.name == 'undefined')
        is_special_vertex = lama_object.id < 0 or is_undefined
        is_new_vertex = lama_object.id == 0 and not is_undefined

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

        if result is not None and not is_new_vertex:
            # Update existing Lama Object.
            object_id = self._update_lama_object(lama_object)
        else:
            # Insert new Lama Object.
            object_id = self._insert_lama_object(lama_object)

        return object_id

    def _insert_lama_object(self, lama_object):
        """Insert a new Lama Object into the core table without any checks

        Do not invoke directly, use self.set_lama_object.
        """
        insert_args = {
            'id_in_world': lama_object.id_in_world,
            'name': lama_object.name,
            'emitter_id': lama_object.emitter_id,
            'emitter_name': lama_object.emitter_name,
            'type': lama_object.type,
            'v0': lama_object.references[0],
            'v1': lama_object.references[1],
        }
        if lama_object.id:
            insert_args['id'] = lama_object.id
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(self.core_table.insert(),
                                        insert_args)
        connection.close()
        object_id = result.inserted_primary_key[0]
        self._set_timestamp(rospy.Time.now())
        return object_id

    def _update_lama_object(self, lama_object):
        """Update a Lama Object without any checks

        Do not invoke directly, use self.set_lama_object.
        """
        # Update the core table, if necessary.
        update_args = {}
        for attr in self.direct_attributes:
            v = getattr(lama_object, attr)
            if v:
                update_args[attr] = v
        if lama_object.references[0]:
            update_args['v0'] = lama_object.references[0]
        if lama_object.references[1]:
            update_args['v1'] = lama_object.references[1]
        if update_args:
            update = self.core_table.update(
                whereclause=(self.core_table.c.id == lama_object.id))
            connection = self.engine.connect()
            with connection.begin():
                connection.execute(update, update_args)
            connection.close()

        self._set_timestamp(rospy.Time.now())
        return lama_object.id

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

    def get_interface_info(self, interface_name):
        table = self.interface_table
        query = table.select()
        query = query.where(table.c.interface_name == interface_name)
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()
        connection.close()
        if not result:
            return
        interface_info = InterfaceInfo()
        interface_info.interface_name = interface_name
        interface_info.interface_type = str(result['interface_type'])
        interface_info.message_type = str(result['message_type'])
        interface_info.get_service_type = str(result['get_service_type'])
        interface_info.set_service_type = str(result['set_service_type'])
        interface_info.timestamp.secs = int(result['timestamp_secs'])
        interface_info.timestamp.nsecs = int(result['timestamp_nsecs'])
        interface_info.get_service_name = self.default_getter_service_name(
            interface_name)
        interface_info.set_service_name = self.default_setter_service_name(
            interface_name)
        return interface_info
