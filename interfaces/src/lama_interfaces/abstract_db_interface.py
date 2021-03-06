# Base class for database interfaces.

from abc import ABCMeta, abstractmethod, abstractproperty
import sqlalchemy

import rospkg
import rospy
import roslib.message

# sqlalchemy engine (argument to sqlalchemy.create_engine).
# Defaults to '"ros_home"/lama.sqlite'.
ros_home = rospkg.get_ros_home()
default_engine_name = 'sqlite:///' + ros_home + '/lama.sqlite'
_engine_name = rospy.get_param('/database_engine', default_engine_name)
del ros_home
del default_engine_name

# Table name for type description
_interfaces_table_name = 'map_interfaces'


class AbstractDBInterface(object):
    __metaclass__ = ABCMeta

    # Database related class attributes.
    engine_name = _engine_name
    engine = sqlalchemy.create_engine(_engine_name)
    metadata = sqlalchemy.MetaData()
    metadata.bind = engine

    def __init__(self, interface_name, getter_srv_type, setter_srv_type,
                 start=False):
        """Build the map interface and possibly start ROS services

        Parameters
        ----------
        - interface_name: string, name of the map interface.
        - getter_srv_type: string, service message to write into the map.
        - setter_srv_type: string, service message to read from the map.
        - start: {True|False}, defaults to True. The ROS services for getter and
            setter will be started only if start is True. If start is False, the
            clients proxies will be None.
        """
        if '@' in interface_name:
            rospy.logerr('@ not allowd in interface name')
            raise ValueError('@ not allowd in interface name')

        get_srv_class = roslib.message.get_service_class(getter_srv_type)
        set_srv_class = roslib.message.get_service_class(setter_srv_type)

        rospy.logdebug('Map interface: {} ({}, {})'.format(interface_name,
                                                           getter_srv_type,
                                                           setter_srv_type))
        rospy.logdebug('Getter class {}'.format(get_srv_class))
        rospy.logdebug('Getter request slots: {}'.format(
            get_srv_class._request_class.__slots__))
        rospy.logdebug('Getter response slots: {}'.format(
            get_srv_class._response_class.__slots__))
        rospy.logdebug('Setter class {}'.format(set_srv_class))
        rospy.logdebug('Setter request slots: {}'.format(
            set_srv_class._request_class.__slots__))
        rospy.logdebug('Setter response slots: {}'.format(
            set_srv_class._response_class.__slots__))

        # getter class
        self.getter_service_name = self.default_getter_service_name(
            interface_name)
        self.getter_service_class = get_srv_class
        self.getter_service_type = getter_srv_type
        # setter class
        self.setter_service_name = self.default_setter_service_name(
            interface_name)
        self.setter_service_class = set_srv_class
        self.setter_service_type = setter_srv_type
        # interface name
        self.interface_name = interface_name

        # Read tables from the possibly existing database.
        self.metadata.reflect()
        # Add the table for type description.
        self.interface_table = self._interface_table()
        self.metadata.create_all()
        # Create new tables.
        self._generate_schema()

        # Possibly start the services.
        self._getter_service = None
        self._setter_service = None
        if start:
            self.start_services()

        # Get the service clients.
        self.getter_service_proxy = rospy.ServiceProxy(
            self.getter_service_name,
            self.getter_service_class)
        self.setter_service_proxy = rospy.ServiceProxy(
            self.setter_service_name,
            self.setter_service_class)

    @classmethod
    def default_getter_service_name(cls, interface_name):
        return interface_name + '_getter'

    @classmethod
    def default_setter_service_name(cls, interface_name):
        return interface_name + '_setter'

    @abstractproperty
    def interface_type(self):
        pass

    @abstractmethod
    def _generate_schema():
        pass

    @abstractmethod
    def getter_callback(self):
        pass

    @abstractmethod
    def setter_callback(self):
        pass

    def _interface_table(self):
        """Return the table for the type description (may already exists).

        Return a table with columns ('interface_name', 'message_type',
        interface_type, timestamp_secs, timestamp_nsecs). If the
        table already exists, it will be returned.
        """
        table = sqlalchemy.Table(_interfaces_table_name,
                                 self.metadata,
                                 sqlalchemy.Column('interface_name',
                                                   sqlalchemy.String,
                                                   unique=True),
                                 sqlalchemy.Column('message_type',
                                                   sqlalchemy.String),
                                 sqlalchemy.Column('get_service_type',
                                                   sqlalchemy.String),
                                 sqlalchemy.Column('set_service_type',
                                                   sqlalchemy.String),
                                 sqlalchemy.Column('interface_type',
                                                   sqlalchemy.String),
                                 sqlalchemy.Column('timestamp_secs',
                                                   sqlalchemy.BigInteger),
                                 sqlalchemy.Column('timestamp_nsecs',
                                                   sqlalchemy.BigInteger),
                                 extend_existing=True)

        return table

    def _add_interface_description(self):
        """Add the inteface description if not already existing

        Add the inteface description with unconflicting
        (interface_name / message_type / interface_type).

        A ValueError is raised if a table with the same name (i.e. same
        interface_name) but different message type and interface type already
        exists.
        """
        # Check for an existing table.
        table = self.interface_table
        name = self.interface_name
        msg_type = self.getter_service_class._response_class._slot_types[0]

        query = table.select(whereclause=(table.c.interface_name == name))
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()

        add_interface = True
        if result:
            add_interface = False
            if (result['message_type'] != msg_type or
                result['interface_type'] != self.interface_type):
                err = ('A table "{}" with message type "{}" and interface ' +
                       'type "{}" already exists, cannot change to ' +
                       '"{}"/"{}"').format(
                           name,
                           result['message_type'], result['interface_type'],
                           msg_type, self.interface_type)
                rospy.logfatal(err)
                raise rospy.ROSException(err)

        # Add the table description.
        if add_interface:
            insert_args = {
                'interface_name': name,
                'message_type': msg_type,
                'interface_type': self.interface_type,
                'get_service_type': self.getter_service_type,
                'set_service_type': self.setter_service_type,
            }
            with connection.begin():
                connection.execute(table.insert(), insert_args)
        connection.close()

    def _get_last_modified(self):
        """Return the date of last modification for this interface

        Return a rospy.Time instance.
        """
        table = self.interface_table
        name = self.interface_name
        query = table.select(whereclause=(table.c.interface_name == name))
        connection = self.engine.connect()
        with connection.begin():
            result = connection.execute(query).fetchone()
        connection.close()

        if not result:
            raise rospy.ServiceException('Corrupted database')
        time = rospy.Time(0)
        if result['timestamp_secs'] is None:
            return time
        time.secs = result['timestamp_secs']
        time.nsecs = result['timestamp_nsecs']

    def _set_timestamp(self, time):
        """Set the timestamp to the given time for this interface

        Parameters
        ----------
        - time: a rospy.Time instance.
        """
        table = self.interface_table
        name = self.interface_name
        update_args = {
            'timestamp_secs': time.secs,
            'timestamp_nsecs': time.nsecs,
        }
        update = table.update().where(table.c.interface_name == name)
        connection = self.engine.connect()
        with connection.begin():
            connection.execute(update, update_args)
        connection.close()

    def has_table(self, table):
        """Return true if table is in the database"""
        if table in self.metadata.tables:
            return True
        # Tables can be created by other instances, update and check again.
        self.metadata.reflect()
        return table in self.metadata.tables

    def start_services(self):
        self._getter_service = rospy.Service(self.getter_service_name,
                                             self.getter_service_class,
                                             self.getter_callback)
        self._setter_service = rospy.Service(self.setter_service_name,
                                             self.setter_service_class,
                                             self.setter_callback)
        rospy.loginfo('Services %s and %s started',
                      self.getter_service_name, self.setter_service_name)
