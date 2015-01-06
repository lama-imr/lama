#!/usr/bin/python
# -*- coding: utf-8 -*-

from lama_interfaces.interface_factory import interface_factory

from test_interfaces_base import DbMessagePassingBase, RosTestCase


class TestDbMessagePassingSerialized(DbMessagePassingBase, RosTestCase):
    def interface_factory(self, n, gs, ss):
        return interface_factory(n, gs, ss)
    interface_type = 'serialization'


if __name__ == '__main__':
    import rostest
    rostest.rosrun('lama_interfaces',
                   'test_db_message_passing_serialized',
                   TestDbMessagePassingSerialized)
