#!/usr/bin/python
# -*- coding: utf-8 -*-

from lama_interfaces.cleartext_interface_factory import cleartext_interface_factory

from test_interfaces_base import DbMessagePassingBase, RosTestCase


class TestDbMessagePassingCleartext(DbMessagePassingBase, RosTestCase):
    def interface_factory(self, n, gs, ss):
        return cleartext_interface_factory(n, gs, ss)
    interface_type = 'cleartext'

if __name__ == '__main__':
    import rostest
    rostest.rosrun('lama_interfaces',
                   'test_db_message_passing_cleartext',
                   TestDbMessagePassingCleartext)
