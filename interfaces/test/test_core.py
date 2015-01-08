#!/usr/bin/python
# -*- coding: utf-8 -*-
# Unit tests for core_interface.py

import unittest

import rospy

from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest
from lama_msgs.msg import LamaObject

vertex1 = LamaObject()
vertex1.id_in_world = 101
vertex1.name = 'vertex1'
vertex1.type = LamaObject.VERTEX

vertex2 = LamaObject()
vertex2.id_in_world = 102
vertex2.name = 'vertex2'
vertex2.type = LamaObject.VERTEX

vertex3 = LamaObject()
vertex3.id_in_world = 103
vertex3.name = 'vertex3'
vertex3.type = LamaObject.VERTEX

vertex3a = LamaObject()
vertex3a.id_in_world = 104
vertex3a.name = 'vertex3'
vertex3a.type = LamaObject.VERTEX

vertex3b = LamaObject()
vertex3b.id_in_world = 103
vertex3b.name = 'vertex3b'
vertex3b.type = LamaObject.VERTEX

edge1 = LamaObject()
edge1.id_in_world = 101
edge1.name = 'edge1'
edge1.type = LamaObject.EDGE

edge2 = LamaObject()
edge2.id_in_world = 102
edge2.name = 'edge2'
edge2.type = LamaObject.EDGE

edge3 = LamaObject()
edge3.id_in_world = 103
edge3.name = 'edge3'
edge3.type = LamaObject.EDGE


class TestCoreInterface(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        rospy.init_node('test_lama_core', anonymous=True)
        self.map_agent = rospy.ServiceProxy('lama_map_agent', ActOnMap)
        self.map_agent.wait_for_service()
        super(TestCoreInterface, self).__init__(*args, **kwargs)

    def assertNonEmpty(self, objects):
        self.assertGreater(len(objects), 0,
                           msg='At least one LamaObject expected')

    def assertObjectEqual(self, object0, object1):
        """Fail if LamaObjects are not equal"""
        self.assertIsInstance(object0, LamaObject,
                              msg='Argument 1 is not a LamaObject')
        self.assertIsInstance(object1, LamaObject,
                              msg='Argument 2 is not a LamaObject')
        self.assertIdEqual(object0, object1)
        self.assertIdInWorldEqual(object0, object1)
        self.assertNameEqual(object0, object1)
        self.assertReferencesEqual(object0, object1)

    def assertIdEqual(self, object0, object1):
        """Fail if id are not equal"""
        self.assertIsInstance(object0, LamaObject,
                              msg='Argument 1 is not a LamaObject')
        self.assertIsInstance(object1, LamaObject,
                              msg='Argument 2 is not a LamaObject')
        self.assertEqual(object0.id, object1.id,
                         msg='id differ: {} != {}'.format(
                             object0.id, object1.id))

    def assertIdInWorldEqual(self, object0, object1):
        """Fail if id_in_world are not equal"""
        self.assertIsInstance(object0, LamaObject,
                              msg='Argument 1 is not a LamaObject')
        self.assertIsInstance(object1, LamaObject,
                              msg='Argument 2 is not a LamaObject')
        self.assertEqual(object0.id_in_world, object1.id_in_world,
                         msg='id_in_world differ: {} != {}'.format(
                             object0.id_in_world, object1.id_in_world))

    def assertNameEqual(self, object0, object1):
        """Fail if name are not equal"""
        self.assertIsInstance(object0, LamaObject,
                              msg='Argument 1 is not a LamaObject')
        self.assertIsInstance(object1, LamaObject,
                              msg='Argument 2 is not a LamaObject')
        self.assertEqual(object0.name, object1.name,
                         msg='name differ: {} != {}'.format(
                             object0.name, object1.name))

    def assertReferencesEqual(self, object0, object1):
        """Fail if references are not equal"""
        self.assertIsInstance(object0, LamaObject,
                              msg='Argument 1 is not a LamaObject')
        self.assertIsInstance(object1, LamaObject,
                              msg='Argument 2 is not a LamaObject')
        self.assertListEqual(object0.references, object1.references)

    def test001_push_vertex(self):
        """Test ActOnMap service PUSH_VERTEX"""
        try:
            response = self.map_agent(object=vertex1,
                                      action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(False,
                            'service did not process request: {}'.format(exc))
        vertex1.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex1)

        try:
            response = self.map_agent(object=vertex2,
                                      action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(
                False,
                'service did not process request vertex2: {}'.format(exc))
        vertex2.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex2)

        try:
            response = self.map_agent(object=vertex3,
                                      action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(
                False,
                'service did not process request vertex3: {}'.format(exc))
        vertex3.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex3)

        try:
            response = self.map_agent(object=vertex3a,
                                      action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(
                False,
                'service did not process request vertex3a: {}'.format(exc))
        vertex3a.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex3a)

        try:
            response = self.map_agent(object=vertex3b,
                                      action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(
                False,
                'service did not process request vertex3b: {}'.format(exc))
        vertex3b.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex3b)

    def test002_push_existing_vertex(self):
        """Test ActOnMap service PUSH_VERTEX with existing vertex"""
        vertex1.name = 'vertex1a'
        try:
            response = self.map_agent(object=vertex1,
                                      action=ActOnMapRequest.PUSH_VERTEX)
        except rospy.ServiceException as exc:
            self.assertTrue(False,
                            'service did not process request: {}'.format(exc))
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex1)

    def test002_pull_vertex(self):
        """Test ActOnMap service PULL_VERTEX"""
        req = LamaObject()
        req.id = vertex1.id
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.PULL_VERTEX)
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], vertex1)

    def test002_pull_vertex_id_in_world(self):
        """Test ActOnMap service GET_VERTEX_LIST according to id_in_world"""
        req = LamaObject()
        req.id_in_world = vertex1.id_in_world
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_VERTEX_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(vertex1.id, [o.id for o in response.objects])
        for obj in response.objects:
            # Workaround a strange bug, where references is a tuple.
            obj.references = list(obj.references)
            self.assertIdInWorldEqual(obj, vertex1)

    def test002_pull_vertex_name(self):
        """Test ActOnMap service GET_VERTEX_LIST according to name"""
        req = LamaObject()
        req.name = vertex1.name
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_VERTEX_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(vertex1.id, [o.id for o in response.objects])
        for obj in response.objects:
            # Workaround a strange bug, where references is a tuple.
            obj.references = list(obj.references)
            self.assertNameEqual(obj, vertex1)

    def test002_push_edge(self):
        """Test ActOnMap service push edge """
        edge1.references[0] = vertex1.id
        edge1.references[1] = vertex2.id
        try:
            response = self.map_agent(object=edge1,
                                      action=ActOnMapRequest.PUSH_EDGE)
        except rospy.ServiceException as exc:
            self.assertTrue(False,
                            'service did not process request: {}'.format(exc))
        edge1.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], edge1)

        edge2.references[0] = vertex2.id
        edge2.references[1] = vertex3.id
        try:
            response = self.map_agent(object=edge2,
                                      action=ActOnMapRequest.PUSH_EDGE)
        except rospy.ServiceException as exc:
            self.assertTrue(False,
                            'service did not process request: {}'.format(exc))
        edge2.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], edge2)

        edge3.references = [vertex2.id, 0]
        self.assertRaises(rospy.ServiceException,
                          self.map_agent,
                          object=edge3, action=ActOnMapRequest.PUSH_EDGE)

        edge3.references = [vertex2.id, vertex2.id]
        try:
            response = self.map_agent(object=edge3,
                                      action=ActOnMapRequest.PUSH_EDGE)
        except rospy.ServiceException as exc:
            self.assertTrue(False,
                            'service did not process request: {}'.format(exc))
        edge3.id = response.objects[0].id
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], edge3)

    def test003_pull_edge(self):
        """Test ActOnMap service PULL_EDGE"""
        req = LamaObject()
        req.id = edge1.id
        response = self.map_agent(object=req, action=ActOnMapRequest.PULL_EDGE)
        self.assertEqual(len(response.objects), 1)
        # Workaround a strange bug, where msg.object.references is a tuple.
        response.objects[0].references = list(response.objects[0].references)
        self.assertObjectEqual(response.objects[0], edge1)

    def test003_pull_edge_id_in_world(self):
        """test ActOnMap service GET_EDGE_LIST according id in world"""
        req = LamaObject()
        req.id_in_world = edge1.id_in_world
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_EDGE_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(edge1.id, [o.id for o in response.objects])
        for obj in response.objects:
            # Workaround a strange bug, where references is a tuple.
            obj.references = list(obj.references)
            self.assertIdInWorldEqual(obj, vertex1)

    def test003_pull_edge_name(self):
        """ test ActOnMap service GET_EDGE_LIST according name"""
        req = LamaObject()
        req.name = edge1.name
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_EDGE_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(edge1.id, [o.id for o in response.objects])
        for obj in response.objects:
            # Workaround a strange bug, where references is a tuple.
            obj.references = list(obj.references)
            self.assertNameEqual(obj, edge1)

    def test003_pull_edge_vertices(self):
        """test ActOnMap service GET_EDGE_LIST according to both vertices"""
        req = LamaObject()
        req.references[0] = edge1.references[0]
        req.references[1] = edge1.references[1]
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_EDGE_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(edge1.id, [o.id for o in response.objects])
        for obj in response.objects:
            # Workaround a strange bug, where references is a tuple.
            obj.references = list(obj.references)
            self.assertReferencesEqual(obj, edge1)

    def test003_pull_edge_start_vertex(self):
        """test ActOnMap service GET_EDGE_LIST according to start vertex"""
        req = LamaObject()
        req.type = LamaObject.EDGE
        req.references[0] = edge1.references[0]
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_EDGE_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(edge1.id, [o.id for o in response.objects])
        for obj in response.objects:
            self.assertEqual(obj.references[0], edge1.references[0],
                             'wrong start vertex, got {}, expected {}'.format(
                                 obj.references[0], edge1.references[0]))

    def test003_get_outgoing_edges(self):
        """test ActOnMap service GET_OUTGOING_EDGES"""
        req = LamaObject()
        req.id = edge1.references[0]
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_OUTGOING_EDGES)
        self.assertNonEmpty(response.objects)
        self.assertIn(edge1.id, [o.id for o in response.objects])
        for obj in response.objects:
            self.assertEqual(obj.references[0], edge1.references[0],
                             'wrong start vertex, got {}, expected {}'.format(
                                 obj.references[0], edge1.references[0]))

    def test003_pull_edge_stop_vertex(self):
        """test ActOnMap service GET_EDGE_LIST according to stop vertex"""
        req = LamaObject()
        req.references[1] = edge1.references[1]
        response = self.map_agent(object=req,
                                  action=ActOnMapRequest.GET_EDGE_LIST)
        self.assertNonEmpty(response.objects)
        self.assertIn(edge1.id, [o.id for o in response.objects])
        for obj in response.objects:
            self.assertEqual(obj.references[1], edge1.references[1],
                             'wrong start vertex, got {}, expected {}'.format(
                                 obj.references[1], edge1.references[1]))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('lama_interfaces',
                   'test_core_interface',
                   TestCoreInterface)
