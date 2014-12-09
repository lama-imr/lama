#!/usr/bin/python

import rospy

from lama_interfaces.core_interface import MapAgentInterface
from lama_interfaces.msg import LamaObject
from lama_interfaces.srv import ActOnMap
from lama_interfaces.srv import ActOnMapRequest

_iface = MapAgentInterface(start=False)
_map_agent = rospy.ServiceProxy(_iface.action_service_name, ActOnMap)


def _get_lama_object_from_id(graph, id_):
    """Return the LamaObject which is a key of graph and has id_ as id"""
    for lama_object in graph.iterkeys():
        if lama_object.id == id_:
            return lama_object
    return None


def get_directed_graph():
    """Return the directed graph as a dict {vertex: [edge0, edge1, ...], ...}

    Return the directed graph as a dict {vertex: [edge0, edge1, ...], ...},
    where vertex is a LamaObject of type vertex and edge0 is a LamaObject
    of type edge and with edge0.references[0] = vertex.
    All vertices are listed as key. All edges are listed as values.
    """
    # Get the vertices (graph keys).
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_VERTEX_LIST
    response = _map_agent.call(map_action)
    graph = {}
    for vertex in response.objects:
        graph[vertex] = []
    # Get the edges (graph values).
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_EDGE_LIST
    response = _map_agent.call(map_action)
    for edge in response.objects:
        first_vertex = _get_lama_object_from_id(graph, edge.references[0])
        if first_vertex is None:
            rospy.logerr(('Vertex {} does not exist although ' +
                          'it is the first vertex of edge {}').format(
                              edge.references[0],
                              edge.id))
            continue
        graph[first_vertex].append(edge)
    return graph


def get_directed_graph_index():
    """Return the directed graph as a dict {vertex_id: [v0_id, v1_id...], ...}

    Return the directed graph as a dict {vertex_id: [v0_id, v1_id, ...], ...},
    where vertex_id is the index of a LamaObject of type vertex and v0_id is
    the index of a LamaObject, meaning that their is an edge from vertex_id to
    v0_id.
    """
    map_graph = get_directed_graph()
    graph = {}
    for vertex, edges in map_graph.iteritems():
        graph[vertex.id] = [e.references[1] for e in edges]
    return graph


def get_edges_with_vertices(v0, v1):
    """Return the list of edges from v0 to v1, as LamaObject"""
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_EDGE_LIST
    response = _map_agent(map_action)
    edges = []
    for edge in response.objects:
        if (edge.references[0] == v0) and (edge.references[1] == v1):
            edges.append(edge)
    return edges


def get_descriptors(object_id, interface, getter):
    """Retrieve the descriptors associated with LamaObject with id object_id

    Return a list of descriptors associated with getter.

    Parameters
    ----------
    - object_id: int, LamaObject's id
    - interface: str, interface name associated with getter
    - getter: ROS ServiceProxy for a ROS message type
    """
    map_action = ActOnMapRequest()
    map_action.action = map_action.GET_DESCRIPTOR_LINKS
    lama_object = LamaObject()
    lama_object.id = object_id
    map_action.object = lama_object
    map_action.interface_name = interface
    response = _map_agent(map_action)
    descriptors = []
    for descriptor_link in response.descriptor_links:
        getter_response = getter(descriptor_link.descriptor_id)
        descriptors.append(getter_response.descriptor)
    return descriptors