# Functionalities to convert the map into Python objects.

import rospy

from lama_msgs.msg import LamaObject

from lama_interfaces.map_agent import MapAgent
from lama_interfaces.srv import ActOnMapRequest

_map_agent = MapAgent()


def get_vertex_from_graph(graph, id_):
    """Return the vertex which is a key of graph and has id_ as id

    Return a LamaObject
    """
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
    graph = {}
    vertices = _map_agent.get_vertex_list()
    if vertices is None:
        return graph
    for vertex in vertices:
        # rospy deserializes arrays as tuples, convert to list.
        vertex.references = list(vertex.references)
        graph[vertex] = []
    # Get the edges (graph values).
    edges = _map_agent.get_edge_list()
    if edges is None:
        return graph
    for edge in edges:
        # rospy deserializes arrays as tuples, convert to list.
        edge.references = list(edge.references)
        first_vertex = get_vertex_from_graph(graph, edge.references[0])
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
    """Return the list of edges from v0 to v1, as LamaObject

    Return the list of edges from v0 to v1, as LamaObject. Return None on
    service error.
    """
    edge = LamaObject()
    edge.type = edge.EDGE
    edge.references = [v0, v1]
    return _map_agent.get_edge_list(edge)


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
    response = _map_agent.proxy(map_action)
    descriptors = []
    for descriptor_link in response.descriptor_links:
        getter_response = getter(descriptor_link.descriptor_id)
        descriptors.append(getter_response.descriptor)
    return descriptors
