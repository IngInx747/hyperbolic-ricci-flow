#pragma once
#ifndef _MESH_GRAPH_H_
#define _MESH_GEAPH_H_

#include <vector>
#include <unordered_set>
#include <unordered_map>

/// Graph
/// @tparam Vertex
/// @tparam Weight of edge
template <class V, class W>
class Graph
{
public:
    using Edge = std::pair<V, std::pair<V, W>>;

    /// Get adjancent table
    /// @return adjancent table
    std::unordered_map<V, std::unordered_map<V, W>>& adj() { return m_G; }

    /// Get shortest distance between source and target
    /// @param [IN]  starting vertex
    /// @param [IN]  ending vertex
    /// @return distance of the shortest path
    W shortest_distance(V source, V target);

    /// Find shortest path from source to target
    /// @param [IN]  starting vertex
    /// @param [IN]  ending vertex
    /// @param [OUT] shortest path where vertices are ordered from target to source
    /// @return distance of the shortest path
    W shortest_path(V source, V target, std::vector<V>& path);

    /// Find representative vertices of every connecting components in the graph
    /// @param [OUT] representative vertices of connecting components
    void connecting_representatives(std::vector<V>& vertices);

    /// Find all connecting components in the graph
    /// @param [OUT] vertices of every connecting component in the graph
    void connecting_components(std::vector<std::vector<V>>& components);

    /// Find representative edges of every cycles in the graph
    /// @param [OUT] representative edges of every cycle in the graph
    void cycle_representatives(std::vector<Edge>& edges);

    /// Find all cycles in the graph
    /// @param [OUT] vertices of every cycle in the graph, stored in ccw/clw order
    void cycle_components(std::vector<std::vector<V>>& components);

    /// Find all detours in the graph
    /// @param [OUT] vertices of every detours in the graph, stored in path order
    void detour_components(std::vector<std::vector<V>>& components);

protected:
    /// Find representative edges of every cycles in the graph
    /// @param [IN]  representative vertices of connecting components
    /// @param [OUT] representative edges of every cycle in the graph
    void _cycle_representatives(const std::vector<V>& cc, std::vector<Edge>& edges);

    /// Find one cycle component in the given connecting component (if any).
    /// @param [IN]  representative vertex of connecting component
    /// @param [OUT] vertices of the cycle, stored in ccw/clw order
    /// @return 1 if any cycle detected, 0 if not
    bool _cycle_component(V rep, std::vector<V>& vertices);

protected:
    // store vertex-edge in form of adjancent table
    std::unordered_map<V, std::unordered_map<V, W>> m_G;
};

#include "MeshGraph.hpp"

#endif // !_MESH_GRAPH_H_
