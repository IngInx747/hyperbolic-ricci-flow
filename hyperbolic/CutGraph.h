#pragma once
#ifndef _CUT_GRAPH_H_
#define _CUT_GRAPH_H_

#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "HyperbolicMesh.h"

#if 1
namespace MeshLib
{
class CutGraph
{
protected:
    using M = CHyperbolicMesh;

public:
    CutGraph() {}

    CutGraph(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh) { m_pMesh = pMesh; }

    void generate(M::CVertex* base = NULL);

    std::unordered_map<M::CEdge*, float>& lengths() { return m_lengths; }

    M::CVertex* base_point() { return m_base; }

protected:
    // prune branches in the sharp-edge graph
    void _prune_branch();

    // construct shortest-path tree rooted at given vertex
    void _shortest_path_tree(M::CVertex* pVs);

    // construct maximum-spanning tree where w(e*) = |\sigma(e)|
    void _maximum_spanning_tree();

protected:
    // reference mesh
    M* m_pMesh = NULL;

    // sharp edges
    std::unordered_map<M::CEdge*, int> m_sharps;

    // edge lengths
    std::unordered_map<M::CEdge*, float> m_lengths;

    // distance from vertex to root
    std::unordered_map<M::CVertex*, float> m_dists;

    // base point
    M::CVertex* m_base = NULL;
};
}

#else
namespace MeshLib
{
class CutGraph
{
public:
    using M = CHyperbolicMesh;

public:
    CutGraph() {}

    CutGraph(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh) { m_pMesh = pMesh; }

    // Generate cut graph
    // The edges on cut graph are marked sharp
    void generate();

    void generate_experimental();

protected:
    //
    void _spread_fire(M::CVertex* vertex);

    // Traverse faces on the mesh
    // Mark uncrossed edges sharp
    void _spread_fire(M::CFace* face);

    // Prune branches in sharp edge graph
    void _prune_branch();

    // Smoothen sharp edge graph
    int _shrink_triangles();

protected:
    //
    M* m_pMesh = NULL;

    // sharp edges
    std::unordered_set<M::CEdge*> m_sharps;
};
} // namespace MeshLib
#endif

#endif // !_CUT_GRAPH_H_