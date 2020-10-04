#pragma once
#ifndef _CUT_GRAPH_H_
#define _CUT_GRAPH_H_

#include <vector>
#include <unordered_set>
#include <unordered_map>

#include "HyperbolicMesh.h"

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

#endif // !_CUT_GRAPH_H_