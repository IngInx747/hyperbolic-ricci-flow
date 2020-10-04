#pragma once
#ifndef _MESH_EDITOR_H_
#define _MESH_EDITOR_H_

#include "HyperbolicMesh.h"

#include <unordered_map>

namespace MeshLib
{
class MeshSlicer
{
public:
    using M = CHyperbolicMesh;

public:
    struct VertexDecorator
    {
        virtual void operator() (M::CVertex* pV0, M::CVertex* pV1) {}
    };

    struct EdgeDecorator
    {
        virtual void operator() (M::CEdge* pE0, M::CEdge* pE1) {}
    };

public:
    MeshSlicer() {}

    MeshSlicer(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh) { m_pMesh = pMesh; }

    void set_edge_decorator(EdgeDecorator* ed) { m_pEdgeDecorator = ed; }

    void set_vertex_decorator(VertexDecorator* vd) { m_pVertexDecorator = vd; }

    // Slice mesh along sharp edges
    void slice_edges();

protected:
    // Step 1: Divide edge into two edges linking the same vertices
    // The functionality is not provided by original createEdge(...)
    void _slice_edge(M::CEdge* pEdge);

    // Step 2: Divide vertex into multiple vertices of each side
    // The functionality is not provided by original createVertex(...)
    void _slice_vertex(M::CVertex* pVertex);

protected:
    // the mesh to cut
    M* m_pMesh = NULL;

    // vertex - most ccw incoming halfedge
    std::unordered_map<M::CVertex*, std::vector<M::CHalfEdge*>> m_map;

    // register a behavior if you want to do anything with newly generated edge
    EdgeDecorator* m_pEdgeDecorator = NULL;

    // register a behavior if you want to do anything with newly generated vertex
    VertexDecorator* m_pVertexDecorator = NULL;
};
}
#endif // !_MESH_EDITOR_H_
