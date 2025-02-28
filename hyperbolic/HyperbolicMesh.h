#pragma once
#ifndef _HYPERBOLIC_MESH_H_
#define _HYPERBOLIC_MESH_H_

#include "Mesh/BaseMesh.h"
#include "Mesh/Edge.h"
#include "Mesh/Face.h"
#include "Mesh/HalfEdge.h"
#include "Mesh/Vertex.h"
#include "Mesh/Boundary.h"
#include "Mesh/Iterators.h"
#include "Parser/parser.h"

#include <vector>

namespace MeshLib
{
class CHyperbolicVertex : public CVertex
{
public:
	CHyperbolicVertex() {}

protected:
};

class CHyperbolicEdge : public CEdge
{
public:
	CHyperbolicEdge() {}

    int& sharp() { return m_sharp; };

    void _from_string();

    void _to_string();

protected:
    // sharp flag
    // Boundary edge has non-0 value
    // Value is set based on edge position in fundamental domain
    int m_sharp = 0;
};

inline void CHyperbolicEdge::_from_string()
{
    CParser parser(m_string);
    for (std::list<CToken*>::iterator titer = parser.tokens().begin(); titer != parser.tokens().end(); titer++)
    {
        CToken* pT = *titer;

        if (pT->m_key == "sharp")
        {
            std::string line = strutil::trim(pT->m_value, "()");
            m_sharp = strutil::parseString<int>(line);
        }
    }
};

inline void MeshLib::CHyperbolicEdge::_to_string()
{
    m_string.clear();

    if (m_sharp)
    {
        m_string.append("sharp=(");
        m_string.append(std::to_string(m_sharp));
        m_string.append(")");
    }
}

class CHyperbolicHalfEdge : public CHalfEdge
{
public:
	CHyperbolicHalfEdge() {}

protected:
};

class CHyperbolicFace : public CFace
{
public:
	CHyperbolicFace() {}

    CPoint& normal() { return m_normal; };

protected:
    CPoint m_normal;
};

template <class T>
struct PtrHash
{
    std::size_t operator()(const T& t) const
    {
        return t->id();
    }
};

template <class T>
struct PtrCompare
{
    bool operator()(const T& a, const T& b) const
    {
        return a->id() < b->id();
    }
};

class CHyperbolicMesh : public CBaseMesh<CHyperbolicVertex, CHyperbolicEdge, CHyperbolicFace, CHyperbolicHalfEdge>
{
protected:
    typedef CHyperbolicVertex   V;
    typedef CHyperbolicEdge     E;
    typedef CHyperbolicFace     F;
    typedef CHyperbolicHalfEdge H;

public:
    typedef V CVertex;
    typedef E CEdge;
    typedef F CFace;
    typedef H CHalfEdge;

    typedef CBoundary<V, E, F, H>                   CBoundary;
    typedef CLoop<V, E, F, H>                       CLoop;

    typedef MeshVertexIterator<V, E, F, H>          MeshVertexIterator;
    typedef MeshEdgeIterator<V, E, F, H>            MeshEdgeIterator;
    typedef MeshFaceIterator<V, E, F, H>            MeshFaceIterator;
    typedef MeshHalfEdgeIterator<V, E, F, H>        MeshHalfEdgeIterator;

    typedef VertexVertexIterator<V, E, F, H>        VertexVertexIterator;
    typedef VertexEdgeIterator<V, E, F, H>          VertexEdgeIterator;
    typedef VertexFaceIterator<V, E, F, H>          VertexFaceIterator;
    typedef VertexInHalfedgeIterator<V, E, F, H>    VertexInHalfedgeIterator;
    typedef VertexOutHalfedgeIterator<V, E, F, H>   VertexOutHalfedgeIterator;

    typedef FaceVertexIterator<V, E, F, H>          FaceVertexIterator;
    typedef FaceEdgeIterator<V, E, F, H>            FaceEdgeIterator;
    typedef FaceHalfedgeIterator<V, E, F, H>        FaceHalfedgeIterator;

public:
    //
    CHyperbolicMesh() {}

    // build structures from other mesh at constructing
    CHyperbolicMesh(const CHyperbolicMesh& mesh)
    {
        //printf("$");
        copy_from(mesh);
    }

    // explicit move contructor for vector resizing
    CHyperbolicMesh(CHyperbolicMesh&& other) noexcept
    {
        //printf("&");
        m_verts = std::move(other.m_verts);
        m_edges = std::move(other.m_edges);
        m_faces = std::move(other.m_faces);
        m_map_vert = std::move(other.m_map_vert);
        m_map_face = std::move(other.m_map_face);
    }

    // clear mesh attributes
    void clear();

    //
    void copy_from(const CHyperbolicMesh& from);

    //
    void copy_to(CHyperbolicMesh& to);

    // resize mesh within [-1, 1] range
    void normalize();

    // compute face and vertex normal
    void computeNormal();

    // re-index vertices make sure vertex id starts from 1
    void reIndexVert();

    // re-index faces
    void reIndexFace();

    // re-index edges
    void reIndexEdge();

protected:
    // edge map

protected:
    // copy structure information
    static void _copy(const CHyperbolicMesh& from, CHyperbolicMesh& to);
};
}

#endif // !_HYPERBOLIC_MESH_H_
