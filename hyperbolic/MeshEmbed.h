#pragma once
#ifndef _MESH_EMBED_H_
#define _MESH_EMBED_H_

#include "HyperbolicMesh.h"

#include <unordered_map>

namespace MeshLib
{
class HyperbolicEmbed
{
public:
    using M = CHyperbolicMesh;

public:
    HyperbolicEmbed() {}

    HyperbolicEmbed(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh) { m_pMesh = pMesh; }

    void set_metrics(std::unordered_map<M::CVertex*, double>& factors);

    void embed();

    std::unordered_map<M::CVertex*, CPoint2>& uv() { return m_uv; }

protected:
    //
    void _embed_first_face(M::CFace* face);

    //
    void _embed_face(M::CFace* face, M::CHalfEdge* halfedge);

protected:
    //
    M* m_pMesh = NULL;

    // uv coordinates on disk
    std::unordered_map<M::CVertex*, CPoint2> m_uv;

    // hyperbolic edge lengths after scaling
    std::unordered_map<M::CEdge*, double> m_lengths;

    //
    std::unordered_map<M::CFace*, bool> m_faceVisit;

    //
    std::unordered_map<M::CVertex*, bool> m_vertVisit;

    //
    bool m_fail_flag = 0;
};
}

#endif // !_MESH_EMBED_H_
