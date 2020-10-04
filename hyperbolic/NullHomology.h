#pragma once
#ifndef _NULL_HOMOLOGY_H_
#define _NULL_HOMOLOGY_H_

#include <unordered_set>
#include <unordered_map>

#include "HyperbolicMesh.h"

namespace MeshLib
{
class NullHomologyDetector
{
public:
    using M = MeshLib::CHyperbolicMesh;

public:
    NullHomologyDetector() {}

    NullHomologyDetector(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh) { m_pMesh = pMesh; };

    int remove_exact_loops();

protected:
    // spread fire over faces on the surface
    void _spread_fire(M::CFace* face);

    // post-processing: unsharp branch edges (these not forming cycle)
    void _prune_branch();

protected:
    //
    M* m_pMesh = NULL;

    // sharp edges
    std::unordered_set<M::CEdge*> m_sharps;

    // faces visited or not map
    std::unordered_map<M::CFace*, bool> m_visit;
};
}

#endif // !_NULL_HOMOLOGY_H_
