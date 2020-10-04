#pragma once
#ifndef _HYPERBOLIC_MAP_H_
#define _HYPERBOLIC_MAP_H_

#include "HyperbolicMesh.h"
#include "RicciFlow.h"
#include "MobiusTransform.h"
#include "Tree.h"

#include <unordered_map>

namespace MeshLib
{
class HyperbolicMap
{
public:
    using M = CHyperbolicMesh;

public:
    HyperbolicMap() {}

    HyperbolicMap(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh);

    void ricci_flow(double threshold, double lambda, int maxSteps);

    void mark_fundamental_domain();

    void slice_fundamental_domain();

    void isometrical_embed();

    void sort_domain_boundaries();

    void compute_fuchsian_group();

    void tesselate_disk(int level = 2);

    void tesselate_disk_single_step(int level = 2);

    void clear_tesselation();

    void compute_geodesic_cycles();

    M& open_mesh() { return m_openMesh; }
    //M& open_mesh() { return *m_pMesh; }

    M& domain_mesh() { return m_domainMesh; }
    //M& domain_mesh() { return *m_pMesh; }
    //M& domain_mesh() { return m_openMesh; }

    std::vector<M>& tesselation_meshes() { return m_tesselationMeshes; }

    std::vector<std::pair<CPoint2, double>>& geodesic_circles() { return m_circles; }

protected:
    // reference mesh
    M* m_pMesh = NULL;

    // sliced mesh
    M m_openMesh;

    // primary domain
    M m_domainMesh;

    // sub domains
    std::vector<M> m_tesselationMeshes;

    // ricci flow
    HyperbolicRicciFlow m_flow;

    // edges of domain that share the same edge on the original mesh
    std::unordered_map<int, int> m_edgeIndexMap;

    // vertex of domain that on the same point on the original mesh
    std::unordered_map<int, int> m_vertIndexMap;

    // domain segments: k |-> [s_k(0), s_k(1)]
    std::unordered_map<int, std::pair<int, int>> m_segments;

    // the Mobius transforms for tesselating whole Poincare disk
    std::unordered_map<int, MobiusTransform<double>> m_fuchsianGroupGenerators;

    // all actions from Fuchsian group
    Tree<int> m_tesselatingActions;

    // for single step disk tesselation
    Tree<int>::PathIterator m_tesselatingIterator;

    // geodesic circles (center, radius)
    std::vector<std::pair<CPoint2, double>> m_circles;
};
};

#endif // !_HYPERBOLIC_MAP_H_
