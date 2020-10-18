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

    // practice hyperbolic Ricci flow on reference mesh
    void ricci_flow(double threshold, double lambda, int maxSteps);

    //
    void greedy_homotopy_generators();

    // generate boundaries forming a fundamental domain of the mesh
    void mark_fundamental_domain();

    // slice the mesh into a fundamental domain
    void slice_fundamental_domain();

    // perform hyperbolically isometric embedding of the fundamental domain
    void isometric_embed();

    // record information of boundary segments (like s(0), s(1), id(s), etc.)
    void sort_domain_boundaries();

    // generate Fuchsian transform generators (based on segments info)
    void compute_fuchsian_group();

    // tessellate Poincare disk (based on the domain and Fuchsian group)
    void tessellate_disk(int level = 2);

    // tessellate Poincare disk, one domain at a time
    void tessellate_disk_single_step(int level = 2);

    // clear tessellating results
    void clear_tessellation();

    // generate hyperbolic lines of each boundary segments
    void compute_geodesic_cycles();

    M& original_mesh() { return *m_pMesh; }

    M& open_mesh() { return m_openMesh; }
    //M& open_mesh() { return *m_pMesh; }

    M& domain_mesh() { return m_domainMesh; }
    //M& domain_mesh() { return *m_pMesh; }
    //M& domain_mesh() { return m_openMesh; }

    //std::vector<M>& tessellation_meshes() { return m_tessellationMeshes; }
    std::vector<std::unordered_map<M::CVertex*, CPoint2>>& tessellation_meshes() { return m_tessellationMeshes; }

    int tessellation_index(int mid) { return m_tessellationIndices[mid]; }

    std::vector<std::pair<CPoint2, double>>& geodesic_circles() { return m_circles; }

protected:
    // reference mesh
    M* m_pMesh = NULL;

    // sliced mesh
    M m_openMesh;

    // primary domain
    M m_domainMesh;

    // sub domains
    //std::vector<M> m_tessellationMeshes;
    std::vector<std::unordered_map<M::CVertex*, CPoint2>> m_tessellationMeshes;

    // ricci flow
    HyperbolicRicciFlow m_flow;

    // edges of domain that share the same edge on the original mesh
    std::unordered_map<int, int> m_edgeIndexMap;

    // vertex of domain that on the same point on the original mesh
    std::unordered_map<int, int> m_vertIndexMap;

    // vertex id of base point
    //int m_base_id;

    // domain segments, k -> [s_k(0), s_k(1)]
    std::unordered_map<int, std::pair<int, int>> m_segments;

    // Mobius transforms, k -> T_k: [s_k(0), s_k(1)] |-> [s_{-k}(0), s_{-k}(1)]
    std::unordered_map<int, MobiusTransform<double>> m_fuchsianGroupGenerators;

    // all actions from Fuchsian group
    Tree<int> m_tessellatingActions;

    // for single step disk tessellation
    Tree<int>::PathIterator m_tessellatingIterator;

    // geodesic lines in form of circle (center, radius)
    std::vector<std::pair<CPoint2, double>> m_circles;

    // color of tessellating domains, id -> k of last Mobius transform T_{-k}
    std::unordered_map<int, int> m_tessellationIndices;
};
};

#endif // !_HYPERBOLIC_MAP_H_
