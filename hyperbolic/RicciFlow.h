#pragma once
#ifndef _RICCI_FLOW_H_
#define _RICCI_FLOW_H_

#include "HyperbolicMesh.h"

#include <unordered_map>

namespace MeshLib
{
class HyperbolicRicciFlow
{
public:
    using M = CHyperbolicMesh;

public:
    HyperbolicRicciFlow() {}

    HyperbolicRicciFlow(M* pMesh) { set_mesh(pMesh); }

    void set_mesh(M* pMesh) { m_pMesh = pMesh; }

    void initialize();

    void compute_metrics_gradient(double threshold, double lambda, int maxSteps);

    void compute_metrics_newton(double threshold, double lambda, int maxSteps);

    std::unordered_map<M::CVertex*, double>& factors() { return m_factors; }

protected:
    // index vertices
    void _initialize_indices();

    // set all factors to 0
    void _initialize_factors();

    // set target curvature to 0
    void _initialize_targets();

    // compute edge lengths after vertex scaling
    void _compute_lengths();

    // compute corner angles after vertex scaling
    void _compute_angles();

    // compute curvatures on vertices
    void _compute_curvatures();

    // compute norm-2 error
    double _compute_error_norm_2();

    // compute norm-inf error
    double _compute_error_norm_inf();

protected:
    //
    M* m_pMesh = NULL;

    // hyperbolic edge lengths
    std::unordered_map<M::CEdge*, double> m_lengths;

    // corner angles
    std::unordered_map<M::CHalfEdge*, double> m_angles;

    // vertex indices
    std::unordered_map<M::CVertex*, int> m_indices;

    // comformal factor of vertex
    std::unordered_map<M::CVertex*, double> m_factors;

    // discrete gaussian curvature of vertex
    std::unordered_map<M::CVertex*, double> m_curvatures;

    // target curvature
    std::unordered_map<M::CVertex*, double> m_target_curvatures;
};
}

#endif // !_RICCI_FLOW_H_
