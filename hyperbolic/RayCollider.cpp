#include "RayCollider.h"

using M = MeshLib::CHyperbolicMesh;

constexpr double kEpsilon = 1e-3;

bool MeshLib::RayCollider::collide(M::CFace* face, const CPoint& org, const CPoint& dir, double& dist) const
{
	M::CHalfEdge* pH = m_pMesh->faceMostCcwHalfEdge(face);
	M::CVertex* pV0 = m_pMesh->halfedgeTarget(pH);
	pH = m_pMesh->faceNextCcwHalfEdge(pH);
	M::CVertex* pV1 = m_pMesh->halfedgeTarget(pH);
	pH = m_pMesh->faceNextCcwHalfEdge(pH);
	M::CVertex* pV2 = m_pMesh->halfedgeTarget(pH);
	CPoint v0 = pV0->point();
	CPoint v1 = pV1->point();
	CPoint v2 = pV2->point();

	CPoint v01 = v1 - v0;
	CPoint v02 = v2 - v0;
	CPoint pvc = dir ^ v02; // T
	double det = v01 * pvc; // ((P1, V02, V01))

#ifdef ENABLE_CULLING
	if (det < 0.) return false;
#else
	if (fabs(det) < kEpsilon) return false;
#endif

	double inv = 1 / det;
	CPoint tvc = org - v0; // P0 - V0
	double u = (tvc * pvc) * inv; // Eq.3
	if (u < 0.0 || u > 1.0) return false;

	CPoint qvc = tvc ^ v01; // S
	double v = (dir * qvc) * inv; // Eq.4
	if (v < 0.0 || u + v > 1.0) return false;

	// distance from ray.origin to hit point
	double t = (v02 * qvc) * inv; // Eq.5

	// update hit distance
	if (t > 0.0 && dist > t)
	{
		dist = t;
		return true; // ray hit primitive in distance
	}
	else return false; // ray hit primitive out of distance
}

M::CFace* MeshLib::RayCollider::collide(const CPoint& org, const CPoint& dir, double& dist) const
{
	M::CFace* ret = NULL;

	// TODO: acceleration structure

	for (M::CFace* pF : m_pMesh->faces())
	{
		if (collide(pF, org, dir, dist))
		{
			ret = pF;
		}
	}

	return ret;
}
