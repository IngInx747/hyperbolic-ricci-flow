#include "MeshEmbed.h"

#include <queue>

using UV = MeshLib::CPoint2;

inline double hyperbolic_length_scale(double length, double u0, double u1)
{
	return std::asinh(std::exp((u0 + u1) * 0.5) * std::sinh(length * 0.5)) * 2;
}

inline double distance_h2e(double hd, UV v0, UV v1)
{
	double n0 = 1 - v0.norm();
	double n1 = 1 - v1.norm();
	return std::sqrt(0.5 * n0 * n1 * (std::cosh(hd) - 1));
}

inline double distance_e2h(double ed, UV v0, UV v1)
{
	double n0 = 1 - v0.norm();
	double n1 = 1 - v1.norm();
	return std::acosh(1 + 2 * ed * ed / (n0 * n1));
}

/// Euclidean cycles intersection
/// @param  [IN]  Coordinate of first point
/// @param  [IN]  Coordinate of second point
/// @param  [IN]  Euclidean radius of first cycle
/// @param  [IN]  Euclidean radius of second cycle
/// @param  [OUT] Coordinate of the intersection point
/// @param  [IN]  Use counter-clockwise order or not
/// @return 1 if embedment succeeded, 0 otherwise
inline bool cycle_intersect_eu(UV v0, UV v1, double r0, double r1, UV& v2, bool ccw = true)
{
	double r = (v1 - v0).norm();
	//printf("r0=%lf, r1=%lf, r=%lf\n", r0, r1, r);
	//assert(r < r0 + r1);
	//assert(r > fabs(r1 - r0));
	//bool result = true;

	//if (r > r0 + r1 || r < fabs(r1 - r0)) result = false;
	if (r > r0 + r1 || r < fabs(r1 - r0)) return false;

	double a = (r0 * r0 - r1 * r1 + r * r) / (2 * r); // distance v0 to pedal of v2 to v0v1
	double h = std::sqrt(r0 * r0 - a * a); // height v2 to v0v1
	double q = a / r;

	UV d = (v1 - v0) / r;
	UV t(-d[1], d[0]);
	if (!ccw) t = t * -1;

	//v2 = v0 + d * a + t * h;
	v2 = v0 * (1 - q) + v1 * q + t * h;

	//return result;
	return true;
}

/// Hyperbolic cycles intersection
/// @param  [IN]  Coordinate of first point on Poincare disk D
/// @param  [IN]  Coordinate of second point
/// @param  [IN]  Hyperbolic radius of first cycle
/// @param  [IN]  Hyperbolic radius of second cycle
/// @param  [OUT] Coordinate of the intersection point
/// @param  [IN]  Use counter-clockwise order or not
/// @return 1 if embedment succeeded, 0 otherwise
inline bool cycle_intersect_hy(UV v0, UV v1, double hr0, double hr1, UV& v2, bool ccw = true)
{
	double k0 = (std::cosh(hr0) - 1) * (1 - v0.norm2()) * 0.5;
	double r0 = std::sqrt(k0 * (k0 + 1 - v0.norm2())) / (k0 + 1);
	UV c0 = v0 / (k0 + 1);

	double k1 = (std::cosh(hr1) - 1) * (1 - v1.norm2()) * 0.5;
	double r1 = std::sqrt(k1 * (k1 + 1 - v1.norm2())) / (k1 + 1);
	UV c1 = v1 / (k1 + 1);

	//printf("nv0=%lf, nv1=%lf\n", v0.norm2(), v1.norm2());
	//printf("nv0=%lf, hr0=%lf, nv1=%lf, hr1=%lf, ", v0.norm2(), hr0, v1.norm2(), hr1);
	//printf("hr0=%lf, hr1=%lf, ", hr0, hr1);
	//printf("k0=%e, k1=%e\n", k0, k1);

	return cycle_intersect_eu(c0, c1, r0, r1, v2, ccw);
}

void MeshLib::HyperbolicEmbed::set_metrics(std::unordered_map<M::CVertex*, double>& factors)
{
	for (M::MeshEdgeIterator eiter(m_pMesh); !eiter.end(); ++eiter)
	{
		M::CEdge* pE = *eiter;
		M::CVertex* pV0 = m_pMesh->edgeVertex1(pE);
		M::CVertex* pV1 = m_pMesh->edgeVertex2(pE);
		double length = m_pMesh->edgeLength(pE);
		double u0 = factors[pV0];
		double u1 = factors[pV1];
		length = hyperbolic_length_scale(length, u0, u1);
		m_lengths[pE] = length;
	}

	//for (M::MeshFaceIterator fiter(m_pMesh); !fiter.end(); ++fiter)
	//{
	//	M::CFace* pF = *fiter;
	//	M::CHalfEdge* hs[3];
	//	M::CEdge* es[3];
	//	double ls[3];
	//
	//	hs[0] = m_pMesh->faceMostCcwHalfEdge(pF);
	//	hs[1] = m_pMesh->faceNextCcwHalfEdge(hs[0]);
	//	hs[2] = m_pMesh->faceNextCcwHalfEdge(hs[1]);
	//	es[0] = m_pMesh->halfedgeEdge(hs[2]); // reorder to convention
	//	es[1] = m_pMesh->halfedgeEdge(hs[0]);
	//	es[2] = m_pMesh->halfedgeEdge(hs[1]);
	//	ls[0] = m_lengths[es[0]];
	//	ls[1] = m_lengths[es[1]];
	//	ls[2] = m_lengths[es[2]];
	//
	//	//printf("r0=%lf, r1=%lf, r2=%lf\n", ls[0], ls[1], ls[2]);
	//
	//	for (int i = 0; i < 3; ++i)
	//	{
	//		double a = ls[i];
	//		double b = ls[(i + 1) % 3];
	//		double c = ls[(i + 2) % 3];
	//		assert(a < b + c);
	//		assert(a > fabs(b - c));
	//	}
	//}
}

void MeshLib::HyperbolicEmbed::embed()
{
	for (M::CVertex* pV : m_pMesh->vertices())
		m_vertVisit[pV] = false;

	for (M::CFace* pF : m_pMesh->faces())
		m_faceVisit[pF] = false;

	M::CFace* pFs = *m_pMesh->faces().rbegin();
	std::queue<M::CFace*> q;

	_embed_first_face(pFs);
	q.push(pFs);

	while (!q.empty())
	{
		M::CFace* pF = q.front();
		q.pop();

		for (M::FaceHalfedgeIterator hiter(pF); !hiter.end(); ++hiter)
		{
			M::CHalfEdge* pH = *hiter;
			M::CHalfEdge* pSymH = m_pMesh->halfedgeSym(pH);
			if (!pSymH) continue;
			M::CFace* pNbrF = m_pMesh->halfedgeFace(pSymH);
			if (m_faceVisit[pNbrF]) continue;
			_embed_face(pNbrF, pSymH);
			q.push(pNbrF);
		}
	}

	if (m_fail_flag)
	{
		printf("[HyperbolicEmbed] Mesh embedment failed. Please check triangulation.\n");
		m_fail_flag = 0;
	}
}

void MeshLib::HyperbolicEmbed::_embed_first_face(M::CFace* face)
{
	M::CHalfEdge* hs[3];
	M::CVertex* vs[3];
	M::CEdge* es[3];

	hs[0] = m_pMesh->faceMostCcwHalfEdge(face);
	hs[1] = m_pMesh->faceNextCcwHalfEdge(hs[0]);
	hs[2] = m_pMesh->faceNextCcwHalfEdge(hs[1]);

	vs[0] = m_pMesh->halfedgeTarget(hs[0]);
	vs[1] = m_pMesh->halfedgeTarget(hs[1]);
	vs[2] = m_pMesh->halfedgeTarget(hs[2]);

	es[0] = m_pMesh->halfedgeEdge(hs[2]); // reorder to convention
	es[1] = m_pMesh->halfedgeEdge(hs[0]);
	es[2] = m_pMesh->halfedgeEdge(hs[1]);

	double hd0 = m_lengths[es[0]];
	double hd1 = m_lengths[es[1]];
	double hd2 = m_lengths[es[2]];

	// embed first edge on real axis
	double ed2 = std::tanh(hd2 * 0.5); // hy -> eu
	m_uv[vs[0]] = UV(0, 0);
	m_uv[vs[1]] = UV(ed2, 0);

	// embed third vertex on the disk
	UV p0 = m_uv[vs[0]];
	UV p1 = m_uv[vs[1]];
	UV p2;
	double hr0 = hd1;
	double hr1 = hd0;

	//printf("r=%lf, r0=%lf, r1=%lf\n", m_lengths[es[2]], r0, r1);
	bool result = cycle_intersect_hy(p0, p1, hr0, hr1, p2);
	if (!result) m_fail_flag = 1;
	m_uv[vs[2]] = p2;

	m_faceVisit[face] = true;
	m_vertVisit[vs[0]] = true;
	m_vertVisit[vs[1]] = true;
	m_vertVisit[vs[2]] = true;
}

void MeshLib::HyperbolicEmbed::_embed_face(M::CFace* face, M::CHalfEdge* halfedge)
{
	m_faceVisit[face] = true;

	M::CHalfEdge* pH0 = m_pMesh->halfedgePrev(halfedge);
	M::CHalfEdge* pH1 = halfedge;
	M::CHalfEdge* pH2 = m_pMesh->halfedgeNext(halfedge);

	M::CVertex* pV0 = m_pMesh->halfedgeTarget(pH0);
	M::CVertex* pV1 = m_pMesh->halfedgeTarget(pH1);
	M::CVertex* pV2 = m_pMesh->halfedgeTarget(pH2);

	M::CEdge* pE0 = m_pMesh->halfedgeEdge(pH2);
	M::CEdge* pE1 = m_pMesh->halfedgeEdge(pH0);
	M::CEdge* pE2 = m_pMesh->halfedgeEdge(pH1);

	assert(m_vertVisit[pV0]);
	assert(m_vertVisit[pV1]);

	if (m_vertVisit[pV2]) return;

	UV p0 = m_uv[pV0];
	UV p1 = m_uv[pV1];
	UV p2;
	double hr0 = m_lengths[pE1];
	double hr1 = m_lengths[pE0];
	//printf("r0=%lf, r1=%lf, ", r0, r1);
	//printf("R=%lf R'=%lf\n", m_lengths[m_pMesh->vertexEdge(pV0, pV1)], (p0-p1).norm());
	//printf("r=%lf, r0=%lf, r1=%lf\n", m_lengths[m_pMesh->vertexEdge(pV0, pV1)], r0, r1);
	//printf("v(%d), v(%d), v(%d), \t", pV0->id(), pV1->id(), pV2->id());
	//if (pV0->boundary()) printf("0 ");
	//if (pV1->boundary()) printf("1 ");
	//if (pV2->boundary()) printf("2 ");
	//printf("v(%lf, %lf), v(%lf, %lf), \t", p0[0], p0[1], p1[0], p1[1]);
	bool result = cycle_intersect_hy(p0, p1, hr0, hr1, p2);
	if (!result) m_fail_flag = 1;
	m_uv[pV2] = p2;

	m_vertVisit[pV2] = true;
}
