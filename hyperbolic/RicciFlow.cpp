#include "RicciFlow.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

inline double euclidean_cosine_law(double a, double b, double c)
{
	double cs = (a * a + b * b - c * c) / (2 * a * b);
	return std::acos(cs);
}

inline double hyperbolic_cosine_law(double a, double b, double c)
{
	double cs = (std::cosh(a) * std::cosh(b) - std::cosh(c)) / (std::sinh(a) * std::sinh(b));
	return std::acos(cs);
}

inline double hyperbolic_triangle_area(double a, double b, double c)
{
	double ci = std::cosh(a);
	double cj = std::cosh(b);
	double ck = std::cosh(c);
	return std::sqrt(1 - ci * ci - cj * cj - ck * ck + 2 * ci * cj * ck) * 0.5;
}

void MeshLib::HyperbolicRicciFlow::initialize()
{
	_initialize_indices();
	_initialize_factors();
	_initialize_targets();
}

void MeshLib::HyperbolicRicciFlow::_initialize_indices()
{
	int id = 0;

	for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
	{
		M::CVertex* pV = *viter;
		m_indices[pV] = id++;
	}
}

void MeshLib::HyperbolicRicciFlow::_initialize_factors()
{
	for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
	{
		M::CVertex* pV = *viter;
		m_factors[pV] = 0;
	}
}

void MeshLib::HyperbolicRicciFlow::_initialize_targets()
{
	for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
	{
		M::CVertex* pV = *viter;
		m_target_curvatures[pV] = 0;
	}
}

inline double hyperbolic_length_scale(double length, double u0, double u1)
{
	return std::asinh(std::exp((u0 + u1) * 0.5) * std::sinh(length * 0.5)) * 2;
}

void MeshLib::HyperbolicRicciFlow::_compute_lengths()
{
	for (M::MeshEdgeIterator eiter(m_pMesh); !eiter.end(); ++eiter)
	{
		M::CEdge* pE = *eiter;
		M::CVertex* pV0 = m_pMesh->edgeVertex1(pE);
		M::CVertex* pV1 = m_pMesh->edgeVertex2(pE);

		// original euclidean length
		CPoint d = pV1->point() - pV0->point();
		double length = d.norm();

		// scaling factors
		double u0 = m_factors[pV0];
		double u1 = m_factors[pV1];

		// hyperbolic length projected to euclidean space
		m_lengths[pE] = hyperbolic_length_scale(length, u0, u1);

		//printf("len = %f ", m_lengths[pE]);
	}
}

void MeshLib::HyperbolicRicciFlow::_compute_angles()
{
	for (M::MeshFaceIterator fiter(m_pMesh); !fiter.end(); ++fiter)
	{
		M::CFace* pF = *fiter;
		M::CHalfEdge* hes[3];
		double lens[3];

		hes[0] = m_pMesh->faceMostCcwHalfEdge(pF);
		hes[1] = m_pMesh->faceNextCcwHalfEdge(hes[0]);
		hes[2] = m_pMesh->faceNextCcwHalfEdge(hes[1]);

		lens[0] = m_lengths[m_pMesh->halfedgeEdge(hes[0])];
		lens[1] = m_lengths[m_pMesh->halfedgeEdge(hes[1])];
		lens[2] = m_lengths[m_pMesh->halfedgeEdge(hes[2])];

		// hE |-> angle between hE and hE->next
		for (int i = 0; i < 3; ++i)
		{
			double a = lens[i];
			double b = lens[(i + 1) % 3];
			double c = lens[(i + 2) % 3];
			//m_angles[hes[i]] = euclidean_cosine_law(a, b, c);
			m_angles[hes[i]] = hyperbolic_cosine_law(a, b, c);
		}
	}
}

void MeshLib::HyperbolicRicciFlow::_compute_curvatures()
{
	for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
	{
		M::CVertex* pV = *viter;
		double k = pV->boundary() ? PI : 2 * PI;

		for (M::VertexInHalfedgeIterator hiter(m_pMesh, pV); !hiter.end(); ++hiter)
		{
			M::CHalfEdge* pH = *hiter;
			k -= m_angles[pH];
		}

		m_curvatures[pV] = k;
		//printf("curv(%d) = %f ", m_indices[pV], m_curvatures[pV]);
	}
}

double MeshLib::HyperbolicRicciFlow::_compute_error_norm_2()
{
	double error = 0;

	for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
	{
		M::CVertex* pV = *viter;

		double dk = m_target_curvatures[pV] - m_curvatures[pV];
		error += dk * dk;
	}

	return std::sqrt(error);
}

double MeshLib::HyperbolicRicciFlow::_compute_error_norm_inf()
{
	double max_error = -FLT_MAX;

	for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
	{
		M::CVertex* pV = *viter;

		double dk = m_target_curvatures[pV] - m_curvatures[pV];
		dk = std::fabs(dk);

		if (dk > max_error)
		{
			max_error = dk;
		}
	}

	return max_error;
}

void MeshLib::HyperbolicRicciFlow::compute_metrics_gradient(double threshold, double lambda, int maxSteps)
{
	for (int t = 0; t < maxSteps; ++t)
	{
		_compute_lengths();
		_compute_angles();
		_compute_curvatures();

		double error2 = _compute_error_norm_2();
		double errorInf = _compute_error_norm_inf();
		printf("[Ricci Flow] Curvature err(2) = %e, err(inf) = %e\n", error2, errorInf);
		if (errorInf < threshold) break;

		// update comformal factors u
		for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
		{
			M::CVertex* pV = *viter;
			double gradient = m_target_curvatures[pV] - m_curvatures[pV];
			m_factors[pV] += gradient * lambda;
		}
	}
}

void MeshLib::HyperbolicRicciFlow::compute_metrics_newton(double threshold, double lambda, int maxSteps)
{
	int nv = m_pMesh->numVertices();

	for (int t = 0; t < maxSteps; ++t)
	{
		_compute_lengths();
		_compute_angles();
		_compute_curvatures();

		double error2 = _compute_error_norm_2();
		double errorInf = _compute_error_norm_inf();
		printf("[Ricci Flow] Curvature err(2) = %e, err(inf) = %e\n", error2, errorInf);
		if (errorInf < threshold) break;

		// set Hussian matrix
		std::vector<Eigen::Triplet<double>> A_coefficients;
		Eigen::SparseMatrix<double> A(nv, nv);
		A.setZero();

		for (M::MeshFaceIterator fiter(m_pMesh); !fiter.end(); ++fiter)
		{
			M::CFace* pF = *fiter;
			M::CHalfEdge* hs[3];
			M::CVertex* vs[3];
			M::CEdge* es[3];
			double ls[3], chs[3];
			int ids[3];

			hs[0] = m_pMesh->faceMostCcwHalfEdge(pF);
			hs[1] = m_pMesh->faceNextCcwHalfEdge(hs[0]);
			hs[2] = m_pMesh->faceNextCcwHalfEdge(hs[1]);
			vs[0] = m_pMesh->halfedgeTarget(hs[0]);
			vs[1] = m_pMesh->halfedgeTarget(hs[1]);
			vs[2] = m_pMesh->halfedgeTarget(hs[2]);
			es[0] = m_pMesh->halfedgeEdge(hs[2]); // reorder to convention
			es[1] = m_pMesh->halfedgeEdge(hs[0]);
			es[2] = m_pMesh->halfedgeEdge(hs[1]);
			ls[0] = m_lengths[es[0]];
			ls[1] = m_lengths[es[1]];
			ls[2] = m_lengths[es[2]];
			chs[0] = std::cosh(ls[0]);
			chs[1] = std::cosh(ls[1]);
			chs[2] = std::cosh(ls[2]);
			ids[0] = m_indices[vs[0]];
			ids[1] = m_indices[vs[1]];
			ids[2] = m_indices[vs[2]];

			double area = hyperbolic_triangle_area(ls[0], ls[1], ls[2]);

			for (int i = 0; i < 3; ++i)
			{
				int j = (i + 1) % 3;
				int k = (i + 2) % 3;

				// update non-diagonal elements
				double coef_nd = (chs[j] + chs[k] - chs[i] - 1) / (chs[i] + 1) / area;
				A_coefficients.push_back(Eigen::Triplet<double>(ids[j], ids[k], coef_nd));
				A_coefficients.push_back(Eigen::Triplet<double>(ids[k], ids[j], coef_nd));

				// update diagonal elements
				double coef_dg = 0;
				coef_dg += -(chs[i] * chs[j] - chs[k]) / (chs[j] + 1);
				coef_dg += -(chs[i] * chs[k] - chs[j]) / (chs[k] + 1);
				coef_dg /= area;
				A_coefficients.push_back(Eigen::Triplet<double>(ids[i], ids[i], coef_dg));
			}
		}

		A.setFromTriplets(A_coefficients.begin(), A_coefficients.end());

		// set b vector in Ax = b where b := \Delta E
		Eigen::VectorXd b(nv);
		b.setZero();

		for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
		{
			M::CVertex* pV = *viter;
			int id = m_indices[pV];
			b(id) = m_target_curvatures[pV] - m_curvatures[pV];
		}

		// solve x where x := \delta u
		Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
		solver.compute(A);

		if (solver.info() != Eigen::Success)
		{
			std::cerr << "Waring: Eigen decomposition failed" << std::endl;
		}

		Eigen::VectorXd x = solver.solve(b);
		if (solver.info() != Eigen::Success)
		{
			std::cerr << "Waring: Eigen decomposition failed" << std::endl;
		}

		// update conformal factors u
		for (M::MeshVertexIterator viter(m_pMesh); !viter.end(); ++viter)
		{
			M::CVertex* pV = *viter;
			int id = m_indices[pV];
			m_factors[pV] -= x(id) * lambda;
			//printf("u = %f ", m_factors[pV]);
		}
		//printf("[Ricci Flow] Curvature error(2) = %f, error(inf) = %f\n", error2, errorInf);
	}
}
