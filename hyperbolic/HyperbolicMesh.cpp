#include "HyperbolicMesh.h"

#include <unordered_map>

void MeshLib::CHyperbolicMesh::clear()
{
	// delete edges
	for (auto eiter = m_edges.begin(); eiter != m_edges.end(); eiter++)
	{
		CEdge* pE = *eiter;
		delete pE;
	}
	m_edges.clear();
	//m_map_edge.clear();

	// delete faces
	for (auto fiter = m_faces.begin(); fiter != m_faces.end(); fiter++)
	{
		F* pF = *fiter;
		std::vector<H*> hs;

		for (FaceHalfedgeIterator hiter(pF); !hiter.end(); ++hiter)
		{
			H* pH = *hiter;
			hs.push_back(pH);
		}

		for (H* h : hs)
			delete h;

		delete pF;
	}
	m_faces.clear();
	m_map_face.clear();

	// delete vertices
	for (auto viter = m_verts.begin(); viter != m_verts.end(); viter++)
	{
		CVertex* pV = *viter;
		delete pV;
	}
	m_verts.clear();
	m_map_vert.clear();
}

void MeshLib::CHyperbolicMesh::copy_from(const CHyperbolicMesh& from)
{
	_copy(from, *this);
}

void MeshLib::CHyperbolicMesh::copy_to(CHyperbolicMesh& to)
{
	_copy(*this, to);
}

void MeshLib::CHyperbolicMesh::normalize()
{
	CPoint s(0, 0, 0);
	for (MeshVertexIterator viter(this); !viter.end(); ++viter)
	{
		V* v = *viter;
		s = s + v->point();
	}
	s = s / numVertices();

	for (MeshVertexIterator viter(this); !viter.end(); ++viter)
	{
		V* v = *viter;
		CPoint p = v->point();
		p = p - s;
		v->point() = p;
	}

	double d = 0;
	for (MeshVertexIterator viter(this); !viter.end(); ++viter)
	{
		V* v = *viter;
		CPoint p = v->point();
		for (int k = 0; k < 3; k++)
		{
			d = (d > fabs(p[k])) ? d : fabs(p[k]);
		}
	}

	for (MeshVertexIterator viter(this); !viter.end(); ++viter)
	{
		V* v = *viter;
		CPoint p = v->point();
		p = p / d;
		v->point() = p;
	}
}

void MeshLib::CHyperbolicMesh::computeNormal()
{
	for (MeshVertexIterator viter(this); !viter.end(); ++viter)
	{
		V* v = *viter;
		CPoint n(0, 0, 0);

		for (VertexFaceIterator vfiter(v); !vfiter.end(); ++vfiter)
		{
			F* pF = *vfiter;

			CPoint p[3];
			H* he = faceHalfedge(pF);
			for (int k = 0; k < 3; k++)
			{
				p[k] = he->target()->point();
				he = halfedgeNext(he);
			}

			CPoint fn = (p[1] - p[0]) ^ (p[2] - p[0]);
			pF->normal() = fn / fn.norm();
			n += fn;
		}

		n = n / n.norm();
		v->normal() = n;
	}
}

void MeshLib::CHyperbolicMesh::reIndexVert()
{
	m_map_vert.clear();
	int id = 1;

	for (V* pV : vertices())
	{
	    pV->id() = id;
		m_map_vert[id] = pV;
		++id;
	}
}

void MeshLib::CHyperbolicMesh::reIndexFace()
{
	m_map_face.clear();
	int id = 1;

	for (F* pF : faces())
	{
		pF->id() = id;
		m_map_face[id] = pF;
		++id;
	}
}

void MeshLib::CHyperbolicMesh::reIndexEdge()
{
	//m_map_edge.clear();
	int id = 1;

	for (E* pE : edges())
	{
		pE->id() = id;
		//m_map_edge[id] = pE;
		++id;
	}
}

void MeshLib::CHyperbolicMesh::_copy(const CHyperbolicMesh& from, CHyperbolicMesh& to)
{
	if (&from == &to) return;

	std::unordered_map<V*, V*> verts;
	std::unordered_map<F*, F*> faces;
	std::unordered_map<E*, E*> edges;

	to.clear();

	// copy vertices
	for (V* vf : from.m_verts)
	{
		V* vt = to.createVertex(vf->id());
		verts[vf] = vt;
		//verts[vt] = vf;

		// copy attributes
		vt->string() = vf->string();
		vt->normal() = vf->normal();
		vt->point() = vf->point();
		vt->uv() = vf->uv();
		vt->boundary() = vf->boundary();
	}

	// copy faces
	for (F* ff : from.m_faces)
	{
		std::vector<V*> vts;

		for (FaceVertexIterator viter(ff); !viter.end(); ++viter)
		{
			V* vf = *viter;
			V* vt = verts[vf];
			vts.push_back(vt);
		}

		F* ft = to.createFace(vts, ff->id()); // handle halfedge and edge
		faces[ff] = ft;
		//faces[ft] = ff;

		// rerange halfedge as the same order in original face
		H* hf = (H*)ff->halfedge();
		H* ht = (H*)ft->halfedge();
		int id = hf->target()->id(); // vertex id

		while (ht->target()->id() != id)
		{
			ht = (H*)ht->he_next();
		}
		ft->halfedge() = ht;

		// copy attributes
		ft->string() = ff->string();
		ft->normal() = ff->normal();
	}

	// copy edges
	for (F* ff : from.m_faces)
	{
		F* ft = faces[ff];

		FaceEdgeIterator eiter_f(ff);
		FaceEdgeIterator eiter_t(ft);

		for (; !eiter_f.end(); ++eiter_f, ++eiter_t)
		{
			E* ef = *eiter_f;
			E* et = *eiter_t;
			edges[ef] = et;
			//edges[et] = ef;
		}
	}

	for (auto& p : edges)
	{
		E* ef = p.first;
		E* et = p.second;

		// copy attributes
		et->string() = ef->string();
		et->sharp() = ef->sharp();
	}
}
