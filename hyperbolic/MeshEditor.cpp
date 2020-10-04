#include "MeshEditor.h"

#include <unordered_set>
#include <unordered_map>

// ----------------------------------------------------------------------------
//
// Mesh Copier
//
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//
// Mesh Cutter
//
// ----------------------------------------------------------------------------
void MeshLib::MeshSlicer::slice_edges()
{
	std::unordered_set<M::CEdge*> edges;
	std::unordered_set<M::CVertex*> vertices;

	for (M::CEdge* pE : m_pMesh->edges())
	{
		if (!pE->sharp()) continue;
		edges.insert(pE);
		vertices.insert(m_pMesh->edgeVertex1(pE));
		vertices.insert(m_pMesh->edgeVertex2(pE));
	}

	for (M::CEdge* pE : edges)
	{
		_slice_edge(pE);
	}

	for (M::CVertex* pV : vertices)
	{
		_slice_vertex(pV);
	}
}

void MeshLib::MeshSlicer::_slice_edge(M::CEdge* pE0)
{
	if (pE0->boundary()) return;

	// V0 >-- H1 --> V1
	// V0 <-- H0 --< V1
	M::CHalfEdge* pH0 = (M::CHalfEdge*)pE0->halfedge(0);
	M::CHalfEdge* pH1 = (M::CHalfEdge*)pE0->halfedge(1);
	M::CVertex* pV0 = (M::CVertex*)pH0->target();
	M::CVertex* pV1 = (M::CVertex*)pH1->target();

	//if (pV0->id() > pV1->id())
	//{
	//	std::swap(pV0, pV1);
	//	std::swap(pH0, pH1);
	//}

	// set vertex as boundary one (IMPORTANT)
	pV0->boundary() = true;
	pV1->boundary() = true;

	// create a new edge
	M::CEdge* pE1 = new M::CEdge;
	//M::CEdge* pE1 = m_pMesh->createEdge(pV0,pV1);
	pE1->id() = m_pMesh->numEdges();
	m_pMesh->edges().push_back(pE1);

	// move one halfedge to the new edge
	pE1->halfedge(0) = pH1;
	pE0->halfedge(1) = NULL;
	pH0->edge() = pE0;
	pH1->edge() = pE1;

	// some methods utilize this attribute (functions like vertexEdge(id, id))
	//pV0->edges().push_back(e);

	// record vertex-edge information
	m_map[pV0].push_back(pH0);
	m_map[pV1].push_back(pH1);

	// Hyperbolic mesh specification
	//pE1->sharp() = -pE0->sharp();

	// Optional behaviors
	if (m_pEdgeDecorator)
	{
		(*m_pEdgeDecorator)(pE0, pE1);
	}
}

void MeshLib::MeshSlicer::_slice_vertex(M::CVertex* pV0)
{
	int numSides = static_cast<int>(m_map[pV0].size());
	if (numSides <= 0) return;

	//           |
	//   side 0  |  side 1
	//           |
	// ----------V----------
	//           |
	//   side n  |  side i
	//           |

	// V--------
	// |\
	// | \ side i
	// |  \

	std::vector<M::CVertex*> sideVertices(numSides);
	std::vector<std::vector<M::CEdge*>> sideEdges(numSides); // edges on sides
	std::vector<std::vector<M::CHalfEdge*>> sideHalfEdges(numSides); // incoming hE on sides
	sideVertices[0] = pV0;

	// generate new vertices (if necessary)
	for (int i = 1; i < numSides; ++i)
	{
		//M::CVertex* pV = new M::CVertex(*pV0);
		//pV->id() = m_pMesh->numVertices();
		//m_pMesh->vertices().push_back(pV);
		int id = m_pMesh->numVertices() + 1; // vertex index starts at 1
		M::CVertex* pV = m_pMesh->createVertex(id);
		sideVertices[i] = pV;
	}

	// sort out attributes from each side
	for (int i = 0; i < numSides; ++i)
	{
		M::CHalfEdge* pCcwH = m_map[pV0][i];

		// set most ccw incoming halfedge temporarily
		pV0->halfedge() = pCcwH;

		// REMINDER: Check vertex was labelled boundary first!
		// Labelling boundary was done in edge slicing step.

		// store halfedges from one side ccwly
		for (M::VertexInHalfedgeIterator hiter(m_pMesh, pV0); !hiter.end(); ++hiter)
		{
			M::CHalfEdge* pH = *hiter;
			sideHalfEdges[i].push_back(pH);
		}

		// store edges from one side ccwly
		//for (M::VertexEdgeIterator eiter(pV0); !eiter.end(); ++eiter)
		//{
		//	M::CEdge* pE = *eiter;
		//	sideEdges[i].push_back(pE);
		//}
	}

	// assign attributes to each side
	for (int i = 0; i < numSides; ++i)
	{
		M::CVertex* pV = sideVertices[i];
		M::CHalfEdge* pCcwH = m_map[pV0][i];

		// set most ccw incoming halfedge
		pV->halfedge() = pCcwH;

		// update halfedge target
		for (M::CHalfEdge* pH : sideHalfEdges[i])
			pH->target() = pV;

		// update vertex adjacent edges
		//pV->edges().clear();
		//for (M::CEdge* pE : sideEdges[i])
		//	pV->edges().push_back(pE);

		// mark new vertex as boundary (IMPORTANT)
		pV->boundary() = true;
	}

	if (m_pVertexDecorator)
	{
		for (int i = 1; i < numSides; ++i)
		{
			M::CVertex* pV = sideVertices[i];
			(*m_pVertexDecorator)(pV0, pV);
		}
	}
}
