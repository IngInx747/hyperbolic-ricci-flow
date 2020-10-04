#include "NullHomology.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>

int MeshLib::NullHomologyDetector::remove_exact_loops()
{
	//std::unordered_map<M::CFace*, bool> visit;
	//std::unordered_set<M::CEdge*> sharps;
    m_sharps.clear();
    m_visit.clear();

	for (M::CFace* pF : m_pMesh->faces())
        m_visit[pF] = false;

	for (M::CEdge* pE : m_pMesh->edges())
        if (pE->sharp())
            m_sharps.insert(pE); // store all sharp edges

	int numNullCycles = 0;
	M::CFace* pFs = *m_pMesh->faces().begin();

    while (pFs)
    {
        // Let fire traverse boundary faces graph, starting with a random face.
        // The fire spread stops by sharp edges. Homological loops do not block
        // fire spreading whole surface while null homological ones do.
        _spread_fire(pFs);

        pFs = NULL; // starting point of next fire traversal

        // Find an edge whose one of two boundary faces is visited
        // while the other one not. Unsharp the edge meanwhile
        // select the face not visited yet as start point.
        for (M::CEdge* pEs : m_sharps)
        {
            M::CFace* pF0 = m_pMesh->edgeFace1(pEs);
            M::CFace* pF1 = m_pMesh->edgeFace2(pEs);

            // exact loop detected if only one of two faces of the edge was visited
            if (m_visit[pF0] ^ m_visit[pF1])
            {
                pFs = m_visit[pF0] ? pF1 : pF0; // select the face not visited
                m_sharps.erase(pEs); // unsharp edge
                ++numNullCycles;
                break;
            }
        }
    }

    //printf("[Null homology] Exact loops number: %d\n", numNullCycles);

    // remove sharp edges on branches
    _prune_branch();

    // unsharp all edges
    for (M::CEdge* pE : m_pMesh->edges())
        pE->sharp() = false;

    // mark cycle edges sharp
    for (M::CEdge* pE : m_sharps)
        pE->sharp() = true;

    return numNullCycles;
}

void MeshLib::NullHomologyDetector::_spread_fire(M::CFace* face)
{
    std::queue<M::CFace*> q;
    m_visit[face] = true;
    q.push(face);

    while (!q.empty())
    {
        M::CFace* pF = q.front();
        q.pop();

        for (M::FaceEdgeIterator eiter(pF); !eiter.end(); ++eiter)
        {
            M::CEdge* pE = *eiter;

            // stop spreading if encounter a sharp edge
            if (m_sharps.find(pE) != m_sharps.end()) continue;

            // find symmertic face with respect of current face
            M::CFace* pSymF = m_pMesh->edgeFace1(pE);
            if (pSymF == pF) pSymF = m_pMesh->edgeFace2(pE);

            if (!m_visit[pSymF]) // not visited yet
            {
                q.push(pSymF);
                m_visit[pSymF] = true; // mark as visited
            }
        }
    }
}

void MeshLib::NullHomologyDetector::_prune_branch()
{
    std::queue<M::CVertex*> q; // valence-1 vertices
    std::unordered_set<M::CVertex*> vSet; // vertices to be pruned
    std::unordered_map<M::CVertex*, int> valences; // vertex |-> adjacent sharp edges number

    for (M::CEdge* pE : m_sharps)
    {
        valences[m_pMesh->edgeVertex1(pE)] += 1;
        valences[m_pMesh->edgeVertex2(pE)] += 1;
    }

    // 1. Compute the valence of each vertex, and record all valence-1 vertices.
    // This step just fetches current valence-1 vertices. During pruning, more
    // valence-1 vertices will be generated.
    for (auto& p : valences)
    {
        M::CVertex* pV = p.first;
        if (valences[pV] == 1)
            vSet.insert(pV);
    }

    for (M::CVertex* pV : vSet)
        q.push(pV);

    // 2. Remove the segments which attached to valence-1 vertices.
    // After getting first batch of valence-1 vertices, prune and find
    // new valence-1 vertices recursively.
    while (!q.empty())
    {
        M::CVertex* pV = q.front(); // valence-1 vertex
        q.pop();

        for (M::VertexEdgeIterator eiter(pV); !eiter.end(); ++eiter)
        {
            M::CEdge* pE = *eiter;
            if (m_sharps.find(pE) == m_sharps.end()) continue;

            M::CVertex* pW = m_pMesh->edgeVertex1(pE);
            if (pW == pV) pW = m_pMesh->edgeVertex2(pE);

            m_sharps.erase(pE); // mark edge unsharp
            valences[pW] -= 1; // the vertex on the other end of edge
            if (valences[pW] == 1) q.push(pW); // enqueue new generated valence-1 vertex
        }
    }
}
