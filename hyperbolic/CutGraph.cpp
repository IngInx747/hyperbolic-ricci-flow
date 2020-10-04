#include "CutGraph.h"

#include <set>
#include <queue>

void MeshLib::CutGraph::generate()
{
    if (!m_pMesh) return;

    m_sharps.clear();

    for (M::CEdge* pE : m_pMesh->edges())
        m_sharps.insert(pE);

    M::CFace* pFs = m_pMesh->faces().back();
    _spread_fire(pFs);

    do {
        _prune_branch();
    } while (_shrink_triangles() > 0);

    // unsharp all edges
    for (M::CEdge* pE : m_pMesh->edges())
        pE->sharp() = 0;

    // mark cycle edges sharp
    for (M::CEdge* pE : m_sharps)
        pE->sharp() = 1;
}

void MeshLib::CutGraph::generate_experimental()
{
    if (!m_pMesh) return;

    m_sharps.clear();

    for (M::CEdge* pE : m_pMesh->edges())
        m_sharps.insert(pE);

    M::CVertex* pVs = m_pMesh->vertices().back();

    int maxValence = INT_MIN;
    for (M::CVertex* pV : m_pMesh->vertices())
    {
        int valence = 0;
        for (M::VertexVertexIterator viter(pV); !viter.end(); ++viter)
            ++valence;
        if (valence >= maxValence)
        {
            maxValence = valence;
            pVs = pV;
        }
    }

    _spread_fire(pVs);

    std::unordered_set<M::CEdge*> vertSpanTree;

    for (M::CEdge* pE : m_pMesh->edges())
        if (m_sharps.find(pE) == m_sharps.end())
            vertSpanTree.insert(pE);

    M::CFace* pFs = m_pMesh->faces().back();
    _spread_fire(pFs);

    for (M::CEdge* pE : vertSpanTree)
        m_sharps.insert(pE);

    do {
        _prune_branch();
    } while (_shrink_triangles() > 0);

    // unsharp all edges
    for (M::CEdge* pE : m_pMesh->edges())
        pE->sharp() = 0;

    // mark cycle edges sharp
    for (M::CEdge* pE : m_sharps)
        pE->sharp() = 1;
}

void MeshLib::CutGraph::_spread_fire(M::CVertex* vertex)
{
    std::queue<M::CVertex*> q;
    std::unordered_map<M::CVertex*, bool> visit;

    for (M::CVertex* pV : m_pMesh->vertices())
        visit[pV] = false;

    visit[vertex] = true;
    q.push(vertex);

    while (!q.empty())
    {
        M::CVertex* pV = q.front();
        q.pop();

        //for (M::VertexVertexIterator viter(pV); !viter.end(); ++viter)
        //{
        //    M::CVertex* pW = *viter;
        //    if (visit[pW]) continue;
        //}

        for (M::VertexOutHalfedgeIterator hiter(m_pMesh, pV); !hiter.end(); ++hiter)
        {
            M::CHalfEdge* pH = *hiter;
            M::CVertex* pW = m_pMesh->halfedgeTarget(pH);

            if (!visit[pW])
            {
                M::CEdge* pE = m_pMesh->halfedgeEdge(pH);
                m_sharps.erase(pE);
                q.push(pW);
                visit[pW] = true;
            }
        }

        if (pV->boundary())
        {
            M::CHalfEdge* pH = m_pMesh->vertexMostCcwInHalfEdge(pV);
            M::CVertex* pW = m_pMesh->halfedgeTarget(pH);

            if (!visit[pW])
            {
                M::CEdge* pE = m_pMesh->halfedgeEdge(pH);
                m_sharps.erase(pE);
                q.push(pW);
                visit[pW] = true;
            }
        }
    }
}

void MeshLib::CutGraph::_spread_fire(M::CFace* face)
{
    std::queue<M::CFace*> q;
    std::unordered_map<M::CFace*, bool> visit;

    for (M::CFace* pF : m_pMesh->faces())
        visit[pF] = false;

    visit[face] = true;
    q.push(face);

    while (!q.empty())
    {
        M::CFace* pF = q.front();
        q.pop();

        for (M::FaceEdgeIterator eiter(pF); !eiter.end(); ++eiter)
        {
            M::CEdge* pE = *eiter;

            // never cross a boundary edge
            if (pE->boundary()) continue;

            // never cross a pre-marked edge (these not in sharp set)
            if (m_sharps.find(pE) == m_sharps.end()) continue;

            M::CFace* pSymF = m_pMesh->edgeFace1(pE);
            if (pSymF == pF) pSymF = m_pMesh->edgeFace2(pE);

            if (!visit[pSymF])
            {
                m_sharps.erase(pE);
                visit[pSymF] = true;
                q.push(pSymF);
            }
        }
    }
}

void MeshLib::CutGraph::_prune_branch()
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

int MeshLib::CutGraph::_shrink_triangles()
{
    int count = 0;
    std::unordered_map<M::CVertex*, int> valences;
    std::set<M::CFace*, PtrCompare<M::CFace*>> faces; // use ordered set to make result stable

    for (M::CEdge* pE : m_sharps)
    {
        if (pE->boundary()) continue;
        faces.insert(m_pMesh->edgeFace1(pE));
        faces.insert(m_pMesh->edgeFace2(pE));
        valences[m_pMesh->edgeVertex1(pE)] += 1;
        valences[m_pMesh->edgeVertex2(pE)] += 1;
    }

    //for (M::CEdge* pE : m_pMesh->edges())
    //{
    //    valences[m_pMesh->edgeVertex1(pE)] = 0;
    //    valences[m_pMesh->edgeVertex2(pE)] = 0;
    //}
    //for (M::CEdge* pE : sharps)
    //{
    //    valences[m_pMesh->edgeVertex1(pE)] += 1;
    //    valences[m_pMesh->edgeVertex2(pE)] += 1;
    //}

    for (M::CFace* pF : faces)
    {
        int nSharp = 0;
        std::vector<M::CEdge*> edges;

        for (M::FaceEdgeIterator eiter(pF); !eiter.end(); ++eiter)
        {
            M::CEdge* pE = *eiter;
            edges.push_back(pE);
            if (m_sharps.find(pE) != m_sharps.end()) ++nSharp;

            // preserve sharp edges on boundary
            if (pE->boundary())
            {
                nSharp = 0;
                break;
            }
        }

        /*
               .------.          .      .
                \    /   ----->   \    /
                 \  /   (unsharp)  \  /
                  \/                \/
         */
        if (nSharp == 3)
        {
            m_sharps.erase(edges[0]);
            ++count;
        }

        /*
               .      .          .------.
                \    /   ----->
                 \  /    (switch
                  \/   sharp edges)
         */
        if (nSharp == 2)
        {
            std::unordered_set<M::CVertex*> vSet;
            M::CVertex* pVc = NULL; // common vertex

            for (int i = 0; i < 3; ++i)
            {
                if (m_sharps.find(edges[i]) != m_sharps.end())
                {
                    M::CVertex* pV = NULL;

                    pV = m_pMesh->edgeVertex1(edges[i]);
                    if (vSet.find(pV) == vSet.end()) { vSet.insert(pV); }
                    else { pVc = pV; break; }

                    pV = m_pMesh->edgeVertex2(edges[i]);
                    if (vSet.find(pV) == vSet.end()) { vSet.insert(pV); }
                    else { pVc = pV; break; }
                }

                if (pVc) break;
            }

            if (pVc && valences[pVc] == 2)
            {
                for (int i = 0; i < 3; ++i)
                {
                    M::CEdge* pE = edges[i];

                    if (m_sharps.find(pE) == m_sharps.end())
                        m_sharps.insert(pE);
                    else
                        m_sharps.erase(pE);
                }

                ++count;
            }
        }
    }

    return count;
}
