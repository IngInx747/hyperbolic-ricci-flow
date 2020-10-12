#pragma once
#ifndef _MESH_GRAPH_HPP_
#define _MESH_GEAPH_HPP_

#include "MeshGraph.h"

#include <limits>
#include <queue>

#include "DisjointSets.h"

template<class V, class W>
W Graph<V, W>::shortest_distance(V source, V target)
{
    using Pair = std::pair<W, V>;

    std::unordered_map<V, W> dist;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair>> pq;

    pq.push(Pair(static_cast<W>(0), source));
    dist[source] = static_cast<W>(0);

    while (!pq.empty())
    {
        V v = pq.top().second;
        pq.pop();

        for (auto& p : m_G[v])
        {
            V u = p.first;
            W w = p.second;

            if (dist.find(u) == dist.end())
                dist[u] = std::numeric_limits<W>::max();

            // rest edge v-u
            if (dist[u] > dist[v] + w)
            {
                dist[u] = dist[v] + w;
                pq.push(std::make_pair(dist[u], u));
            }
        }
    }

    if (dist.find(target) == dist.end())
        return std::numeric_limits<W>::max();

    return dist[target];
}

template<class V, class W>
W Graph<V, W>::shortest_path(V source, V target, std::vector<V>& path)
{
    using Pair = std::pair<W, V>;

    std::unordered_map<V, W> dist;
    std::unordered_map<V, V> parent;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair> > pq;

    pq.push(Pair(static_cast<W>(0), source));
    dist[source] = static_cast<W>(0);
    parent[source] = source;

    while (!pq.empty())
    {
        V v = pq.top().second;
        pq.pop();

        for (auto& p : m_G[v])
        {
            V u = p.first;
            W w = p.second;

            if (dist.find(u) == dist.end())
                dist[u] = std::numeric_limits<W>::max();

            // rest edge v-u
            if (dist[u] > dist[v] + w)
            {
                dist[u] = dist[v] + w;
                pq.push(std::make_pair(dist[u], u));
                parent[u] = v;
            }
        }
    }

    if (parent.find(target) == parent.end())
        return std::numeric_limits<W>::max();

    V vertex = target;
    path.push_back(vertex);

    while (vertex != source)
    {
        vertex = parent[vertex];
        path.push_back(vertex);
    }

    return dist[target];
}

template<class V, class W>
void Graph<V, W>::shortest_path_tree(V root, std::vector<Edge>& tree)
{
    using Pair = std::pair<W, V>;

    std::unordered_map<V, W> dist;
    std::unordered_map<V, V> parent;
    std::priority_queue<Pair, std::vector<Pair>, std::greater<Pair> > pq;

    pq.push(Pair(static_cast<W>(0), root));
    dist[root] = static_cast<W>(0);
    //parent[root] = root;

    while (!pq.empty())
    {
        V v = pq.top().second;
        pq.pop();

        for (auto& p : m_G[v])
        {
            V u = p.first;
            W w = p.second;

            if (dist.find(u) == dist.end())
                dist[u] = std::numeric_limits<W>::max();

            // rest edge v-u
            if (dist[u] > dist[v] + w)
            {
                dist[u] = dist[v] + w;
                pq.push(std::make_pair(dist[u], u));
                parent[u] = v;
            }
        }
    }

    for (auto& p : parent)
    {
        V v = p.first;
        V u = p.second; // parent of v
        W d = dist[v]; // distance to root
        tree.push_back(Edge(v, std::make_pair(u, d)));
    }
}

template<class V, class W>
void Graph<V, W>::connecting_representatives(std::vector<V>& vertices)
{
    std::unordered_map<V, bool> visit;
    std::unordered_map<V, V> parent;

    for (auto& p : m_G)
    {
        V v = p.first;
        parent[v] = v;
        visit[v] = false;
    }

    for (auto& p : m_G)
    {
        V rep = p.first;
        if (visit[rep]) continue;

        std::queue<V> q;
        q.push(rep);
        visit[rep] = true;

        while (!q.empty())
        {
            V v = q.front();
            V pv = parent[v];
            q.pop();

            for (auto& p : m_G[v])
            {
                V u = p.first;

                if (u == pv) continue; // v comes from u in BFS

                if (!visit[u])
                {
                    visit[u] = true;
                    parent[u] = v;
                    q.push(u);
                }
            }
        }

        vertices.push_back(rep);
    }
}

template<class V, class W>
void Graph<V, W>::connecting_components(std::vector<std::vector<V>>& components)
{
    std::unordered_map<V, bool> visit;
    std::unordered_map<V, V> parent;

    for (auto& p : m_G)
    {
        V v = p.first;
        parent[v] = v;
        visit[v] = false;
    }

    for (auto& p : m_G)
    {
        V rep = p.first;
        if (visit[rep]) continue;

        std::vector<V> vertices;
        std::queue<V> q;
        q.push(rep);
        visit[rep] = true;

        while (!q.empty())
        {
            V v = q.front();
            V pv = parent[v];
            vertices.push_back(v);
            q.pop();

            for (auto& p : m_G[v])
            {
                V u = p.first;

                if (u == pv) continue; // v comes from u in BFS

                if (!visit[u])
                {
                    visit[u] = true;
                    parent[u] = v;
                    q.push(u);
                }
            }
        }

        components.push_back(vertices);
    }
}

template<class V, class W>
void Graph<V, W>::_cycle_representatives(const std::vector<V>& cc, std::vector<Edge>& edges)
{
    enum BFSStatus
    {
        NOT_VISITED,
        ENQUEUED,
        VISITED,
    };

    std::unordered_map<V, BFSStatus> visit;
    std::unordered_map<V, V> parent;
    std::queue<V> q;

    for (auto& p : m_G)
    {
        V v = p.first;
        parent[v] = v;
        visit[v] = NOT_VISITED;
    }

    // start with vertex of each connecting component
    for (V v : cc)
    {
        visit[v] = VISITED;
        q.push(v);
    }

    // construct minimum spanning tree conceptually
    // BFS to find out all cycle representatives
    while (!q.empty())
    {
        V v = q.front();
        V pv = parent[v];
        visit[v] = VISITED;
        q.pop();

        for (auto& p : m_G[v])
        {
            V u = p.first;
            W w = p.second;

            if (u == pv) continue; // v comes from u in BFS

            if (visit[u] == NOT_VISITED)
            {
                visit[u] = ENQUEUED;
                parent[u] = v;
                q.push(u);
            }
            else if (visit[u] == VISITED) // cycle detected
            {
                edges.push_back(Edge(v, std::make_pair(u, w)));
            }
        }
    }
}

template<class V, class W>
void Graph<V, W>::cycle_representatives(std::vector<Edge>& edges)
{
    std::vector<V> repVerts; // connecting components representatives

    // sort out connecting components representatives
    connecting_representatives(repVerts);

    // sort out cycle components representatives
    _cycle_representatives(repVerts, edges);
}

template<class V, class W>
void Graph<V, W>::cycle_components(std::vector<std::vector<V>>& components)
{
    using Edge = std::pair<V, std::pair<V, W>>;

    std::vector<Edge> repEdges; // cycle components representatives

    // sort out cycle components representatives
    cycle_representatives(repEdges);

    // sort out vertices forming cycle for each representative
    // Break all representative edges, then restore one edge
    // everytime and detect cycle in the graph. There is only
    // one cycle during each attempt.

    // break all representative edges
    for (Edge& edge : repEdges)
    {
        V v = edge.first;
        V u = edge.second.first;
        m_G[v].erase(u);
        m_G[u].erase(v);
    }

    // detect cycle
    for (Edge& edge : repEdges)
    {
        V v = edge.first;
        V u = edge.second.first;
        W w = edge.second.second;

        // restore one representative edge
        m_G[v][u] = w;
        m_G[u][v] = w;

        // find cycle
        std::vector<V> vCycle;
        if (_cycle_component(v, vCycle))
            components.push_back(vCycle);

        // break the representative edge again
        m_G[v].erase(u);
        m_G[u].erase(v);
    }

    // restore all representative edges
    for (Edge& edge : repEdges)
    {
        V v = edge.first;
        V u = edge.second.first;
        W w = edge.second.second;
        m_G[v][u] = w;
        m_G[u][v] = w;
    }
}

template<class V, class W>
bool Graph<V, W>::_cycle_component(V rep, std::vector<V>& vertices)
{
    std::unordered_map<V, bool> visit;
    std::unordered_map<V, V> parent;
    std::queue<V> q;
    V v0, v1; // wave front

    for (auto& p : m_G)
    {
        V v = p.first;
        parent[v] = v;
        visit[v] = false;
    }

    q.push(rep);
    visit[rep] = true;
    v0 = v1 = rep;

    while (!q.empty())
    {
        V v = q.front();
        V pv = parent[v];
        q.pop();

        for (auto& p : m_G[v])
        {
            V u = p.first;

            if (!visit[u])
            {
                visit[u] = true;
                parent[u] = v;
                q.push(u);
            }
            else if (u != pv) // cycle detected
            {
                v0 = u;
                v1 = v;
                break;
            }
        }
    }

    vertices.clear();
    if (v0 == v1) return false; // no cycle detected

    /*
    * -------> V <-------
    * |                 |
    * |                 |
    * |                 |
    * <--- V0 ... V1 --->
    */

    while (v0 != rep)
    {
        vertices.push_back(v0);
        v0 = parent[v0];
    }
    std::reverse(vertices.begin(), vertices.end());

    while (v1 != rep)
    {
        vertices.push_back(v1);
        v1 = parent[v1];
    }
    vertices.push_back(rep);

    return true;
}

template<class V, class W>
void Graph<V, W>::detour_components(std::vector<std::vector<V>>& components)
{
    std::unordered_map<V, int> valences;
    std::unordered_set<V> pivots; // vertex with valences > 2

    for (auto& p : m_G)
    {
        V v = p.first;
        valences[v] = static_cast<int>(m_G[v].size());
        if (valences[v] > 2) pivots.insert(v);
    }

    for (V pivot : pivots)
    {
        for (auto& p : m_G[pivot])
        {
            std::vector<V> detour(1, pivot);

            V curr = p.first;
            V prev = pivot;

            if (valences[curr] < 1) continue; // detour has been traveled

            --valences[prev];
            --valences[curr];

            while (pivots.find(curr) == pivots.end())
            {
                if (valences[curr] < 1)
                {
                    detour.clear();
                    break; // encounter deadend
                }

                detour.push_back(curr);
                V next = prev;

                for (auto& pp : m_G[curr]) // curr node has only 2 neighbors, one is prev
                {
                    next = pp.first;
                    if (next != prev) break;
                }

                prev = curr;
                curr = next;

                --valences[prev];
                --valences[curr];
            }

            if (pivots.find(curr) != pivots.end())
                detour.push_back(curr);

            if (detour.size() > 1)
                components.push_back(detour);
        }
    }
}

template<class V, class W>
inline void Graph<V, W>::prune_branches()
{
    std::queue<V> q; // valence-1 vertices
    std::unordered_set<V> vSet; // vertices to be pruned
    std::unordered_map<V, int> valences; // vertex |-> adjacent sharp edges number

    // 1. Compute the valence of each vertex, and record all valence-1 vertices.
    // This step just fetches current valence-1 vertices. During pruning, more
    // valence-1 vertices will be generated.
    for (auto& p : m_G)
    {
        V v = p.first;
        int nv = (int)p.second.size();
        valences[v] = nv;
        if (nv <= 1) vSet.insert(v);
    }

    for (V v : vSet)
        q.push(v);

    // 2. Remove the segments which attached to valence-1 vertices.
    // After getting first batch of valence-1 vertices, prune and find
    // new valence-1 vertices recursively.
    while (!q.empty())
    {
        V v = q.front(); // valence-1 vertex
        q.pop();

        int nv = valences[v];

        if (nv == 1)
        {
            V u = m_G[v].begin()->first;
            m_G[u].erase(v);
            m_G.erase(v);
            valences.erase(v);
            --valences[u];
            if (valences[u] <= 1) q.push(u);
        }
        else // nv == 0
        {
            m_G.erase(v);
            valences.erase(v);
        }
    }
}

template<class V, class W>
void Graph<V, W>::maximum_spanning_tree(std::vector<Edge>& tree)
{
    using Pair = std::pair<W, Edge>;

    std::priority_queue<Pair, std::vector<Pair>, std::less<Pair>> pq;
    DisjointSets<V> uf;

    for (auto& p : m_G)
    {
        V v = p.first;
        uf.insert(v);
    }

    for (auto& vert : m_G)
    {
        V v = vert.first;

        for (auto& edge : vert.second)
        {
            V u = edge.first;
            W w = edge.second;
            pq.push(std::make_pair(w, Edge(v, std::make_pair(u, w))));
        }
    }

    while (!pq.empty())
    {
        Edge edge = pq.top().second;
        pq.pop();

        V v = edge.first;
        V u = edge.second.first;

        if (uf.find(v) != uf.find(u))
        {
            uf.join(v, u);
            tree.push_back(edge);
        }
    }
}

#endif // !_MESH_GRAPH_HPP_