#include "HyperbolicMap.h"

#include <complex>

#include "MeshGraph.h"
#include "RicciFlow.h"
#include "CutGraph.h"
#include "MeshEditor.h"
#include "MeshEmbed.h"

inline int compute_chi(MeshLib::CHyperbolicMesh& mesh)
{
    return mesh.numVertices() + mesh.numFaces() - mesh.numEdges();
}

inline double hyperbolic_length_scale(double length, double u0, double u1)
{
    return std::asinh(std::exp((u0 + u1) * 0.5) * std::sinh(length * 0.5)) * 2;
}

void MeshLib::HyperbolicMap::set_mesh(M* pMesh)
{
    m_pMesh = pMesh;
    m_pMesh->reIndexVert();
    m_flow.set_mesh(m_pMesh);
    m_flow.initialize();
}

void MeshLib::HyperbolicMap::ricci_flow(double threshold, double lambda, int maxSteps)
{
    m_flow.compute_metrics_newton(threshold, lambda, maxSteps);
    //m_flow.compute_metrics_gradient(threshold, lambda, maxSteps);
}

void MeshLib::HyperbolicMap::greedy_homotopy_generators()
{
    M& mesh = original_mesh();

    int chi = compute_chi(mesh);
    int genus = (2 - chi) / 2;

    printf("[Hyperbolic Map] V+F-E = %d, genus = %d, Canonical FD = %d\n", chi, genus, genus * 4);

    // generate cut graph to form a fundamental domain
    CutGraph cutGraph(&mesh);
    std::unordered_map<M::CVertex*, double> factors;

    for (auto& p : m_flow.factors())
    {
        int id = p.first->id();
        double u = p.second;

        M::CVertex* pVt = mesh.idVertex(id);
        assert(pVt);
        factors[pVt] = u;
    }

    for (M::CEdge* pE : mesh.edges())
    {
        M::CVertex* pV0 = mesh.edgeVertex1(pE);
        M::CVertex* pV1 = mesh.edgeVertex2(pE);
        double length = mesh.edgeLength(pE);
        double u0 = factors[pV0];
        double u1 = factors[pV1];
        length = hyperbolic_length_scale(length, u0, u1);
        cutGraph.lengths()[pE] = (float)length;
    }

    cutGraph.generate();
    m_base_id = mesh.vertexId(cutGraph.base_point());
}

void MeshLib::HyperbolicMap::mark_fundamental_domain()
{
    M& mesh = original_mesh();

    int chi = compute_chi(mesh);
    int genus = (2 - chi) / 2;

    //
    Graph<M::CVertex*, M::CEdge*> fundDomGraph;

    // populate graph for next steps
    for (M::CEdge* pE : mesh.edges())
    {
        if (!pE->sharp()) continue;

        M::CVertex* pV = mesh.edgeVertex1(pE);
        M::CVertex* pW = mesh.edgeVertex2(pE);

        fundDomGraph.adj()[pV][pW] = pE;
        fundDomGraph.adj()[pW][pV] = pE;
    }

    // the vertices on fundamental domain
    std::vector<std::vector<M::CVertex*>> vFundDom;

    fundDomGraph.detour_components(vFundDom);

    printf("[Hyperbolic Map] Generated fundamental domain edges number : %zd\n", vFundDom.size() * 2);

    // make result stable
    std::sort(vFundDom.begin(), vFundDom.end(), [](auto& a, auto& b) { return a.size() > b.size(); });

    // distinguish edges based on their component order in fundamental domain
    for (int i = 0; i < vFundDom.size(); ++i)
    {
        auto& vertices = vFundDom[i];

        for (int j = 0; j < vertices.size() - 1; ++j)
        {
            M::CVertex* pV0 = vertices[j];
            M::CVertex* pV1 = vertices[j + 1];
            M::CEdge* pE = fundDomGraph.adj()[pV0][pV1];
            pE->sharp() = i + 1;
        }
    }
}

void MeshLib::HyperbolicMap::slice_fundamental_domain()
{
    M& mesh = open_mesh();
    mesh.copy_from(*m_pMesh);

    // cut edges along fundamental domain
    // divide sharp edge into two edges
    struct HyperbolicEdgeDecorator : public MeshSlicer::EdgeDecorator
    {
        HyperbolicEdgeDecorator(
            std::unordered_map<int, int>& map)
            : m_map(map)
        {}

        virtual void operator() (M::CEdge* pE0, M::CEdge* pE1)
        {
            // Hyperbolic mesh specification
            pE1->sharp() = pE0->sharp();
            m_map[pE0->id()] = pE1->id();
            m_map[pE1->id()] = pE0->id();
        }

        std::unordered_map<int, int>& m_map;
    } ed(m_edgeIndexMap);

    //struct HyperbolicVertexDecorator : public MeshSlicer::VertexDecorator
    //{
    //    HyperbolicVertexDecorator(
    //        std::unordered_map<M::CVertex*, double>& factors)
    //        : m_us(factors)
    //    {}
    //
    //    virtual void operator() (M::CVertex* pV0, M::CVertex* pV1)
    //    {
    //        m_us[pV1] = m_us[pV0];
    //    }
    //
    //    std::unordered_map<M::CVertex*, double>& m_us;
    //} vd(m_factors);

    struct HyperbolicVertexDecorator : public MeshSlicer::VertexDecorator
    {
        HyperbolicVertexDecorator(
            std::unordered_map<int, int>& id)
            : m_id(id)
        {}

        virtual void operator() (M::CVertex* pV0, M::CVertex* pV1)
        {
            pV1->point() = pV0->point();
            pV1->normal() = pV0->normal();
            pV1->uv() = pV0->uv();
            pV1->string() = pV0->string();

            int id0 = pV0->id();
            int id1 = pV1->id();
            m_id[id1] = id0;
            //m_id[id0] = id1;
            //printf("%d <- %d\n", id0, id1);
        }

        std::unordered_map<int, int>& m_id;
    } vd(m_vertIndexMap);

    // slice mesh along sharp edges
    MeshSlicer slicer(&mesh);
    slicer.set_edge_decorator(&ed);
    slicer.set_vertex_decorator(&vd);
    slicer.slice_edges();

    int chi = compute_chi(mesh);

    printf("[Hyperbolic Map] Mesh has been cut. V+F-E = %d\n", chi);
}

void MeshLib::HyperbolicMap::isometric_embed()
{
    int chi = compute_chi(open_mesh());

    if (chi != 1)
    {
        printf("[Hyperbolic Map] Cannot flatten mesh with chi = %d. Please slice mesh first.\n", chi);
        return;
    }

    M& mesh = domain_mesh();
    mesh.copy_from(open_mesh());

    // copy conformal factors
    std::unordered_map<M::CVertex*, double> factors;

    for (auto& p : m_flow.factors())
    {
        int id = p.first->id();
        double u = p.second;
    
        M::CVertex* pVt = mesh.idVertex(id);
        assert(pVt);
        factors[pVt] = u;
        //printf("u(%d) = %lf\n", id, factors[pVt]);
    }

    // factors of these newly generated vertices after slicing
    for (auto& p : m_vertIndexMap)
    {
        int idt = p.first;
        int idf = p.second;
        M::CVertex* pVf = mesh.idVertex(idf);
        M::CVertex* pVt = mesh.idVertex(idt);
        assert(pVf);
        assert(pVt);
        double u = factors[pVf];
        factors[pVt] = u;
        //printf("u(%d) = u(%d) = %lf\n", idt, idf, factors[pVt]);
        //printf("u(%d) -> u(%d)\n", pVt->id(), pVf->id());
    }

    HyperbolicEmbed embed(&mesh);

    // set embedding lengths
    for (M::MeshEdgeIterator eiter(&mesh); !eiter.end(); ++eiter)
    {
        M::CEdge* pE = *eiter;
        M::CVertex* pV0 = mesh.edgeVertex1(pE);
        M::CVertex* pV1 = mesh.edgeVertex2(pE);
        double length = mesh.edgeLength(pE);
        double u0 = factors[pV0];
        double u1 = factors[pV1];
        length = hyperbolic_length_scale(length, u0, u1);
        embed.lengths()[pE] = length;
    }

    embed.embed();
    
    for (auto& p : embed.uv())
    {
        M::CVertex* pV = p.first;
        CPoint2 uv = p.second;
        pV->point() = CPoint(uv[0], uv[1], 0);
    }

    printf("[Hyperbolic Map] Mesh has been embedded into Poincare Disk.\n");

    //for (M::CVertex* pV : m_pMesh->vertices())
    //{
    //    CPoint n;
    //
    //    if (!pV->boundary()) continue;
    //
    //    for (M::VertexFaceIterator fiter(pV); !fiter.end(); ++fiter)
    //    {
    //        M::CFace* pF = *fiter;
    //        n += pF->normal();
    //    }
    //    n /= n.norm();
    //
    //    pV->point() += n * 0.01;
    //}
}

void MeshLib::HyperbolicMap::sort_domain_boundaries()
{
    int chi = compute_chi(open_mesh());
    int genus = (2 - chi) / 2;
    M& mesh = domain_mesh();
#if 0
    M::CVertex* pStarV = mesh.idVertex(m_base_id);
    std::unordered_set<M::CVertex*> basePointSet;

    basePointSet.insert(pStarV);

    for (auto& p : m_vertIndexMap)
    {
        int idt = p.first;
        int idf = p.second;

        if (idf == m_base_id)
        {
            M::CVertex* pV = mesh.idVertex(idt);
            basePointSet.insert(pV);
        }
    }

    for (M::CVertex* pV : basePointSet)
    {
        printf("%d -> %d\n", pV->id(), m_base_id);
    }

    // beginning vertex of each segment with segment index
    std::vector<M::CVertex*> basePoints; // v_1, v_2, ..., v_{4g}, v_0
    M::CVertex* pIterV = pStarV;

    // sort out segments along boundary ccwly
    do {
        M::CHalfEdge* pH = mesh.vertexMostClwOutHalfEdge(pIterV);
        M::CEdge* pE = mesh.halfedgeEdge(pH);

        if (basePointSet.find(pIterV) != basePointSet.end())
            basePoints.push_back(pIterV);

        pIterV = mesh.halfedgeTarget(pH); // next vertex, ccw along boundary
    } while (pIterV != pStarV);

    int nv = (int)basePoints.size() - 1; // 4g
    assert(nv == genus * 4);

    // segment id k |-> [s_k(0), s_k(1)]
    m_segments.clear();

    for (int i = 0; i < nv; ++i)
    {
        M::CVertex* pV0 = basePoints[i];
        M::CVertex* pV1 = basePoints[i + 1];
        int vid0 = pV0->id();
        int vid1 = pV1->id();

        int gid = i / 4; // group id 0~(g-1)
        int cls = i % 4; // class id 0~3
        int sid = 0;

        if (cls == 0) sid = gid * 2 + 1;
        else if (cls == 1) sid = gid * 2 + 2;
        else if (cls == 2) sid = -(gid * 2 + 1);
        else if (cls == 3) sid = -(gid * 2 + 2);

        m_segments[sid] = std::make_pair(vid0, vid1);
    }

#else
    // look for a vertex on domain boundary
    M::CVertex* pStaV = NULL;
    for (M::CVertex* pV : mesh.vertices())
    {
        if (pV->boundary()) // <- be cautious (make sure boundary flag is valid)
        {
            pStaV = pV;
            break;
        }
    }
    if (!pStaV)
    {
        printf("[Hyperbolic Map] Please do hyperbolically isometric embedment first.\n");
        return;
    }
    
    // go to the nearest intersection between two arbitrary adjacent segments
    while (true)
    {
        M::CHalfEdge* pHi = mesh.vertexMostCcwInHalfEdge(pStaV);
        M::CHalfEdge* pHo = mesh.vertexMostClwOutHalfEdge(pStaV);
        int si = mesh.halfedgeEdge(pHi)->sharp();
        int so = mesh.halfedgeEdge(pHo)->sharp();
        if (si != so) break;
        pStaV = mesh.halfedgeTarget(pHo);
    }
    
    // beginning vertex of each segment with segment index
    std::vector<std::pair<M::CVertex*, int>> starters;
    
    M::CVertex* pVs = pStaV;
    
    // sort out segment with its beginning vertex
    do {
        M::CHalfEdge* pHi = mesh.vertexMostCcwInHalfEdge(pVs);
        M::CHalfEdge* pHo = mesh.vertexMostClwOutHalfEdge(pVs);
        int si = mesh.halfedgeEdge(pHi)->sharp();
        int so = mesh.halfedgeEdge(pHo)->sharp();
    
        // meet an intersection vertex
        if (si != so) starters.emplace_back(pVs, so);
    
        // go to next vertex, ccw along boundary
        pVs = mesh.halfedgeTarget(pHo);
    } while (pVs != pStaV);

    // segment id k |-> [s_k(0), s_k(1)]
    m_segments.clear();

    for (int i = 0; i < starters.size(); ++i)
    {
        int id = starters[i].second; // segment id
        int j = (i + 1) % starters.size();
        M::CVertex* pV0 = starters[i].first;
        M::CVertex* pV1 = starters[j].first;
        int vid0 = pV0->id();
        int vid1 = pV1->id();
        if (m_segments.find(id) != m_segments.end()) id = -id;
        m_segments[id] = std::make_pair(vid0, vid1);
    }
#endif
    printf("[Hyperbolic Map] Domain boundaries:\n");
    for (auto& p : m_segments)
    {
        int s = p.first;
        M::CVertex* pV0 = mesh.idVertex(p.second.first);
        M::CVertex* pV1 = mesh.idVertex(p.second.second);
        printf("s_{%d}(0) = (%lf, %lf) -> ", s, pV0->point()[0], pV0->point()[1]);
        printf("s_{%d}(1) = (%lf, %lf)\n", s, pV1->point()[0], pV1->point()[1]);
    }
    printf("[Hyperbolic Map] End of domain boundaries.\n");
}

void MeshLib::HyperbolicMap::compute_fuchsian_group()
{
    if (m_segments.empty())
    {
        printf("[Hyperbolic Map] Please sort domain boundaries first.\n");
        return;
    }

    M& mesh = domain_mesh();
    std::complex<double> I(1, 0);

    // compute Mobius transform \alpha_k mapping s_{k}[0,1] to s_{-k}[1,0]
    for (auto& p : m_segments)
    {
        int s = p.first; // segment index k
        M::CVertex* pVp0 = mesh.idVertex(p.second.first);
        M::CVertex* pVp1 = mesh.idVertex(p.second.second);
        M::CVertex* pVm0 = mesh.idVertex(m_segments[-s].first);
        M::CVertex* pVm1 = mesh.idVertex(m_segments[-s].second);
        //printf("vid = %d\n", pVp0->id());

        std::complex<double> p0(pVp0->point()[0], pVp0->point()[1]);
        std::complex<double> p1(pVp1->point()[0], pVp1->point()[1]);
        std::complex<double> m0(pVm0->point()[0], pVm0->point()[1]);
        std::complex<double> m1(pVm1->point()[0], pVm1->point()[1]);

        // this complex is of the form e^{i*\theta} whose norm equals to 1.0
        std::complex<double> et = (m0 - m1) * (I - conj(p0) * p1) / ((p1 - p0) * (I - conj(m1) * m0));
        //printf("%lf\n", norm(et));

        // compute isometric Mobius transformation
        std::complex<double> a = et - conj(p0) * m1;
        std::complex<double> b = m1 - et * p0;
        std::complex<double> c = et * conj(m1) - conj(p0);
        std::complex<double> d = I - et * p0 * conj(m1);
        m_fuchsianGroupGenerators[s] = MobiusTransform<double>(a, b, c, d);
    }

    printf("[Hyperbolic Map] Compute Fuchsian group...\n");
    for (auto& p : m_fuchsianGroupGenerators)
    {
        int s = p.first;
        const MobiusTransform<double>& mt = p.second;
        printf("s_{%d} -> s_{%d} : ", s, -s);
        std::cout << mt << std::endl;
    }
    printf("[Hyperbolic Map] Fuchsian group has been generated.\n");
}

void MeshLib::HyperbolicMap::tessellate_disk(int level)
{
    if (m_fuchsianGroupGenerators.empty())
    {
        printf("[Hyperbolic Map] Please do hyperbolically isometric embedment first.\n");
        return;
    }

    if (m_tessellatingActions.empty() || m_tessellatingActions.level() != level)
    {
        std::vector<int> domains;
        for (auto& p : m_fuchsianGroupGenerators) domains.push_back(p.first);
        m_tessellatingActions.span_complete_tree(domains, level);
    }

    // reset
    clear_tessellation();

    M& mesh = domain_mesh();

    for (Tree<int>::PathIterator iter(m_tessellatingActions); !iter.end(); ++iter)
    {
        auto path = *iter;

        // elder implementation
        //m_tessellationMeshes.emplace_back(domain_mesh());
        //M& mesh = m_tessellationMeshes.back();

        m_tessellationMeshes.emplace_back();
        auto& uvs = m_tessellationMeshes.back();
        MobiusTransform<double> mt;

        // composite Mobius transforms
        printf("[Hyperbolic Map] Fuchsian action : (");
        for (int s : path)
        {
            printf("%d, ", s);
            mt += m_fuchsianGroupGenerators[s];
        }
        printf("\b\b)\n");

        // apply transforms to current mesh
        for (M::CVertex* pV : mesh.vertices())
        {
            std::complex<double> uv(pV->point()[0], pV->point()[1]);
            uv = mt(uv);
            //pV->point() = CPoint(uv.real(), uv.imag(), 0);
            uvs[pV] = CPoint2(uv.real(), uv.imag());
        }

        // label mesh with id of last Mobius transform
        m_tessellationIndices[iter.count()] = path.back();
    }

    printf("[Hyperbolic Map] Poincare disk tessellation finished.\n");
    printf("[Hyperbolic Map] Generated new meshes: %zd\n", m_tessellationMeshes.size());
}

void MeshLib::HyperbolicMap::tessellate_disk_single_step(int level)
{
    if (m_fuchsianGroupGenerators.empty())
    {
        printf("[Hyperbolic Map] Please do hyperbolically isometric embedment first.\n");
        return;
    }

    if (m_tessellatingActions.empty() || m_tessellatingActions.level() != level)
    {
        std::vector<int> domains;
        for (auto& p : m_fuchsianGroupGenerators) domains.push_back(p.first);
        m_tessellatingActions.span_complete_tree(domains, level);
        clear_tessellation();
    }

    if (m_tessellatingIterator.end())
    {
        m_tessellatingIterator.reset(m_tessellatingActions);
    }

    auto path = *m_tessellatingIterator;
    int id = m_tessellatingIterator.count();

    if (m_tessellationMeshes.size() <= id)
    {
        m_tessellationMeshes.resize(id + 1);
    }

    // copy whole domain (too memory-consuming)
    //M& mesh = m_tessellationMeshes[id];
    //mesh.copy_from(domain_mesh());

    auto& uvs = m_tessellationMeshes[id];
    MobiusTransform<double> mt;

    M& mesh = domain_mesh();

    // composite Mobius transforms
    printf("[Hyperbolic Map] Fuchsian action : (");
    for (int s : path)
    {
        printf("%d, ", s);
        mt += m_fuchsianGroupGenerators[s];
    }
    printf("\b\b)\n");

    // apply transforms to current mesh
    for (M::CVertex* pV : mesh.vertices())
    {
        std::complex<double> uv(pV->point()[0], pV->point()[1]);
        uv = mt(uv);
        //pV->point() = CPoint(uv.real(), uv.imag(), 0);
        uvs[pV] = CPoint2(uv.real(), uv.imag());
    }

    // label mesh with id of last Mobius transform
    m_tessellationIndices[id] = path.back();

    ++m_tessellatingIterator;
}

void MeshLib::HyperbolicMap::clear_tessellation()
{
    m_tessellatingIterator.clear();
    m_tessellationMeshes.clear();
    m_tessellationIndices.clear();
}

void MeshLib::HyperbolicMap::compute_geodesic_cycles()
{
    m_circles.clear();
    M& mesh = domain_mesh();

    for (auto& p : m_segments)
    {
        int s = p.first;
        int vid0 = p.second.first;
        int vid1 = p.second.second;
        M::CVertex* pV0 = mesh.idVertex(vid0);
        M::CVertex* pV1 = mesh.idVertex(vid1);
        CPoint p0 = pV0->point();
        CPoint p1 = pV1->point();
        double u0 = p0[0];
        double v0 = p0[1];
        double u1 = p1[0];
        double v1 = p1[1];
        double n0 = u0 * u0 + v0 * v0;
        double n1 = u1 * u1 + v1 * v1;
        double det = u0 * v1 - u1 * v0; // cross p0, p1
        double a = 0.5 * (v1 * (n0 + 1) - v0 * (n1 + 1)) / det;
        double b = 0.5 * (u0 * (n1 + 1) - u1 * (n0 + 1)) / det;
        CPoint2 center = CPoint2(a, b);
        double radius = std::sqrt(a * a + b * b - 1);
        //printf("%lf, %lf, %lf\n", a, b, radius);
        m_circles.emplace_back(center, radius);
    }
}
