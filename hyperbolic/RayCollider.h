#pragma once
#ifndef RAY_COLLIDER_H
#define RAY_COLLIDER_H

#include "HyperbolicMesh.h"

namespace MeshLib
{
    class RayCollider
    {
    protected:
        using M = CHyperbolicMesh;

    public:
        RayCollider() {}

        RayCollider(M* pMesh) { set_mesh(pMesh); }

        void set_mesh(M* pMesh) { m_pMesh = pMesh; }

        bool collide(M::CFace* face, const CPoint& org, const CPoint& dir, double& dist) const;

        M::CFace* collide(const CPoint& org, const CPoint& dir, double& dist) const;

    protected:
        M* m_pMesh = NULL;
    };
}

#endif // !RAY_COLLIDER_H
