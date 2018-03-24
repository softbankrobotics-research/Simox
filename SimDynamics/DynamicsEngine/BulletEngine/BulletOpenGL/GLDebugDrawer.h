#ifndef GL_DEBUG_DRAWER_H
#define GL_DEBUG_DRAWER_H

#include "LinearMath/btIDebugDraw.h"



class GLDebugDrawer : public btIDebugDraw
{
    int m_debugMode;

public:

    GLDebugDrawer();
    ~GLDebugDrawer() override;

    void    drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor) override;

    void    drawLine(const btVector3& from, const btVector3& to, const btVector3& color) override;

    void    drawSphere(const btVector3& p, btScalar radius, const btVector3& color) override;

    void    drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha) override;

    void    drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) override;

    void    reportErrorWarning(const char* warningString) override;

    void    draw3dText(const btVector3& location, const char* textString) override;

    void    setDebugMode(int debugMode) override;

    int     getDebugMode() const override
    {
        return m_debugMode;
    }

};

#endif//GL_DEBUG_DRAWER_H
