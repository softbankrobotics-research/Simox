#include "Qt3DCustomCameraController.h"

#include <iostream>

#include <Qt3DRender/QCamera>

SimoxGui::Qt3DCustomCameraController::Qt3DCustomCameraController(Qt3DCore::QNode *parent)
    :QAbstractCameraController(parent)
{
}

void SimoxGui::Qt3DCustomCameraController::moveCamera(const Qt3DExtras::QAbstractCameraController::InputState &state, float dt)
{
    Qt3DRender::QCamera *theCamera = camera();

    if (theCamera == nullptr)
        return;

    if (state.leftMouseButtonActive)
    {
        if(state.shiftKeyActive || state.altKeyActive)
        {
            // Translate
            theCamera->translate(QVector3D(-1.0f * state.rxAxisValue * linearSpeed(), -1.0f * state.ryAxisValue * linearSpeed(), 0) * dt);
        }
        else
        {
            // Orbit
            theCamera->panAboutViewCenter(-1.0f * (state.rxAxisValue * lookSpeed()) * dt);
            theCamera->tiltAboutViewCenter(-1.0f * (state.ryAxisValue * lookSpeed()) * dt);
        }
    }
    else if(state.rightMouseButtonActive)
    {
        theCamera->rollAboutViewCenter(-1.0f * (state.ryAxisValue * lookSpeed()) * dt);
    }
    else if(state.middleMouseButtonActive)
    {
        theCamera->viewAll();
    }

    // Translate
    theCamera->translate(QVector3D(0, 0, state.tzAxisValue * linearSpeed() * 1.5f) * dt, theCamera->DontTranslateViewCenter);
}
