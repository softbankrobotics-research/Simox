/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    Gui
* @author     Philipp Schmidt
* @copyright  2018 Philipp Schmidt
*             GNU Lesser General Public License
*
*/

#include "Qt3DCustomCameraController.h"
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
