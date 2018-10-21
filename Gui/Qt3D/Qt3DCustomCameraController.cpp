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
#include <cmath>

SimoxGui::Qt3DCustomCameraController::Qt3DCustomCameraController(QSize windowSize, float sensitivity, Qt3DCore::QNode *parent)
    : Qt3DCore::QEntity(parent)
    , keyboardDevice(new Qt3DInput::QKeyboardDevice(this))
    , keyboardHandler(new Qt3DInput::QKeyboardHandler(this))
    , mouseDevice(new Qt3DInput::QMouseDevice(this))
    , mouseHandler(new Qt3DInput::QMouseHandler(this))
    , frameAction(new Qt3DLogic::QFrameAction(this))
    , pressed(false)
    , posX(0)
    , posY(0)
    , pantiltrollratio(0.0f)
    , sensitivity(sensitivity)
    , windowSize(windowSize)
    , leftOfCenter(false)
{
    keyboardHandler->setSourceDevice(keyboardDevice);
    keyboardHandler->setFocus(true);
    QObject::connect(keyboardHandler, &Qt3DInput::QKeyboardHandler::pressed,
                     this, [=] (Qt3DInput::QKeyEvent *event) {
    });
    QObject::connect(keyboardHandler, &Qt3DInput::QKeyboardHandler::released,
                     this, [=] (Qt3DInput::QKeyEvent *event) {
    });

    mouseHandler->setSourceDevice(mouseDevice);
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::pressed,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
        posX = event->x();
        posY = event->y();

        int xDiffToCenter = abs(posX - (this->windowSize.width() / 2));
        int yDiffToCenter = abs(posY - (this->windowSize.height() / 2));
        float diffVecLenght = sqrt(xDiffToCenter * xDiffToCenter + yDiffToCenter * yDiffToCenter);
        float windowLength = sqrt(this->windowSize.width() * this->windowSize.width() + this->windowSize.height() * this->windowSize.height()) / 2;
        pantiltrollratio = std::min(1.0f, diffVecLenght / windowLength);
        leftOfCenter = posX <= (this->windowSize.width() / 2);

        pressed = true;
    });
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::released,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
        pressed = false;
    });
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::pressAndHold,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
    });
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::positionChanged,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
        if(!pressed)
            return;

        float yaw = (event->x() - posX) * 0.5f;
        float pitch = (event->y() - posY) * 0.5f;
        pitch = (pitch > 89.0f) ? 89.0f : (pitch < - 89.0f) ? -89.0f : pitch;

        this->camera->panAboutViewCenter(-yaw * sensitivity * (1 - pantiltrollratio));
        this->camera->tiltAboutViewCenter(pitch * sensitivity * (1 - pantiltrollratio));
        this->camera->rollAboutViewCenter(pitch * sensitivity * pantiltrollratio * (leftOfCenter ? -1.0f : 1.0f));

        posY = event->y();
        posX = event->x();
    });

    this->addComponent(frameAction);
    QObject::connect(frameAction, &Qt3DLogic::QFrameAction::triggered,
                         this, [=] (float dt) {

    });
}

SimoxGui::Qt3DCustomCameraController::~Qt3DCustomCameraController()
{

}

void SimoxGui::Qt3DCustomCameraController::setCamera(Qt3DRender::QCamera *camera)
{
    if (camera && this->camera != camera)
    {
        camera->setParent(this);
        this->camera = camera;
    }
}

void SimoxGui::Qt3DCustomCameraController::setWindowSize(QSize size)
{
    this->windowSize = size;
}
