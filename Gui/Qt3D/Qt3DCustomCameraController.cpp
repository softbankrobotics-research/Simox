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

SimoxGui::Qt3DCustomCameraController::Qt3DCustomCameraController(Qt3DCore::QNode *parent)
    : Qt3DCore::QEntity(parent)
    , keyboardDevice(new Qt3DInput::QKeyboardDevice(this))
    , keyboardHandler(new Qt3DInput::QKeyboardHandler(this))
    , mouseDevice(new Qt3DInput::QMouseDevice(this))
    , mouseHandler(new Qt3DInput::QMouseHandler(this))
    , frameAction(new Qt3DLogic::QFrameAction(this))
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
    });
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::released,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
    });
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::pressAndHold,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
    });
    QObject::connect(mouseHandler, &Qt3DInput::QMouseHandler::positionChanged,
                     this, [=] (Qt3DInput::QMouseEvent *event) {
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
