/*!
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
#ifndef _Gui_Qt3DCustomCameraController_h_
#define _Gui_Qt3DCustomCameraController_h_

#include "../SimoxGuiImportExport.h"

#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DInput/QKeyboardDevice>
#include <Qt3DInput/QKeyboardHandler>
#include <Qt3DInput/QMouseDevice>
#include <Qt3DInput/QMouseHandler>
#include <Qt3DLogic/QFrameAction>

namespace SimoxGui
{
    class SIMOX_GUI_IMPORT_EXPORT Qt3DCustomCameraController : public Qt3DCore::QEntity
    {
    public:
        Qt3DCustomCameraController(Qt3DCore::QNode *parent = nullptr);
        ~Qt3DCustomCameraController();

        void setCamera(Qt3DRender::QCamera* camera);

    private:
        Qt3DRender::QCamera* camera;

        Qt3DInput::QKeyboardDevice *keyboardDevice;
        Qt3DInput::QKeyboardHandler *keyboardHandler;
        Qt3DInput::QMouseDevice *mouseDevice;
        Qt3DInput::QMouseHandler *mouseHandler;
        Qt3DLogic::QFrameAction *frameAction;

        int posX;
        int posY;
        bool pressed;
    };
}

#endif
