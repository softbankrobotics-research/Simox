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
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#include "CoinViewer.h"

#include <Inventor/actions/SoLineHighlightRenderAction.h>

CoinViewer::CoinViewer(QWidget *parent) :
    SoQtExaminerViewer(parent, "", true, BUILD_POPUP)
{
    sceneSep = new SoSeparator;
    sceneSep->ref();

    setBackgroundColor(SbColor(1, 1, 1));
    setAccumulationBuffer(true);
    setAntialiasing(true, 4);
    setGLRenderAction(new SoLineHighlightRenderAction);
    setFeedbackVisibility(true);
    setSceneGraph(sceneSep);

    viewAll();
}

CoinViewer::~CoinViewer()
{
    sceneSep->unref();
}

SoSeparator *CoinViewer::getScene()
{
    return sceneSep;
}
