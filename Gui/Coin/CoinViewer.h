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

#ifndef _Gui_CoinViewer_h_
#define _Gui_CoinViewer_h_

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>

class CoinViewer : public SoQtExaminerViewer
{
    public:
        CoinViewer(QWidget *parent);
        ~CoinViewer();

        SoSeparator *getScene();

    protected:
        SoSeparator *sceneSep;
};

#endif 
