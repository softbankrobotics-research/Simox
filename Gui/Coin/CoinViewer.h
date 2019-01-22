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
* @author     Peter Kaiser, Adrian Knobloch
* @copyright  2016,2017 Peter Kaiser, Adrian Knobloch
*             GNU Lesser General Public License
*
*/

#pragma once

#include "../AbstractViewer.h"

#include <VirtualRobot/Visualization/CoinVisualization/CoinSelectionGroup.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/draggers/SoDragger.h>

#include "SoGLHighlightRenderAction.h"

#include <unordered_set>

namespace SimoxGui
{
    class SIMOX_GUI_IMPORT_EXPORT CoinViewer : public AbstractViewer, public SoQtExaminerViewer
    {
    public:
        CoinViewer(QWidget *parent);
        ~CoinViewer() override;

        virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected() const override;
        virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected(const std::string &layer, bool recursive=true) const override;

        std::vector<VirtualRobot::SelectionGroupPtr> getAllSelectedGroups() const override;

        virtual QImage getScreenshot() const override;

        virtual void resetView() override;
        virtual void viewAll() override;

        virtual void setAntialiasing(unsigned short quality) override;
        virtual unsigned short getAntialiasing() const override;

        virtual void setBackgroundColor(const VirtualRobot::Visualization::Color& color) override;
        virtual VirtualRobot::Visualization::Color getBackgroundColor() const override;

        void setCameraConfiguration(const CameraConfigurationPtr& config) override;
        CameraConfigurationPtr getCameraConfiguration() const override;

        void setMutex(std::shared_ptr<std::recursive_mutex> m) override;

    protected:
        virtual void _addVisualization(const VirtualRobot::VisualizationPtr &visualization) override;
        virtual void _removeVisualization(const VirtualRobot::VisualizationPtr &visualization) override
        {
            _removeVisualization(visualization, nullptr);
        }
        bool _removeVisualization(const VirtualRobot::VisualizationPtr &visualization, const VirtualRobot::SelectionGroupPtr& group);


        /*!
        * \brief actualRedraw Reimplement the redraw method in order to lock engine mutex
        */
        void actualRedraw(void) override;

        QWidget *parent;

        SoSeparator *sceneSep;
        SoUnits *unitNode;
        SoSelection* selectionNode;
        struct SelectionGroupData
        {
            SoSeparator* node;
            size_t selectionChangedCallbackId;
            SoDragger* dragger;
            size_t manipulatorSetCallbackId;
        };
        std::map<std::shared_ptr<VirtualRobot::CoinSelectionGroup>, SelectionGroupData> selectionGroups;
        void setDragger(SelectionGroupData &data, VirtualRobot::ManipulatorType t, const VirtualRobot::SelectionGroupPtr &selectionGroup);

        static void manipulatorStartCallback(void* userdata, SoDragger* dragger);
        static void manipulatorValueChangedCallback(void* userdata, SoDragger* dragger);
        static void manipulatorFinishCallback(void* userdata, SoDragger* dragger);

        VirtualRobot::Visualization::Color backgroundColor;

        size_t selectionGroupChangedCallbackId;

        SoGLHighlightRenderAction* highlightRenderAction;
    };
    typedef std::shared_ptr<CoinViewer> CoinViewerPtr;
}
