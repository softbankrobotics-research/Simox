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

#ifndef _Gui_Qt3DViewer_h_
#define _Gui_Qt3DViewer_h_

#include "../ViewerInterface.h"
#include "Qt3DCustomCameraController.h"

#include <unordered_set>
#include <QWidget>
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QEntity>
#include <Qt3DCore/QNode>
#include <Qt3DRender/QRenderCapture>

namespace VirtualRobot {
    class Qt3DSelectionGroup;
}

namespace SimoxGui
{

    class SIMOX_GUI_IMPORT_EXPORT Qt3DViewer : public ViewerInterface, public Qt3DExtras::Qt3DWindow
    {
    public:

        Qt3DViewer(QWidget *parent);
        ~Qt3DViewer();

        virtual void addVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) override;
        virtual void removeVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) override;
        virtual std::vector<VirtualRobot::VisualizationPtr> getAllVisualizations() const override;
        virtual std::vector<VirtualRobot::VisualizationPtr> getAllVisualizations(const std::string &layer) const override;
        virtual bool hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const override;
        virtual bool hasVisualization(const std::string &layer, const VirtualRobot::VisualizationPtr &visualization) const override;
        virtual void clearLayer(const std::string &layer) override;
        virtual bool hasLayer(const std::string &layer) const override;
        virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected() const override;
        virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected(const std::string &layer) const override;
        virtual QImage getScreenshot() const override;
        virtual void resetView() override;
        virtual void viewAll() override;
        virtual void setAntialiasing(unsigned short quality) override;
        virtual unsigned short getAntialiasing() const override;
        virtual void setBackgroundColor(const VirtualRobot::Visualization::Color &color) override;
        virtual VirtualRobot::Visualization::Color getBackgroundColor() const override;

    private:
        struct Layer
        {
            Layer();
            ~Layer();

            void addVisualization(const VirtualRobot::VisualizationPtr& visualization);
            bool removeVisualization(const VirtualRobot::VisualizationPtr& visualization);
            bool hasVisualization(const VirtualRobot::VisualizationPtr& visualization) const;

            void clear();

            std::unordered_set<VirtualRobot::VisualizationPtr> visualizations;
        };
        Layer& requestLayer(const std::string& name);

        void _addVisualization(const VirtualRobot::VisualizationPtr &visualization);
        void _removeVisualization(const VirtualRobot::VisualizationPtr &visualization);

        std::map<std::string, Layer> layers;

        QWidget* parent;
        Qt3DCore::QEntity* scene;
        Qt3DCustomCameraController* camController;
        Qt3DRender::QRenderCapture* capture;
        VirtualRobot::Visualization::Color backgroundColor;

        struct SelectionGroupData
        {
            size_t selectionChangedCallbackId;
        };
        std::map<std::shared_ptr<VirtualRobot::Qt3DSelectionGroup>, SelectionGroupData> selectionGroups;
    };
    typedef std::shared_ptr<Qt3DViewer> Qt3DViewerPtr;
}

#endif 
