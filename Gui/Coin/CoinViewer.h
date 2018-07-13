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

#ifndef _Gui_CoinViewer_h_
#define _Gui_CoinViewer_h_

#include "../ViewerInterface.h"

#include <VirtualRobot/Visualization/CoinVisualization/CoinSelectionGroup.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSelection.h>

#include <unordered_set>

namespace SimoxGui
{
    class SIMOX_GUI_IMPORT_EXPORT CoinViewer : public ViewerInterface, public SoQtExaminerViewer
    {
    public:
        CoinViewer(QWidget *parent);
        ~CoinViewer();

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

        virtual void setBackgroundColor(const VirtualRobot::Visualization::Color& color) override;
        virtual VirtualRobot::Visualization::Color getBackgroundColor() const override;

    protected:
        struct Layer
        {
            Layer();
            ~Layer();

            void addVisualization(const VirtualRobot::VisualizationPtr& visu);
            bool removeVisualization(const VirtualRobot::VisualizationPtr& visu);
            bool hasVisualization(const VirtualRobot::VisualizationPtr& visu) const;

            void clear();

            std::unordered_set<VirtualRobot::VisualizationPtr> visualizations;
        };

        Layer& requestLayer(const std::string& name);

        void _addVisualization(const VirtualRobot::VisualizationPtr &visualization);
        void _removeVisualization(const VirtualRobot::VisualizationPtr &visualization);

        QWidget *parent;

        SoSeparator *sceneSep;
        SoUnits *unitNode;
        SoSelection* selectionNode;
        std::map<std::string, Layer> layers;
        struct SelectionGroupData
        {
            SoSeparator* node;
            size_t visuAddedCallbackId;
            size_t selectionChangedCallbackId;
        };
        std::map<std::shared_ptr<VirtualRobot::CoinSelectionGroup>, SelectionGroupData> selectionGroups;

        VirtualRobot::Visualization::Color backgroundColor;
    };
    typedef std::shared_ptr<CoinViewer> CoinViewerPtr;
}

#endif 
