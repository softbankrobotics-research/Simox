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
* @copyright  2016 Peter Kaiser
*             GNU Lesser General Public License
*
*/

#pragma once

#include "SimoxGuiImportExport.h"

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Visualization/Visualization.h>
#include <VirtualRobot/Visualization/VisualizationSet.h>

#include <QtGui/QtGui>
#include <QImage>

#include <string>

namespace SimoxGui
{

class SIMOX_GUI_IMPORT_EXPORT AbstractViewer
{
public:
    AbstractViewer();
    void addVisualization(const VirtualRobot::VisualizationPtr &visualization, const std::string &layer = "");
    inline void addVisualizations(const std::vector<VirtualRobot::VisualizationPtr> &visualizations, const std::string &layer = "");
    void removeVisualization(const VirtualRobot::VisualizationPtr &visualization, const std::string &layer = "");
    void removeVisualizationOnAllLayers(const VirtualRobot::VisualizationPtr &visualization);

    bool hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const;
    bool hasVisualization(const VirtualRobot::VisualizationPtr &visualization, const std::string &layer, bool recursive=true) const;

    std::vector<VirtualRobot::VisualizationPtr> getAllVisualizations() const;
    std::vector<VirtualRobot::VisualizationPtr> getAllVisualizations(const std::string &layer, bool recursive=true) const;

    void addLayer(const std::string& layer);
    void removeLayer(const std::string& layer);
    void removeAllLayer();
    bool hasLayer(const std::string &layer) const;

    void clearLayer(const std::string &layer, bool recursive=false);

    void setLayerSeparator(char sep);
    char getLayerSeparator() const;

    virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected() const = 0;
    virtual std::vector<VirtualRobot::VisualizationPtr> getAllSelected(const std::string &layer, bool recursive=true) const = 0;

    virtual QImage getScreenshot() const = 0;

    virtual void resetView() = 0;
    virtual void viewAll() = 0;

    virtual void setAntialiasing(unsigned short quality) = 0;
    virtual unsigned short getAntialiasing() const = 0;

    virtual void setBackgroundColor(const VirtualRobot::Visualization::Color& color) = 0;
    virtual VirtualRobot::Visualization::Color getBackgroundColor() const = 0;

    /*!
     * If set, the drawing is protected by this mutex. This overwrites the default mutex.
     */
    void setMutex(std::shared_ptr<std::recursive_mutex> m);


    /*!
     * \return This lock allows to safely access the viewer's scene graph.
     */
    std::shared_ptr<std::lock_guard<std::recursive_mutex>> getScopedLock();

protected:

    std::shared_ptr<std::recursive_mutex> mutex;
    virtual void _addVisualization(const VirtualRobot::VisualizationPtr &visualization) = 0;
    virtual void _removeVisualization(const VirtualRobot::VisualizationPtr &visualization) = 0;

    struct Layer
    {
        Layer(AbstractViewer* viewer) : viewer(viewer) {}
        ~Layer()
        {
            clear(true);
        }
        void addVisualization(const VirtualRobot::VisualizationPtr &visualization)
        {
            viewer->addVisualizationP(visualization);
            visualizations.push_back(visualization);
        }
        void removeVisualization(const VirtualRobot::VisualizationPtr &visualization)
        {
            auto it = std::find(visualizations.begin(), visualizations.end(), visualization);
            if (it != visualizations.end())
            {
                viewer->removeVisualizationP(visualization);
                visualizations.erase(it);
            }
        }
        bool hasVisualization(const VirtualRobot::VisualizationPtr &visualization, bool recursive) const
        {
            auto it = std::find(visualizations.begin(), visualizations.end(), visualization);
            if (it != visualizations.end())
            {
                return true;
            }
            if (recursive)
            {
                for (const auto& l : childLayers)
                {
                    if (l.second.hasVisualization(visualization, true))
                    {
                        return true;
                    }
                }
            }
            return false;
        }
        void clear(bool recursive=true)
        {
            for (const auto& visu : visualizations)
            {
                viewer->removeVisualizationP(visu);
            }
            visualizations.clear();
            if (recursive)
            {
                for (auto& l : childLayers)
                {
                    l.second.clear(true);
                }
                childLayers.clear();
            }
        }

        std::vector<VirtualRobot::VisualizationPtr> visualizations;
        std::map<std::string, Layer> childLayers;
        AbstractViewer* viewer;
    };

    Layer& requestLayer(const std::string& layer);

    void removeVisualizationOnAllLayers(Layer& l, const VirtualRobot::VisualizationPtr &visualization);

    void getAllVisualizations(const Layer& l, std::vector<VirtualRobot::VisualizationPtr>& visus) const;

    void addVisualizationP(const VirtualRobot::VisualizationPtr& visu);

    void removeVisualizationP(const VirtualRobot::VisualizationPtr& visu);

    std::vector<std::string> splitLayerString(const std::string& layer) const;


    struct VisualizationInfo
    {
        size_t addedCallbackId;
        size_t removedCallbackId;
    };
    std::map<VirtualRobot::VisualizationSetPtr, VisualizationInfo> visualizationSetInfos;
    Layer baseLayer;
    char layerSeparator;

};
typedef std::shared_ptr<AbstractViewer> AbstractViewerPtr;

}
