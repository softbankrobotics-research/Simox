#include "AbstractViewer.h"

#include <set>

SimoxGui::AbstractViewer::AbstractViewer(const std::shared_ptr<std::recursive_mutex>& m) :
    mutex(m),
    baseLayer(this),
    layerSeparator('/')
{
}

SimoxGui::AbstractViewer::~AbstractViewer()
{
    auto visus = getAllVisualizations();
    for (const auto& v : visus)
    {
        removeVisualization(v);
    }
}

void SimoxGui::AbstractViewer::addVisualization(const VirtualRobot::VisualizationPtr &visualization, const std::string &layer)
{
    auto l = getScopedLock();
    visualization->addMutex(mutex);
    requestLayer(layer).addVisualization(visualization);
}

void SimoxGui::AbstractViewer::addVisualizations(const std::vector<VirtualRobot::VisualizationPtr> &visualizations, const std::string &layer)
{
    for (const auto& visu : visualizations)
    {
        addVisualization(visu, layer);
    }
}

void SimoxGui::AbstractViewer::removeVisualization(const VirtualRobot::VisualizationPtr &visualization, const std::string &layer)
{
    auto l = getScopedLock();
    requestLayer(layer).removeVisualization(visualization);
    visualization->removeMutex(mutex);
}

void SimoxGui::AbstractViewer::removeVisualizationOnAllLayers(const VirtualRobot::VisualizationPtr &visualization)
{
    auto l = getScopedLock();
    removeVisualizationOnAllLayers(baseLayer, visualization);
    visualization->removeMutex(mutex);
}

bool SimoxGui::AbstractViewer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization) const
{
    auto l = getScopedLock();
    return baseLayer.hasVisualization(visualization, true);
}

bool SimoxGui::AbstractViewer::hasVisualization(const VirtualRobot::VisualizationPtr &visualization, const std::string &layer, bool recursive) const
{
    auto lock = getScopedLock();
    std::vector<std::string> subLayers = splitLayerString(layer);
    const Layer* l = &baseLayer;
    for (const auto& subL : subLayers)
    {
        const auto it = l->childLayers.find(subL);
        if (it != l->childLayers.end())
        {
            l = &it->second;
        }
        else
        {
            return false;
        }
    }
    return l->hasVisualization(visualization, recursive);
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::AbstractViewer::getAllVisualizations() const
{
    auto l = getScopedLock();
    std::vector<VirtualRobot::VisualizationPtr> visus;
    getAllVisualizations(baseLayer, visus);
    return visus;
}

std::vector<VirtualRobot::VisualizationPtr> SimoxGui::AbstractViewer::getAllVisualizations(const std::string &layer, bool recursive) const
{
    auto lock = getScopedLock();
    std::vector<std::string> subLayers = splitLayerString(layer);
    const Layer* l = &baseLayer;
    for (const auto& subL : subLayers)
    {
        const auto it = l->childLayers.find(subL);
        if (it != l->childLayers.end())
        {
            l = &it->second;
        }
        else
        {
            return std::vector<VirtualRobot::VisualizationPtr>();
        }
    }
    if (recursive)
    {
        std::vector<VirtualRobot::VisualizationPtr> visus;
        getAllVisualizations(*l, visus);
        return visus;
    }
    else
    {
        return l->visualizations;
    }
}

std::vector<VirtualRobot::SelectionGroupPtr> SimoxGui::AbstractViewer::getAllGroups() const
{
    auto visus = getAllVisualizations();
    std::set<VirtualRobot::SelectionGroupPtr> g;
    for (const auto& v : visus)
    {
        g.insert(v->getSelectionGroup());
    }
    return std::vector<VirtualRobot::SelectionGroupPtr>(g.begin(), g.end());
}

void SimoxGui::AbstractViewer::addLayer(const std::string &layer)
{
    auto l = getScopedLock();
    requestLayer(layer);
}

void SimoxGui::AbstractViewer::removeLayer(const std::string &layer)
{
    auto lock = getScopedLock();
    std::vector<std::string> subLayers = splitLayerString(layer);
    Layer* l = &baseLayer;
    std::string lastName = subLayers.back();
    subLayers.pop_back();
    for (const auto& subL : subLayers)
    {
        auto it = l->childLayers.find(subL);
        if (it != l->childLayers.end())
        {
            l = &it->second;
        }
        else
        {
            return;
        }
    }
    auto it = l->childLayers.find(lastName);
    if (it != l->childLayers.end())
    {
        l->childLayers.erase(it);
    }
}

void SimoxGui::AbstractViewer::removeAllLayers()
{
    auto l = getScopedLock();
    baseLayer.clear(true);
}

bool SimoxGui::AbstractViewer::hasLayer(const std::string &layer) const
{
    auto lock = getScopedLock();
    std::vector<std::string> subLayers = splitLayerString(layer);
    const Layer* l = &baseLayer;
    for (const auto& subL : subLayers)
    {
        auto it = l->childLayers.find(subL);
        if (it != l->childLayers.end())
        {
            l = &it->second;
        }
        else
        {
            return false;
        }
    }
    return true;
}

void SimoxGui::AbstractViewer::clearLayer(const std::string &layer, bool recursive)
{
    auto lock = getScopedLock();
    std::vector<std::string> subLayers = splitLayerString(layer);
    Layer* l = &baseLayer;
    for (const auto& subL : subLayers)
    {
        auto it = l->childLayers.find(subL);
        if (it != l->childLayers.end())
        {
            l = &it->second;
        }
        else
        {
            return;
        }
    }
    l->clear(recursive);
}

void SimoxGui::AbstractViewer::setLayerSeparator(char sep)
{
    auto lock = getScopedLock();
    layerSeparator = sep;
}

char SimoxGui::AbstractViewer::getLayerSeparator() const
{
    auto lock = getScopedLock();
    return layerSeparator;
}

std::shared_ptr<std::lock_guard<std::recursive_mutex> > SimoxGui::AbstractViewer::getScopedLock() const
{
    std::shared_ptr<std::lock_guard<std::recursive_mutex>> l(nullptr);
    if (mutex)
    {
        l.reset(new std::lock_guard<std::recursive_mutex>(*mutex));
    }

    return l;
}

SimoxGui::AbstractViewer::Layer &SimoxGui::AbstractViewer::requestLayer(const std::string &layer)
{
    std::vector<std::string> subLayers = splitLayerString(layer);
    Layer* l = &baseLayer;
    for (const auto& subL : subLayers)
    {
        auto it = l->childLayers.find(subL);
        if (it != l->childLayers.end())
        {
            l = &it->second;
        }
        else
        {
            l->childLayers.emplace(subL, Layer(this));
            l = &l->childLayers.find(subL)->second;
        }
    }
    return *l;
}

void SimoxGui::AbstractViewer::removeVisualizationOnAllLayers(SimoxGui::AbstractViewer::Layer &l, const VirtualRobot::VisualizationPtr &visualization)
{
    l.removeVisualization(visualization);
    for (auto& childL : l.childLayers)
    {
        removeVisualizationOnAllLayers(childL.second, visualization);
    }
}

void SimoxGui::AbstractViewer::getAllVisualizations(const SimoxGui::AbstractViewer::Layer &l, std::vector<VirtualRobot::VisualizationPtr> &visus) const
{
    visus.insert(visus.end(), l.visualizations.begin(), l.visualizations.end());
    for (const auto& childL : l.childLayers)
    {
        getAllVisualizations(childL.second, visus);
    }
}

void SimoxGui::AbstractViewer::addVisualizationP(const VirtualRobot::VisualizationPtr &visu)
{
    VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visu);
    if (set)
    {
        auto& infos = visualizationSetInfos[set];
        infos.addedCallbackId = set->addVisualizationAddedCallback([this](const VirtualRobot::VisualizationPtr& visu)
        {
            addVisualizationP(visu);
        });
        infos.removedCallbackId = set->addVisualizationRemovedCallback([this](const VirtualRobot::VisualizationPtr& visu)
        {
            removeVisualizationP(visu);
        });
        for (const auto& v : set->getVisualizations())
        {
            addVisualizationP(v);
        }
    }
    else
    {
        _addVisualization(visu);
    }
}

void SimoxGui::AbstractViewer::removeVisualizationP(const VirtualRobot::VisualizationPtr &visu)
{
    VirtualRobot::VisualizationSetPtr set = std::dynamic_pointer_cast<VirtualRobot::VisualizationSet>(visu);
    if (set)
    {
        auto it = visualizationSetInfos.find(set);
        VR_ASSERT(it != visualizationSetInfos.end());
        set->removeVisualizationAddedCallback(it->second.addedCallbackId);
        set->removeVisualizationRemovedCallback(it->second.removedCallbackId);
        visualizationSetInfos.erase(it);
        for (const auto& v : set->getVisualizations())
        {
            removeVisualizationP(v);
        }
    }
    else
    {
        _removeVisualization(visu);
    }
}

std::vector<std::string> SimoxGui::AbstractViewer::splitLayerString(const std::string &layer) const
{
    std::vector<std::string> subLayers;
    std::stringstream ss(layer);
    std::string token;
    while (std::getline(ss, token, layerSeparator)) {
        subLayers.push_back(token);
    }
    return subLayers;
}
