/**
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*/

#include "Qt3DVisualization.h"
#include "Qt3DVisualizationSet.h"
#include "Qt3DVisualizationFactory.h"

namespace VirtualRobot
{
    Qt3DVisualizationSet::Qt3DVisualizationSet(const std::vector<VisualizationPtr>& visualizations) : VisualizationSet(visualizations)
    {
    }

    std::string Qt3DVisualizationSet::toXML(const std::string &basePath, int tabs) const
    {
        VR_ERROR_ONCE_NYI;
    }

    std::string Qt3DVisualizationSet::toXML(const std::string &basePath, const std::string &filename, int tabs) const
    {
        VR_ERROR_ONCE_NYI;
    }

    bool Qt3DVisualizationSet::saveModel(const std::string &modelPath, const std::string &filename)
    {
        VR_ERROR_ONCE_NYI;
    }
}
