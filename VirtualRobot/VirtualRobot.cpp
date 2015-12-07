
#include "VirtualRobot.h"
#include "Visualization/VisualizationFactory.h"

namespace VirtualRobot
{

    void init(int argc, char* argv[], const std::string appName)
    {
        boost::shared_ptr<VisualizationFactory> v = VisualizationFactory::first(NULL);
        if (v)
        {
            v->init(argc, argv, appName);
        }
    }

}