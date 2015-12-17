
#include "VirtualRobot.h"
#include "Visualization/VisualizationFactory.h"

namespace VirtualRobot
{
    std::string globalAppName;

    void init(int &argc, char* argv[], const std::string &appName)
    {
        globalAppName = appName;
        boost::shared_ptr<VisualizationFactory> v = VisualizationFactory::first(NULL);
        if (v)
        {
            v->init(argc, argv, globalAppName);
        }
    }

}
