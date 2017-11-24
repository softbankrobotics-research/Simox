
#include "Model/Model.h"
#include "Visualization/VisualizationFactory.h"

namespace VirtualRobot
{
    std::string globalAppName;

    void init(const std::string &appName)
    {
        // cnstruct some standard values

        static int argc = 1;
        static char** argv = new char*[1];
        argv[0] = new char[appName.length() + 1];
        strcpy(argv[0], appName.c_str());

        init(argc, argv, appName);
    }

    void init(int &argc, char* argv[], const std::string &appName)
    {
        globalAppName = appName;
        std::shared_ptr<VisualizationFactory> v = VisualizationFactory::first(nullptr);
        if (v)
        {
            v->init(argc, argv, globalAppName);
        }
    }

}
