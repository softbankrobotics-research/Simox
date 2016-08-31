#include "ModelNodeSet.h"
#include "VirtualRobot.h"
#include "Model.h"
#include "Nodes/ModelNode.h"


namespace VirtualRobot
{

    ModelNodeSet::ModelNodeSet(const std::string &name, ModelWeakPtr model, const std::vector<ModelNodePtr> &modelNodes,
                               const ModelNodePtr kinematicRoot, const ModelNodePtr tcp)
    {
        this->name = name;
        this->model = model;
        this->modelNodes = modelNodes;
        this->kinematicRoot = kinematicRoot;
        this->tcp = tcp;


        if (modelNodes.size() > 0)
        {
            if (!kinematicRoot)
            {
                if (ModelPtr tmp = model.lock()) this->kinematicRoot = tmp->getRootNode();
            }
            if (!tcp)
            {
                    this->tcp = modelNodes.at(modelNodes.size() - 1);
            }
        }
        else
        {
            VR_WARNING << "Initializing empty ModelNodeSet" << endl;
        }
    }
}
