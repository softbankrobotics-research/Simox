/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/


#include "CoinVisualizationSet.h"
#include "CoinVisualizationNode.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>

#include <algorithm>

// For the VRML2.0 export
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/nodes/SoRotation.h>

namespace VirtualRobot
{

    CoinVisualizationSet::CoinVisualizationSet(const VisualizationNodePtr visualizationNode) :
        VisualizationSet(visualizationNode)
    {
        selection = NULL;
        visuRoot = NULL;
        color = NULL;
    }

    CoinVisualizationSet::CoinVisualizationSet(const std::vector<VisualizationNodePtr>& visualizationNodes) :
        VisualizationSet(visualizationNodes)
    {
        selection = NULL;
        visuRoot = NULL;
        color = NULL;
    }

    CoinVisualizationSet::~CoinVisualizationSet()
    {
        if (selection)
        {
            selection->unref();
        }
        if (visuRoot)
        {
            visuRoot->unref();
        }
        /*if (color)
        {
            color->unref();
        }*/
    }

    bool CoinVisualizationSet::buildVisualization()
    {
        if (selection)
        {
            return true;
        }

        selection = new SoSelection;
        selection->ref();
        selection->policy = SoSelection::TOGGLE;

        visuRoot = new SoSeparator;
        visuRoot->ref();

        if(isSelectable)
        {
            selection->addChild(visuRoot);
        }

        SoSeparator* visualization = new SoSeparator();
        //SoMatrixTransform *mtr = new SoMatrixTransform;
        /*SbMatrix m(reinterpret_cast<SbMat*>(globalPose.data()));
        mtr->matrix.setValue(m);
        selection->addChild(mtr);*/
        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        visualization->addChild(u);

        color = new SoMaterial();
        visualization->addChild(color);

        visuRoot->addChild(visualization);

        for(auto visualizationNode : visualizations)
        {
            std::shared_ptr<CoinVisualizationNode> coinVisualizationNode = std::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);

            if (coinVisualizationNode && coinVisualizationNode->getCoinVisualization())
            {
                visualization->addChild(coinVisualizationNode->getCoinVisualization());
            }
        }
        return true;
    }

    bool CoinVisualizationSet::highlight(unsigned int which, bool enable)
    {
        if (which >= visualizations.size())
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        return highlight(visualizations[which], enable);
    }

    bool CoinVisualizationSet::highlight(SoNode* visu, bool enable)
    {
        if (!visu)
        {
            return false;
        }

        if (enable)
        {
            selection->select(visu);
        }
        else
        {
            selection->deselect(visu);
        }

        selection->touch();
        return true;
    }

    bool CoinVisualizationSet::highlight(VisualizationNodePtr visualizationNode, bool enable)
    {
        if (!selection)
        {
            return false;
        }

        if (!isVisualizationNodeRegistered(visualizationNode))
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        std::shared_ptr<CoinVisualizationNode> coinVisualizationNode = std::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);


        if (coinVisualizationNode)
        {
            return highlight(coinVisualizationNode->getCoinVisualization(), enable);
        }

        return false;
    }

    bool CoinVisualizationSet::highlight(bool enable)
    {
        for (size_t i = 0; i < visualizations.size(); i++)
        {
            highlight(i, enable);
        }

        return true;
    }

    /**
     * This method iterates over the entries in member
     * CoinVisualization::visualizationNodes and stores the return value of
     * CoinVisualizationNode::getCoinVisualization() in an SoSeparator if the
     * processed node is of type CoinVisualizationNode.
     * Afterwards the SoSeparator is returned.
     */
    SoNode* CoinVisualizationSet::getCoinVisualization(bool selectable)
    {
        isSelectable = selectable;
        buildVisualization();
        return selectable? selection : visuRoot;
    }

    /**
     * \return new instance of VirtualRobot::CoinVisualization with the same set of robot nodes.
     */
    VirtualRobot::VisualizationPtr CoinVisualizationSet::clone()
    {
        return VisualizationPtr(new CoinVisualizationSet(visualizations));
    }

    void CoinVisualizationSet::colorize(Visualization::Color c)
    {

        buildVisualization();
        if (!selection || !color)
        {
            return;
        }

        if (c.isNone())
        {

            color->setOverride(FALSE);
        }
        else
        {
            color->diffuseColor = SbColor(c.r, c.g, c.b);
            color->ambientColor = SbColor(0, 0, 0);
            color->transparency = std::max(0.0f, c.transparency);
            color->diffuseColor.setIgnored(FALSE);
            color->setOverride(TRUE);
        }
    }

    void CoinVisualizationSet::setTransparency(float transparency)
    {
        buildVisualization();
        if (!selection || !color)
        {
            return;
        }

        if (transparency < 0)
            transparency = 0;
        if (transparency > 1.0f)
            transparency = 1.0f;

        if (transparency==1)
        {
            color->diffuseColor.setIgnored(FALSE);
            color->setOverride(FALSE);
        }
        else
        {
            color->transparency = transparency;
            color->diffuseColor.setIgnored(TRUE);
            color->setOverride(TRUE);
        }
    }

	void CoinVisualizationSet::exportToVRML2(std::string filename, bool useRotation)
    {

        SoSeparator* root = new SoSeparator;
        root->ref();
        SoRotation* rotationX = new SoRotation();
        rotationX->ref();
        SoRotation* rotationZ = new SoRotation();
        rotationZ->ref();

        if(useRotation)
        {
            rotationX->rotation.setValue(SbVec3f(-1, 0, 0), float(M_PI / 2));
            rotationZ->rotation.setValue(SbVec3f(0, 0, 1), float(M_PI));
            root->addChild(rotationX);
            root->addChild(rotationZ);
        }
        root->addChild(this->getCoinVisualization());

        printf("Converting...\n");
        SoToVRML2Action tovrml2;
        tovrml2.apply(root);
        SoVRMLGroup* newroot = tovrml2.getVRML2SceneGraph();
        newroot->ref();
        root->unref();
        rotationZ->unref();
        rotationX->unref();

        printf("Writing...\n");

        SoOutput out;
        out.openFile(filename.c_str());
        out.setHeaderString("#VRML V2.0 utf8");
        SoWriteAction wra(&out);
        wra.apply(newroot);
        out.closeFile();
        newroot->unref();
    }


} // namespace VirtualRobot
