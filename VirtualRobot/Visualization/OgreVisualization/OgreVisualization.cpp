/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2016 Nikolaus Vahrenkamp
*/


#include "OgreVisualization.h"
#include "OgreVisualizationNode.h"

#include <algorithm>



namespace VirtualRobot
{

    OgreVisualization::OgreVisualization(const VisualizationNodePtr visualizationNode) :
        Visualization(visualizationNode)
    {
        selection = NULL;
        color = NULL;
    }

    OgreVisualization::OgreVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes) :
        Visualization(visualizationNodes)
    {
        selection = NULL;
        color = NULL;
    }

    OgreVisualization::~OgreVisualization()
    {
        // todo !!!
        /*if (selection)
        {
            selection->unref();
        }
        if (color)
        {
            color->unref();
        }*/
    }

    bool OgreVisualization::buildVisualization()
    {
        // todo !!!
        /*
        if (selection)
        {
            return true;
        }

        selection = new SoSelection;
        selection->ref();
        selection->policy = SoSelection::TOGGLE;
        SoSeparator* visualization = new SoSeparator();

        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        visualization->addChild(u);

        color = new SoMaterial();
        visualization->addChild(color);

        selection->addChild(visualization);

        BOOST_FOREACH(VisualizationNodePtr visualizationNode, visualizationNodes)
        {
            boost::shared_ptr<OgreVisualizationNode> OgreVisualizationNode = boost::dynamic_pointer_cast<OgreVisualizationNode>(visualizationNode);

            if (OgreVisualizationNode && OgreVisualizationNode->getOgreVisualization())
            {
                visualization->addChild(OgreVisualizationNode->getOgreVisualization());
            }
        }*/
        return true;
    }

    bool OgreVisualization::highlight(unsigned int which, bool enable)
    {
        /*
        if (which >= visualizationNodes.size())
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        return highlight(visualizationNodes[which], enable);
        */
        return false;
    }


    bool OgreVisualization::highlight(VisualizationNodePtr visualizationNode, bool enable)
    {
        /*if (!selection)
        {
            return false;
        }

        if (!isVisualizationNodeRegistered(visualizationNode))
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        boost::shared_ptr<OgreVisualizationNode> OgreVisualizationNode = boost::dynamic_pointer_cast<OgreVisualizationNode>(visualizationNode);


        if (OgreVisualizationNode)
        {
            return highlight(OgreVisualizationNode->getOgreVisualization(), enable);
        }*/

        return false;
    }


    void* OgreVisualization::getOgreVisualization()
    {
        buildVisualization();
        return selection;
    }

    /**
     * \return new instance of VirtualRobot::OgreVisualization with the same set of robot nodes.
     */
    VirtualRobot::VisualizationPtr OgreVisualization::clone()
    {
        return VisualizationPtr(new OgreVisualization(visualizationNodes));
    }

    void OgreVisualization::colorize(VisualizationFactory::Color c)
    {
        /*
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
        * */
    }

    void OgreVisualization::setTransparency(float transparency)
    {
        /*
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
        * */
    }

} // namespace VirtualRobot
