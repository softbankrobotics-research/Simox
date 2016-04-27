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
        sceneNode = NULL;
        color = NULL;
        ogreRenderer = OgreRenderer::getOgreRenderer();
    }

    OgreVisualization::OgreVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes) :
        Visualization(visualizationNodes)
    {
        sceneNode = NULL;
        color = NULL;
        ogreRenderer = OgreRenderer::getOgreRenderer();
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
        if (sceneNode)
        {
            return true;
        }

        sceneNode = ogreRenderer->getSceneManager()->createSceneNode();

        //color = new SoMaterial();
        //visualization->addChild(color);
        //selection->addChild(visualization);

        BOOST_FOREACH(VisualizationNodePtr visualizationNode, visualizationNodes)
        {
            OgreVisualizationNodePtr ovn = boost::dynamic_pointer_cast<VirtualRobot::OgreVisualizationNode>(visualizationNode);

            if (ovn && ovn->getOgreVisualization())
            {
                sceneNode->addChild(ovn->getOgreVisualization());
            }
        }
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


    Ogre::SceneNode *OgreVisualization::getOgreVisualization()
    {
        buildVisualization();
        return sceneNode;
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
