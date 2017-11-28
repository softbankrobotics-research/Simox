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
* @package    MotionPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _MotionPlanning_RrtWorkspaceVisualization_h_
#define _MotionPlanning_RrtWorkspaceVisualization_h_

#include "../MotionPlanning.h"
#include "VirtualRobot/Model/Model.h"
#include "VirtualRobot/Model/Model.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace MotionPlanning
{

    /*!
     *
     * A visualization of an RRT search tree.
     * @see CoinRrtWorkspaceVisualization
     *
     */
    class MOTIONPLANNING_IMPORT_EXPORT RrtWorkspaceVisualization
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            Robot must have a node with name TCPName.
            The visualizations are build by determining the TCP's position in workspace according to the configurations of a path or tree .
        */
        RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, CSpacePtr cspace, const std::string& TCPName);
        RrtWorkspaceVisualization(VirtualRobot::RobotPtr robot, VirtualRobot::JointSetPtr robotNodeSet, const std::string& TCPName);

        enum ColorSet
        {
            eRed,
            eGreen,
            eBlue,
            eCustom
        };

        /*!
        */
        virtual ~RrtWorkspaceVisualization();

        /*!
            Clears all visualizations.
        */
        virtual void reset();

        /*!
            Set name of TCP joint. Does not affect already added paths or trees.
        */
        void setTCPName(const std::string TCPName);


        /*!
            Add visualization of a path in cspace.
        */
        virtual bool addCSpacePath(const CSpacePathPtr& path, RrtWorkspaceVisualization::ColorSet colorSet = eBlue);
        virtual void setPathStyle(float lineSize = 4.0f, float nodeSize = 15.0f, float /*renderComplexity*/ = 1.0f);

        /*!
            Add visualization of a tree (e.g an RRT) in cspace.
        */
        virtual bool addTree(const CSpaceTreePtr &tree, RrtWorkspaceVisualization::ColorSet colorSet = eRed);
        virtual void setTreeStyle(float lineSize = 1.0f, float nodeSize = 15.0f, float /*renderComplexity*/ = 0.1f);

        /*!
            Add visualization of a configuration in cspace.
        */
        virtual bool addConfiguration(const Eigen::VectorXf& c, RrtWorkspaceVisualization::ColorSet colorSet = eGreen, float nodeSizeFactor = 1.0f);

        /*!
            Set the custom line and node color. Does not affect already added trees or paths.
        */
        virtual void setCustomColor(float nodeR, float nodeG, float nodeB, float lineR = 0.5f, float lineG = 0.5f, float lineB = 0.5f);

        /*!
            Set tree nodes with status flag equal to given parameter to the specified color.
        */
        virtual void colorizeTreeNodes(int status, ColorSet colorSet);

        virtual VirtualRobot::VisualizationSetPtr getVisualization()
        {
            return visualization;
        }

    protected:
        void init(); // is called by constructor

        VirtualRobot::RobotPtr robot;
        CSpacePtr cspace;
        VirtualRobot::JointSetPtr robotNodeSet;
        VirtualRobot::RobotNodePtr TCPNode;

        std::string TCPName;

        float pathLineSize, pathNodeSize;
        float treeLineSize, treeNodeSize;

        struct RenderColors
        {
            float nodeR, nodeG, nodeB, lineR, lineG, lineB;
        };

        std::map<ColorSet, RenderColors> colors;
        std::map<int, ColorSet> treeNodeStatusColor;

        VirtualRobot::VisualizationSetPtr visualization;
    };

    typedef std::shared_ptr<RrtWorkspaceVisualization> RrtWorkspaceVisualizationPtr;

} // namespace MotionPlanning

#endif // _MotionPlanning_RrtWorkspaceVisualization_h_
