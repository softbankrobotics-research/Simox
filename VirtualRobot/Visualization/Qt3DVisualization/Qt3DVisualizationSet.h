/*!
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
* @package    VirtualRobot
* @author     Philipp Schmidt
* @copyright  2017 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Qt3DVisualizationSet_h_
#define _VirtualRobot_Qt3DVisualizationSet_h_

#include "../VisualizationSet.h"

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT Qt3DVisualizationSet : public VisualizationSet
    {
    public:
        Qt3DVisualizationSet(const std::vector<VisualizationPtr>& visualizations);
        ~Qt3DVisualizationSet();

        virtual void setGlobalPose(const Eigen::Matrix4f &m) override;
        virtual size_t addPoseChangedCallback(std::function<void (const Eigen::Matrix4f &)> f) override;
        virtual void removePoseChangedCallback(size_t id) override;
        virtual size_t addSelectionChangedCallback(std::function<void (bool)> f) override;
        virtual void removeSelectionChangedCallback(size_t id) override;
        virtual bool hasManipulator(ManipulatorType t) const override;
        virtual std::vector<ManipulatorType> getAddedManipulatorTypes() const override;
        virtual void setFilename(const std::string &filename, bool boundingBox) override;
        virtual std::string getFilename() const override;
        virtual bool usedBoundingBoxVisu() const override;
        virtual VisualizationPtr clone() const override;
        virtual std::string toXML(const std::string &basePath, int tabs) const override;
        virtual std::string toXML(const std::string &basePath, const std::string &filename, int tabs) const override;
        virtual bool saveModel(const std::string &modelPath, const std::string &filename) override;

    protected:
        virtual void _addManipulator(ManipulatorType t) override;
        virtual void _removeManipulator(ManipulatorType t) override;
        virtual void _removeAllManipulators() override;
    };

    typedef std::shared_ptr<Qt3DVisualizationSet> Qt3DVisualizationSetPtr;
}

#endif
