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
* @package    VirtualRobot
* @author     Stefan Ulbrich
* @copyright  2014 Stefan Ulbrich
*             GNU Lesser General Public License
*
*/
#include "RobotImporterFactory.h"
#include <Eigen/Core>
#include "../VirtualRobotException.h"

#include <boost/algorithm/string.hpp>

using namespace std;

namespace VirtualRobot
{

bool RobotImporterFactory::loadEndEffector(const RobotPtr &robot, const string &filename)
{
    if (!robot)
        return false;



    return true;
}

string RobotImporterFactory::getAllFileFilters()
    {
        vector<string> filter;
        for(string subclass: RobotImporterFactory::getSubclassList())
        {
            filter.push_back(RobotImporterFactory::fromName(subclass, nullptr)->getFileFilter());
        }
        return boost::algorithm::join(filter, ";;");
    }

    string RobotImporterFactory::getAllExtensions()
    {
        vector<string> filter;
        for(string subclass : RobotImporterFactory::getSubclassList())
        {
            string extension = RobotImporterFactory::fromName(subclass, nullptr)->getFileExtension();
            filter.push_back("*." + extension);
        }
        return boost::algorithm::join(filter, " ");
    }

    RobotImporterFactoryPtr RobotImporterFactory::fromFileExtension(string type, void* params)
    {
        for(string subclass: RobotImporterFactory::getSubclassList())
        {
            string subclassType = RobotImporterFactory::fromName(subclass, nullptr)->getFileExtension();

            if (type.compare(subclassType) == 0)
            {
                return RobotImporterFactory::fromName(subclass, params);
            }
        }
        THROW_VR_EXCEPTION(string("Unkown file extension: ") + type);
        return RobotImporterFactoryPtr();
    }

    RobotImporterFactoryPtr RobotImporterFactory::fromFilepath(const boost::filesystem::path& path, void *params)
    {
        return fromFileExtension(path.extension().string(), params);
    }


} // namespace VirtualRobot

