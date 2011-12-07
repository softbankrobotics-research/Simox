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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_TriMeshModel_h_
#define _VirtualRobot_TriMeshModel_h_

#include "../VirtualRobotImportExport.h"
#include "../MathTools.h"
#include "../BoundingBox.h"
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace VirtualRobot {

class VIRTUAL_ROBOT_IMPORT_EXPORT TriMeshModel
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3);
	void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, Eigen::Vector3f& normal);
	static Eigen::Vector3f CreateNormal(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3);
	void addFace(const MathTools::TriangleFace& face);
	void addVertex(const Eigen::Vector3f& vertex);
	void clear();
	void flipVertexOrientations();

	void print();
	Eigen::Vector3f getCOM();
	bool getSize(Eigen::Vector3f &storeMinSize, Eigen::Vector3f &storeMaxSize);
	bool checkFacesHaveSameEdge(const MathTools::TriangleFace& face1, const MathTools::TriangleFace& face2, std::vector<std::pair<int, int> >& commonVertexIds) const;
	unsigned int checkAndCorrectNormals(bool inverted);

	std::vector<Eigen::Vector3f> vertices;
	std::vector<MathTools::TriangleFace> faces;
	BoundingBox boundingBox;
};
} // namespace VirtualRobot

#endif /* _VirtualRobot_TriMeshModel_h_ */
