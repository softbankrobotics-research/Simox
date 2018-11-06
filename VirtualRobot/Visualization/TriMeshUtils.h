/*
 * This file is part of ArmarX.
 *
 * Copyright (C) 2011-2017, High Performance Humanoid Technologies (H2T), Karlsruhe Institute of Technology (KIT), all rights reserved.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    ArmarX
 * @author     Mirko Waechter( mirko.waechter at kit dot edu)
 * @date       2018
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */
#pragma once

#include "TriMeshModel.h"

namespace VirtualRobot {

class TriMeshUtils
{
public:
    static void CreateBoxTriangles(std::vector<TriMeshModel::triangle> &triangles, const Eigen::Matrix4f &globalPose, float width = 50.f, float height = 50.f, float depth = 50.f);
    static std::vector<TriMeshModel::triangle> CreateBoxTriangles(const Eigen::Matrix4f &globalPose, float width = 50.f, float height = 50.f, float depth = 50.f);
    static TriMeshModelPtr CreateBox(const Eigen::Matrix4f &globalPose, float width = 50.f, float height = 50.f, float depth = 50.f,
                                     const VisualizationFactory::Color &color = VisualizationFactory::Color::Gray(), const std::vector<VisualizationFactory::Color>& colors = {});
    static TriMeshModelPtr CreateSparseBoxGrid(const Eigen::Matrix4f &globalPose, const std::vector<Eigen::Vector3f>& positions,
                                               float width = 50.f, float height = 50.f, float depth = 50.f,
                                               const VisualizationFactory::Color &color = VisualizationFactory::Color::Gray(),
                                               const std::vector<VisualizationFactory::Color>& colors = {});

};

} // namespace VirtualRobot

