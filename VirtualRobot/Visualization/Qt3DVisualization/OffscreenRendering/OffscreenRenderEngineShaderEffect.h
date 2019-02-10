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
* @author     Philipp Schmidt
* @copyright  2019 Philipp Schmidt
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_OffscreenRenderEngineShaderEffect_h_
#define _VirtualRobot_OffscreenRenderEngineShaderEffect_h_

#include <Qt3DRender/QEffect>
#include <Qt3DRender/QTechnique>
namespace VirtualRobot
{
    class OffscreenRenderEngineShaderEffect : public Qt3DRender::QEffect
    {
    public:
        explicit OffscreenRenderEngineShaderEffect(const QString& name, const QUrl& vertexShader, const QUrl& fragmentShader, Qt3DCore::QNode *parent = nullptr);

        QList<Qt3DRender::QFilterKey *> passCriteria() const;

    private:
        Qt3DRender::QTechnique *technique;
        Qt3DRender::QRenderPass *pass;
        Qt3DRender::QFilterKey *passCriterion;
    };
}

#endif // _VirtualRobot_OffscreenRenderEngineShaderEffect_h_
