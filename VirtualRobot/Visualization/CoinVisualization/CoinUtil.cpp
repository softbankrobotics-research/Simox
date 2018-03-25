#include "CoinUtil.h"


namespace VirtualRobot {
namespace CoinUtil {

SbMatrix getSbMatrix(const Eigen::Matrix4f &m)
{
    SbMatrix res(reinterpret_cast<const SbMat*>(m.data()));
    return res;
}

SoMatrixTransform *getMatrixTransform(Eigen::Matrix4f &m)
{
    SoMatrixTransform* mt = new SoMatrixTransform;
    SbMatrix m_(reinterpret_cast<SbMat*>(m.data()));
    mt->matrix.setValue(m_);
    return mt;
}



} // namespace CoinUtil
} // namespace VirtualRobot
