#ifndef COINUTIL_H
#define COINUTIL_H

#include <Eigen/Geometry>
#include <Inventor/SbMatrix.h>
#include <Inventor/nodes/SoMatrixTransform.h>

namespace VirtualRobot {
    namespace CoinUtil {

        SbMatrix getSbMatrix(const Eigen::Matrix4f &m);
        SoMatrixTransform* getMatrixTransform(Eigen::Matrix4f& m);

    }
}

#endif
