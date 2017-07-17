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
* @package    GraspPlanning
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _MotionPlanning_CoinConvexHullVisualization_h_
#define _MotionPlanning_CoinConvexHullVisualization_h_


#include "../../GraspPlanning.h"
#include "../ConvexHullVisualization.h"

class SoNode;
class SoSeparator;
class SoCallbackAction;
class SoPrimitiveVertex;

namespace GraspPlanning
{
    /*!
    *
    * \class CoinConvexHullVisualization
    *
    * A Coin3D related visualization of a convex hull
    */
    class GRASPPLANNING_IMPORT_EXPORT CoinConvexHullVisualization : virtual public ConvexHullVisualization
    {
    public:
        /*!
            Constructor
        */
        CoinConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr convHull, bool useFirst3Coords = true);
        CoinConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr convHull);

        ~CoinConvexHullVisualization();

        SoSeparator* getCoinVisualization();

    protected:

        void buildVisu();
        SoSeparator* createConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr& convHull);
        SoSeparator* createConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr& convHull, bool buseFirst3Coords);


        SoSeparator* visualization;

    };

    typedef std::shared_ptr<CoinConvexHullVisualization> CoinConvexHullVisualizationPtr;


}

#endif
