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
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2017 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#pragma once

#include "../GraspStudio.h"

#include <vector>


namespace GraspStudio
{

    struct GraspPlannerEvaluation
    {
        GraspPlannerEvaluation();
    
        void print();
    
        static std::string GetCSVHeader();
    
        std::string toCSVString() const;
    
    
        int nrGraspsGenerated;
        int nrGraspsValid;
        int nrGraspsInvalidCollision;
        int nrGraspsInvalidFC; // or quality
        int nrGraspsInvalidContacts;
    
        int nrGraspsValidPrecision;
        int nrGraspsValidPower;
    
        std::vector<float> timeGraspMS;     // time per grasp generation
        std::vector<float> graspScore;      //grasp quality
        std::vector<bool> graspValid;       //grasp valid
        std::vector<bool> graspTypePower;   //grasp type
    
        bool fcCheck;
        float minQuality;
        
    };

    
    std::ostream& operator<< (std::ostream& os, const GraspPlannerEvaluation& rhs);

}

