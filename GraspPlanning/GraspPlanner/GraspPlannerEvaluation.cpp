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
#include "GraspPlannerEvaluation.h"

namespace GraspStudio
{

GraspPlannerEvaluation::GraspPlannerEvaluation()
{
    nrGraspsGenerated = 0;
    nrGraspsValid = 0;
    nrGraspsInvalidCollision = 0;
    nrGraspsInvalidFC = 0;
    nrGraspsInvalidContacts = 0;
    nrGraspsValidPrecision = 0;
    nrGraspsValidPower = 0;
    fcCheck = false;
    minQuality = 0.0f;
}

void GraspPlannerEvaluation::print()
{
    std::cout << (*this) << std::endl;
}

std::string GraspPlannerEvaluation::GetCSVHeader()
{
    std::stringstream fs;
    fs << "minQuality" << ","
       << "nrGraspsGenerated" << ","
       << "nrGraspsValid" << ","
       << "nrGraspsInValid" << ","
       << "nrGraspsValidPrecision" << ","
       << "nrGraspsValidPower" << ","
       << "nrGraspsInvalidCollision" << ","
       << "nrGraspsInvalidFC" << ","
       << "nrGraspsInvalidContacts" << ","
       << "AverageDurationMS" << ","
       << "percPowerGrasps" << ","
       << "percPrecGraps" << ","
       << "avgScore";
    return fs.str();
}

std::string GraspPlannerEvaluation::toCSVString() const
{
    float timeAcc = 0;
    float avgTime = 0;
    float scoreAcc = 0;
    int nrGrasps = static_cast<int>(timeGraspMS.size());
    int nrValid = 0;
    int nrPower = 0;
    int nrPrecision = 0;
    for (size_t i = 0; i < timeGraspMS.size(); i++)
    {
        timeAcc+=timeGraspMS.at(i);
        if (graspValid.at(i))
        {
            nrValid++;
            scoreAcc+=graspScore.at(i);
        }
        if (graspTypePower.at(i))
            nrPower++;
        else
            nrPrecision++;
    }
    float percPower = 0;
    float percPrec = 0;
    if (nrGrasps > 0)
    {
        percPower = static_cast<float>(nrPower) / static_cast<float>(nrGrasps);
        percPrec = static_cast<float>(nrPrecision) / static_cast<float>(nrGrasps);
        avgTime = timeAcc / static_cast<float>(nrGrasps);
    }
    float avgScore = 0;
    if (nrValid > 0)
    {
        avgScore = scoreAcc / nrValid;
    }
    
    
    std::stringstream fs;
    fs << minQuality << ","
       << nrGraspsGenerated << ","
       << nrGraspsValid << ","
       << (nrGraspsInvalidCollision+nrGraspsInvalidFC+nrGraspsInvalidContacts) << ","
       << nrGraspsValidPrecision << ","
       << nrGraspsValidPower << ","
       << nrGraspsInvalidCollision << ","
       << nrGraspsInvalidFC << ","
       << nrGraspsInvalidContacts << ","
       << avgTime << ","
       << percPower << ","
       << percPrec << ","
       << avgScore;
    return fs.str();
}

std::ostream& operator<<(std::ostream& os, const GraspPlannerEvaluation& rhs)
{
    os << "---------------- GRASP PLANNER EVALUATION -----------" << std::endl;
    if (rhs.fcCheck)
        os << "ForceClosure check: true" << std::endl;
    else
        os << "ForceClosure check: false" << std::endl;
    os << "Min Quality: " << rhs.minQuality << std::endl;
    
    os << "nrGraspsGenerated: " << rhs.nrGraspsGenerated << std::endl;
    os << "nrGraspsValid: " << rhs.nrGraspsValid << std::endl;
    os << "nrGraspsInValid: " << (rhs.nrGraspsInvalidCollision + rhs.nrGraspsInvalidFC 
                                  + rhs.nrGraspsInvalidContacts) << std::endl;
    os << "nrGraspsValidPrecision: " << rhs.nrGraspsValidPrecision << std::endl;
    os << "nrGraspsValidPower: " << rhs.nrGraspsValidPower << std::endl;
    os << "nrGraspsInvalidCollision: " << rhs.nrGraspsInvalidCollision << std::endl;
    os << "nrGraspsInvalidFC: " << rhs.nrGraspsInvalidFC << std::endl;
    os << "nrGraspsInvalidContacts: " << rhs.nrGraspsInvalidContacts << std::endl;
    VR_ASSERT (rhs.timeGraspMS.size() == rhs.graspScore.size());
    VR_ASSERT (rhs.timeGraspMS.size() == rhs.graspValid.size());
    VR_ASSERT (rhs.timeGraspMS.size() == rhs.graspTypePower.size());
    float timeAcc = 0;
    float scoreAcc = 0;
    int nrGrasps = static_cast<int>(rhs.timeGraspMS.size());
    int nrValid = 0;
    int nrPower = 0;
    int nrPrecision = 0;
    for (size_t i = 0; i < rhs.timeGraspMS.size(); i++)
    {
        timeAcc += rhs.timeGraspMS.at(i);
        if (rhs.graspValid.at(i))
        {
            nrValid++;
            scoreAcc += rhs.graspScore.at(i);
        }
        if (rhs.graspTypePower.at(i))
            nrPower++;
        else
            nrPrecision++;
    }
    os << "Time complete: " << timeAcc << " ms" << std::endl;
    if (nrGrasps > 0)
    {
        os << "Avg time per grasp (valid&invalid): " <<
              static_cast<float>(timeAcc) / static_cast<float>(nrGrasps) << " ms" << std::endl;
        float percPower = static_cast<float>(nrPower) / static_cast<float>(nrGrasps);
        float percPrec = static_cast<float>(nrPrecision) / static_cast<float>(nrGrasps);
        os << "Precision grasps: " << nrPrecision << " -> " << percPrec*100 << "%" << std::endl;
        os << "Power grasps: " << nrPower << " -> " << percPower*100 << "%" << std::endl;
    }
    if (nrValid > 0)
    {
        os << "Avg score (valid):" << scoreAcc / nrValid << std::endl;
    }
    if (rhs.nrGraspsGenerated > 0)
        os << "Percentage of valid grasps: " 
           << static_cast<float>(rhs.nrGraspsValid) / static_cast<float>(rhs.nrGraspsGenerated) * 100.0f 
           << "%" << std::endl;
    
    return os;
}


}

