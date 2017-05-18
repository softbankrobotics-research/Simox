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
#ifndef __GENERAL_GRASP_PLANNER_EVALUATION_H__
#define __GENERAL_GRASP_PLANNER_EVALUATION_H__

#include "../GraspStudio.h"
#include <vector>

namespace GraspStudio
{

struct GraspPlannerEvaluation
{
    GraspPlannerEvaluation()
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

    void print()
    {
        std::cout << "---------------- GRASP PLANNER EVALUATION -----------" << std::endl;
        if (fcCheck)
            std::cout << "ForceClosure check: true" << std::endl;
        else
            std::cout << "ForceClosure check: false" << std::endl;
        std::cout << "Min Quality: " << minQuality << std::endl;

        std::cout << "nrGraspsGenerated:" << nrGraspsGenerated << std::endl;
        std::cout << "nrGraspsValid:" << nrGraspsValid << std::endl;
        std::cout << "nrGraspsInValid:" << nrGraspsInvalidCollision+nrGraspsInvalidFC+nrGraspsInvalidContacts << std::endl;
        std::cout << "nrGraspsValidPrecision:" << nrGraspsValidPrecision << std::endl;
        std::cout << "nrGraspsValidPower:" << nrGraspsValidPower << std::endl;
        std::cout << "nrGraspsInvalidCollision:" << nrGraspsInvalidCollision << std::endl;
        std::cout << "nrGraspsInvalidFC:" << nrGraspsInvalidFC << std::endl;
        std::cout << "nrGraspsInvalidContacts:" << nrGraspsInvalidContacts << std::endl;
        VR_ASSERT (timeGraspMS.size() == graspScore.size());
        VR_ASSERT (timeGraspMS.size() == graspValid.size());
        VR_ASSERT (timeGraspMS.size() == graspTypePower.size());
        float timeAcc = 0;
        float scoreAcc = 0;
        int nrGrasps = (int)timeGraspMS.size();
        int nrValid = 0;
        int nrPower = 0;
        int nrPrecision = 0;
        for (size_t i=0;i<timeGraspMS.size();i++)
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
        std::cout << "Time complete: " << timeAcc << " ms" << std::endl;
        if (nrGrasps>0)
        {
            std::cout << "Avg time per grasp (valid&invalid):" << (float)timeAcc / (float)nrGrasps << " ms" << std::endl;
            float percPower = (float)nrPower / (float)nrGrasps;
            float percPrec = (float)nrPrecision / (float)nrGrasps;
            std::cout << "Precision grasps:" << nrPrecision << " -> " << percPrec*100 << "%" << std::endl;
            std::cout << "Power grasps:" << nrPower << " -> " << percPower*100 << "%" << std::endl;
        }
        if (nrValid>0)
        {
            std::cout << "Avg score (valid):" << scoreAcc / nrValid << std::endl;
        }
        if (nrGraspsGenerated>0)
            std::cout << "Percentage of valid grasps:" << (float)nrGraspsValid / (float)nrGraspsGenerated * 100.0f << "%" << std::endl;
    }

    static std::string GetCSVHeader()
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

    std::string toCSVString() const
    {
        float timeAcc = 0;
        float scoreAcc = 0;
        int nrGrasps = (int)timeGraspMS.size();
        int nrValid = 0;
        int nrPower = 0;
        int nrPrecision = 0;
        for (size_t i=0;i<timeGraspMS.size();i++)
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
        if (nrGrasps>0)
        {
            percPower = (float)nrPower / (float)nrGrasps;
            percPrec = (float)nrPrecision / (float)nrGrasps;
        }
        float avgScore = 0;
        if (nrValid>0)
        {
            avgScore = scoreAcc / nrValid ;
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
           << (float)timeAcc / (float)nrGrasps << ","
           << percPower << ","
           << percPrec << ","
           << avgScore;
        return fs.str();
    }


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

}

#endif
