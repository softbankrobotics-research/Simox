#include "BoundingBox.h"
#include "MathTools.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/Visualization.h>

namespace VirtualRobot
{

    BoundingBox::BoundingBox()
    {
        clear();
    }

    bool BoundingBox::planeGoesThrough(const MathTools::Plane& p)
    {
        bool left = false;
        bool right = false;

        std::vector < Eigen::Vector3f > points = getPoints();

        for (int i = 0; i < 8; i++)
            if (MathTools::onNormalPointingSide(points[i], p))
            {
                left = true;
            }
            else
            {
                right = true;
            }

        if (!left || !right)
        {
            return false;
        }

        return true;
    }


    BoundingBox::BoundingBox(const std::vector< Eigen::Vector3f >& p)
    {
        clear();
        if (p.size() != 0)
        {
            addPoints(p);
        }
    }

    std::vector <Eigen::Vector3f> BoundingBox::getPoints() const
    {
        std::vector < Eigen::Vector3f > points;

        if (std::isnan(min(0)) || std::isnan(min(1)) || std::isnan(min(2)) ||
                std::isnan(max(0)) || std::isnan(max(1)) || std::isnan(max(2)))
        {
            return points;
        }

        points.push_back(Eigen::Vector3f(min(0), min(1), min(2)));
        points.push_back(Eigen::Vector3f(min(0), min(1), max(2)));
        points.push_back(Eigen::Vector3f(min(0), max(1), min(2)));
        points.push_back(Eigen::Vector3f(min(0), max(1), max(2)));

        points.push_back(Eigen::Vector3f(max(0), min(1), min(2)));
        points.push_back(Eigen::Vector3f(max(0), min(1), max(2)));
        points.push_back(Eigen::Vector3f(max(0), max(1), min(2)));
        points.push_back(Eigen::Vector3f(max(0), max(1), max(2)));

        return points;
    }

    void BoundingBox::print()
    {
        cout << "* Bounding Box\n";
        std::streamsize pr = cout.precision(2);
        cout << "** min <" << min(0) << "," << min(1) << "," << min(2) << ">\n";
        cout << "** max <" << max(0) << "," << max(1) << "," << max(2) << ">\n";
        cout.precision(pr);

    }

    void BoundingBox::addPoints(const std::vector < Eigen::Vector3f >& p)
    {
        for (size_t i = 0; i < p.size(); i++)
        {
            addPoint(p[i]);
        }
    }

    void BoundingBox::addPoints(const BoundingBox& bbox)
    {
        std::vector <Eigen::Vector3f> p = bbox.getPoints();
        addPoints(p);
    }

    void BoundingBox::addPoint(const Eigen::Vector3f& p)
    {
        for (int j = 0; j < 3; j++)
        {
            if (std::isnan(min(j)) || p(j) < min(j))
            {
                min(j) = p(j);
            }

            if (std::isnan(max(j)) || p(j) > max(j))
            {
                max(j) = p(j);
            }
        }
    }

    Eigen::Vector3f BoundingBox::getMax() const
    {
        return max;
    }

    Eigen::Vector3f BoundingBox::getMin() const
    {
        return min;
    }


    void BoundingBox::clear()
    {
        min(0) = NAN;
        min(1) = NAN;
        min(2) = NAN;
        max(0) = NAN;
        max(1) = NAN;
        max(2) = NAN;
    }

    void BoundingBox::transform(const Eigen::Matrix4f& pose)
    {
        if (std::isnan(min(0)) || std::isnan(min(1)) || std::isnan(min(2)) ||
                std::isnan(max(0)) || std::isnan(max(1)) || std::isnan(max(2)))
        {
            return;
        }

        Eigen::Vector3f result[8];
        std::vector<Eigen::Vector3f> result3;
        result[0] << min(0), min(1), min(2);
        result[1] << max(0), min(1), min(2);
        result[2] << min(0), max(1), min(2);
        result[3] << max(0), max(1), min(2);
        result[4] << min(0), min(1), max(2);
        result[5] << max(0), min(1), max(2);
        result[6] << min(0), max(1), max(2);
        result[7] << max(0), max(1), max(2);
        Eigen::Matrix4f m;

        for (int i = 0; i < 8; i++)
        {
            m.setIdentity();
            m.block(0, 3, 3, 1) = result[i];
            m = pose * m;
            result3.push_back(m.block(0, 3, 3, 1));
        }

        // now find min max values
        min << FLT_MAX, FLT_MAX, FLT_MAX;
        max << -FLT_MAX, -FLT_MAX, -FLT_MAX;

        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (result3[i](j) < min(j))
                {
                    min(j) = result3[i](j);
                }

                if (result3[i](j) > max(j))
                {
                    max(j) = result3[i](j);
                }
            }
        }
    }

    void BoundingBox::scale(const Eigen::Vector3f &scaleFactor)
    {
        if (std::isnan(min(0)) || std::isnan(min(1)) || std::isnan(min(2)) ||
                std::isnan(max(0)) || std::isnan(max(1)) || std::isnan(max(2)))
        {
            return;
        }

        for (int i = 0; i < 3; i++)
        {
            min(i) *= scaleFactor(i);
            max(i) *= scaleFactor(i);
        }
    }

    std::string BoundingBox::toXML(int tabs, bool skipMatrixTag)
    {
            std::stringstream ss;
            std::string t;

            for (int i = 0; i < tabs; i++)
            {
                t += "\t";
            }

            if (!skipMatrixTag)
            {
                ss << t << "<BoundingBox>\n";
            }

            ss << t << "\t<min x='" << min(0) << "' y='" << min(1) << "' z='" << min(2) << "'/>\n";
            ss << t << "\t<max x='" << max(0) << "' y='" << max(1) << "' z='" << max(2) << "'/>\n";

            if (!skipMatrixTag)
            {
                ss << t << "</BoundingBox>\n";
            }

            return ss.str();
    }

    void BoundingBox::enlarge(const Eigen::Vector3f &v)
    {
        min -= v;
        max += v;
    }

    VisualizationPtr BoundingBox::getVisualization(bool wireFrame) const
    {
        Eigen::Vector3f mi = getMin();
        Eigen::Vector3f ma = getMax();
        float x1 = std::min(mi(0), ma(0));
        float x2 = std::max(mi(0), ma(0));
        float y1 = std::min(mi(1), ma(1));
        float y2 = std::max(mi(1), ma(1));
        float z1 = std::min(mi(2), ma(2));
        float z2 = std::max(mi(2), ma(2));
        float x = x1 + (x2 - x1) * 0.5f;
        float y = y1 + (y2 - y1) * 0.5f;
        float z = z1 + (z2 - z1) * 0.5f;
        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        gp(0, 3) = x;
        gp(1, 3) = y;
        gp(2, 3) = z;

        float width = x2 - x1;
        float height = y2 - y1;
        float depth = z2 - z1;

        auto box = VisualizationFactory::getInstance()->createBox(width, height, depth);
        box->setStyle(wireFrame ? Visualization::DrawStyle::wireframe : Visualization::DrawStyle::normal);
        box->setGlobalPose(gp);
        return box;
    }
}
