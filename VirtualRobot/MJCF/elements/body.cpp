#include "body.h"

#include "../Document.h"


using namespace mjcf;


const std::string Inertial::tag     = "inertial";
const std::string Joint::tag        = "joint";
const std::string FreeJoint::tag    = "freejoint";
const std::string Geom::tag         = "geom";
const std::string Site::tag         = "site";
const std::string Camera::tag       = "camera";
const std::string Light::tag        = "light";
const std::string Body::tag         = "body";
const std::string Worldbody::tag        = "worldbody";


void Inertial::inertiaFromMatrix(const Eigen::Matrix3f& matrix)
{
    if (matrix.isDiagonal(1e-9f))
    {
        this->diaginertia = matrix.diagonal();
    }
    else
    {
        /* Full inertia matrix M. Since M is 3-by-3 and symmetric, 
         * it is specified using only 6 numbers in the following order: 
         * M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3).
         * -- http://www.mujoco.org/book/XMLreference.html#inertial 
         */
        
        Eigen::Vector6f inertia;
        inertia << matrix(0, 0), matrix(1, 1), matrix(2, 2), 
                matrix(0, 1), matrix(0, 2), matrix(1, 2);
        this->fullinertia = inertia;
    }
}

bool Body::hasMass() const
{
    return hasChild<Geom>() || hasChild<Inertial>();
}

Body Body::addBody(const std::string& name)
{
    Body body = addChild<Body>();

    if (!name.empty())
    {
        body.name = name;
    }

    return body;
}

Inertial Body::addInertial()
{
    return addChild<Inertial>();
}

Inertial Body::addInertial(const Eigen::Vector3f& pos, float mass, const Eigen::Matrix3f& matrix)
{
    Inertial inertial = addInertial();
    
    inertial.pos = pos;
    inertial.mass = mass;

    if (matrix.isDiagonal(document().getFloatCompPrecision()))
    {
        inertial.diaginertia = matrix.diagonal();
    }
    else
    {
        /* from mujoco xml reference:
         * "Full inertia matrix M. Since M is 3-by-3 and symmetric, it is
         * specified using only 6 numbers in the following order:
         * M(1,1), M(2,2), M(3,3), M(1,2), M(1,3), M(2,3)."*/
        Eigen::Vector6f inertia;
        inertia << matrix(0, 0), matrix(1, 1), matrix(2, 2),
                matrix(0, 1), matrix(0, 2), matrix(1, 2);
        inertial.fullinertia = inertia;
    }
    
    return inertial;
}

Inertial Body::addDummyInertial()
{
    Inertial inertial = addInertial();

    inertial.pos = Eigen::Vector3f(0, 0, 0);
    inertial.mass = document().getDummyMass();
    inertial.diaginertia = Eigen::Vector3f::Ones();

    return inertial;
}

Joint Body::addJoint()
{
    return addChild<Joint>();
}

FreeJoint Body::addFreeJoint()
{
    return addChild<FreeJoint>();
    
}

Geom Body::addGeom(const std::string& type)
{
    Geom geom = addChild<Geom>();
    
    geom.type = type;
    
    return geom;
}

Geom Body::addGeomMesh(const std::string& meshName, const std::string& materialName)
{
    Geom geom = addGeom("mesh");
    
    geom.mesh = meshName;
    if (!materialName.empty())
    {
        geom.material = materialName;
    }
    
    return geom;
}


Body Worldbody::addMocapBody(const std::string& name, float geomSize)
{
    Body mocap = addChild<Body>();
    mocap.name = name;
    mocap.mocap = true;
    // add geom for visualization

    Geom geom = mocap.addChild<Geom>();
    geom.type = "box";
    geom.size = geomSize * Eigen::Vector3f::Ones();

    return mocap;
}

Body Worldbody::addBody(const std::string& name, const std::string& childClass)
{
    Body body = addChild<Body>();

    if (!name.empty())
    {
        body.name = name;
    }
    if (!childClass.empty())
    {
        body.childclass = childClass;
    }

    return body;
}
