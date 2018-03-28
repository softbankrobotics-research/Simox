#include "GaussianImplicitSurface3DNormals.h"

using namespace math;

GaussianImplicitSurface3DNormals::GaussianImplicitSurface3DNormals()
{

}

void GaussianImplicitSurface3DNormals::Calculate(ContactList samples, float noise, float normalNoise, float normalScale)
{
    ContactList shiftedSamples;
    std::vector<Vec3> points;
    std::vector<Vec3> pointsOriginal;
    mean = Average(samples);
    for(Contact d : samples){        
        Vec3 shiftedPos = d.Position()- mean;
        shiftedSamples.push_back(Contact(shiftedPos, d.Normal()*normalScale));
        points.push_back(shiftedPos);
        pointsOriginal.push_back(d.Position());
    }
    R = 0;
    for(Vec3 p1 : points){
        for(Vec3 p2 : points){
            if((p1-p2).norm() > R) R= (p1-p2).norm();
        }
    }
    R = std::sqrt(R);

    covariance = CalculateCovariance(pointsOriginal, R, noise, normalNoise);
    VectorXf y(samples.size()*4);
    for (uint i = 0; i < samples.size(); i++)
    {
         y(i*4) = 0;
         y(i * 4 + 1) = samples.at(i).Normal().x();
         y(i * 4 + 2) = samples.at(i).Normal().y();
         y(i * 4 + 3) = samples.at(i).Normal().z();
    }
    alpha = MatrixSolve(covariance, y);
    this->samples = shiftedSamples;
}


float GaussianImplicitSurface3DNormals::Get(Vec3 pos)
{
    return Predict(pos);
}

VectorXf GaussianImplicitSurface3DNormals::getCux(Vec3 pos)
{
     VectorXf Cux (samples.size() * 4);
     for (std::size_t i = 0; i < samples.size(); i++)
     {
        Cux(i * 4) = Kernel(pos, samples.at(i).Position(), R);
        Cux(i * 4 + 1) = Kernel_dj(pos, samples.at(i).Position(), R, 0);
        Cux(i * 4 + 2) = Kernel_dj(pos, samples.at(i).Position(), R, 1);
        Cux(i * 4 + 3) = Kernel_dj(pos, samples.at(i).Position(), R, 2);
    }
    return Cux;//VectorXf;
}
float GaussianImplicitSurface3DNormals::Predict(Vec3 pos)
{
    VectorXf Cux(4*samples.size());
    pos = pos - mean;
    Cux=getCux(pos);
    return Cux.dot(alpha);

}
MatrixXf GaussianImplicitSurface3DNormals::CalculateCovariance(std::vector<Vec3> points, float R, float noise, float normalNoise)
{
    MatrixXf covariance = MatrixXf(points.size()*4, points.size()*4);

    for (size_t i = 0; i < points.size()*4; i++)
    {
        for (size_t j = i; j < points.size()*4; j++)
        {
            float cov = Kernel(points.at(i/4), points.at(j/4), R, i%4 , j%4);
            covariance(i, j) = cov;
            covariance(j, i) = cov;
        }
    }
    for (size_t i = 0; i < points.size(); i++)
    {
        if(i%4==0){
         covariance(i, i) += i % 4 == 0 ? noise*noise : normalNoise;
        }

    }
    return covariance;   
}    
VectorXf GaussianImplicitSurface3DNormals::MatrixSolve(MatrixXf a, VectorXf b)
{
    return a.colPivHouseholderQr().solve(b);
}

Vec3 GaussianImplicitSurface3DNormals::Average(ContactList samples)
{
    Vec3 sum = Vec3(0,0,0);
    if (samples.size() == 0) return sum;
    for(Contact d : samples){
        sum += d.Position();
    }
    return sum / samples.size();
}

float GaussianImplicitSurface3DNormals::Kernel(Vec3 p1, Vec3 p2, float R)
{
    float r = (p1-p2).norm();
    return 2 * r*r*r + 3 * R * r*r + R*R*R;
}

float GaussianImplicitSurface3DNormals::Kernel_dx(float x, float /*y*/, float /*z*/, float r, float R)
{
    return 6 * x * (r + R);
}

float GaussianImplicitSurface3DNormals::Kernel_ddx(float x, float /*y*/, float /*z*/, float r, float R)
{
    if (r == 0) return 6 * R;
    return 6 * (R + r) + 6 * x * x / r;
}

float GaussianImplicitSurface3DNormals::Kernel_dxdy(float x, float y, float /*z*/, float r, float /*R*/)
{
    if (r == 0) return 0;
    return 6 * x * y / r;
}


float GaussianImplicitSurface3DNormals::Kernel(Vec3 p1, Vec3 p2, float R, int i, int j)
{
    if (i == 0 && j == 0) return Kernel(p1, p2, R);
    if (i == 0) return Kernel_dj(p1, p2, R, j - 1);
    if (j == 0) return Kernel_di(p1, p2, R, i - 1);
    return Kernel_didj(p1, p2, R, i - 1, j - 1);
}

float GaussianImplicitSurface3DNormals::Kernel_di(Vec3 p1, Vec3 p2, float R, int i)
{
    float r = (p1-p2).norm();
    float x = p1.x() - p2.x();
    float y = p1.y() - p2.y();
    float z = p1.z() - p2.z();
    swap(x, y, z, i);
    return Kernel_dx(x, y, z, r, R);
}
float GaussianImplicitSurface3DNormals::Kernel_dj(Vec3 p1, Vec3 p2, float R, int j)
{
   float r = (p1-p2).norm();
   float x = p1.x() - p2.x();
   float y = p1.y() - p2.y();
   float z = p1.z() - p2.z();
   swap(x, y, z, j);
   return -Kernel_dx(x, y, z, r, R);
}

float GaussianImplicitSurface3DNormals::Kernel_didj(Vec3 p1, Vec3 p2, float R, int i, int j)
{
   float r = (p1-p2).norm();
   float x = p1.x() - p2.x();
   float y = p1.y() - p2.y();
   float z = p1.z() - p2.z();
   if (i == j)
   {
       swap(x, y, z, i);       
       return -Kernel_ddx(x, y, z, r, R);
   }
    std::vector<float> a(3);
    a.at(0)=x;
    a.at(1)=y;
    a.at(2)=z;

    x = a.at(i);
    y = a.at(j);
    z = a.at(3 - i - j);
    return -Kernel_dxdy(x, y, z, r, R);

}


void GaussianImplicitSurface3DNormals::swap(float &x, float &y, float &z, int index) //TODO ref
{
   switch (index)
   {
       case 0: break;
       case 1: math::Helpers::Swap(x, y); break;
       case 2: math::Helpers::Swap(x, z); break;
       default: throw std::invalid_argument("index must be >= 0 and <= 2" );
    }
}

