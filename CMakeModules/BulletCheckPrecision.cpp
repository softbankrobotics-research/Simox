#pragma GCC diagnostic ignored "-Wunused-variable"
#include <btBulletCollisionCommon.h>

int main()
{
    btVector3 vec(0.0, 1.0, 0.0);
    btStaticPlaneShape(vec, 0.0);

    return 0;
}
