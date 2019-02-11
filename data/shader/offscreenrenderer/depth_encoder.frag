#version 130

out vec4 fragColor;

varying float distToCamera;

const float maxDistance = 100000.0;

void main()
{
    float distance = min(maxDistance, distToCamera);
    int distanceTicks = int(distance / (maxDistance / (pow(256.0, 3.0))));

    int r = distanceTicks % 256;
    int g = (distanceTicks / 256) % 256;
    int b = (distanceTicks / 256 / 256) % 256;

    fragColor = vec4(r / 256.0, g / 256.0, b / 256.0, 1.0);
}

