#version 130

in vec4 vertexPosition;
in vec3 vertexNormal;

uniform mat4 mvp;
uniform mat4 modelView;
uniform mat3 modelViewNormal;

varying float distToCamera;

void main()
{
    vec4 cs_position = modelView * vertexPosition;
    distToCamera = -cs_position.z;
    gl_Position = mvp * vertexPosition;
}
