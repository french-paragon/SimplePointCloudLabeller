#version 150

attribute vec3 in_location;

uniform mat4 matrixViewProjection;

uniform float minZ;
uniform float maxZ;

out float heightProp;

void main(void)
{
    gl_Position = matrixViewProjection * vec4(in_location, 1.);
    gl_PointSize = 2.0;

    heightProp = (in_location.z - minZ)/(maxZ - minZ);
}
