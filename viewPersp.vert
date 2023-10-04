#version 150

attribute vec3 in_location;
attribute vec3 in_color;

uniform mat4 matrixViewProjection;

uniform bool displayPointsColor = false;

uniform float minZ;
uniform float maxZ;

out float heightProp;
out vec4 pointCol;

void main(void)
{
    gl_Position = matrixViewProjection * vec4(in_location, 1.);
    gl_PointSize = 2.0;

    heightProp = (in_location.z - minZ)/(maxZ - minZ);

    if (displayPointsColor) {
        pointCol = vec4(in_color, 1);
    } else {
        pointCol = vec4(0,0,0,1);
    }
}
