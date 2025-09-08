#version 150

attribute vec3 in_location;
attribute vec3 in_color;
attribute float in_cluster_mask;

uniform mat4 matrixViewProjection;

uniform bool displayPointsColor = false;

uniform int passId;

uniform float minZ;
uniform float maxZ;

uniform float ptSize;

out float heightProp;
out vec4 pointCol;
out float pointInCluster;

void main(void)
{
    gl_Position = matrixViewProjection * vec4(in_location, 1.);

    gl_PointSize = ptSize;

    heightProp = (in_location.z - minZ)/(maxZ - minZ);

    if (displayPointsColor) {
        pointCol = vec4(in_color, 1);
    } else {
        pointCol = vec4(0,0,0,1);
    }

    pointInCluster = in_cluster_mask;

    if (in_cluster_mask < 0.5 && passId > 1) {
        gl_PointSize = 0.3*gl_PointSize;
    }
}
