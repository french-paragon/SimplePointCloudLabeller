#version 150

uniform bool displayPointsColor;
uniform int passId;

in float heightProp;
in vec4 pointCol;
in float pointInCluster;

void main(void)
{

    if (pointInCluster < 0.5) {
        if (passId <= 1) {
            gl_FragColor = vec4(0.8, 0.8, 0.8, 1.0);
        } else {
            return;
        }
        return;
    }

    if (displayPointsColor) {
        gl_FragColor = pointCol;
        if (passId > 1) {
            gl_FragColor[3] = 0.5;
        }
        return;
    }

    vec4 color1 = vec4(0.0, 0.1, 0.9, 1.0);
    vec4 color2 = vec4(0.9, 0.9, 0.0, 1.0);
    vec4 color3 = vec4(0.9, 0.0, 0.1, 1.0);

    vec4 c1;
    vec4 c2;

    if (heightProp < 0.5) {
        c1 = color1;
        c2 = color2;
    } else {
        c1 = color2;
        c2 = color3;
    }

    float fullProp = 2*heightProp;

    if (fullProp >= 1) {
        fullProp -= 1;
    }

    vec4 composedColor = fullProp*c2 + (1-fullProp)*c1;

    if (passId > 1) {
        gl_FragColor = composedColor;
    } else {
        gl_FragColor = composedColor;
    }
}
