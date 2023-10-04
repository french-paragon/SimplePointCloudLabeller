#version 150

uniform bool displayPointsColor;

in float heightProp;
in vec4 pointCol;

void main(void)
{
    if (displayPointsColor) {
        gl_FragColor = pointCol;
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

    gl_FragColor = fullProp*c2 + (1-fullProp)*c1;
}
