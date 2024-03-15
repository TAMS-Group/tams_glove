#version 330

in vec2 in_vert;

void main() {
    gl_Position = vec4(in_vert * vec2(2.0, 2.0) - vec2(1.0, 1.0), 0.0, 1.0);
}