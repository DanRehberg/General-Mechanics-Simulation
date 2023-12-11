#ifndef __SHADERS_HPP__
#define __SHADERS_HPP__

const char* vertShader = R"(
#version 460
layout (location = 0) in uint index;

layout (std140, binding = 0) uniform view
{
	mat4 ortho;
};

uniform vec3 positions[2];
uniform vec3 color;

out vec4 col;

void main()
{
	gl_Position = ortho * vec4(positions[index].xyz, 1.0);
	col = vec4(color.xyz, 1.0);
}

)";

const char* fragShader = R"(
#version 460

in vec4 col;

out vec4 color;

void main()
{
	color = col;
}

)";

#endif