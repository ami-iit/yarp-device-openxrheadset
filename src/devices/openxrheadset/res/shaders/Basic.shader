#shader vertex
#version 330 core

layout(location = 0) in vec4 position;
layout(location = 1) in vec2 texCoord;

out vec2 v_TexCoord;

uniform mat4 u_M;
uniform mat4 u_P;
mat4 u_MVP;

void main()
{
    u_MVP = u_P * u_M;
    gl_Position = u_MVP * position;
    v_TexCoord = texCoord;
};

#shader fragment
#version 330 core

layout(location = 0) out vec4 color;

in vec2 v_TexCoord;

//uniform vec4 u_Color;
uniform sampler2D u_Texture;
uniform bool u_UseAlpha;

void main()
{
    vec4 texColor = texture(u_Texture, v_TexCoord);
    color = vec4(texColor.r, texColor.g, texColor.b, u_UseAlpha ? texColor.a : 1.0);
};
