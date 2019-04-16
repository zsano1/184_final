#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2,uv).r;
  return 0.0;
}

void main() {
  // YOUR CODE HERE
  float u=v_uv.x;
  float v=v_uv.y;
  float w=u_texture_2_size.x;
  float height=u_texture_2_size.y;
  float du=(h(vec2(u+1/w,v))-h(vec2(u,v)))*u_height_scaling*u_normal_scaling;
  float dv=(h(vec2(u,v+1/height))-h(vec2(u,v)))*u_height_scaling*u_normal_scaling;
  vec3 n0=vec3(-du,-dv,1);
  vec3 c1=v_tangent.xyz;
  vec3 c2=cross(v_normal.xyz,v_tangent.xyz);
  vec3 c3=v_normal.xyz;
  mat3 tbn=mat3(c1,c2,c3);
  vec3 nd=tbn*n0;
  nd=normalize(nd);
  // (Placeholder code. You will want to replace it.)
  vec3 dir=normalize(u_light_pos-v_position.xyz);
  vec3 dir2=normalize(u_cam_pos-v_position.xyz);
  vec3 factor1=0.1*u_light_intensity/(dot(dir,dir))*max(0.0,dot(nd,dir));
  vec3 h=normalize((dir+dir2)/2);
  vec3 factor2=0.1*u_light_intensity/(dot(dir,dir))*pow(max(0.0,dot(nd,h)),4);
  float Ia=0.3;
  vec3 factor3=vec3(Ia);
  // (Placeholder code. You will want to replace it.)
  out_color.xyz = factor1+factor2+factor3;

  out_color.a = 1;
}

