#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  vec3 dir=u_light_pos-v_position.xyz;
  vec3 dir2=u_cam_pos-v_position.xyz;
  vec3 factor1=0.1*u_light_intensity/(dot(dir,dir))*max(0.0,dot(v_normal.xyz,dir));
  vec3 h=(dir+dir2)/2;
  vec3 factor2=0.1*u_light_intensity/(dot(dir,dir))*pow(max(0.0,dot(v_normal.xyz,h)),4);
  float Ia=0.3;
  vec3 factor3=vec3(Ia);
  // (Placeholder code. You will want to replace it.)
  out_color.xyz =factor1+factor2+factor3;
  out_color.a = 1;
}

