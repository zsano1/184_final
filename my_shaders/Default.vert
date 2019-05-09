#version 330

uniform mat4 u_model;
uniform mat4 u_view_projection;
uniform vec3 u_gravity;
uniform int pause;
uniform float u_delta_t;
uniform float u_damping;
uniform float u_mass;
uniform samplerBuffer u_points;
uniform samplerBuffer u_pinned;
uniform float u_ks;
uniform vec3 u_sphere_origin;
uniform float u_radius;
uniform int u_exist_sphere;
uniform float u_friction_sphere;

layout(location = 0) in vec3 position;
layout(location = 1) in int pinned;
layout(location = 2) in vec3 last_position;
layout(location = 3) in vec4 spring_structural;
layout(location = 4) in vec4 spring_shearing;
layout(location = 5) in vec4 spring_bending;
out vec3 outPosition;
out vec3 out_last_position;

int count;
vec3 self_collide(int k, float relax_length, int type, vec3 accummulate) {
    vec3 force = vec3(0.0, 0.0, 0.0);
    if (k == -1) {
        return force;
    }
    vec3 p = texelFetch(u_points, k).rgb;
    vec3 direction = normalize(p - position);
    float length = distance(p, position);
    float thickness=0.0095;
    if(length<=2*thickness){
        vec3 correction=direction*2*thickness+p-position;
        accummulate+=correction;
        count+=1;
    }
    return accummulate;
}

vec3 spring_force(int k, float relax_length, int type) {
    vec3 force = vec3(0.0, 0.0, 0.0);
    if (k == -1) {
        return force;
    }
    vec3 p = texelFetch(u_points, k).rgb;
    vec3 direction = normalize(p - position);
    float length = distance(p, position);
    force = u_ks * (length - relax_length) * direction;
    if (type == 2) {
        force = 0.2 * force;
    }
    return force;
}



vec3 constrain_changes(int k, float relax_length, vec3 new_position) {
    vec3 p = texelFetch(u_points, k).rgb;
    vec3 position_change = vec3(0, 0, 0);
    if (k == -1) {
        return position_change;
    }
    float length = distance(p, new_position);
    if (length > relax_length * 1.1) {
        vec3 direction = normalize(p - new_position);
        float len = length - relax_length * 1.1;
        if (texelFetch(u_pinned, k).r == 1) {
            position_change = len * direction;
        }
        else {
            position_change = (len / 2.0) * direction;
        }
    }
    return position_change;
}

vec3 collide_sphere(vec3 p) {
    vec3 direction = normalize(p - u_sphere_origin);
    float len = distance(p, u_sphere_origin);
    vec3 new_position = p;
    if (len <= u_radius) {
        vec3 correction = u_sphere_origin + direction * u_radius - last_position;
        new_position = last_position + (1.0 - u_friction_sphere) * correction;
    }
    return new_position;
}
vec3 collide_plane(vec3 p){
    vec3 point=vec3(0,0,0);
    vec3 normal=vec3(0,1,0);
    float friction=0.5;
    float SURFACE_OFFSET=0.0001;
    vec3 vector_new = p - point;
    vec3 vector_last = last_position - point;
    if (dot(vector_new, normal) * dot(vector_last, normal) <= 0) {
        vec3 unit = normalize(normal);
        vec3 tangent = p - dot(unit, vector_new) * unit;
        vec3 vector;
        if (dot(vector_last, normal) < 0) {
            vector = tangent - normal * SURFACE_OFFSET - last_position;
        }
        else {
            vector = tangent + normal * SURFACE_OFFSET - last_position;
        }
        p = last_position + (1.0 - friction) * vector;
    }
    return p;
}


void main()
{
    out_last_position = position;
    if (u_damping == 0.0 && pause == 0 && u_delta_t == 0.0) {
        outPosition = position;
    }
    else {
        // Points move towards their original position...
        if (pause == 0 && pinned == 0) {
            // outPosition = 0.99 * position + 0.01 * last_position + u_delta_t * u_gravity;
            // vec3 a = (1.5 * spring + 0.3 * spring_bending) / u_mass;
            
            vec3 force = vec3(0.0, 0.0, 0.0);
            force += u_mass * u_gravity;
            force += spring_force(int(spring_structural.x), 0.02, 0);
            force += spring_force(int(spring_structural.y), 0.02, 0);
            force += spring_force(int(spring_structural.z), 0.02, 0);
            force += spring_force(int(spring_structural.w), 0.02, 0);
            force += spring_force(int(spring_shearing.x), 0.028284, 1);
            force += spring_force(int(spring_shearing.y), 0.028284, 1);
            force += spring_force(int(spring_shearing.z), 0.028284, 1);
            force += spring_force(int(spring_shearing.w), 0.028284, 1);
            force += spring_force(int(spring_bending.x), 0.04, 2);
            force += spring_force(int(spring_bending.y), 0.04, 2);
            force += spring_force(int(spring_bending.z), 0.04, 2);
            force += spring_force(int(spring_bending.w), 0.04, 2);

            vec3 accummulate=vec3(0.0,0.0,0.0);
            int count=0;
            accummulate= self_collide(int(spring_structural.x), 0.02, 0,accummulate);
            accummulate= self_collide(int(spring_structural.y), 0.02, 0,accummulate);
            accummulate= self_collide(int(spring_structural.z), 0.02, 0,accummulate);
            accummulate= self_collide(int(spring_structural.w), 0.02, 0,accummulate);
            accummulate= self_collide(int(spring_shearing.x), 0.028284, 1,accummulate);
            accummulate= self_collide(int(spring_shearing.y), 0.028284, 1,accummulate);
            accummulate= self_collide(int(spring_shearing.z), 0.028284, 1,accummulate);
            accummulate= self_collide(int(spring_shearing.w), 0.028284, 1,accummulate);
            accummulate= self_collide(int(spring_bending.x), 0.04, 2,accummulate);
            accummulate= self_collide(int(spring_bending.y), 0.04, 2,accummulate);
            accummulate= self_collide(int(spring_bending.z), 0.04, 2,accummulate);
            accummulate= self_collide(int(spring_bending.w), 0.04, 2,accummulate);

            
            vec3 a = force / u_mass;
            // outPosition = position - 0.01 * texelFetch(u_points, 1).rgb;
            outPosition = position + u_damping * (position - last_position) + u_delta_t * u_delta_t * a;

            outPosition=collide_plane(outPosition);
            if(count!=0){
                outPosition+=accummulate/count/30;
            }
            if (u_exist_sphere == 1) {
                outPosition = collide_sphere(outPosition);
            }
            outPosition += constrain_changes(int(spring_structural.x), 0.02, outPosition);
            outPosition += constrain_changes(int(spring_structural.y), 0.02, outPosition);
            outPosition += constrain_changes(int(spring_structural.z), 0.02, outPosition);
            outPosition += constrain_changes(int(spring_structural.w), 0.02, outPosition);
            outPosition += constrain_changes(int(spring_shearing.x), 0.028284, outPosition);
            outPosition += constrain_changes(int(spring_shearing.y), 0.028284, outPosition);
            outPosition += constrain_changes(int(spring_shearing.z), 0.028284, outPosition);
            outPosition += constrain_changes(int(spring_shearing.w), 0.028284, outPosition);
            outPosition += constrain_changes(int(spring_bending.x), 0.04, outPosition);
            outPosition += constrain_changes(int(spring_bending.y), 0.04, outPosition);
            outPosition += constrain_changes(int(spring_bending.z), 0.04, outPosition);
            outPosition += constrain_changes(int(spring_bending.w), 0.04, outPosition);
        }
        else {
            outPosition = position;
        }
    }
    gl_Position = u_view_projection * vec4(outPosition, 1.0);
}

