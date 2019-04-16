#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D dir=pm.position-origin;
  double distance=dir.norm();
  if(distance<=radius){
    dir.normalize();
    Vector3D correct_pos=origin+dir*radius;
    Vector3D correction=correct_pos-pm.last_position;
    pm.position=correction*(1.0-friction)+pm.last_position;
  }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
