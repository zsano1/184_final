#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}void Cloth::buildGrid() {
    // TODO (Part 1): Build a grid of masses and springs.
    double unit_w=width/num_width_points;
    double unit_h=height/num_height_points;
    for(int i=0;i<num_height_points;i++){
        for(int j=0;j<num_width_points;j++){
            Vector3D p;
            bool pin;
            if(orientation==HORIZONTAL){
                p=Vector3D(j*unit_w,1.0,i*unit_h);
            }
            else{
                p=Vector3D(j*unit_w,i*unit_h,(double)rand()/(double)RAND_MAX/1000.0);
            }
            PointMass mass= PointMass(p, false);
            point_masses.emplace_back(mass);
        }
    }
    for(int i=0;i<pinned.size();i++){
        vector<int> coor=pinned[i];
        int ind=coor[0]+coor[1]*num_width_points;
        point_masses[ind].pinned = true;
    }
    for(int i=0;i<num_height_points;i++){
        for(int j=0;j<num_width_points;j++){
            int now=num_width_points*i+j;
            int left=now-1;
            int above=num_width_points*(i-1)+j;
            int upper_right=num_width_points*(i-1)+j+1;
            int upper_left=upper_right-2;
            int left2=num_width_points*i+j-2;
            int above2=num_width_points*(i-2)+j;
            if(j>=1){
                Spring sp=Spring(&point_masses[now],&point_masses[left],STRUCTURAL);
                springs.emplace_back(sp);
            }
            if(i>=1){
                Spring sp=Spring(&point_masses[now],&point_masses[above],STRUCTURAL);
                springs.emplace_back(sp);
            }
            if((i>=1)&&(j>=1)){
                Spring sp=Spring(&point_masses[now],&point_masses[upper_left],SHEARING);
                springs.emplace_back(sp);
            }
            if((i>=1)&&(j<num_width_points-1)){
                Spring sp=Spring(&point_masses[now],&point_masses[upper_right],SHEARING);
                springs.emplace_back(sp);
            }
            if(j>=2){
                Spring sp=Spring(&point_masses[now],&point_masses[left2],BENDING);
                springs.emplace_back(sp);
            }
            if(i>=2){
                Spring sp=Spring(&point_masses[now],&point_masses[above2],BENDING);
                springs.emplace_back(sp);
            }
        }
    }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D external=Vector3D(0,0,0);
  for(Vector3D &ext:external_accelerations){
      external+=ext;
  }
  Vector3D force=external*mass;
  for(PointMass &ma:point_masses){
      ma.forces=force;
  }
  for(Spring &sp: springs){
      double magnitude=(sp.pm_b->position-sp.pm_a->position).norm();
      Vector3D dir=(sp.pm_b->position-sp.pm_a->position);
      dir.normalize();
      if((cp->enable_structural_constraints&&sp.spring_type==STRUCTURAL)||(cp->enable_shearing_constraints&&sp.spring_type==SHEARING)){
          Vector3D F=cp->ks*(magnitude-sp.rest_length)*dir;
          sp.pm_a->forces+=F;
          sp.pm_b->forces-=F;
      }
      if(cp->enable_bending_constraints&&sp.spring_type==BENDING){
          Vector3D F=0.2*cp->ks*(magnitude-sp.rest_length)*dir;
          sp.pm_a->forces+=F;
          sp.pm_b->forces-=F;
      }
  }
  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for(PointMass &ma:point_masses){
      if(!ma.pinned){
      Vector3D a=ma.forces/mass;
      Vector3D new_pos=ma.position+(1-cp->damping/100.0)*(ma.position-ma.last_position)+a*pow(delta_t,2);
      ma.last_position=ma.position;
      ma.position=new_pos;
  }}

  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (PointMass &pm : point_masses) {
        self_collide(pm, simulation_steps);
    }

  // TODO (Part 3): Handle collisions with other primitives.
    for (PointMass &pm : point_masses) {
        for (CollisionObject *co : *collision_objects) {
            co->collide(pm);
        }
    }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
    for(Spring &sp:springs){
        Vector3D dir=(sp.pm_b->position-sp.pm_a->position);
        double length=dir.norm();
        bool correction=(length>1.1*sp.rest_length);
        if(correction){
            double correction_val=length-1.1*sp.rest_length;
            dir.normalize();
            Vector3D cor_pos=correction_val*dir;
            if(!sp.pm_a->pinned &&!sp.pm_b->pinned){
                sp.pm_a->position+=cor_pos/2;
                sp.pm_b->position-=cor_pos/2;
            }
            else if(!sp.pm_a->pinned &&sp.pm_b->pinned){
                sp.pm_a->position+=cor_pos;
            }
            else if(!sp.pm_b->pinned &&sp.pm_a->pinned){
                sp.pm_b->position-=cor_pos;
            }
        }
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();
  for(PointMass &ma:point_masses){
      float val=hash_position(ma.position);
      if(!map.count(val)){
          map[val]=new vector<PointMass *>();
      }
      map[val]->emplace_back(&ma);
  }
  // TODO (Part 4): Build a spatial map out of all of the point masses.

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float v=hash_position(pm.position);
  int count=0;
  vector<PointMass *> *sets=map[v];
  Vector3D accumulate=Vector3D(0,0,0);
  for(PointMass *ma:*sets){
    if(!(ma->position==pm.position)){
      Vector3D dir=pm.position-ma->position;
      double distance=dir.norm();
      if(distance<=2*thickness){
          dir.normalize();
          Vector3D correction=dir*2*thickness+ma->position-pm.position;
          accumulate+=correction;
          count+=1;
      }
    }
  }
  if(count!=0)
  pm.position=pm.position+accumulate/count/simulation_steps;

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w=3*width/num_width_points;
  float h=3*height/num_height_points;
  float t=max(w,h);
  float x_f=(pos.x-fmod(pos.x,w))/w;
  float y_f=(pos.y-fmod(pos.y,h))/h;
  float z_f=(pos.z-fmod(pos.z,t))/t;
  return x_f+y_f*10+z_f*100;

  return 0.f;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
