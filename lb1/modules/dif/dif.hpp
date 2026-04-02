#pragma once

#include <glm/glm.hpp>
#include <glm/ext.hpp>

using namespace glm;

#define CUBE_SIZE 1.0
#define SPHERE_RADIUS 0.8
#define CYLINDER_RADIUS 0.6
#define CYLINDER_HEIGHT 1.5

enum BodyType {
    BODY_CUBE,
    BODY_SPHERE,
    BODY_CYLINDER
};

struct Context {
    double M_inv;           // 1/M
    dmat3 I_inv;            // 1/I
    
    double mass;            // масса тела
    double volume;          // объем тела
    double ro_liquid;       // плотность жидкости
    double ro_air;          // плотность воздуха
    double g;               // ускорение свободного падения
    double drag_coef_linear;// коэффициент линейного сопротивления
    bool drag_enabled;      // включено ли сопротивление
    double start_time;      // время старта
    
    BodyType body_type;     // тип тела
};

struct RigidBody {
    dvec3 r;                // позиция
    dvec3 l;                // импульс
    dvec3 L;                // момент импульса
    dquat q;                // ориентация
};

dvec3 CalculateForces(const RigidBody &rb, const Context &context, double time);
dvec3 CalculateTorque(const RigidBody &rb, const Context &context, double time);
double CalculateImmersedVolume(const RigidBody &rb, const Context &context);

RigidBody f_rigidbody(const RigidBody &rb, const Context &context, double time);
RigidBody MulRB(const RigidBody &rb, double num);
RigidBody SumRB(const RigidBody &r1, const RigidBody &r2);
double SolveRungeKutta4(RigidBody &rb, const Context &context, double h, double cur_time, int body_index, int current_body);

void InitCube(Context &context, double mass);
void InitSphere(Context &context, double mass);
void InitCylinder(Context &context, double mass);

dvec3 GetCylinderPointWorld(const RigidBody &rb, const dvec3 &local_point);
void GetCylinderYRange(const RigidBody &rb, double radius, double height, double &min_y, double &max_y);
double CalculateCylinderImmersedVolumeWithRotation(const RigidBody &rb, const Context &context, double y_water_level = 0.0);
dvec3 CalculateCylinderImmersedCenter(const RigidBody &rb, const Context &context, double y_water_level = 0.0);
dvec3 GetImmersedCenter(const RigidBody &rb, const Context &context);

double CalculateCubeImmersedVolumeWithRotation(const RigidBody &rb, double y_water_level = 0.0);
dvec3 CalculateCubeImmersedCenter(const RigidBody &rb, double y_water_level = 0.0);

double CalculateSphereImmersedVolumeWithRotation(const RigidBody &rb, double y_water_level = 0.0);
dvec3 CalculateSphereImmersedCenter(const RigidBody &rb, double y_water_level = 0.0);