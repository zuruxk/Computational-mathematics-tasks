#pragma once

#define SIZE 50

#include <glm/glm.hpp>
#include <glm/ext.hpp>

using namespace glm;

struct Context {
    double M_inv;   // 1/M
    dmat3 I_inv;    // 1/I

    double mass;        // body mass
    double volume;      // body volume
    double ro_liquid;   // liquid density
    double ro_air;      // air density
    double g;           // gravitational acceleration
    double drag_coef;   // drag coefficient
    bool drag_enabled;  // is drag enabled
    double start_time;  // start time
};

struct RigidBody {
    dvec3 r, l, L;  // position, impulse, moment of impulse
    dquat q;        // rotation quaternion
    dvec3 v;        // velocity
};

dvec3 CalculateForces (const RigidBody &rb, const Context &context, double time);
dvec3 CalculateTorque (const RigidBody &rb, const Context &context, double time);
RigidBody f_rigidbody (const RigidBody &rb, const Context &context, double time);
RigidBody MulRB (const RigidBody &rb, double num);
RigidBody SumRB (const RigidBody &r1, const RigidBody &r2);

double SolveRungeKutta4 (RigidBody &rb, const Context &context, double h, double cur_time);