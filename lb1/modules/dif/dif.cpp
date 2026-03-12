#include <cmath>
#include <iostream>

#include "dif.hpp"

dvec3 CalculateForces (const RigidBody &rb, const Context &context, double time) {
    dvec3 gravity(0, -context.mass * context.g, 0);

    double half_height = pow(context.volume, 1.0/3.0) / 2;
    double immersion = 0.0;
    double current_rho;
    
    if (rb.r.y > half_height) {
        immersion = 0.0;
        current_rho = context.ro_air;
    }
    else if (rb.r.y < -half_height) {
        immersion = 1.0;
        current_rho = context.ro_liquid;
    }
    else {
        immersion = (half_height - rb.r.y) / (2 * half_height);
        immersion = std::max(0.0, std::min(1.0, immersion));
        current_rho = context.ro_air * (1 - immersion) + context.ro_liquid * immersion;
    }

    double archimedes_force = current_rho * (context.volume * immersion) * context.g;
    dvec3 archimedes(0, archimedes_force, 0);
    
    dvec3 total_force = gravity + archimedes;

    if (context.drag_enabled) {
        dvec3 velocity = rb.l * context.M_inv;
        double speed = glm::length(velocity);

        double drag_coef_air = 0.5;
        double drag_coef_water = 150.0;

        double current_drag_coef = drag_coef_air * (1 - immersion) + drag_coef_water * immersion;

        if (speed > 0.001) {
            double drag_force = current_drag_coef * speed;
            dvec3 drag = -drag_force * (velocity / speed);
            total_force += drag;
        }
    }
    
    return total_force;
}

dvec3 CalculateTorque (const RigidBody &rb, const Context &context, double time) {
    dvec3 torque(0, 0, 0);
    if (context.drag_enabled) {
        dmat3 R(rb.q);
        dvec3 omega = R * context.I_inv * glm::transpose(R) * rb.L;

        double half_height = pow(context.volume, 1.0/3.0) / 2;
        double current_rho;
        
        if (rb.r.y > half_height) {
            current_rho = context.ro_air;
        }
        else if (rb.r.y < -half_height) {
            current_rho = context.ro_liquid;
        }
        else {
            double immersion = (half_height - rb.r.y) / (2 * half_height);
            immersion = std::max(0.0, std::min(1.0, immersion));
            current_rho = context.ro_air * (1 - immersion) + context.ro_liquid * immersion;
        }

        double rotational_drag = 0.1 * current_rho / context.ro_air;
        torque = -rotational_drag * omega;
    }
    return torque;
}

RigidBody f_rigidbody (const RigidBody &rb, const Context &context, double time) {
    RigidBody dt;
    dt.r = rb.l * context.M_inv;
    dmat3 R(rb.q);
    dvec3 omega = R * context.I_inv * glm::transpose(R) * rb.L;
    dt.q = 0.5 * dquat(0, omega) * rb.q;
    dvec3 Force = CalculateForces(rb, context, time);
    dt.l = Force;
    dvec3 Torque = CalculateTorque(rb, context, time);
    dt.L = Torque;

    return dt;
}

RigidBody MulRB (const RigidBody &rb, double num) {
    RigidBody res;
    res.r = rb.r * num;
    res.q = rb.q * num;
    res.l = rb.l * num;
    res.L = rb.L * num;

    return res;
}

RigidBody SumRB (const RigidBody &r1, const RigidBody &r2) {
    RigidBody res;
    res.r = r1.r + r2.r;
    res.q = r1.q + r2.q;
    res.l = r1.l + r2.l;
    res.L = r1.L + r2.L;

    return res;
}

double SolveRungeKutta4 (RigidBody &rb, const Context &context, double h, double cur_time) {
    RigidBody k1, k2, k3, k4;
    
    k1 = f_rigidbody(rb, context, cur_time);

    RigidBody temp = SumRB(rb, MulRB(k1, h / 2.0));
    k2 = f_rigidbody(temp, context, cur_time + h / 2.0);

    temp = SumRB(rb, MulRB(k2, h / 2.0));
    k3 = f_rigidbody(temp, context, cur_time + h / 2.0);

    temp = SumRB(rb, MulRB(k3, h));
    k4 = f_rigidbody(temp, context, cur_time + h);

    RigidBody delta = SumRB(
        SumRB(k1, MulRB(k2, 2.0)),
        SumRB(MulRB(k3, 2.0), k4) );
    rb = SumRB(rb, MulRB(delta, h / 6.0));
    rb.q = glm::normalize(rb.q);

    dvec3 velocity = rb.l * context.M_inv;

    double E_kin_trans = 0.5 * context.mass * dot(velocity, velocity);
    dmat3 R = dmat3(rb.q);
    dmat3 I_inv_world = R * context.I_inv * transpose(R);
    dvec3 omega = I_inv_world * rb.L;
    double E_kin_rot = 0.5 * dot(omega, rb.L);
    
    double E_pot = context.mass * context.g * rb.r.y;
    double half_height = pow(context.volume, 1.0/3.0) / 2;
    double current_rho;
    
    if (rb.r.y > half_height) {
        current_rho = context.ro_air;
    }
    else if (rb.r.y < -half_height) {
        current_rho = context.ro_liquid;
    }
    else {
        double immersion = (half_height - rb.r.y) / (2 * half_height);
        immersion = std::max(0.0, std::min(1.0, immersion));
        current_rho = context.ro_air * (1 - immersion) + context.ro_liquid * immersion;
    }
    
    double E_pot_arch = -current_rho * context.volume * context.g * rb.r.y;
    double total_energy = E_kin_trans + E_kin_rot + E_pot + E_pot_arch;
    
    static int counter = 0;
    if (counter++ % 50 == 0) {
        if (!context.drag_enabled) {
            std::cout<<"Energy check (RK4): "<<total_energy<<std::endl;
        }
    }

    return total_energy;
}