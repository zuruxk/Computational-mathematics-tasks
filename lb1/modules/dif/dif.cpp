#include <cmath>
#include <iostream>
#include <algorithm>
#include "dif.hpp"

static bool last_drag_state = true;
static double last_energy_check = 0;

double CalculateImmersedVolume(const RigidBody &rb, const Context &context) {
    double y_pos = rb.r.y;
    double immersed_volume = 0.0;
    
    switch (context.body_type) {
        case BODY_CUBE: {
            double half_height = CUBE_SIZE / 2.0;
            double top = y_pos + half_height;
            double bottom = y_pos - half_height;
            
            if (top <= 0.0) {
                immersed_volume = 1.0;
            }
            else if (bottom >= 0.0) {
                immersed_volume = 0.0;
            }
            else {
                double immersed_height = (y_pos > 0) ? half_height - y_pos : half_height + y_pos;
                immersed_volume = immersed_height / (2.0 * half_height);
            }
            break;
        }
        
        case BODY_SPHERE: {
            double R = SPHERE_RADIUS;
            if (y_pos + R <= 0.0) {
                immersed_volume = 1.0;
            }
            else if (y_pos - R >= 0.0) {
                immersed_volume = 0.0;
            }
            else {
                double h = std::min(R, std::max(0.0, -y_pos + R));
                immersed_volume = (h * h * (3.0 * R - h)) / (4.0 * R * R * R);
            }
            break;
        }
        
        case BODY_CYLINDER: {
            double R = CYLINDER_RADIUS;
            double H = CYLINDER_HEIGHT;
            double half_height = H / 2.0;
            double top = y_pos + half_height;
            double bottom = y_pos - half_height;
            
            if (top <= 0.0) {
                immersed_volume = 1.0;
            } else if (bottom >= 0.0) {
                immersed_volume = 0.0;
            } else {
                double immersed_height = (y_pos > 0) ? half_height - y_pos : half_height + y_pos;
                immersed_volume = immersed_height / H;
            }
            break;
        }
    }
    
    return std::max(0.0, std::min(1.0, immersed_volume));
}

dvec3 CalculateForces(const RigidBody &rb, const Context &context, double time) {
    dvec3 gravity(0, -context.mass * context.g, 0);
    
    double immersion = CalculateImmersedVolume(rb, context);
    double archimedes_force = context.ro_liquid * context.volume * immersion * context.g;
    dvec3 archimedes(0, archimedes_force, 0);
    
    dvec3 total_force = gravity + archimedes;
    
    if (context.drag_enabled) {
        dvec3 velocity = rb.l * context.M_inv;
        double speed = length(velocity);
        
        if (speed > 0.001) {
            double rho_medium = context.ro_air * (1.0 - immersion) + context.ro_liquid * immersion;
            
            double drag_coef_shape = 1.0;
            switch (context.body_type) {
                case BODY_CUBE: drag_coef_shape = 1.05; break;
                case BODY_SPHERE: drag_coef_shape = 0.47; break;
                case BODY_CYLINDER: drag_coef_shape = 0.82; break;
            }
            
            double area = 0.0;
            switch (context.body_type) {
                case BODY_CUBE: area = CUBE_SIZE * CUBE_SIZE; break;
                case BODY_SPHERE: area = M_PI * SPHERE_RADIUS * SPHERE_RADIUS; break;
                case BODY_CYLINDER: area = 2.0 * CYLINDER_RADIUS * CYLINDER_HEIGHT; break;
            }
            
            double drag_force_magnitude = 0.5 * rho_medium * drag_coef_shape * area * speed * speed;
            dvec3 drag = -drag_force_magnitude * (velocity / speed);
            total_force += drag;
        }
    }
    
    return total_force;
}

dvec3 CalculateTorque(const RigidBody &rb, const Context &context, double time) {
    dvec3 torque(0, 0, 0);
    
    if (context.drag_enabled) {
        dmat3 R = dmat3(rb.q);
        dvec3 omega = R * context.I_inv * transpose(R) * rb.L;
        
        double immersion = CalculateImmersedVolume(rb, context);
        double rho_medium = context.ro_air * (1.0 - immersion) + context.ro_liquid * immersion;
        
        double rotational_drag = 0.1 * rho_medium / context.ro_air;
        torque = -rotational_drag * omega;
    }
    
    return torque;
}


RigidBody f_rigidbody(const RigidBody &rb, const Context &context, double time) {
    RigidBody dt;
    
    dt.r = rb.l * context.M_inv;
    
    dmat3 R = dmat3(rb.q);
    dvec3 omega = R * context.I_inv * transpose(R) * rb.L;

    dt.q = 0.5 * dquat(0.0, omega.x, omega.y, omega.z) * rb.q;
    dt.l = CalculateForces(rb, context, time);
    dt.L = CalculateTorque(rb, context, time);
    
    return dt;
}

RigidBody MulRB(const RigidBody &rb, double num) {
    RigidBody res;
    res.r = rb.r * num;
    res.q = rb.q * num;
    res.l = rb.l * num;
    res.L = rb.L * num;
    return res;
}

RigidBody SumRB(const RigidBody &r1, const RigidBody &r2) {
    RigidBody res;
    res.r = r1.r + r2.r;
    res.q = r1.q + r2.q;
    res.l = r1.l + r2.l;
    res.L = r1.L + r2.L;
    return res;
}

double SolveRungeKutta4(RigidBody &rb, const Context &context, double h, double cur_time, int body_index, int current_body) {
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
        SumRB(MulRB(k3, 2.0), k4)
    );
    
    rb = SumRB(rb, MulRB(delta, h / 6.0));
    rb.q = normalize(rb.q);
    
    dvec3 velocity = rb.l * context.M_inv;
    double E_kin_trans = 0.5 * context.mass * dot(velocity, velocity);
    
    dmat3 R = dmat3(rb.q);
    dvec3 omega = R * context.I_inv * transpose(R) * rb.L;
    double E_kin_rot = 0.5 * dot(omega, rb.L);
    
    double E_pot = context.mass * context.g * rb.r.y;
    
    double immersion = CalculateImmersedVolume(rb, context);
    double rho_medium = context.ro_air * (1.0 - immersion) + context.ro_liquid * immersion;
    double E_pot_arch = -rho_medium * context.volume * immersion * context.g * rb.r.y;
    
    double total_energy = E_kin_trans + E_kin_rot + E_pot + E_pot_arch;
    
    static double last_energy_print = 0;
    
    if (body_index == current_body && cur_time - last_energy_print > 0.2) {
        std::cout << "\nenergy check\n";
        std::cout << "time: " << cur_time << " s\n";
        std::cout << "total energy: " << total_energy << "\n";
        std::cout << "  E_kin_trans: " << E_kin_trans << "\n";
        std::cout << "  E_kin_rot: " << E_kin_rot << "\n";
        std::cout << "  E_pot: " << E_pot << "\n";
        std::cout << "  E_pot_arch: " << E_pot_arch << "\n";
        std::cout << "  Position Y: " << rb.r.y << "\n";
        last_energy_print = cur_time;
    }
    
    return total_energy;
}

void InitCube(Context &context, double mass) {
    context.body_type = BODY_CUBE;
    context.mass = mass;
    context.volume = CUBE_SIZE * CUBE_SIZE * CUBE_SIZE;
    context.M_inv = 1.0 / mass;
    
    double I = (1.0/6.0) * mass * CUBE_SIZE * CUBE_SIZE;
    context.I_inv = dmat3(0);
    for (int i = 0; i < 3; i++)
        context.I_inv[i][i] = 1.0 / I;
}

void InitSphere(Context &context, double mass) {
    context.body_type = BODY_SPHERE;
    context.mass = mass;
    context.volume = (4.0/3.0) * M_PI * pow(SPHERE_RADIUS, 3);
    context.M_inv = 1.0 / mass;
    
    double I = (2.0/5.0) * mass * SPHERE_RADIUS * SPHERE_RADIUS;
    context.I_inv = dmat3(0);
    for (int i = 0; i < 3; i++)
        context.I_inv[i][i] = 1.0 / I;
}

void InitCylinder(Context &context, double mass) {
    context.body_type = BODY_CYLINDER;
    context.mass = mass;
    context.volume = M_PI * CYLINDER_RADIUS * CYLINDER_RADIUS * CYLINDER_HEIGHT;
    context.M_inv = 1.0 / mass;
    
    double I_xy = (1.0/12.0) * mass * (3.0 * CYLINDER_RADIUS * CYLINDER_RADIUS + CYLINDER_HEIGHT * CYLINDER_HEIGHT);
    double I_z = 0.5 * mass * CYLINDER_RADIUS * CYLINDER_RADIUS;
    
    context.I_inv = dmat3(0);
    context.I_inv[0][0] = 1.0 / I_xy;
    context.I_inv[1][1] = 1.0 / I_xy;
    context.I_inv[2][2] = 1.0 / I_z;
}