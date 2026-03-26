#include <cmath>
#include <iostream>
#include <algorithm>
#include <vector>
#include "dif.hpp"

double CalculateImmersedVolume(const RigidBody &rb, const Context &context) {
    switch (context.body_type) {
        case BODY_CUBE:
            return CalculateCubeImmersedVolumeWithRotation(rb, 0.0);
        
        case BODY_SPHERE:
            return CalculateSphereImmersedVolumeWithRotation(rb, 0.0);
        
        case BODY_CYLINDER:
            return CalculateCylinderImmersedVolumeWithRotation(rb, context, 0.0);
    }
    
    return 0.0;
}

// Численное вычисление объема погруженной части куба с учетом вращения
double CalculateCubeImmersedVolumeWithRotation(const RigidBody &rb, double y_water_level) {
    double half = CUBE_SIZE / 2.0;
    
    // Находим min и max Y куба с учетом поворота
    std::vector<dvec3> local_points;
    for (int i = -1; i <= 1; i += 2) {
        for (int j = -1; j <= 1; j += 2) {
            for (int k = -1; k <= 1; k += 2) {
                local_points.push_back(dvec3(i * half, j * half, k * half));
            }
        }
    }
    
    dmat3 R = dmat3(rb.q);
    double min_y = INFINITY;
    double max_y = -INFINITY;
    
    for (const auto &local_pt : local_points) {
        double world_y = (rb.r + R * local_pt).y;
        min_y = std::min(min_y, world_y);
        max_y = std::max(max_y, world_y);
    }
    
    if (min_y >= y_water_level) return 0.0;
    if (max_y <= y_water_level) return 1.0;
    
    // Численное интегрирование по слоям
    const int num_slices = 500000;
    double slice_height = CUBE_SIZE / num_slices;
    double immersed_volume = 0.0;
    double slice_volume = CUBE_SIZE * CUBE_SIZE * slice_height;
    
    for (int i = 0; i < num_slices; i++) {
        double local_y = -half + (i + 0.5) * slice_height;
        dvec3 world_center = rb.r + R * dvec3(0, local_y, 0);
        
        if (world_center.y < y_water_level) {
            immersed_volume += slice_volume;
        }
    }
    
    double total_volume = CUBE_SIZE * CUBE_SIZE * CUBE_SIZE;
    return std::min(1.0, std::max(0.0, immersed_volume / total_volume));
}

// Численное вычисление центра масс погруженной части куба с учетом вращения
dvec3 CalculateCubeImmersedCenter(const RigidBody &rb, double y_water_level) {
    double half = CUBE_SIZE / 2.0;
    
    std::vector<dvec3> local_points;
    for (int i = -1; i <= 1; i += 2) {
        for (int j = -1; j <= 1; j += 2) {
            for (int k = -1; k <= 1; k += 2) {
                local_points.push_back(dvec3(i * half, j * half, k * half));
            }
        }
    }
    
    dmat3 R = dmat3(rb.q);
    double min_y = INFINITY;
    double max_y = -INFINITY;
    
    for (const auto &local_pt : local_points) {
        double world_y = (rb.r + R * local_pt).y;
        min_y = std::min(min_y, world_y);
        max_y = std::max(max_y, world_y);
    }
    
    if (min_y >= y_water_level) return rb.r;
    if (max_y <= y_water_level) return rb.r;
    
    const int num_slices = 500000;
    double slice_height = CUBE_SIZE / num_slices;
    double total_immersed_volume = 0.0;
    dvec3 weighted_sum(0.0);
    double slice_volume = CUBE_SIZE * CUBE_SIZE * slice_height;
    
    for (int i = 0; i < num_slices; i++) {
        double local_y = -half + (i + 0.5) * slice_height;
        dvec3 world_center = rb.r + R * dvec3(0, local_y, 0);
        
        if (world_center.y < y_water_level) {
            total_immersed_volume += slice_volume;
            weighted_sum += slice_volume * world_center;
        }
    }
    
    return (total_immersed_volume > 0) ? weighted_sum / total_immersed_volume : rb.r;
}

// Численное вычисление объема погруженной части сферы с учетом вращения
double CalculateSphereImmersedVolumeWithRotation(const RigidBody &rb, double y_water_level) {
    double R = SPHERE_RADIUS;
    
    // Находим min и max Y сферы с учетом вращения
    double min_y = rb.r.y - R;
    double max_y = rb.r.y + R;
    
    if (min_y >= y_water_level) return 0.0;
    if (max_y <= y_water_level) return 1.0;
    
    // Численное интегрирование по слоям для сферы
    const int num_slices = 500000;
    double slice_height = 2.0 * R / num_slices;
    double immersed_volume = 0.0;
    
    dmat3 R_mat = dmat3(rb.q);
    
    for (int i = 0; i < num_slices; i++) {
        double local_y = -R + (i + 0.5) * slice_height;
        dvec3 world_center = rb.r + R_mat * dvec3(0, local_y, 0);
        
        if (world_center.y < y_water_level) {
            // Вычисляем радиус сечения на данной высоте
            double r_slice = std::sqrt(R * R - local_y * local_y);
            double slice_volume = M_PI * r_slice * r_slice * slice_height;
            immersed_volume += slice_volume;
        }
    }
    
    double total_volume = (4.0/3.0) * M_PI * R * R * R;
    return std::min(1.0, std::max(0.0, immersed_volume / total_volume));
}

// Численное вычисление центра масс погруженной части сферы с учетом вращения
dvec3 CalculateSphereImmersedCenter(const RigidBody &rb, double y_water_level) {
    double R = SPHERE_RADIUS;
    
    double min_y = rb.r.y - R;
    double max_y = rb.r.y + R;
    
    if (min_y >= y_water_level) return rb.r;
    if (max_y <= y_water_level) return rb.r;
    
    // Численное интегрирование по слоям
    const int num_slices = 500000;
    double slice_height = 2.0 * R / num_slices;
    double total_immersed_volume = 0.0;
    dvec3 weighted_sum(0.0);
    
    dmat3 R_mat = dmat3(rb.q);
    
    for (int i = 0; i < num_slices; i++) {
        double local_y = -R + (i + 0.5) * slice_height;
        dvec3 world_center = rb.r + R_mat * dvec3(0, local_y, 0);
        
        if (world_center.y < y_water_level) {
            double r_slice = std::sqrt(R * R - local_y * local_y);
            double slice_volume = M_PI * r_slice * r_slice * slice_height;
            total_immersed_volume += slice_volume;
            weighted_sum += slice_volume * world_center;
        }
    }
    
    return (total_immersed_volume > 0) ? weighted_sum / total_immersed_volume : rb.r;
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
            
            double drag_coef_shape;
            double area;
            switch (context.body_type) {
                case BODY_CUBE:
                    drag_coef_shape = 1.05;
                    area = CUBE_SIZE * CUBE_SIZE;
                    break;
                case BODY_SPHERE:
                    drag_coef_shape = 0.47;
                    area = M_PI * SPHERE_RADIUS * SPHERE_RADIUS;
                    break;
                case BODY_CYLINDER:
                    drag_coef_shape = 0.82;
                    area = 2.0 * CYLINDER_RADIUS * CYLINDER_HEIGHT;
                    break;
            }
            
            double drag_force_magnitude = 0.5 * rho_medium * drag_coef_shape * area * speed * speed;
            total_force += -drag_force_magnitude * (velocity / speed);
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
        
        torque = -0.1 * (rho_medium / context.ro_air) * omega;
    }
    
    double immersion = CalculateImmersedVolume(rb, context);
    if (immersion > 0.0 && immersion < 1.0) {
        double archimedes_magnitude = context.ro_liquid * context.volume * immersion * context.g;
        dvec3 archimedes_force(0, archimedes_magnitude, 0);
        dvec3 center_immersed = GetImmersedCenter(rb, context);
        torque += cross(center_immersed - rb.r, archimedes_force);
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
    RigidBody k1 = f_rigidbody(rb, context, cur_time);
    
    RigidBody temp = SumRB(rb, MulRB(k1, h / 2.0));
    RigidBody k2 = f_rigidbody(temp, context, cur_time + h / 2.0);
    
    temp = SumRB(rb, MulRB(k2, h / 2.0));
    RigidBody k3 = f_rigidbody(temp, context, cur_time + h / 2.0);
    
    temp = SumRB(rb, MulRB(k3, h));
    RigidBody k4 = f_rigidbody(temp, context, cur_time + h);
    
    RigidBody delta = SumRB(
        SumRB(k1, MulRB(k2, 2.0)),
        SumRB(MulRB(k3, 2.0), k4)
    );
    
    rb = SumRB(rb, MulRB(delta, h / 6.0));
    rb.q = normalize(rb.q);
    
    // Расчет энергии для отладки
    dvec3 velocity = rb.l * context.M_inv;
    double E_kin_trans = 0.5 * context.mass * dot(velocity, velocity);
    
    dmat3 R = dmat3(rb.q);
    dvec3 omega = R * context.I_inv * transpose(R) * rb.L;
    double E_kin_rot = 0.5 * dot(omega, rb.L);
    
    double E_pot = context.mass * context.g * rb.r.y;
    
    double immersion = CalculateImmersedVolume(rb, context);
    dvec3 immersed_center = GetImmersedCenter(rb, context);
    double E_pot_arch = -context.ro_liquid * context.volume * immersion * context.g * immersed_center.y;
    
    double total_energy = E_kin_trans + E_kin_rot + E_pot + E_pot_arch;
    
    static double last_energy_print = 0;
    if (body_index == current_body && cur_time - last_energy_print > 0.05) {
        std::cout << "\nenergy check\n";
        std::cout << "time: " << cur_time << " s\n";
        std::cout << "total energy: " << total_energy << "\n";
        std::cout << "  E_kin_trans: " << E_kin_trans << "\n";
        std::cout << "  E_kin_rot: " << E_kin_rot << "\n";
        std::cout << "  E_pot: " << E_pot << "\n";
        std::cout << "  E_pot_arch: " << E_pot_arch << "\n";
        std::cout << "  Position Y: " << rb.r.y << "\n";
        std::cout << "  Immersed center Y: " << immersed_center.y << "\n";
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

dvec3 GetCylinderPointWorld(const RigidBody &rb, const dvec3 &local_point) {
    return rb.r + dmat3(rb.q) * local_point;
}

void GetCylinderYRange(const RigidBody &rb, double radius, double height, double &min_y, double &max_y) {
    std::vector<dvec3> local_points;
    
    for (int i = 0; i < 8; i++) {
        double angle = 2.0 * M_PI * i / 8.0;
        local_points.push_back(dvec3(radius * cos(angle), -height/2.0, radius * sin(angle)));
    }
    local_points.push_back(dvec3(0, -height/2.0, 0));
    
    for (int i = 0; i < 8; i++) {
        double angle = 2.0 * M_PI * i / 8.0;
        local_points.push_back(dvec3(radius * cos(angle), height/2.0, radius * sin(angle)));
    }
    local_points.push_back(dvec3(0, height/2.0, 0));
    
    min_y = INFINITY;
    max_y = -INFINITY;
    
    for (const auto &local_pt : local_points) {
        double world_y = GetCylinderPointWorld(rb, local_pt).y;
        min_y = std::min(min_y, world_y);
        max_y = std::max(max_y, world_y);
    }
}

double CalculateCylinderImmersedVolumeWithRotation(const RigidBody &rb, const Context &context, double y_water_level) {
    double radius = CYLINDER_RADIUS;
    double height = CYLINDER_HEIGHT;
    
    double min_y, max_y;
    GetCylinderYRange(rb, radius, height, min_y, max_y);
    
    if (min_y >= y_water_level) return 0.0;
    if (max_y <= y_water_level) return 1.0;
    
    const int num_slices = 500000;
    double slice_height = height / num_slices;
    double immersed_volume = 0.0;
    
    dmat3 R = dmat3(rb.q);
    double slice_volume = M_PI * radius * radius * slice_height;
    
    for (int i = 0; i < num_slices; i++) {
        double local_y = -height/2.0 + (i + 0.5) * slice_height;
        dvec3 world_center = rb.r + R * dvec3(0, local_y, 0);
        
        if (world_center.y < y_water_level) {
            immersed_volume += slice_volume;
        }
    }
    
    double total_volume = M_PI * radius * radius * height;
    return std::min(1.0, std::max(0.0, immersed_volume / total_volume));
}

dvec3 CalculateCylinderImmersedCenter(const RigidBody &rb, const Context &context, double y_water_level) {
    double radius = CYLINDER_RADIUS;
    double height = CYLINDER_HEIGHT;
    
    double min_y, max_y;
    GetCylinderYRange(rb, radius, height, min_y, max_y);
    
    if (min_y >= y_water_level) return rb.r;
    if (max_y <= y_water_level) return rb.r;
    
    const int num_slices = 500000;
    double slice_height = height / num_slices;
    double total_immersed_volume = 0.0;
    dvec3 weighted_sum(0.0);
    
    dmat3 R = dmat3(rb.q);
    double slice_volume = M_PI * radius * radius * slice_height;
    
    for (int i = 0; i < num_slices; i++) {
        double local_y = -height/2.0 + (i + 0.5) * slice_height;
        dvec3 world_center = rb.r + R * dvec3(0, local_y, 0);
        
        if (world_center.y < y_water_level) {
            total_immersed_volume += slice_volume;
            weighted_sum += slice_volume * world_center;
        }
    }
    
    return (total_immersed_volume > 0) ? weighted_sum / total_immersed_volume : rb.r;
}

dvec3 GetImmersedCenter(const RigidBody &rb, const Context &context) {
    switch (context.body_type) {
        case BODY_CUBE:
            return CalculateCubeImmersedCenter(rb, 0.0);
        
        case BODY_SPHERE:
            return CalculateSphereImmersedCenter(rb, 0.0);
        
        case BODY_CYLINDER:
            return CalculateCylinderImmersedCenter(rb, context, 0.0);
    }
    
    return rb.r;
}