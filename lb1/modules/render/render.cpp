#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <vector>
#include "render.hpp"

#define WIDTH 1200
#define HEIGHT 700

double simulation_speed = 1.0;
bool real_time_mode = true;
double physics_time = 0.0;
double last_real_time = 0.0;

double OldTime = -1, DeltaTime;
std::vector<Context> contexts;
std::vector<RigidBody> bodies;
int current_body = 0;
double simulation_time = 0.0;

void DrawCube(double size) {
    double s = size / 2.0;
    glBegin(GL_QUADS);

    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(s, s, -s);
    glVertex3f(-s, s, -s);
    glVertex3f(-s, s, s);
    glVertex3f(s, s, s);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(s, -s, s);
    glVertex3f(-s, -s, s);
    glVertex3f(-s, -s, -s);
    glVertex3f(s, -s, -s);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(s, s, s);
    glVertex3f(-s, s, s);
    glVertex3f(-s, -s, s);
    glVertex3f(s, -s, s);

    glColor3f(1.0f, 1.0f, 0.0f);
    glVertex3f(s, -s, -s);
    glVertex3f(-s, -s, -s);
    glVertex3f(-s, s, -s);
    glVertex3f(s, s, -s);

    glColor3f(1.0f, 0.0f, 1.0f);
    glVertex3f(-s, s, s);
    glVertex3f(-s, s, -s);
    glVertex3f(-s, -s, -s);
    glVertex3f(-s, -s, s);

    glColor3f(0.0f, 1.0f, 1.0f);
    glVertex3f(s, s, -s);
    glVertex3f(s, s, s);
    glVertex3f(s, -s, s);
    glVertex3f(s, -s, -s);
    
    glEnd();
}

void DrawSphere(double radius) {
    glutSolidSphere(radius, 20, 20);
}

void DrawCylinder(double radius, double height) {
    GLUquadric *quadric = gluNewQuadric();

    glRotated(-90, 1, 0, 0);

    glPushMatrix();
    glTranslated(0, 0, -height/2); 
    gluCylinder(quadric, radius, radius, height, 20, 20);
    glPopMatrix();

    glPushMatrix();
    glTranslated(0, 0, -height/2);
    gluDisk(quadric, 0, radius, 20, 1);
    glPopMatrix();

    glPushMatrix();
    glTranslated(0, 0, +height/2);
    gluDisk(quadric, 0, radius, 20, 1);
    glPopMatrix();
    
    gluDeleteQuadric(quadric);
}

void DrawAxes() {
    glBegin(GL_LINES);
    
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-30.0f, 0.0f, 0.0f);
    glVertex3f(30.0f, 0.0f, 0.0f);
    
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, -30.0f, 0.0f);
    glVertex3f(0.0f, 30.0f, 0.0f);
    
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, -30.0f);
    glVertex3f(0.0f, 0.0f, 30.0f);
    
    glEnd();
}

void DrawWaterSurface(float y_level) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(0.0f, 0.5f, 1.0f, 0.2f);

    glBegin(GL_QUADS);
    glVertex3f(-18.0f, y_level, -18.0f);
    glVertex3f(18.0f, y_level, -18.0f);
    glVertex3f(18.0f, y_level, 18.0f);
    glVertex3f(-18.0f, y_level, 18.0f);
    glEnd();

    glColor4f(0.0f, 0.3f, 0.8f, 0.1f);
    glBegin(GL_LINES);
    for (int i = -18; i <= 18; i += 3) {
        glVertex3f(i, y_level, -18.0f);
        glVertex3f(i, y_level, 18.0f);
        glVertex3f(-18.0f, y_level, i);
        glVertex3f(18.0f, y_level, i);
    }
    glEnd();
    
    glDisable(GL_BLEND);
}

void DrawBottom(float y_level) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glColor4f(0.8f, 0.6f, 0.2f, 0.8f);
    glBegin(GL_QUADS);
    glVertex3f(-18.0f, y_level, -18.0f);
    glVertex3f(18.0f, y_level, -18.0f);
    glVertex3f(18.0f, y_level, 18.0f);
    glVertex3f(-18.0f, y_level, 18.0f);
    glEnd();

    glColor4f(0.0f, 0.3f, 0.8f, 0.1f);
    glBegin(GL_LINES);
    for (int i = -18; i <= 18; i += 3) {
        glVertex3f(i, y_level, -18.0f);
        glVertex3f(i, y_level, 18.0f);
        glVertex3f(-18.0f, y_level, i);
        glVertex3f(18.0f, y_level, i);
    }
    glEnd();
    
    glDisable(GL_BLEND);
}

void Reshape(int W, int H) {
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (double)W / H, 1, 200);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    for (size_t i = 0; i < bodies.size(); i++) {
        double energy = SolveRungeKutta4(bodies[i], contexts[i], DeltaTime, physics_time, i, current_body);
        
        double bottom_y = bodies[i].r.y;
        switch (contexts[i].body_type) {
            case BODY_CUBE: bottom_y -= CUBE_SIZE/2; break;
            case BODY_SPHERE: bottom_y -= SPHERE_RADIUS; break;
            case BODY_CYLINDER: bottom_y -= CYLINDER_HEIGHT/2; break;
        }
        
        if (bottom_y < -15.0) {
            bodies[i].r.y = -15.0 + (contexts[i].body_type == BODY_SPHERE ? SPHERE_RADIUS : 
                           (contexts[i].body_type == BODY_CUBE ? CUBE_SIZE/2 : CYLINDER_HEIGHT/2));
            bodies[i].l = dvec3(0, 0, 0);
            bodies[i].L = dvec3(0, 0, 0);
        }
    }
    
    static double last_print = 0;
    if (physics_time - last_print > 1.0) {
        std::cout << "\rTime: " << std::fixed << std::setprecision(2) 
                  << physics_time << "s | Speed: " << simulation_speed << "x | "
                  << "Mode: " << (contexts[0].drag_enabled ? "DRAG" : "NO DRAG") << " | "
                  << "Time mode: " << (real_time_mode ? "REAL" : "FIXED") << "    " << std::flush;
        
        static int counter = 0;
        if (counter++ % 5 == 0) {
            std::cout << std::endl;
            for (size_t i = 0; i < bodies.size(); i++) {
                std::cout << "  Body " << i << " pos: " << bodies[i].r.y << "m" << std::endl;
            }
        }
        last_print = physics_time;
    }
    
    glLoadIdentity();
    gluLookAt(30, 15, 50,
              0, 0, 0,
              0, 1, 0);
    
    DrawAxes();
    DrawWaterSurface(0.0f);
    DrawBottom(-15.0f);
    
    for (size_t i = 0; i < bodies.size(); i++) {
        glPushMatrix();

        glTranslated(bodies[i].r.x, bodies[i].r.y, bodies[i].r.z - 10.0 * i);

        dquat q = normalize(bodies[i].q);
        double angle = degrees(2.0 * acos(q.w));
        glRotated(angle, q.x, q.y, q.z);
        
        switch (contexts[i].body_type) {
            case BODY_CUBE:
                glColor3f(1.0f, 0.0f, 0.0f);
                DrawCube(CUBE_SIZE);
                break;
            case BODY_SPHERE:
                glColor3f(0.0f, 1.0f, 0.0f);
                DrawSphere(SPHERE_RADIUS);
                break;
            case BODY_CYLINDER:
                glColor3f(0.0f, 0.0f, 1.0f);
                DrawCylinder(CYLINDER_RADIUS, CYLINDER_HEIGHT);
                break;
        }
        
        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3f(1, 1, 1);
        glVertex3f(0, 0, 0);
        glEnd();
        
        glPopMatrix();
    }
    
    glutSwapBuffers();
}

void Idle() {
    long current_clock = clock();
    double current_real_time = (double)current_clock / CLOCKS_PER_SEC;
    
    if (OldTime == -1) {
        OldTime = current_clock;
        last_real_time = current_real_time;
    }
    
    if (real_time_mode) {
        double real_dt = current_real_time - last_real_time;

        if (real_dt > 0.002) real_dt = 0.002;

        DeltaTime = real_dt * simulation_speed;
        
        last_real_time = current_real_time;
    }
    else {
        DeltaTime = (double)(current_clock - OldTime) / CLOCKS_PER_SEC;
        if (DeltaTime > 0.002) DeltaTime = 0.002;
        DeltaTime *= simulation_speed;
    }
    
    physics_time += DeltaTime;
    simulation_time = physics_time;
    
    OldTime = current_clock;
    glutPostRedisplay();
}

void Keyboard(unsigned char Key, int MouseX, int MouseY) {
    if (Key == 27) exit(0);
    
    if (Key == 'd' || Key == 'D') {
        for (auto &context : contexts) {
            context.drag_enabled = !context.drag_enabled;
        }
        std::cout << "Drag " << (contexts[0].drag_enabled ? "ENABLED" : "DISABLED") << std::endl;
    }
    
    if (Key == 'r' || Key == 'R') {
        for (size_t i = 0; i < bodies.size(); i++) {
            bodies[i].q = dquat(1, 0, 0, 0);
            bodies[i].l = dvec3(0, 0, 0);
            bodies[i].L = dvec3(0.5, 0.2, 0.1);
        }
        bodies[0].r = dvec3(0, 20, 0);
        bodies[1].r = dvec3(0, 20, 20);
        bodies[2].r = dvec3(10, 20, 10);

        physics_time = 0.0;
        std::cout << "Position reset" << std::endl;
    }
    
    if (Key == 'c' || Key == 'C') {
        current_body = (current_body + 1) % bodies.size();
        std::cout << "Selected body " << current_body << std::endl;
    }

    if (Key == '+') {
        simulation_speed *= 1.5;
        std::cout << "Simulation speed: " << simulation_speed << "x" << std::endl;
    }
    
    if (Key == '-') {
        simulation_speed /= 1.5;
        if (simulation_speed < 0.1) simulation_speed = 0.1;
        std::cout << "Simulation speed: " << simulation_speed << "x" << std::endl;
    }
    
    if (Key == ' ') {
        real_time_mode = !real_time_mode;
        std::cout << "Time mode: " << (real_time_mode ? "REAL TIME" : "FIXED STEP") << std::endl;
    }
    
    if (Key == 'p' || Key == 'P') {
        static bool paused = false;
        paused = !paused;
        if (paused) {
            glutIdleFunc(NULL);
            std::cout << "PAUSED" << std::endl;
        }
        else {
            glutIdleFunc(Idle);
            last_real_time = (double)clock() / CLOCKS_PER_SEC;
            std::cout << "RESUMED" << std::endl;
        }
    }
}

void Run(int argc, char *argv[]) {
    bodies.resize(3);
    contexts.resize(3);
    
    InitCube(contexts[0], 2400.0);
    bodies[0].r = dvec3(0, 5, 0);
    bodies[0].q = dquat(1, 0, 0, 0);
    bodies[0].l = dvec3(0, 0, 0);
    bodies[0].L = dvec3(300.0, 2.0, 1.0);
    
    InitSphere(contexts[1], 1000.0);
    bodies[1].r = dvec3(0, 5, 20);
    bodies[1].q = dquat(1, 0, 0, 0);
    bodies[1].l = dvec3(0, 0, 0);
    bodies[1].L = dvec3(0.0, 0.0, 0.0);
    
    InitCylinder(contexts[2], 255.0);
    bodies[2].r = dvec3(10, 5, 10);
    bodies[2].q = dquat(1, 0, 0, 0);
    bodies[2].l = dvec3(0, 0, 0);
    bodies[2].L = dvec3(200.0, 0.0, 0.0);
    
    for (auto &context : contexts) {
        context.ro_liquid = 977.0;
        context.ro_air = 1.2;
        context.g = 9.81;
        context.drag_coef_linear = 0.5;
        context.drag_enabled = false;
        context.start_time = 0.0;
    }
    
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Bodies motion in fluid");
    
    glClearColor(0.8, 0.8, 0.8, 0);
    glEnable(GL_DEPTH_TEST);
    
    glutReshapeFunc(Reshape);
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
    glutKeyboardFunc(Keyboard);
    
    std::cout << "Controls:\n";
    std::cout << "  ESC - exit\n";
    std::cout << "  D - toggle drag force\n";
    std::cout << "  R - reset position\n";
    std::cout << "  C - switch selected body\n";
    std::cout << "  P - pause\n";
    
    glutMainLoop();
}