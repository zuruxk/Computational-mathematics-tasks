#include <cmath>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "render.hpp"

#define WIDTH 1000
#define HEIGHT 600
#define TIME_SCALE 1000

double OldTime = -1, DeltaTime;
RigidBody rb;
Context context;
double simulation_time = 0.0;

void DrawCube (int size) {
    glBegin(GL_QUADS);

    glColor3f(0.0f, size, 0.0f);
    glVertex3f(size, size, -size);
    glVertex3f(-size, size, -size);
    glVertex3f(-size, size, size);
    glVertex3f(size, size, size);

    glColor3f(size, 0.5f, 0.0f);
    glVertex3f(size, -size, size);
    glVertex3f(-size, -size, size);
    glVertex3f(-size, -size, -size);
    glVertex3f(size, -size, -size);

    glColor3f(size, 0.0f, 0.0f);
    glVertex3f(size, size, size);
    glVertex3f(-size, size, size);
    glVertex3f(-size, -size, size);
    glVertex3f(size, -size, size);

    glColor3f(size, size, 0.0f);
    glVertex3f(size, -size, -size);
    glVertex3f(-size, -size, -size);
    glVertex3f(-size, size, -size);
    glVertex3f(size, size, -size);

    glColor3f(0.0f, 0.0f, size);
    glVertex3f(-size, size, size);
    glVertex3f(-size, size, -size);
    glVertex3f(-size, -size, -size);
    glVertex3f(-size, -size, size);

    glColor3f(size, 0.0f, size);
    glVertex3f(size, size, -size);
    glVertex3f(size, size, size);
    glVertex3f(size, -size, size);
    glVertex3f(size, -size, -size);

    glEnd();
}

void DrawAxes () {
    glBegin(GL_LINES);

    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-500.0f, 0.0f, 0.0f);
    glVertex3f(500.0f, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, -500.0f, 0.0f);
    glVertex3f(0.0f, 500.0f, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, -500.0f);
    glVertex3f(0.0f, 0.0f, 500.0f);
    
    glEnd();
}

void DrawWaterSurface (float y_level) {
    glColor4f(0.0f, 0.5f, 1.0f, 0.3f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glBegin(GL_QUADS);
    glVertex3f(-500.0f, y_level, -500.0f);
    glVertex3f(500.0f, y_level, -500.0f);
    glVertex3f(500.0f, y_level, 500.0f);
    glVertex3f(-500.0f, y_level, 500.0f);
    glEnd();
    
    glDisable(GL_BLEND);
}

void Reshape (int W, int H) {
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, (double)W / H, 1, 500);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void Display () {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    double energy = SolveRungeKutta4(rb, context, DeltaTime, simulation_time);

    static double last_print = 0;
    if (OldTime - last_print > 1.0) {
        std::cout<<"Time: "<<std::fixed<<std::setprecision(2)<<simulation_time<<"s, pos: "<<rb.r.y<<"m"<<std::endl;
        last_print = simulation_time;
    }

    glPushMatrix();
        gluLookAt(80, 100, 300, 
        0, 10, -20,
        0, 1, 0 );

    DrawAxes();
    DrawWaterSurface(0.0f);
    glTranslated(rb.r.x, rb.r.y, rb.r.z);
    rb.q = glm::normalize(rb.q);

    double angle = glm::degrees(2.0 * acos(rb.q.w));
    glRotated(angle, rb.q.x, rb.q.y, rb.q.z);
    
    DrawCube(SIZE);

    glPointSize(5);
    glBegin(GL_POINTS);
    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glEnd();

    glPopMatrix();
    glFlush();
    glutSwapBuffers();
}

void Idle () {
    long Time = clock();
    if (OldTime == -1) {
        OldTime = Time;
    }
    
    DeltaTime = (double)(Time - OldTime) / CLOCKS_PER_SEC;
    if (DeltaTime > 0.02) DeltaTime = 0.02;
    
    simulation_time += DeltaTime;
    
    OldTime = Time;
    glutPostRedisplay();
}

void Keyboard (unsigned char Key, int MouseX, int MouseY) {
    if (Key == 27)
    exit(0);

    if (Key == 'd' || Key == 'D') {
        context.drag_enabled = !context.drag_enabled;
        std::cout<<"Drag "<<(context.drag_enabled ? "ENABLED" : "DISABLED")<<std::endl;
    }

    if (Key == 'r' || Key == 'R') {
        rb.r = dvec3(0, 5, -20);
        rb.l = dvec3(0, 0, 0);
        rb.L = dvec3(10, 5, 0);
        rb.q = dquat(1, 0, 0, 0);
        std::cout<<"Position reset"<< std::endl;
    }
}

void Run (int argc, char *argv[]) {
    double mass = 60.0;
    double side = 0.464;
    double volume = side * side * side;

    context.M_inv = 1.0 / mass;

    double I = (1.0/6.0) * mass * side * side;
    for (int i = 0; i < 3; i++)
        context.I_inv[i][i] = 1.0 / I;

    context.mass = mass;
    context.volume = volume;
    context.ro_liquid = 800.0;
    context.ro_air = 1.2;
    context.g = 9.81;
    context.drag_coef = 150.0;
    context.drag_enabled = false;
    context.start_time = 0.0;

    rb.r = dvec3(0, 70, -20);
    rb.q = dquat(1, 0, 0, 0);
    rb.l = dvec3(0, 0, 0);
    rb.L = dvec3(10, 5, 0);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutInitWindowPosition(0, 0);
    glutCreateWindow("Body motion in fluid");

    glClearColor(0.8, 0.8, 0.8, 0);

    glutReshapeFunc(Reshape);
    glutDisplayFunc(Display);
    glutIdleFunc(Idle);
    glutKeyboardFunc(Keyboard);

    glEnable(GL_DEPTH_TEST);

    std::cout<<"Controls:\n";
    std::cout<<"  ESC - exit\n";
    std::cout<<"  D - toggle drag force\n";
    std::cout<<"  R - reset position\n";

    glutMainLoop();
}