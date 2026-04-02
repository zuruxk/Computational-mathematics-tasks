#pragma once

#include <GL/glut.h>
#include <vector>
#include "../dif/dif.hpp"

extern std::vector<Context> contexts;
extern std::vector<RigidBody> bodies;
extern int current_body;

void DrawCube(double size);
void DrawSphere(double radius);
void DrawCylinder(double radius, double height);
void DrawAxes();
void DrawWaterSurface(float y_level);
void DrawBottom(float y_level);
void Reshape(int W, int H);
void Display();
void Idle();
void Keyboard(unsigned char Key, int MouseX, int MouseY);
void Run(int argc, char *argv[]);