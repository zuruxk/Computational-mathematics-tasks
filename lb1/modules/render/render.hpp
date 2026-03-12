#pragma once

#include <GL/glut.h>

#include "../dif/dif.hpp"

extern Context context;
extern RigidBody rb;

void DrawCube (int size);
void DrawAxes ();
void DrawWaterSurface (float y_level);
void Reshape (int W, int H);
void Display ();
void Idle ();
void Keyboard (unsigned char Key, int MouseX, int MouseY);

void Run (int argc, char *argv[]);