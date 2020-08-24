#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <Eigen/Dense>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <vector>
#include <algorithm>
#include <string.h>

using namespace Eigen;
typedef unsigned char boolean_T;
using namespace std;

#define pi 3.141592653589793

extern double m0 ;
extern double m1 ;
extern double m2 ;
extern double m3 ;

extern double a0 ;
extern double d3 ;
extern double d4 ;
extern double a1 ;
extern double a2 ;
extern double a3 ;
extern double d1 ;
extern double d2 ;
extern double tool ;

extern double cosinetheta ; //ÃèÊö»úĞµ±ÛÓëÔ²°ë¾¶µÄ¼Ğ½ÇÓàÏÒÖµ

extern Vector2d theta1Range;
extern Vector2d theta2Range;
extern Vector2d theta3Range;
extern Vector2d theta4Range;

extern double tinterval;

void error(string msg);
void warning(string msg);

#endif