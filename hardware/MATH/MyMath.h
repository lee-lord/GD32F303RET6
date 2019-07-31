#ifndef _MYMATH_H_
#define _MYMATH_H_
#include "sysType.h"
#define fabs(x)  (x>0?x:-x)

#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 # define PI M_PI_F
#endif
#ifndef M_PI_2
 # define M_PI_2 1.570796326794897f
#endif

#ifndef TwoPI
#define  TwoPI 6.283185307179586f
#endif
//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))

float fast_atan2(float y, float x);


float InvSqrt(float x);
float Q_rsqrt( float number );
float f32cos(float x);
float f32sin(float x);
void fmod32(float * x,float mod);

  
#define one_by_sqrt3 0.57735026919f
#define two_by_sqrt3 1.15470053838f
#define sqrt3_by_2 0.86602540378f
#define PAIbyTri  1.047197551196598f   


#endif

