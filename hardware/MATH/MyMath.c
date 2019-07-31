
#include <stdint.h>
#include "sysType.h"
#include "MyMath.h"
#define EXTRA_PRECISION
/////-PI ---- PI
#define FBPI  1.273240f
#define F2CPI -0.405285f


#define FAST_ATAN2_PIBY2_FLOAT  1.5707963f
// fast_atan2 - faster version of atan2
//      126 us on AVR cpu vs 199 for regular atan2
//      absolute error is < 0.005 radians or 0.28 degrees
//      origin source: https://gist.github.com/volkansalma/2972237/raw/
float fast_atan2(float y, float x)
{
   if (x == 0.0f) {
       if (y > 0.0f) {
           return FAST_ATAN2_PIBY2_FLOAT;
       }
       if (y == 0.0f) {
           return 0.0f;
       }
       return -FAST_ATAN2_PIBY2_FLOAT;
   }
   float atan;
   float z = y/x;
   if (fabs( z ) < 1.0f) {
       atan = z / (1.0f + 0.28f * z * z);
       if (x < 0.0f) {
           if (y < 0.0f) {
               return atan - PI;
           }
           return atan + PI;
       }
   } else {
       atan = FAST_ATAN2_PIBY2_FLOAT - (z / (z * z + 0.28f));
       if (y < 0.0f) {
           return atan - PI;
       }
   }
   return atan;
}


//const float B = 1.273240f;//4.0f/PI; 
  //const float C = -0.405285f;//-4.0f/(PI*PI);
 // const float B = 4/PI; 
 //    const float C = -4/(PI*PI);
  void fmod32(float * x,float mod)
{
    int32_t tmp =0;
  tmp = (int32_t)(*x/mod);//2*pi
  *x = *x - ((float)tmp)*mod;//(-2*PI --- 2*PI)
  //  if(*x>PI) *x-=TwoPI;
  // else if(*x<-PI) *x+=TwoPI;

}

  float f32sin(float x)
{
    float y=0.0f;
#if 0
  while(x>PI) x-=TwoPI;
  while(x<-PI) x+=TwoPI;
#else  
  int32_t tmp =0;
  if(fabs(x)>TwoPI){
  tmp = (int32_t)(x/TwoPI);//2*pi
  x = x - ((float)tmp)*TwoPI;//(-2*PI --- 2*PI)
  }
   if(x>PI) x-=TwoPI;
  else if(x<-PI) x+=TwoPI;
#endif
     y = FBPI * x + F2CPI * x * fabs(x);
#ifdef EXTRA_PRECISION // const float Q = 0.775; 
 // const float P = 0.225f;

   y = 0.225f * (y * fabs(y) - y) + y;

 // Q * y + P * y * fabs(y)

#endif 
  return y;
 }


  float f32cos(float x)
{ 
   x+=M_PI_2;
#if 0
  while(x>PI) x-=TwoPI;
  while(x<-PI) x+=TwoPI;
#else
   int32_t tmp =0;
   if(fabs(x)>TwoPI){
    tmp =  (int32_t)(x/TwoPI);//2*pi
    x =x - ((float)tmp)*TwoPI;
   }
   
    if(x>PI) x-=TwoPI;
    else if(x<-PI) x+=TwoPI;
#endif
      float y = FBPI * x + F2CPI * x * fabs(x);
#ifdef EXTRA_PRECISION // const float Q = 0.775; 
   // const float P = 0.225f;

    y = 0.225f * (y * fabs(y) - y) + y;

 // Q * y + P * y * fabs(y)

#endif 
  return y;
 }

///https://blog.csdn.net/xhhjin/article/details/7007875
///https://diducoder.com/sotry-about-sqrt.html

  float Q_rsqrt( float number )
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;
    x2 = number * 0.5F;
    y   = number;
    i   = * ( long * ) &y;   // evil floating point bit level hacking
    i   = 0x5f3759df - ( i >> 1 ); // what the fuck?
    y   = * ( float * ) &i;
    y   = y * ( threehalfs - ( x2 * y * y ) ); // 1st iteration
    // y   = y * ( threehalfs - ( x2 * y * y ) ); // 2nd iteration, this can be removed
 
    // #ifndef Q3_VM
    // #ifdef __linux__
    //      assert( !isnan(y) ); // bk010122 - FPE?
    // #endif
    // #endif
    return y;
}
// 作者：jjwwwww 
// 来源：CSDN 
// 原文：https://blog.csdn.net/jjwwwww/article/details/82628549 
// 版权声明：本文为博主原创文章，转载请附上博文链接！


  float InvSqrt(float x)
{
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating VALUE 
    i = 0x5f375a86- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy

    return 1/x;
}


// 2D vector length
float pythagorous2(float a, float b) {
  
  return Q_rsqrt(a*a+b*b);//sqrtf(sq(a)+sq(b));
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
  return Q_rsqrt(a*a+b*b+c*c);//sqrtf(sq(a)+sq(b)+sq(c));
}

// https://blog.csdn.net/qq_26499321/article/details/73724763
