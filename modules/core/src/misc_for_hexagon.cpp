/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


#include "precomp.hpp"

#ifdef __hexagon__
#include "hexagon_protos.h"

namespace cv
{

//////////////////////////////// 2D Point_<int> for hexagon ////////////////////////////////

float norm(const Point_<int>& pt)
{
    return std::sqrt((float)pt.x*pt.x + (float)pt.y*pt.y);
}

int dot(const Point_<int>& a, const Point_<int>& b)
{ return saturate_cast<int>(a.x*b.x + a.y*b.y); }

Point_<int>& operator += (Point_<int>& a, const Point_<int>& b)
{
    a.x = saturate_cast<int>(a.x + b.x);
    a.y = saturate_cast<int>(a.y + b.y);
    return a;
}

Point_<int>& operator -= (Point_<int>& a, const Point_<int>& b)
{
    a.x = saturate_cast<int>(a.x - b.x);
    a.y = saturate_cast<int>(a.y - b.y);
    return a;
}

Point_<int>& operator *= (Point_<int>& a, int b)
{
    a.x = saturate_cast<int>(a.x*b);
    a.y = saturate_cast<int>(a.y*b);
    return a;
}

Point_<int>& operator *= (Point_<int>& a, float b)
{
    a.x = saturate_cast<int>(a.x*b);
    a.y = saturate_cast<int>(a.y*b);
    return a;
}

Point_<int>& operator *= (Point_<int>& a, double b)
{
    a.x = saturate_cast<int>(a.x*b);
    a.y = saturate_cast<int>(a.y*b);
    return a;
}

bool operator == (const Point_<int>& a, const Point_<int>& b)
{ return a.x == b.x && a.y == b.y; }

bool operator != (const Point_<int>& a, const Point_<int>& b)
{ return a.x != b.x || a.y != b.y; }

Point_<int> operator + (const Point_<int>& a, const Point_<int>& b)
{ return Point_<int>( saturate_cast<int>(a.x + b.x), saturate_cast<int>(a.y + b.y) ); }

Point_<int> operator - (const Point_<int>& a, const Point_<int>& b)
{ return Point_<int>( saturate_cast<int>(a.x - b.x), saturate_cast<int>(a.y - b.y) ); }

Point_<int> operator - (const Point_<int>& a)
{ return Point_<int>( saturate_cast<int>(-a.x), saturate_cast<int>(-a.y) ); }

Point_<int> operator * (const Point_<int>& a, int b)
{ return Point_<int>( saturate_cast<int>(a.x*b), saturate_cast<int>(a.y*b) ); }

Point_<int> operator * (int a, const Point_<int>& b)
{ return Point_<int>( saturate_cast<int>(b.x*a), saturate_cast<int>(b.y*a) ); }

Point_<int> operator * (const Point_<int>& a, float b)
{ return Point_<int>( saturate_cast<int>(a.x*b), saturate_cast<int>(a.y*b) ); }

Point_<int> operator * (float a, const Point_<int>& b)
{ return Point_<int>( saturate_cast<int>(b.x*a), saturate_cast<int>(b.y*a) ); }

Point_<int> operator * (const Point_<int>& a, double b)
{ return Point_<int>( saturate_cast<int>(a.x*b), saturate_cast<int>(a.y*b) ); }

Point_<int> operator * (double a, const Point_<int>& b)
{ return Point_<int>( saturate_cast<int>(b.x*a), saturate_cast<int>(b.y*a) ); }

//////////////////////////////// 2D Point_<float> for hexagon ////////////////////////////////

double norm(const Point_<float>& pt)
{
    return std::sqrt((double)pt.x*pt.x + (double)pt.y*pt.y);
}

float dot(const Point_<float>& a, const Point_<float>& b)
{
    float t = Q6_R_sfmpy_RR(a.x, b.x);
    t       = Q6_R_sfmpyacc_RR(t, a.y, b.y);
    return t;
}

Point_<float>& operator += (Point_<float>& a, const Point_<float>& b)
{
    a.x = Q6_R_sfadd_RR(a.x, b.x);
    a.y = Q6_R_sfadd_RR(a.y, b.y);
    return a;
}

Point_<float>& operator -= (Point_<float>& a, const Point_<float>& b)
{
    a.x = Q6_R_sfsub_RR(a.x, b.x);
    a.y = Q6_R_sfsub_RR(a.y, b.y);
    return a;
}

Point_<float>& operator *= (Point_<float>& a, int b)
{
    float t = (float)b;
    a.x = Q6_R_sfmpy_RR(a.x, t);
    a.y = Q6_R_sfmpy_RR(a.y, t);
    return a;
}

Point_<float>& operator *= (Point_<float>& a, float b)
{
    a.x = Q6_R_sfmpy_RR(a.x, b);
    a.y = Q6_R_sfmpy_RR(a.y, b);
    return a;
}

Point_<float>& operator *= (Point_<float>& a, double b)
{
    a.x = saturate_cast<float>(a.x*b);
    a.y = saturate_cast<float>(a.y*b);
    return a;
}

bool operator == (const Point_<float>& a, const Point_<float>& b)
{ return a.x == b.x && a.y == b.y; }

bool operator != (const Point_<float>& a, const Point_<float>& b)
{ return a.x != b.x || a.y != b.y; }

Point_<float> operator + (const Point_<float>& a, const Point_<float>& b)
{ return Point_<float>( Q6_R_sfadd_RR(a.x, b.x), Q6_R_sfadd_RR(a.y, b.y) ); }

Point_<float> operator - (const Point_<float>& a, const Point_<float>& b)
{ return Point_<float>( Q6_R_sfsub_RR(a.x, b.x), Q6_R_sfsub_RR(a.y, b.y) ); }

Point_<float> operator - (const Point_<float>& a)
{ return Point_<float>( (-a.x), (-a.y) ); }

Point_<float> operator * (const Point_<float>& a, int b)
{
    float t = (float)b;
    return Point_<float>( Q6_R_sfmpy_RR(a.x, t), Q6_R_sfmpy_RR(a.y, t) );
}

Point_<float> operator * (int a, const Point_<float>& b)
{
    float t = (float)a;
    return Point_<float>( Q6_R_sfmpy_RR(b.x, t), Q6_R_sfmpy_RR(b.y, t) );
}

Point_<float> operator * (const Point_<float>& a, float b)
{ return Point_<float>( Q6_R_sfmpy_RR(a.x, b), Q6_R_sfmpy_RR(a.y, b) ); }

Point_<float> operator * (float a, const Point_<float>& b)
{ return Point_<float>( Q6_R_sfmpy_RR(b.x, a), Q6_R_sfmpy_RR(b.y, a) ); }

Point_<float> operator * (const Point_<float>& a, double b)
{ return Point_<float>( saturate_cast<float>(a.x*b), saturate_cast<float>(a.y*b) ); }

Point_<float> operator * (double a, const Point_<float>& b)
{ return Point_<float>( saturate_cast<float>(b.x*a), saturate_cast<float>(b.y*a) ); }


//////////////////////////////// 3D Point3_<float> for hexagon ////////////////////////////////

Point3_<float>& operator += (Point3_<float>& a, const Point3_<float>& b)
{
    a.x = saturate_cast<float>(a.x + b.x);
    a.y = saturate_cast<float>(a.y + b.y);
    a.z = saturate_cast<float>(a.z + b.z);
    return a;
}

Point3_<float>& operator -= (Point3_<float>& a, const Point3_<float>& b)
{
    a.x = saturate_cast<float>(a.x - b.x);
    a.y = saturate_cast<float>(a.y - b.y);
    a.z = saturate_cast<float>(a.z - b.z);
    return a;
}

Point3_<float>& operator *= (Point3_<float>& a, int b)
{
    a.x = saturate_cast<float>(a.x*b);
    a.y = saturate_cast<float>(a.y*b);
    a.z = saturate_cast<float>(a.z*b);
    return a;
}

Point3_<float>& operator *= (Point3_<float>& a, float b)
{
    a.x = saturate_cast<float>(a.x*b);
    a.y = saturate_cast<float>(a.y*b);
    a.z = saturate_cast<float>(a.z*b);
    return a;
}

Point3_<float>& operator *= (Point3_<float>& a, double b)
{
    a.x = saturate_cast<float>(a.x*b);
    a.y = saturate_cast<float>(a.y*b);
    a.z = saturate_cast<float>(a.z*b);
    return a;
}

bool operator == (const Point3_<float>& a, const Point3_<float>& b)
{ return a.x == b.x && a.y == b.y && a.z == b.z; }

bool operator != (const Point3_<float>& a, const Point3_<float>& b)
{ return a.x != b.x || a.y != b.y || a.z != b.z; }

Point3_<float> operator + (const Point3_<float>& a, const Point3_<float>& b)
{ return Point3_<float>( saturate_cast<float>(a.x + b.x),
                      saturate_cast<float>(a.y + b.y),
                      saturate_cast<float>(a.z + b.z)); }

Point3_<float> operator - (const Point3_<float>& a, const Point3_<float>& b)
{ return Point3_<float>( saturate_cast<float>(a.x - b.x),
                        saturate_cast<float>(a.y - b.y),
                        saturate_cast<float>(a.z - b.z)); }

Point3_<float> operator - (const Point3_<float>& a)
{ return Point3_<float>( saturate_cast<float>(-a.x),
                      saturate_cast<float>(-a.y),
                      saturate_cast<float>(-a.z) ); }

Point3_<float> operator * (const Point3_<float>& a, int b)
{
    float t = (float)b;
    return Point3_<float>(  Q6_R_sfmpy_RR(a.x, t),
                            Q6_R_sfmpy_RR(a.y, t),
                            Q6_R_sfmpy_RR(a.z, t) );
}

Point3_<float> operator * (int a, const Point3_<float>& b)
{
    float t = (float)a;
    return Point3_<float>(  Q6_R_sfmpy_RR(b.x, t),
                            Q6_R_sfmpy_RR(b.y, t),
                            Q6_R_sfmpy_RR(b.z, t) );
}

Point3_<float> operator * (const Point3_<float>& a, float b)
{
    return Point3_<float>(  Q6_R_sfmpy_RR(a.x, b),
                            Q6_R_sfmpy_RR(a.y, b),
                            Q6_R_sfmpy_RR(a.z, b) );
}


Point3_<float> operator * (float a, const Point3_<float>& b)
{
    return Point3_<float>(  Q6_R_sfmpy_RR(b.x, a),
                            Q6_R_sfmpy_RR(b.y, a),
                            Q6_R_sfmpy_RR(b.z, a) );

}

Point3_<float> operator * (const Point3_<float>& a, double b)
{ return Point3_<float>( saturate_cast<float>(a.x*b),
                      saturate_cast<float>(a.y*b),
                      saturate_cast<float>(a.z*b) ); }

Point3_<float> operator * (double a, const Point3_<float>& b)
{ return Point3_<float>( saturate_cast<float>(b.x*a),
                      saturate_cast<float>(b.y*a),
                      saturate_cast<float>(b.z*a) ); }

Point3_<float>& mpyacc(Point3_<float>& a, const Point3_<float>& b, int c)
{
    float t = (float)c;
    a.x = Q6_R_sfmpyacc_RR(a.x, b.x, t);
    a.y = Q6_R_sfmpyacc_RR(a.y, b.y, t);
    a.z = Q6_R_sfmpyacc_RR(a.z, b.z, t);
    return a;
}

Point3_<float>& mpyacc(Point3_<float>& a, int b, const Point3_<float>& c)
{
    float t = (float)b;
    a.x = Q6_R_sfmpyacc_RR(a.x, c.x, t);
    a.y = Q6_R_sfmpyacc_RR(a.y, c.y, t);
    a.z = Q6_R_sfmpyacc_RR(a.z, c.z, t);
    return a;
}

Point3_<float>& mpyacc(Point3_<float>& a, const Point3_<float>& b, float c)
{
    a.x = Q6_R_sfmpyacc_RR(a.x, b.x, c);
    a.y = Q6_R_sfmpyacc_RR(a.y, b.y, c);
    a.z = Q6_R_sfmpyacc_RR(a.z, b.z, c);
    return a;
}

Point3_<float>& mpyacc(Point3_<float>& a, float b, const Point3_<float>& c)
{
    a.x = Q6_R_sfmpyacc_RR(a.x, c.x, b);
    a.y = Q6_R_sfmpyacc_RR(a.y, c.y, b);
    a.z = Q6_R_sfmpyacc_RR(a.z, c.z, b);
    return a;
}

Point3_<float>& mpynac(Point3_<float>& a, const Point3_<float>& b, int c)
{
    float t = (float)c;
    a.x = Q6_R_sfmpynac_RR(a.x, b.x, t);
    a.y = Q6_R_sfmpynac_RR(a.y, b.y, t);
    a.z = Q6_R_sfmpynac_RR(a.z, b.z, t);
    return a;
}

Point3_<float>& mpynac(Point3_<float>& a, int b, const Point3_<float>& c)
{
    float t = (float)b;
    a.x = Q6_R_sfmpynac_RR(a.x, c.x, t);
    a.y = Q6_R_sfmpynac_RR(a.y, c.y, t);
    a.z = Q6_R_sfmpynac_RR(a.z, c.z, t);
    return a;
}

Point3_<float>& mpynac(Point3_<float>& a, const Point3_<float>& b, float c)
{
    a.x = Q6_R_sfmpynac_RR(a.x, b.x, c);
    a.y = Q6_R_sfmpynac_RR(a.y, b.y, c);
    a.z = Q6_R_sfmpynac_RR(a.z, b.z, c);
    return a;
}

Point3_<float>& mpynac(Point3_<float>& a, float b, const Point3_<float>& c)
{
    a.x = Q6_R_sfmpynac_RR(a.x, c.x, b);
    a.y = Q6_R_sfmpynac_RR(a.y, c.y, b);
    a.z = Q6_R_sfmpynac_RR(a.z, c.z, b);
    return a;
}


}

#endif


/* End of file. */
