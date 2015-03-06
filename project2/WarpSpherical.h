///////////////////////////////////////////////////////////////////////////
//
// NAME
//  WarpSpherical.h -- warp a flat (perspective) image into spherical
//      coordinates or undo radial lens distortion
//
// SPECIFICATION (WarpRDField)
//  CImg<float> WarpRDField(CShape sh, float f, float k1, float k2)
// PARAMETERS
//  f                   focal length, in pixels
//  k1, k2              radial distortion parameters
//
// DESCRIPTION
//  WarpRDField produces a pixel coordinate image suitable
//  for correcting radial distortion in an image.
//
//  Use the output of WarpRDField as the input to WarpLocal,
//  along with a source and destination image, to actually
//  perform the warp.
//
// SPECIFICATION (WarpSphericalField)
//  CImg<float> WarpSphericalField(CShape sh, float f, const CTransform3x3 &r)
// PARAMETERS
//  sh                  shape of destination image
//  f                   focal length, in pixels
//  r                   rotation matrix
//
// DESCRIPTION
//  WarpSphericalField produces a pixel coordinate image suitable
//  for mapping the image into spherical coordinates.
//
//  Use the output of WarpSphericalField as the input to 
//  WarpLocal, along with a source and destination image, to actually
//  perform the warp.
//
// SEE ALSO
//  WarpSpherical.cpp implementation
//  WarpImage.h         image warping code
//  Image.h             image class definition
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////


#include "CImg.h" 

using namespace cimg_library;

#ifndef M_PI 
#define M_PI    3.1415926536f
#endif // M_PI

CImg<float> WarpRDField(CShape srcSh, CShape dstSh, float f, float k1, float k2);

CImg<float> WarpSphericalField(CShape srcSh, CShape dstSh, float f, const CTransform3x3 &r);
