///////////////////////////////////////////////////////////////////////////
//
// NAME
//  BlendImages.h -- blend together a set of overlapping images
//
// SPECIFICATION
//  CImg<float> BlendImages(CImagePositionV ipv, float f, float blendRadius);
//
// PARAMETERS
//  ipv                 list (vector) of images and their locations
//  f                   focal length
//  blendRadius         half-width of transition region (in pixels)
//
// DESCRIPTION
//  This routine takes a collection of images aligned more or less horizontally
//  and stitches together a mosaic.
//
//  The images can be blended together any way you like, but I would recommend
//  using a soft halfway blend of the kind Steve presented in the first lecture.
//
// SEE ALSO
//  BlendImages.cpp     implementation
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////
#include "CImg.h"
using namespace cimg_library;

struct CImagePosition
{
    CImg<float> img;         // image
	CTransform3x3 position; // global rotation
};

typedef std::vector<CImagePosition> CImagePositionV;

CImg<float> BlendImages(CImagePositionV& ipv, float f, float blendWidth);
