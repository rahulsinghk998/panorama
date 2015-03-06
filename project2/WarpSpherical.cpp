///////////////////////////////////////////////////////////////////////////
//
// NAME
//  WarpSpherical.h -- warp a flat (perspective) image into spherical
//      coordinates or undo radial lens distortion
//
// SEE ALSO
//  WarpSpherical.h   longer description
//
//  R. Szeliski and H.-Y. Shum.
//  Creating full view panoramic image mosaics and texture-mapped models.
//  Computer Graphics (SIGGRAPH'97), pages 251-258, August 1997.
//
// Copyright ?Richard Szeliski, 2001.  See Copyright.h for more details
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "WarpSpherical.h"
#include <math.h>



/******************* TO DO *********************
 * warpRDField:
 *	INPUT:
 *		srcSh: shape (width, height, number of channels) of source image
 *		dstSh: shape of destination image
 *		f: focal length
 *		k1, k2: radial distortion parameters
 *	OUTPUT:
 *		Return an image containing (u,v) coordinates for applying
 *		radial distortion.
 *			Note that this is inverse warping, i.e., this routine will be
 *		actually used to map from planar coordinates with radial distortion
 *		to planar coordinates without radial distortion.
 *
 */
CImg<float> WarpRDField(CShape srcSh, CShape dstSh, float f, float k1, float k2)
{
    // Set up the pixel coordinate image
    dstSh.nBands = 2;
    CImg<float> uvImg(dstSh.width, dstSh.height, 1, dstSh.nBands);   // (u,v) coordinates

    // Fill in the values
    for (int y = 0; y < dstSh.height; y++)
    {
        //float *uv = &uvImg.Pixel(0, y, 0);
        //for (int x = 0; x < dstSh.width; x++, uv += 2)
		for (int x = 0; x < dstSh.width; x++)
        {
			// (x,y) is the planar image coordinates
            // (xf,yf) is the planar coordinates

            float xf = (x - 0.5f*dstSh.width) / f;
            float yf = (y - 0.5f*dstSh.height) / f;

			// (xt,yt) are intermediate coordinates to which you can
			// apply the radial distortion
            float xt = 0;
			float yt = 0;
 
			// *** BEGIN TODO ***
			// add code to distort with radial distortion coefficients
			// k1 and k2
			float r2 = pow(xf, 2) + pow(yf, 2);
			float divisor = 1.0f + k1 * r2 + k2 * pow(r2, 2);
			xt = xf / divisor;
			yt = yf / divisor;

			// *** END TODO ***

            // Convert back to regular pixel coordinates and store
            float xn = 0.5f*srcSh.width  + xt*f;
            float yn = 0.5f*srcSh.height + yt*f;

			uvImg(x, y, 0, 0) = xn;
			uvImg(x, y, 0, 1) = yn;
            /*uv[0] = xn;
            uv[1] = yn;*/
        }
    }
    return uvImg;
}

/******************* TO DO *********************
 * warpSphericalField:
 *	INPUT:
 *		srcSh: shape (width, height, number of channels) of source image
 *		dstSh: shape of destination image
 *		f: focal length in pixels (provided on the project web page, or measured by yourself)
 *		r: rotation matrix
 *	OUTPUT:
 *		Return an image containing (u,v) coordinates for mapping pixels from
 *		spherical coordinates to planar image coordinates.
 *			Note that this is inverse warping, i.e., this routine will be
 *		actually used to map from planar coordinates to spherical coordinates.
 *
 */
CImg<float> WarpSphericalField(CShape srcSh, CShape dstSh, float f, const CTransform3x3 &r)
{
    // Set up the pixel coordinate image
    dstSh.nBands = 2;
	CImg<float> uvImg(dstSh.width, dstSh.height, 1, dstSh.nBands);   // (u,v) coordinates

    // Fill in the values
    for (int y = 0; y < dstSh.height; y++)
    {
        //float *uv = &uvImg.Pixel(0, y, 0);
        for (int x = 0; x < dstSh.width; x++)
        {
			// (x,y) is the spherical image coordinates. 
            // (xf,yf) is the spherical coordinates, e.g., xf is the angle theta
			// and yf is the angle phi

			float xf = 2*M_PI * (x - 0.5f*dstSh.width) / dstSh.width;
			float yf = M_PI * (y - 0.5f*dstSh.height) / dstSh.height;

			// (xt,yt,zt) are intermediate coordinates to which you can
			// apply the spherical correction
            float xt=0, yt=0, zt=0;
			CVector3 p;
 
			// *** BEGIN TODO ***
			// add code to apply the spherical correction, i.e.,
			// compute the Euclidean coordinates, rotate according to
			// r, and project the point to the z=1 plane at
			// (xt/zt,yt/zt,1)
			//
			// don't forget to deal with points that have negative
			// z values appropriately, or you'll have each point on
			// the z=1 plane mapping to two opposite points on the
			// sphere
            xt = sin(xf) * cos(yf);
            yt = sin(yf);
            zt = cos(xf) * cos(yf);

            p[0] = xt;  p[1] = yt;  p[2] = zt;
            p = r * p;
            
            if (p[2] <= 0) continue;
			
            xt = p[0] / p[2];
			yt = p[1] / p[2];


			// *** END TODO ***

            // Convert back to regular pixel coordinates and store
            float xn = 0.5f*srcSh.width  + xt*f;
            float yn = 0.5f*srcSh.height + yt*f;
            uvImg(x, y, 0, 0) = xn;
			uvImg(x, y, 0, 1) = yn;
        }
    }
    return uvImg;
}
