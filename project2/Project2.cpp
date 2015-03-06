///////////////////////////////////////////////////////////////////////////
//
// NAME
//  Project2.cpp -- command-line (shell) interface to project 2 code
//
// SYNOPSIS
//  Project2 rdWarp input.tga output.tga f k1 k2
//  Project2 alignAll images.txt orientations.txt minMatches f nRANSAC RANSACthresh [sift]
//  Project2 blendAll orientations.txt outfile.tga f blendRadius
//  Project2 script script.cmd
//
// PARAMTERS
//  input.tga       input image
//  output.tga      output image
//
//  f               focal length in pixels
//  k1, k2          radial distortion parameters
//
//  input*.f        input feature set
//
//  nRANSAC         number of RANSAC iterations
//  RANSACthresh    RANSAC distance threshold for inliers
//  minMatches      required number of matches between two images
//  sift            the word "sift"
//
//  images.txt      file of image filenames and feature filenames
//
//  blendRadius		radius of the blending function
//
//  script.cmd      script file (command line file)
//
// DESCRIPTION
//  This file contains a set of simple command line processes from which
//  you can build a simple automatic cylindrical stitching application.
//
//  Use the programs here to perform a series of operations such as:
//  1. warp all of the images into spherical coordinate (and undo radial distortion)
//  2. align pairs of images using a feature matcher
//  3. read in all of the images and perform pairwise blends
//     to obtain a final (rectified and trimmed) mosaic
//
// TIPS
//  To become familiar with this code, single-step through a couple of
//  examples.  Also, if you are running inside the debugger, place
//  a breakpoint inside the catch statement and at the last statement
//  in main() so you can see the error message before the program exits.
//
// SEE ALSO
//  Image.h             basic image class documentation
//
///////////////////////////////////////////////////////////////////////////

#include "ImageLib/ImageLib.h"
#include "WarpSpherical.h"
#include "BlendImages.h"
#include "GlobalAlign.h"


int main(int argc, const char *argv[]);     // forward declaration

CImg<float> ConvertToCImg(CByteImage& src)
{
	int width = src.Shape().width;
	int height = src.Shape().height;
	int nBands = src.Shape().nBands;
	CImg<float> dst(width, height, 1, nBands);
	for(int h = 0 ; h < height ; ++h)
	{
		for(int w = 0 ; w < width ; ++w)
		{
			for(int b = 0 ; b < nBands ; ++b)
			{
				dst(w, h, 0, b) = src.Pixel(w, h, b);
			}
		}
	}
	return dst;
}

CByteImage ConvertToImglib(CImg<float>& src)
{
	int width = src.width();
	int height = src.height();
	int nBands = src.spectrum();
	CByteImage dst(width, height, nBands);
	for(int h = 0 ; h < height ; ++h)
	{
		for(int w = 0 ; w < width ; ++w)
		{
			for(int b = 0 ; b < nBands ; ++b)
			{
				dst.Pixel(w, h, b) = src(w, h, 0, b);
			}
		}
	}
	return dst;
}

int RDWarp(int argc, const char *argv[])
{
    // Warp the input image to correct for radial distortion
    if (argc < 7)
    {
        printf("usage: %s input.tga output.tga f k1 k2\n", argv[1]);
        return -1;
    }
    const char *infile  = argv[2];
    const char *outfile = argv[3];
	float f             = (float) atof(argv[4]);
    float k1            = (float) atof(argv[5]);
    float k2            = (float) atof(argv[6]);
	
	CByteImage tmp;
    ReadFile(tmp, infile);
	if (tmp.Shape().nBands != 4)
        tmp = ConvertToRGBA(tmp);

	CImg<float> src = ConvertToCImg(tmp);

	CShape sh = tmp.Shape();
    CImg<float> uv = WarpRDField(sh, sh, f, k1, k2);
	CImg<float> tmpDst = src.warp(uv);

	CByteImage dst = ConvertToImglib(tmpDst);
	
    WriteFile(dst, outfile);
    return 0;
}

int AlignAll(int argc, const char *argv[])
{
    // Compute global alignment for all images
    if (argc < 9)
    {
        printf("usage: %s images.txt orientations.txt minMatches f nRANSAC RANSACthresh bundleAdjustIters [sift]\n", argv[1]);
        return -1;
    }
    const char *i_list    = argv[2];
    const char *outfile   = argv[3];
	int minMatches        = atoi(argv[4]);
	float f               = (float) atof(argv[5]);
	int nRANSAC           = atoi(argv[6]);
	double RANSACthresh   = atof(argv[7]);
	int bundleAdjustIters = atoi(argv[8]);

	bool sift = false;
	if ((argc >= 10) && (strcmp(argv[9], "sift") == 0))
		sift = true;
	
	vector<FeatureSet> fs;
	FeatureSet tempFeatures;

    // Open the list of images
    FILE *stream = fopen(i_list, "r");
    if (stream == 0)
        throw CError("%s: could not open the file %s", argv[1], i_list);

    // Construct the list of images and translations
    char line[1024], infile1[1024], infile2[1024];
    int n;
    for (n = 0; fgets(line, 1024, stream); n++)
    {
        // Read the line and push the image onto the list
        if (sscanf(line, "%s %s", infile1, infile2) != 2)
            throw CError("%s: error reading %s", argv[1], i_list);

		if (sift)
			tempFeatures.load_sift(infile2);
		else
			tempFeatures.load(infile2);

		fs.push_back(tempFeatures);
    }

    fclose(stream);

	// read in an image, just to get the dimensions
	CByteImage img;
	ReadFile(img, infile1);
	int width = img.Shape().width;
	int height = img.Shape().height;

	vector<CTransform3x3> ms;
	AlignMatrix am;

	// Perform the alignment.
	if (initGlobalAlign(fs, minMatches, eRotate3D, f, width, height, nRANSAC, RANSACthresh, am, ms) == -1)
		throw CError("%s: image graph not connected", argv[1]);

	// Do bundle adjustment.
	bundleAdjust(fs, eRotate3D, f, width, height, am, ms, bundleAdjustIters);

	// Fix the rotation matrices.
	fixRotations(ms);

	FILE *outstream = fopen(outfile, "w");
	stream = fopen(i_list, "r");

	for (int i=0; i<n; i++) {
		// write the image filename, feature filename, and alignment
		fgets(line, 1024, stream);
		sscanf(line, "%s %s", infile1, infile2);
		if (sift)
			fprintf(outstream, "%s %s %f %f %f %f %f %f %f %f %f\n", infile1, infile2,
					ms[i][0][0], -ms[i][0][1], ms[i][0][2],
					-ms[i][1][0], ms[i][1][1], -ms[i][1][2],
					ms[i][2][0], -ms[i][2][1], ms[i][2][2]);
		else
			fprintf(outstream, "%s %s %f %f %f %f %f %f %f %f %f\n", infile1, infile2,
					ms[i][0][0], ms[i][0][1], ms[i][0][2],
					ms[i][1][0], ms[i][1][1], ms[i][1][2],
					ms[i][2][0], ms[i][2][1], ms[i][2][2]);
	}

	fclose(stream);
	fclose(outstream);

    return 0;
}

int BlendAll(int argc, const char *argv[])
{
    // Blend a sequence of images given the global transformations
    if (argc < 6)
    {
        printf("usage: %s orientations.txt outimg.tga f blendRadius", argv[1]);
        return -1;
    }
    const char *o_list  = argv[2];
    const char *outfile = argv[3];
	float f             = (float) atof(argv[4]);
    float blendRadius   = (float) atof(argv[5]);

    // Open the list of image orientations
    FILE *stream = fopen(o_list, "r");
    if (stream == 0)
        throw CError("%s: could not open the file %s", argv[1], o_list);

    // Construct the list of images and translations
    CImagePositionV ipList;
    char line[1024], infile1[1024], infile2[1024];
    int n;
    for (n = 0; fgets(line, 1024, stream); n++)
    {
        // Compute the global position
        CImagePosition ip;
		float r00,r01,r02,r10,r11,r12,r20,r21,r22;

        // Read the line and push the image onto the list
        if (sscanf(line, "%s %s %f %f %f %f %f %f %f %f %f", infile1, infile2,
				&r00, &r01, &r02,
				&r10, &r11, &r12,
				&r20, &r21, &r22) != 11)
            throw CError("%s: error reading %s", argv[1], o_list);
		ip.position[0][0] = r00;
		ip.position[0][1] = r01;
		ip.position[0][2] = r02;
		ip.position[1][0] = r10;
		ip.position[1][1] = r11;
		ip.position[1][2] = r12;
		ip.position[2][0] = r20;
		ip.position[2][1] = r21;
		ip.position[2][2] = r22;
		CByteImage tmp;
        ReadFile(tmp, infile1);
		ip.img = ConvertToCImg(tmp);
        ipList.push_back(ip);
    }

    fclose(stream);
    CImg<float> tmpPesult = BlendImages(ipList, f, blendRadius);
	CByteImage result = ConvertToImglib(tmpPesult);
    WriteFile(result, outfile);
    return 0;
}

int Script(int argc, const char *argv[])
{
    // Read a series of commands from a script file
    //  (an alternative to a shell-level command file)
    if (argc < 3)
    {
        printf("usage: %s script.cmd\n", argv[1]);
        return -1;
    }
    FILE *stream = fopen(argv[2], "r");
    if (stream == 0)
        throw CError("Could not open %s", argv[2]);

    // Process each command line
    char line[1024];
    while (fgets(line, 1024, stream))
    {
        fprintf(stderr, line);
        if (line[0] == '/' && line[1] == '/')
            continue;   // skip the comment line
        char *ptr = line;
        char *argv2[256];
        int argc2;
        for (argc2 = 0; argc2 < 256 && *ptr; argc2++)
        {
            while (*ptr && isspace(*ptr)) ptr++;
            argv2[argc2] = ptr;
            while (*ptr && !isspace(*ptr)) ptr++;
            if (*ptr)
                *ptr++ = 0;     // null terminate the argument
        }
        if (argc2 < 2)
            continue;

        // Call the dispatch routine
        int code = main(argc2, (const char **) argv2);
        if (code)
            return code;
    }
    fclose(stream);
    return 0;
}

int main(int argc, const char *argv[])
{
    try
    {
        // Branch to processing code based on first argument
        if (argc > 1 && strcmp(argv[1], "rdWarp") == 0)
            return RDWarp(argc, argv);
        else if (argc > 1 && strcmp(argv[1], "alignAll") == 0)
            return AlignAll(argc, argv);
        else if (argc > 1 && strcmp(argv[1], "blendAll") == 0)
            return BlendAll(argc, argv);
        else if (argc > 1 && strcmp(argv[1], "script") == 0)
            return Script(argc, argv);
		else {
			printf("usage: \n");
	        printf("	%s rdWarp input.tga output.tga f k1 k2\n", argv[0]);
			printf("	%s alignAll images.txt orientations.txt minMatches f nRANSAC RANSACthresh bundleAdjustIters [sift]\n", argv[0]);
			printf("	%s blendAll orientations.txt outimg.tga f blendRadius\n", argv[0]);
			printf("	%s script script.cmd\n", argv[0]);
		}
    }
    catch (CError &err) {
        printf(err.message);
        return -1;
    }
    return 0;
}
