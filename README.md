# panorama
panorama image stitching based on SIFT features

Thanks for visiting my Github page!

This project is a solution for Assignment 2 of the course Image Stitching in Image Processing and Computer Vision (ENGG 5104, CUHK, 2015 Spring: www.cse.cuhk.edu.hk/engg5104).

Based on extracted SIFT features from several images, the project align every 2 images using RANSAC algorithm, and stitching all the images to a common plane. For more principle and technical details, refer to project2_pano.htm.pdf.

How to run my code:

1. Download the whole project2 folder.

2. Use Visual Studio 2010 (other platform never tested) to open project file project2.vcxproj.

3. Run the code. If there exist some debugging warnings, ignore them.

4. Go to the /project2/Release, you can find that the executable file project2.exe has been generated. Also, in this folder, the images set for test is also included. Note that their sift features have already been exetracted and recorded in 0.key ~ 6.key. Therefore, when you run my code, you can skip SIFT exetraction step.

5. Open Windows terminal, go to the /Release folder.

6. Run command ‘project2 alignAll image_list.txt orientations.txt 10 595 100 10 8 sift’. This step execute global alignment to all the images listed in image_list.txt, and the output information of transformation between every 2 images is stored in orientation.txt.

7. Run command 'project2 blendAll orientations.txt outimg.tga 572 200'. This step blends the images and produce the final panorama image as outimg.tga. Use software like IrfanView to open it and change to more common picture format like JPEG.
