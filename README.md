This library contains the implementations of the **[GJK](https://en.wikipedia.org/wiki/Gilbert–Johnson–Keerthi_distance_algorithm)** and **[EPA](http://uu.diva-portal.org/smash/get/diva2:343820/FULLTEXT01.pdf)** algorithms.
They are used a lot in the domain of computer graphics, to:

 * detect collisions between pairs of convex shapes
 * get the closest pair of points between pairs of convex shapes
 * get the penetration vector between pairs of convex shapes

With respect to other similar solvers, the one proposed allows to easily:

 * account for roto-traslation for the shapes, without explicitly computes the vertices coordinates after the roto-traslation
 * manage template objects represting 3d coordinates in order to use your favourite algebra library ([Eigen](https://eigen.tuxfamily.org/dox/group__TutorialMatrixArithmetic.html), etc...)
 
Before diving into the code, have a look at ./doc/GJK_EPA.pdf.

This library is stand-alone and completely **cross platform**. Use [CMake](https://cmake.org) to configure the project.
 
The relevant code is contained in ./src, while ./Samples contains samples showing how to use this library.
In particular, after running the samples, some .json files will be produced storing the results.
A specific python script can be run to visualize results as similarly done below:
 
 ![Sample](sample.png)
