All files are contained in ./content/

./content/GJK.h, ./content/GJK.cpp and ./content/Hull.h contain all the functionalities for performing proximity queries with the GJK or EPA algorithm. 

With respect to other similar solvers, the one proposed allow the possibility to:
 -specify a roto-traslation for the shapes, without explicitly computes the vertices coordinates after the roto-traslation
 -the shapes to consider are passed as list of 3D coordinates. The type describing the coordinates is a template, therefore you can use with no 
  further effort the 3D representation that you prefer.

./content/GJK_EPA.pdf explains the theory behind the lines of codes as well as how to use the GJK_EPA class.

Samples are contained in:
./content/Sample_01/Main_01.cpp
./content/Sample_01/Main_02.cpp
./content/Sample_01/Main_03.cpp

Results of the Samples can be displayed using the script python ./Result_visualization/Main.py