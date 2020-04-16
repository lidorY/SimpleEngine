# SimpleEngine
Simple demonstrative 3D graphics engine (Runs on the CPU).

# Instructions:
Add the relevant files to a MSVC project.
Download Eigen math package from here: http://eigen.tuxfamily.org/index.php?title=Main_Page
!!Use version 3.3.7!!
Add its header files path (...\eigen-3.3.7\Eigen\) to the MSVC project properties as an "Additional Include Directory".
(Project Properties -> C/C++ -> General -> Additional Include Directories <Eigen header file path goes here>)
This requirement will be dropped out after I'll implement a basic Linear Alegbra utility code..

Compile and enjoy learning!
* This code requires lingdi as it's using WinAPI functions to paint pixels on the screen, hence you should use MSVC to
 compile it and it will only run on windows. 
