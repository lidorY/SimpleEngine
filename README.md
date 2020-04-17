# SimpleEngine
Simple demonstrative 3D graphics engine (Runs on the CPU).

# Instructions:
Add the relevant files to a MSVC project. <br>
Download Eigen math package from here: http://eigen.tuxfamily.org/index.php?title=Main_Page <br>
!!Use version 3.3.7!! <br>
Add its header files path (...\eigen-3.3.7\) to the MSVC project properties as an "Additional Include Directory". <br>
(Project Properties -> C/C++ -> General -> Additional Include Directories <Eigen header file path goes here>) <br>
This requirement will be dropped out after I'll implement a basic Linear Alegbra utility code.. <br>
<br>
Compile and enjoy learning! <br>
 <br>
* This code requires "wingdi" support as it's using WinAPI functions to paint pixels on the screen.<br>
 Therefore, you should use MSVC to compile it and it will only run on windows. <br>
* The Code is in C++17 standart, you need to make sure that the MSVC project is configured accordingly.<br>
 (Project Properties -> C/C++ -> Language -> C++ Language Standard <ISO C++ 17 standard or above>)
