# Final Project for Computer Graphics (CSCI 371)

I created physics-based portals in a digital environment by using recursive-rasterization techniques that users can pass through in order to traverse the environment. 
I achieved the recursive portal rendering by using an incremental counter within OpenGl's stencil buffer and repeated matrix transforms.
The inspiration for this project was the game <a href="https://store.steampowered.com/app/400/Portal/">Portal</a> developed by Valve.

I developped the code within `main.cpp`. In order to run, the program requires `cow.cpp` and `snail.cpp`, which are two files developped by Jim Bern, the professor of the course,
to be used on each of our assignments. `cow.cpp` is a wrapper around GLFW (a windowing library for OpenGL) that provides some utility functions to speed up development. `snail.cpp`
is a simple linear algebra library to perform matrix operations.

Click below for a demo video of the portals:
[![Portal Project Demo](https://img.youtube.com/vi/zv5bBVZ7tww/maxresdefault.jpg)](https://www.youtube.com/watch?v=zv5bBVZ7tww)
