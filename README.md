# :earth_americas: Hello, World :earth_africa:

This project takes in a .obj file and renders a 3D object with texture mapping and bump mapping.

## Built With

* C++
* Eigen library for linear algebra
* OpenGL (version 4.1.0)
* GLSL (version 4.10)
* GLFW

## User Input

* 'u' scales the earth up
* 'd' scales the earth down
* 'r' to rotate the earth
* 'z' to spin the scene with the camera
* '-' to zoom out with the camera (perspective mode only)
* 'b' toggles between terrain bump map to bricks bump map

## Texture Mapping

Daytime texture map:

![alt text](https://github.com/EricaHD/HelloWorld/blob/master/READMEpictures/daytexture.jpg)

Nighttime texture map:

PICTURE

Combined texture map, linearly interpolating between day and night depending on Phong shading computed at that point:

PICTURE

## Bump Mapping

Topographical bump map:

PICTURE

Brick bump map:

PICTURE

## Acknowledgements

[Object data made by Glenn Campbell, ideastudio on www.free3d.com](https://free3d.com/3d-model/planet-earth-99065.html)

Maps supplied by NASA.