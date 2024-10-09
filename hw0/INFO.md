# The .obj file
- contains a list of vertices & list of triangular faces to specify the geometry
  of a 3D model.

- **World Space** - coordinate system in which the vertices are specified

### Structure of an .obj file
```
v x1 y1 z1  
v x2 y2 z2  
v x3 y3 z3  
...  
v xm ym zm  
f face0v1 face0v2 face0v3  
f face1v1 face1v2 face1v3  
f face2v1 face2v2 face2v3  
...  
f facenv1 facenv2 facenv3
```

The lines that begin with a v contain vertex data, whereas the lines that begin with a f contain face data.

Each “vertex line” starts with a v and follows the v with three floating point numbers: the x, y, and z coordinates (in that order) of a vertex in the described 3D model. Each “face line” starts with a f and follows the f with three integers specifying the three vertices that make up a triangular face in the 3D model. For example, the following line:

`f 1 8 37`

specifies a face that is made up of the first, eighth, and thirty-seventh specified vertices in the file. Note that this means the vertices are 1-indexed

### Definitions
- Image: grid of y rows by x columns of pixels, where y and x are positive integers. We often refer to images as pixel grids.
- Pixel: a square in the pixel grid. For colored images, each pixel has three values associated with it.
- Maximum Pixel Intensity: the highest value a pixel can have.
- PPM Image Format: a primitive image format that represents an image as an ASCII text file with a .ppm extension.
  - Contains information about the image size, maximum pixel intensity, and RGB
    data for each individual pixel. This format is very inefficient.

Consider the following example:

```P3  
3 4  
255  
255 255 255  
128 128 128  
0 0 0  
255 0 0  
0 255 0  
0 0 255  
255 255 0  
255 0 255  
0 255 255  
128 0 0  
0 128 0  
0 0 128```

- The first line is a required header for PPM files and is always P3. This line simply indicates that the file is a PPM image and serves no other purpose.

- The second line specifies the x and y resolutions of the image in that order. In the example above, the line “3 4” specifies that the image is of size 4x3 pixels. In other words, this image has 4 rows of pixels vertically along the y-direction and 3 columns of pixels horizontally along the x-direction.

- The third line indicates the maximum pixel intensity. In the above example, we set the maximum pixel intensity to 255.

- Each line after that specifies the RGB values for an individual pixel, specified left to right, top to bottom.