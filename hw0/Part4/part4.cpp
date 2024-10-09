/*
Output a PPM image of a colored circle centered on a black background.

Input:
    xres, yres: resolution of the image (positive, even integers)

The diameter of the circle should be equal to half of min(xres, yres).
*/
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " xres yres" << std::endl;
        return 1;
    }

    int xres = atoi(argv[1]);
    int yres = atoi(argv[2]);

    int centerx = xres / 2;
    int centery = yres / 2;
    int radius = std::min(xres, yres) / 4;

    std::cout << "P3" << std::endl;
    std::cout << xres << ' ' << yres << std::endl;
    std::cout << "255" << std::endl;

    for (int y = 0; y < yres; y++) {
        for (int x = 0; x < xres; x++) {
            int dist = (x - centerx) * (x - centerx) + (y - centery) * (y - centery);
            if (dist <= radius * radius) {
                std::cout << "0 0 0" << std::endl;
            } else {
                std::cout << "255 255 255" << std::endl;
            }
        }
    }
}