#include <vector>

struct KeyFrame {
    int frame;
    float translation[3];
    float scale[3];
    float rotation[4];
};

struct Script {
    int n_frames;
    std::vector<KeyFrame *> *keyframes;
};
