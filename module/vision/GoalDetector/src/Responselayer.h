#include <stdio.h>
#include <vector>

class ResponseLayer {
public:
    int width, step, filter;
    std::vector<float> responses;
    std::vector<unsigned char> laplacian;

    ResponseLayer(int width, int step, int filter) {
        if (width > 0) {

            this->width  = width;
            this->step   = step;
            this->filter = filter;

            responses.reserve(width);
            laplacian.reserve(width);
        }
        else {
            printf("ERROR: in ResponseLayer constructor: w was not greater than 0\n");
        }
    }

    inline unsigned char getLaplacian(unsigned int column) {
        return laplacian[column];
    }

    inline unsigned char getLaplacian(unsigned int column, std::vector<ResponseLayer>::iterator src) {
        int scale = this->width / src->width;
        return laplacian[scale * column];
    }

    inline float getResponse(unsigned int column) {
        return responses[column];
    }

    inline float getResponse(unsigned int column, std::vector<ResponseLayer>::iterator src) {
        int scale = this->width / src->width;
        return responses[scale * column];
    }
};
