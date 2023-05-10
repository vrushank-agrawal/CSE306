#define _CRT_SECURE_NO_WARNINGS 1
#include <algorithm>
#include <random>
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb/stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "../stb/stb_image.h"

#define ITERATIONS 100

/*  ------------------------ VECTOR ---------------------------------   */

class Vector {
public:
    explicit Vector(double x = 0, double y = 0, double z = 0) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    double norm2() const {
        return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
    }
    double norm() const {
        return sqrt(norm2());
    }
    Vector normalize() {
        double n = norm();
        Vector temp;
        temp[0] = data[0]/n;
        temp[1] = data[1]/n;
        temp[2] = data[2]/n;
        return temp;
    }
    double operator[](int i) const { return data[i]; };
    double& operator[](int i) { return data[i]; };
    double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
    return Vector(a*b[0], a*b[1], a*b[2]);
}
Vector operator*(const Vector& a, const double b) {
    return Vector(a[0]*b, a[1]*b, a[2]*b);
}
Vector operator*(const Vector& a, const Vector& b) {
    return Vector(a[0]*b[0], a[1]*b[1], a[2]*b[2]);
}
Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}

/*  ------------------------ COLOR MATCHING ---------------------------------   */

Vector random_direction() {
    double r1 = ((double) rand() / (RAND_MAX));
    double r2 = ((double) rand() / (RAND_MAX));
    double x = cos(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double y = sin(2 * M_PI * r1) * sqrt(r2 * (1 - r2));
    double z = 1 - 2 * r2;
    return Vector(x, y, z);
}

void color_matching(int W, int H, int C, int M_C, unsigned char* imageSource, unsigned char* colorSource) {
    size_t total_pixels = W * H;
    std::vector<std::pair<int, int>> projI(total_pixels);
    std::vector<std::pair<int, int>> projM(total_pixels);
    Vector image_pixel, color_pixel, v;

	for (size_t i = 0; i < ITERATIONS; i++) {
		v = random_direction();

		// Projections
		for (size_t i = 0; i < total_pixels; i++) {
			unsigned char *Out_I = imageSource + i * C;
			unsigned char *Out_M = colorSource + i * M_C;
			image_pixel = Vector(Out_I[0], Out_I[1], Out_I[2]);
			color_pixel = Vector(Out_M[0], Out_M[1], Out_M[2]);
			projI[i] = std::make_pair(dot(image_pixel, v), i);
			projM[i] = std::make_pair(dot(color_pixel, v), i);
		}

		// Sorting
        std::sort(projI.begin(), projI.end());
        std::sort(projM.begin(), projM.end());

		// Advect initial point cloud
		for (size_t i = 0; i < total_pixels; i++) {
			unsigned char *Out = imageSource + projI[i].second * C;
			image_pixel = Vector(Out[0], Out[1], Out[2]) + (projM[i].first - projI[i].first) * v;
			for (size_t j = 0; j < 3; j++)
				Out[j] = image_pixel[j];
		}
	}
}

// /*  ------------------------ MAIN ---------------------------------   */

int main(int argc, char **argv) {

	int I_W, I_H, I_C;
	int M_W, M_H, M_C;

	unsigned char *imageSource = stbi_load("input.png", &I_W, &I_H, &I_C, 0);
	unsigned char *colorSource = stbi_load("model.png", &M_W, &M_H, &M_C, 0);

	// Run color matching implicitly on the imageSource
	color_matching(I_W, I_H, I_C, M_C, imageSource, colorSource);

	stbi_write_png("output.png", I_W, I_H, I_C, &imageSource[0], 0);

	return 0;
}