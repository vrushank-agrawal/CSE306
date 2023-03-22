#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

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
    void normalize() {
        double n = norm();
        data[0] /= n;
        data[1] /= n;
        data[2] /= n;
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
Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
 
class Sphere {
public:
    explicit Sphere(Vector center, double radius, Vector color, bool is_mirror = false, bool is_transparent = false) {
        radius = radius;
        center = center;
        color = color;
        is_mirror = is_mirror;
        is_transparent = is_transparent;
    }

    bool intersect(const Ray& ray, Vector &P, Vector &N, double &t) {
        bool intersection = false;
        double delta = pow(dot(ray.direction, ray.origin - center), 2) - ((ray.origin - center).norm2() - radius*radius);

        if (delta >= 0) {
            double t1 = dot(ray.direction, ray.origin - center) - sqrt(delta);
            double t2 = dot(ray.direction, ray.origin - center) + sqrt(delta);
            if (t1 > 0) {
                P = ray.origin + t1 * ray.direction;
                N = (P - center) / (P - center).norm();
                intersection = true;
            }
            if (t2 > 0) {
                P = ray.origin + t2 * ray.direction;
                N = (P - center) / (P - center).norm();
                intersection = true;
            }
        }
        return intersection;
    }

    Vector center;
    double radius;
    Vector color;
    bool is_mirror;
    bool is_transparent;
};
 
class Ray {
public:
    explicit Ray(Vector origin, Vector direction) {
        origin = origin;
        direction = direction;
    }
    Vector origin;
    Vector direction;
};

class Scene {
public:

    void addSphere(Sphere sphere) {
        spheres.push_back(sphere);
    }

    bool intersect(const Ray& ray, Vector &P, Vector &N, double &t, int &id) {
        bool intersection = false;
        // t = numeric_limits<double>::max;
        for (int i = 0; i < spheres.size(); i++) {
            if (spheres[i].intersect(ray, localP, localN, localT)) {
                if (localT < t) {
                    t = localT;
                    P = localP;
                    N = localN;
                    intersection = true;
                    id = i;
                }
            }
        }
        return intersection;
    }

    Vector getcolor(const Ray& ray, int depth) {

        if (depth < 0)
            return Vector(0, 0, 0);

        if (intersect(ray, localP, localN, localT, id)) {
            if (spheres[id].is_mirror) {
                Ray reflection = Ray(localP, ray.direction - 2 * dot(ray.direction, localN) * localN);
                return getcolor(reflection, depth - 1);
            }
            else {
                Vector lightDir = Vector(0, 0, 1);
                lightDir.normalize();
                double lightIntensity = std::max(0.0, dot(lightDir, localN));
                return spheres[id].color * lightIntensity;
            }
        }

        Vector P, N;
        double t;
        int id = 0;
        if (intersect(ray, P, N, t, id)) {
            Vector lightDir = Vector(0, 0, 1);
            lightDir.normalize();
            double lightIntensity = std::max(0.0, dot(lightDir, N));
            return spheres[id].color * lightIntensity;
        }
        return Vector(0, 0, 0);
    }

    std::vector<Sphere> spheres;
    Vector localP;
    Vector localN;
    double localT;
    double intensity;
    Vector light;
};

class Camera {
public:
    explicit Camera(Vector center, double angle, double height, double width) {
        center = center;
        angle = angle;
        height = height;
        width = width;
    }

    Vector center;
    double angle;
    double height;
    double width;
};

int main() {
    int W = 512;
    int H = 512;
 
    Camera camera(Vector(0, 0, 55), 60, H, W);

    // define a scene
    Scene scene;
    scene.intensity = 2E10;
    scene.light = Vector(-10, 20 ,40);

    // create spheres from the lecture notes
    Sphere sphere(Vector(0, 0, 0), 10, Vector(1, 0, 0));
    Sphere left(Vector(0, 0, 1000), 940, Vector(0, 1, 0));
    Sphere right(Vector(0, 0, -1000), 940, Vector(0, 0, 1));
    Sphere top(Vector(0, 1000, 0), 940, Vector(1, 1, 1));
    Sphere bottom(Vector(0, -1000, 0), 940, Vector(1, 1, 1));

    // add spheres to the scene
    scene.addSphere(sphere);
    scene.addSphere(left);
    scene.addSphere(right);
    scene.addSphere(top);
    scene.addSphere(bottom);

    std::vector<unsigned char> image(W * H * 3, 0);
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            Vector dir = Vector(j - W / 2, i - H / 2, 0);
            dir.normalize();

            image[(i * W + j) * 3 + 0] = 255;
            image[(i * W + j) * 3 + 1] = 0;
            image[(i * W + j) * 3 + 2] = 0;
        }
    }

    return 0;
}


// convert this binary string to a hex string