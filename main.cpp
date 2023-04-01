#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

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

struct Intersection {
    Vector P;
    Vector N;
    Vector color;
    double t;
    double refractive_index;
    bool reflects;
    bool is_transparent;

    bool intersects = false;

    Intersection(double refractive_index = 1.0,
                 bool reflects = false,
                 bool is_transparent = false,
                 Vector color = Vector(0, 0, 0)){
        refractive_index = refractive_index;
        reflects = reflects;
        is_transparent = is_transparent;
        color = color;
    }
};

class Ray {
    public:
        Vector origin;
        Vector dir;
        explicit Ray(Vector origin, Vector direction) {
            origin = origin;
            dir = direction;
        }
};

class Sphere {
    private:
        Vector center;
        Vector color;
        double radius;
        double refractive_index;
        bool reflects;
        bool is_transparent;
        int id;

    public:
        explicit Sphere(Vector center,
                        double radius,
                        Vector color,
                        bool reflects = false,
                        bool is_transparent = false,
                        double refractive_index = 1.0) {
            radius = radius;
            center = center;
            color = color;
            reflects = reflects;
            is_transparent = is_transparent;
            refractive_index = refractive_index;
        }

        Intersection intersect(const Ray& ray) {

            Intersection I(refractive_index, reflects, is_transparent, color);

            double delta = pow(dot(ray.dir, ray.origin - center), 2)
                            - ((ray.origin - center).norm2() - pow(radius, 2));

            if (delta >= 0.) {
                double t1 = dot(ray.dir, ray.origin - center) - sqrt(delta);
                double t2 = dot(ray.dir, ray.origin - center) + sqrt(delta);
                I.t = (t1>0) ? t1 : ((t2>0) ? t2 : 0.0);
                calculate_intersection(&I, ray);
            }

            return I;
        }

        void calculate_intersection(Intersection *I, const Ray& ray) {
            I->P = ray.origin + (I->t * ray.dir);
            I->N = (I->P - center) / (I->P - center).norm();
            I->t = I->t;
            I->intersects = true;
        }
};

class Scene {
    private:
        int id;
        std::vector<Sphere> spheres;
        Vector LightSource;
        Vector localP;
        Vector localN;
        double intensity = 2E10;

    public:
        void addSphere(Sphere sphere) { spheres.push_back(sphere); }
        void setLightSource(Vector light) { LightSource = light; }
        void setIntensity(double intensity) { intensity = intensity; }

        Intersection intersect(const Ray& ray) {
            Intersection I_main, I_temp;
            double t = std::numeric_limits<double>::max();
            for (auto& sphere : spheres) {
                I_temp = sphere.intersect(ray);
                if (I_temp.intersects && I_temp.t < t) {
                    t = I_temp.t;
                    I_main = I_temp;
                }
            }
            return I_main;
        }

        Vector getcolor(const Ray& ray, int depth) {
            if (depth < 0) return Vector(0, 0, 0);

            Intersection I = intersect(ray);

            if (I.intersects) {
                double eps = 1e-10;
                localP = I.P + eps * I.N;
                localN = I.N;

                if (I.reflects) {
                    Ray reflected_ray = Ray(localP, ray.dir - 2 * dot(ray.dir, localN) * localN);
                    return getcolor(reflected_ray, depth - 1);
                }

                if (I.refractive_index != 1.) {
                    double N_u = dot(ray.dir, localN);
                    double n1 = (N_u > 0) ? I.refractive_index : 1.0;
                    double n2 = (N_u > 0) ? 1. : I.refractive_index;

                    localN = (N_u > 0) ? (-1.)*localN : localN;
                    localP = I.P - eps * localN;
                    N_u = dot(ray.dir, localN);

                    Vector T = (n1/n2) * (ray.dir - N_u * localN);
                    Vector N = -1. * sqrt(1. - T.norm2()) * localN;
                    Ray refracted_ray = Ray(localP, T + N);
                    return getcolor(refracted_ray, depth - 1);
                }

                double d = (LightSource - localP).norm();
                Vector w_i = (LightSource - localP) / d;
                Ray light_ray = Ray(LightSource, w_i*(-1.));
                Intersection I_light = intersect(light_ray);
                bool visible = !(I_light.intersects && I_light.t <= d);
                return I.color / M_PI * intensity / (4*M_PI*d*d) * visible * std::max(0., dot(w_i, localN));
            }

            return Vector(0, 0, 0);
        }
};

int main() {
    int W = 512;
    int H = 512;

    std::cout<<"Entered"<<std::endl;

    // define a scene
    Scene scene;
    scene.setLightSource(Vector(-10, 20 ,40));

    // create spheres from the lecture notes
    Sphere sphere(Vector(0, 0, 0), 10, Vector(1, 0, 0));
    Sphere left(Vector(0, 0, 1000), 940, Vector(0, 1, 0));
    Sphere front(Vector(0, 0, -1000), 940, Vector(0, 1, 0));
    Sphere right(Vector(0, 0, -1000), 940, Vector(0, 0, 1));
    Sphere back(Vector(0, 0, 1000), 940, Vector(1, 0, 1));
    Sphere ceiling(Vector(0, 1000, 0), 940, Vector(1, 1, 1));
    Sphere floor(Vector(0, -1000, 0), 940, Vector(1, 1, 1));

    // add spheres to the scene
    scene.addSphere(sphere);
    scene.addSphere(left);
    scene.addSphere(front);
    scene.addSphere(right);
    scene.addSphere(back);
    scene.addSphere(ceiling);
    scene.addSphere(floor);

    std::vector<unsigned char> image(W * H * 3, 0);
    Vector Q = Vector(0, 0, 55);
    int max_depth = 5;
    double alpha = 1.0472; // 60 deg
    double gamma = 2.2;

    std::cout<<"Rendering..."<<std::endl;

    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            Vector dir = Vector(Q[0]+j+0.5-W/2,
                                Q[1]+i+0.5-H/2,
                                Q[2]-W/(2*tan(alpha/2)));
            Ray ray = Ray(Q, (dir-Q)/(dir-Q).norm());
            Vector color = scene.getcolor(ray, max_depth);
            image[(i * W + j) * 3 + 0] = std::min(255., pow(color[0], 1./gamma)*255);
            image[(i * W + j) * 3 + 1] = std::min(255., pow(color[1], 1./gamma)*255);
            image[(i * W + j) * 3 + 2] = std::min(255., pow(color[2], 1./gamma)*255);
        }
    }

    std::cout<<"Done!"<<std::endl;
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    return 0;
}
