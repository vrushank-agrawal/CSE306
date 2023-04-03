#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
#include <cmath>
#include <limits>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

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
    double t = 0.;
    double refractive_index;
    bool reflects;
    bool intersects = false;

    Intersection( Vector color = Vector(0., 0., 0.),
                  double refractive_index = 1.,
                  bool reflects = false){
        this->color = color;
        this->refractive_index = refractive_index;
        this->reflects = reflects;
    }
};

class Ray {
    public:
        Vector O;
        Vector dir;
        explicit Ray(Vector O, Vector dir) {
            this->O = O;
            this->dir = dir;
        }
};

class Sphere {
    private:
        Vector C;
        Vector color;
        double radius;
        double refractive_index;
        bool reflects;

    public:
        explicit Sphere(Vector C,
                        double radius,
                        Vector color,
                        bool reflects = false,
                        double refractive_index = 1.) {
            this->C = C;
            this->radius = radius;
            this->color = color;
            this->reflects = reflects;
            this->refractive_index = refractive_index;
        }

        Intersection intersect(const Ray &ray) {
            Intersection I(this->color, this->refractive_index, this->reflects);

            Vector origin_center = ray.O - this->C;
            double delta = pow(dot(ray.dir, origin_center), 2)
                            - (dot(origin_center, origin_center)
                            - pow(radius, 2));

            if (delta >= 0.) {
                double t1 = dot(ray.dir, -1.*origin_center) - sqrt(delta);
                double t2 = dot(ray.dir, -1.*origin_center) + sqrt(delta);
                I.t = (t1>0) ? t1 : ((t2>0) ? t2 : 0.0);
                I.intersects = (t2 < 0.) ? false : true;
            }

            I.P = ray.O + (I.t * ray.dir);
            I.N = I.P - this->C;
            I.N.normalize();
            return I;
        }
};

class Scene {
    private:
        std::vector<Sphere> spheres;
        Vector S;
        double intensity = 1e6;

    public:
        explicit Scene(Vector light) { S = light; }
        void addSphere(Sphere sphere) { spheres.push_back(sphere); }
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

        Vector getColor(const Ray& ray, int depth) {
            if (depth < 0) return Vector(0, 0, 0);

            Intersection I = intersect(ray);

            if (I.intersects) {
                double eps = 1e-10;
                Vector localP = I.P + (eps * I.N);
                Vector localN = I.N;

                if (I.reflects) {
                    Ray reflected_ray = Ray(localP, ray.dir - (2*dot(ray.dir, localN) * localN));
                    return getColor(reflected_ray, depth - 1);
                }

                if (I.refractive_index != 1.) {
                    double N_u = dot(ray.dir, localN);
                    double n1 = (N_u > 0) ? I.refractive_index : 1.0;
                    double n2 = (N_u > 0) ? 1. : I.refractive_index;

                    localN = (N_u > 0) ? -1.*localN : localN;
                    localP = I.P - eps * localN;
                    N_u = dot(ray.dir, localN);

                    Vector T = (n1/n2) * (ray.dir - N_u * localN);
                    Vector N = -1. * sqrt(1. - T.norm2()) * localN;
                    Ray refracted_ray = Ray(localP, T + N);
                    return getColor(refracted_ray, depth - 1);
                }

                double d = (S - localP).norm();
                Vector w_i = (S - localP) / d;
                Ray light_ray = Ray(S, w_i*(-1.));
                Intersection I_light = intersect(light_ray);
                bool visible = !(I_light.intersects && I_light.t <= d);
                return I.color / M_PI * intensity / (4*M_PI*d*d) * visible * std::max(0., dot(w_i, localN));
            }

            return Vector(0, 0, 0);
        }
};

int main() {

    Scene scene = Scene(Vector(-10, 20, 40));

    // let's have fun!
    Sphere sphere1 = Sphere(Vector(20, 0, 0), 10, Vector(1., 1., 1.), true, 1.5);
    Sphere sphere2 = Sphere(Vector(0, 0, 0), 10, Vector(1., 1., 1.), false, 1.5);
    Sphere sphere3 = Sphere(Vector(-20, 0, 0), 10, Vector(1., 1., 1.), false);
    // create spheres from the lecture notes
    Sphere left = Sphere(Vector(0, 1000, 0), 940, Vector(0, 1, 0));
    Sphere front = Sphere(Vector(0, 0, -1000), 940, Vector(1, 0, 0));
    Sphere right = Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 1));
    Sphere back = Sphere(Vector(0, 0, 1000), 940, Vector(1, 0, 1));
    Sphere ceiling = Sphere(Vector(0, 1000, 0), 940, Vector(1, 1, 1));
    Sphere floor = Sphere(Vector(0, -1000, 0), 940, Vector(1, 1, 1));

    // add spheres to the scene
    scene.addSphere(sphere1);
    scene.addSphere(sphere2);
    scene.addSphere(sphere3);
    scene.addSphere(left);
    scene.addSphere(front);
    scene.addSphere(right);
    scene.addSphere(back);
    scene.addSphere(ceiling);
    scene.addSphere(floor);

    int W = 1024;
    int H = 1024;
    std::vector<unsigned char> image(W*H * 3, 0);
    Vector Q = Vector(0, 0, 55);
    double angle = 1.0472; // 60 deg
    double gamma = 2.2;
    int max_depth = 5;

    for (int i = 0; i < H; i++)
        for (int j = 0; j < W; j++) {
            Vector V = Vector(Q[0]+j+0.5-W/2,
                              Q[1]-i-0.5+H/2,
                              Q[2]-W/(2*tan(angle/2)));

            Vector temp = V - Q;
            temp.normalize();
            Ray ray = Ray(Q, temp);
            Vector color = scene.getColor(ray, max_depth);

            image[(i * W + j) * 3 + 0] = std::min(255., pow(color[0], 1./gamma)*255);
            image[(i * W + j) * 3 + 1] = std::min(255., pow(color[1], 1./gamma)*255);
            image[(i * W + j) * 3 + 2] = std::min(255., pow(color[2], 1./gamma)*255);
        }

    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    return 0;
}