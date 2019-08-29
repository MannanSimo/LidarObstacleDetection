#ifndef LIDAR_H_
#define LIDAR_H_

#include "../render/render.h"
#include <ctime>
#include <chrono>

const double pi = 3.1415;

struct Ray {
  Vect3 origin;
  Vect3 direction;
  Vect3 castPosition;
  double resolution;
  double castDistance;

  Ray(Vect3 initOrigin, double horizontalAngle, double verticalAngle,
      double initResolution)
      : origin(initOrigin),
        resolution(initResolution),
        direction(resolution * cos(verticalAngle) * cos(horizontalAngle),
                  resolution * cos(verticalAngle) * sin(horizontalAngle),
                  resolution * sin(verticalAngle)),
        castPosition(origin),
        castDistance(0) {}

  void rayCast(const std::vector<Car> &cars, double minDistance,
               double maxDistance, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
               double slopeAngle, double sderr) {
    castPosition = origin;
    castDistance = 0;

    bool collision = false;

    while (!collision && castDistance < maxDistance) {
      castPosition = castPosition + direction;
      castDistance += resolution;

      collision = (castPosition.z <= castPosition.x * tan(slopeAngle));

      if (!collision && castDistance < maxDistance) {
        for (Car car : cars) {
          collision |= car.checkCollision(castPosition);
          if (collision) break;
        }
      }
    }

    if ((castDistance >= minDistance) && (castDistance <= maxDistance)) {
      double rx = ((double)rand() / (RAND_MAX));
      double ry = ((double)rand() / (RAND_MAX));
      double rz = ((double)rand() / (RAND_MAX));
      cloud->points.push_back(pcl::PointXYZ(castPosition.x + rx * sderr,
                                            castPosition.y + ry * sderr,
                                            castPosition.z + rz * sderr));
    }
  }
};

struct Lidar {
  std::vector<Ray> rays;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  std::vector<Car> cars;
  Vect3 position;
  double groundSlope;
  double minDistance;
  double maxDistance;
  double resolution;
  double sderr;

  Lidar(std::vector<Car> setCars, double setGroundSlope)
      : cloud(new pcl::PointCloud<pcl::PointXYZ>()), position(0, 0, 2.6) {
    minDistance = 5.0;
    maxDistance = 50.0;
    resolution = 0.2;
    sderr = 0.2;
    cars = setCars;
    groundSlope = setGroundSlope;
    int numLayers = 8;
    double steepestAngle = 30.0 * (-pi / 180);
    double angleRange = 26.0 * (pi / 180);
    double horizontalAngleInc = pi / 64;
    double angleIncrement = angleRange / numLayers;

    for (double angleVertical = steepestAngle;
         angleVertical < steepestAngle + angleRange;
         angleVertical += angleIncrement) {
      for (double angle = 0; angle <= 2 * pi; angle += horizontalAngleInc) {
        Ray ray(position, angle, angleVertical, resolution);
        rays.push_back(ray);
      }
    }
  }

  ~Lidar() {}

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan() {
    cloud->points.clear();
    auto startTime = std::chrono::steady_clock::now();
    for (Ray ray : rays)
      ray.rayCast(cars, minDistance, maxDistance, cloud, groundSlope, sderr);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime);
    cout << "ray casting took " << elapsedTime.count() << " milliseconds"
         << endl;
    cloud->width = cloud->points.size();
    cloud->height = 1;
    return cloud;
  }
};

#endif
