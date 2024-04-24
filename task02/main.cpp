#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2)
{
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}

/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe)
{
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, ps, pe);
  auto d = area(dir + ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f)
  {
    return 1;
  }
  return 0;
}

/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe)
{
  // comment out below to do the assignment
  // return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic bezier
  // p(t) = ((1 - t)^2) * ps + 2 * (1 - t) * t * pc + (t^2) * pe = org + s * dir ( s >= 0, 0 <= t <= 1)
  // ((1 - t)^2) * ps + 2 * (1 - t) * t * pc + (t^2) * pe - org) * normal(dir) = s * dir * normal(dir) = 0
  // (t^2) * (pe - 2 * pc + ps) * normal(dir) + t * 2 * (pc - ps) * normal(dir) + (ps - org) * normal(dir) = 0

  // solve the quadratic equation for t: a*t^2 + b*t + c = 0
  const auto a = (pe - 2 * pc + ps).dot(Eigen::Vector2f(-dir[1], dir[0]));
  const auto b = 2 * (pc - ps).dot(Eigen::Vector2f(-dir[1], dir[0]));
  const auto c = (ps - org).dot(Eigen::Vector2f(-dir[1], dir[0]));
  const auto D = b * b - 4 * a * c;
  const auto t1 = (-b + std::sqrt(D)) / (2 * a);
  const auto t2 = (-b - std::sqrt(D)) / (2 * a);

  // compute the intersection points and check if they are on the ray
  const Eigen::Vector2f p1 = (1 - t1) * (1 - t1) * ps + 2 * (1 - t1) * t1 * pc + t1 * t1 * pe;
  const Eigen::Vector2f p2 = (1 - t2) * (1 - t2) * ps + 2 * (1 - t2) * t2 * pc + t2 * t2 * pe;
  const auto s1 = (p1 - org).dot(dir);
  const auto s2 = (p2 - org).dot(dir);

  if (D == 0)
  {
    if (t1 >= 0 && t1 <= 1 && s1 >= 0)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else if (D > 0)
  {
    if (t1 >= 0 && t1 <= 1 && s1 >= 0)
    {
      if (t2 >= 0 && t2 <= 1 && s2 >= 0)
      {
        return 2;
      }
      else
      {
        return 1;
      }
    }
    else
    {
      if (t2 >= 0 && t2 <= 1 && s2 >= 0)
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
  }
  else
  {
    return 0;
  }
}

int main()
{
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0)
  { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih)
  {
    for (unsigned int iw = 0; iw < width; ++iw)
    {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(60., 20.);           // search direction
      int count_cross = 0;
      for (const auto &loop : loops)
      { // loop over loop (letter R have internal/external loops)
        for (const auto &edge : loop)
        { // loop over edge in the loop
          if (edge.is_bezier)
          { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          }
          else
          { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1)
      {                                // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
