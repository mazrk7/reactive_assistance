#include <reactive_assistance/dist_util.hpp>

namespace reactive_assistance
{
  // Transform a point 'p' relative to a frame defined by the angle 'th' and origin point 'org'
  void transformPoint(const geometry_msgs::Point &org, double th, geometry_msgs::Point &p)
  {
    double s = std::sin(th);
    double c = std::cos(th);

    // Deviation from origin
    double x = p.x - org.x;
    double y = p.y - org.y;

    // Apply translation and 2d rotation about th (inverse transformation matrix)
    p.x = c * x + s * y;
    p.y = c * y - s * x;
  }

  // Check if a 'target' angle is between two other angles (i.e. the interior of the gap)
  // Look at https://www.xarg.org/2010/06/is-an-angle-between-two-other-angles/ for implementation
  bool isBetweenAngles(double target, double first, double second)
  {
    // Normalise angles
    target = mod2pi(target);
    first = mod2pi(first);
    second = mod2pi(second);

    // Handles loop around problem
    if (first < second)
    {
      return ((first <= target) && (target <= second));
    }
    else
    {
      return ((first <= target) || (target <= second));
    }
  }

  // Check if two lines (p1->p2) and (p3->p4) intersect one another
  // Look at http://paulbourke.net/geometry/pointlineplane/ for implementation
  bool lineIntersect(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, const geometry_msgs::Point &p3,
                     const geometry_msgs::Point &p4, geometry_msgs::Point &out)
  {
    double denom = (p4.y - p3.y) * (p2.x - p1.x) - (p4.x - p3.x) * (p2.y - p1.y);
    double numera = (p4.x - p3.x) * (p1.y - p3.y) - (p4.y - p3.y) * (p1.x - p3.x);
    double numerb = (p2.x - p1.x) * (p1.y - p3.y) - (p2.y - p1.y) * (p1.x - p3.x);

    // The two lines are coincident (i.e. overlay exactly one another)
    if (almostEqual(numera, 0.0) && almostEqual(numerb, 0.0) && almostEqual(denom, 0.0))
    {
      // Take halfway point as intersection
      out.x = (p1.x + p2.x) / 2.0;
      out.y = (p1.y + p2.y) / 2.0;
      out.z = 0.0;

      return true;
    }

    // The two lines are parallel
    if (almostEqual(denom, 0.0))
    {
      return false;
    }

    double ua = numera / denom;
    double ub = numerb / denom;

    // Test if 'ua' and 'ub' both lie between 0 and 1
    if ((ua < 0.0) || (ua > 1.0) || (ub < 0.0) || (ub > 1.0))
    {
      // Return false as both unknowns were not within the range
      return false;
    }

    // Intersection is within both line segments
    out.x = p1.x + ua * (p2.x - p1.x);
    out.y = p1.y + ua * (p2.y - p1.y);
    out.z = 0.0;

    return true;
  }

  // Check if a line defined by p1->p2 intersects with a circle (c.x, c.y) of radius 'r'
  // Look at https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm for explanation
  // Return whether an intersection occurred and the closest intersecting point as 'out'
  bool circleIntersect(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2, const geometry_msgs::Point &c,
                       double r, geometry_msgs::Point &out)
  {
    geometry_msgs::Point d, f;

    // Direction vector from start to end
    d.x = p2.x - p1.x;
    d.y = p2.y - p1.y;

    // Vector from circle centre to start
    f.x = p1.x - c.x;
    f.y = p1.y - c.y;

    // Quadratic equation to solve for intersects, with coefficients A, B, C:
    // t^2*d.dot(d) + 2t*f.dot(d) + (f.dot(f) - r*r) = 0
    double A = d.x * d.x + d.y * d.y;         // d.dot(d)
    double B = 2.0 * (f.x * d.x + f.y * d.y); // 2 * f.dot(d)
    double C = f.x * f.x + f.y * f.y - r * r; // f.dot(f) - r*r

    double discr = B * B - 4.0 * A * C;

    // No intersecting point
    if (discr < 0.0)
    {
      return false;
    }
    else
    {
      // There exists a solution to the quadratic equation
      double sqrt_discr = std::sqrt(discr);

      // Either solution may be on/off the line segment, need to test both
      // t1 always smaller as 'discr' & 'A' are always > 0
      double t1 = (-B - sqrt_discr) / (2.0 * A);
      double t2 = (-B + sqrt_discr) / (2.0 * A);

      // Closer intersection is t1
      if (t1 >= 0.0 && t1 <= 1.0)
      {
        out.x = p1.x + t1 * d.x;
        out.y = p1.y + t1 * d.y;
        out.z = 0.0;

        return true;
      }

      // No t1 intersect so either within circle
      // or completely beyond it, check t2
      if (t2 >= 0.0 && t2 <= 1.0)
      {
        out.x = p1.x + t2 * d.x;
        out.y = p1.y + t2 * d.y;
        out.z = 0.0;

        return true;
      }

      // No intersection solution along line segment
      return false;
    }
  }
} /* namespace reactive_assistance */