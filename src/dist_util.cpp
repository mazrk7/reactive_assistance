#include <reactive_assistance/dist_util.hpp>

namespace reactive_assistance 
{
  // Transform a point 'p' relative to a frame defined by the angle 'th' and origin point 'org'
  void transformPoint(const geometry_msgs::Point& org, double th, geometry_msgs::Point& p)
  {
    double s = std::sin(th);
    double c = std::cos(th);

    // Deviation from origin
    double x = p.x-org.x;
    double y = p.y-org.y;

    // Apply translation and 2d rotation about th (inverse transformation matrix)
    p.x = c*x + s*y;
    p.y = c*y - s*x;
  }

  // Check if a 'target' angle is between two other angles (i.e. the interior of the gap)
  bool isBetweenAngles(double target, double first, double second) 
  {
    // Normalise angles
    target = modpi(target);
    first = modpi(first);
    second = modpi(second);

    // Handles loop around problem
    if (first < second)
      return first <= target && target <= second;
    else
      return first <= target || target <= second;
  }

  // Check if two lines (p1->p2) and (p3->p4) intersect one another
  // Look at Paul Bourke (http://paulbourke.net/geometry/pointlineplane/) for implementation
  bool lineIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& p3, const geometry_msgs::Point& p4, geometry_msgs::Point& out)
  {
    double det = (p4.y - p3.y)*(p2.x - p1.x) - (p4.x - p3.x)*(p2.y - p1.y);

    if (almostEqual(det, 0.0))
    {
      return false;
    }
    else
    {
      double ua = ((p4.x - p3.x)*(p1.y - p3.y) - (p4.y - p3.y)*(p1.x - p3.x))/det;
      double ub = ((p2.x - p1.x)*(p1.y - p3.y) - (p2.y - p1.y)*(p1.x - p3.x))/det;

      out.x = p1.x + ua*(p2.x - p1.x);
      out.y = p1.y + ub*(p2.y - p1.y);
      out.z = 0.0;

      // Test if ua and ub lie between 0 and 1 to check intersection of line segments
      // If either lie in that range then there is an intersection 
      return (0.0 < ua && ua < 1.0) && (0.0 < ub && ub < 1.0);
    }
  }

  // Check if a line defined by p1->p2 intersects with a circle (cx, cy) of radius 'r'
  // Return intersecting point as 'out', if two solutions then take the closest of the intersects
  bool circleIntersect(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const geometry_msgs::Point& c, double r, geometry_msgs::Point& out)
  { 
    geometry_msgs::Point v1, v2, dv;

    // Transform line segments to local coordinates i.e. relative to c
    v1.x = p1.x - c.x;
    v1.y = p1.y - c.y;
    v2.x = p2.x - c.x;
    v2.y = p2.y - c.y;

    // Difference vector
    dv.x = v2.x - v1.x;
    dv.y = v2.y - v1.y;

    // Quadratic equation to solve for intersect, with coefficients A, B, C
    double A = dv.x*dv.x + dv.y*dv.y;
    double B = 2.0*(dv.x*v1.x + dv.y*v1.y);
    double C = v1.x*v1.x + v1.y*v1.y - r*r;

    double discr = B*B - 4.0*A*C;

    // No intersection
    if (discr < 0.0)
    {
      return false;
    }
    else if (almostEqual(discr, 0.0))
    {
      // One real intersection solution
      double u = -B / (2.0 * A);

      out.x = p1.x + u*dv.x;
      out.y = p1.y + u*dv.y;
      out.z = 0.0;

      // Use 'u' value to determine whether intersect is on line segment
      return (0.0 < u && u < 1.0);
    }
    else
    {
      double sqrt_discr = std::sqrt(discr);

      double u1 = (-B + sqrt_discr) / (2.0 * A);
      double u2 = (-B - sqrt_discr) / (2.0 * A);
    
      // Two intersections on line segment? Chord...
      if ((0.0 < u1 && u1 < 1.0) && (0.0 < u2 && u2 < 1.0))
      {
        // Take closest of the two intersects to circle centre
        geometry_msgs::Point inter1;
        inter1.x = p1.x + u1*dv.x;
        inter1.y = p1.y + u1*dv.y;
        inter1.z = 0.0;

        geometry_msgs::Point inter2;
        inter2.x = p1.x + u2*dv.x;
        inter2.y = p1.y + u2*dv.y;
        inter2.z = 0.0;

        out = (dist(inter1, c) < dist(inter2, c)) ? inter1 : inter2;

        return true;
      }
      // Line segment intersection for first solution
      else if (0.0 < u1 && u1 < 1.0)
      {
        out.x = p1.x + u1*dv.x;
        out.y = p1.y + u1*dv.y;
        out.z = 0.0;

        return true;
      }
      // Line segment intersection for second solution
      else if (0.0 < u2 && u2 < 1.0)
      {
        out.x = p1.x + u2*dv.x;
        out.y = p1.y + u2*dv.y;
        out.z = 0.0;

        return true;
      }
      // No intersection on line segment!
      else 
      {
        return false;
      }
    }
  }
} /* namespace reactive_assistance */