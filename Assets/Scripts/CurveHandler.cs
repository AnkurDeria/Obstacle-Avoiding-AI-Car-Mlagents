using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurveHandler : MonoBehaviour
{
    public List<Vector2> m_convexhull = new List<Vector2>();
    public class ClockwiseComparer : IComparer<Vector2>
    {
        private Vector2 m_Origin;
        
        /// <summary>
        ///     Gets or sets the origin.
        /// </summary>
        /// <value>The origin.</value>
        public Vector2 origin { get { return m_Origin; } set { m_Origin = value; } }

        /// <summary>
        ///     Initializes a new instance of the ClockwiseComparer class.
        /// </summary>
        /// <param name="origin">Origin.</param>
        public ClockwiseComparer(Vector2 origin)
        {
            m_Origin = origin;
        }

        /// <summary>
        ///     Compares two objects and returns a value indicating whether one is less than, equal to, or greater than the other.
        /// </summary>
        /// <param name="first">First.</param>
        /// <param name="second">Second.</param>
        public int Compare(Vector2 first, Vector2 second)
        {
            return IsClockwise(first, second, m_Origin);
        }

        /// <summary>
        ///     Returns 1 if first comes before second in clockwise order.
        ///     Returns -1 if second comes before first.
        ///     Returns 0 if the points are identical.
        /// </summary>
        /// <param name="first">First.</param>
        /// <param name="second">Second.</param>
        /// <param name="origin">Origin.</param>
        public static int IsClockwise(Vector2 first, Vector2 second, Vector2 origin)
        {
            if (first == second)
                return 0;

            Vector2 firstOffset = first - origin;
            Vector2 secondOffset = second - origin;

            float angle1 = Mathf.Atan2(firstOffset.x, firstOffset.y);
            float angle2 = Mathf.Atan2(secondOffset.x, secondOffset.y);

            if (angle1 < angle2)
                return -1;

            if (angle1 > angle2)
                return 1;

            // Check to see which point is closest
            return (firstOffset.sqrMagnitude < secondOffset.sqrMagnitude) ? -1 : 1;
        }
    }

    int findSide(Vector2 _point1, Vector2 _point2, Vector2 _point_maxdist)
    {
        int val = lineDist(_point1, _point2, _point_maxdist);

        if (val > 0)
            return 1;
        if (val < 0)
            return -1;
        return 0;
    }

    // returns _points value proportional to the distance 
    // between the point _point_maxdist and the line joining the 
    // _points _point1 and _point2 
    int lineDist(Vector2 _point1, Vector2 _point2, Vector2 _point_maxdist)
    {
        return (int)((_point_maxdist.y - _point1.y) * (_point2.x - _point1.x) -
                   (_point2.y - _point1.y) * (_point_maxdist.x - _point1.x));
    }


    void quickHull(List<Vector2> _points, int _size, Vector2 _point1, Vector2 _point2, int _side)
    {
        int _index = -1;
        int _max_dist = 0;

        // finding the point with maximum distance 
        // from L and also on the specified _side of L. 
        for (int i = 0; i < _size; i++)
        {
            int _temp = Mathf.Abs(lineDist(_point1, _point2, _points[i]));
            if (findSide(_point1, _point2, _points[i]) == _side && _temp > _max_dist)
            {
                _index = i;
                _max_dist = _temp;
            }
        }

        // If no point is found, add the end _points 
        // of L to the convex hull. 
        if (_index == -1)
        {
            if (!m_convexhull.Contains(_point1)) m_convexhull.Add(_point1);
            if (!m_convexhull.Contains(_point2)) m_convexhull.Add(_point2);
            //Debug.Log("Point 1 = " + _point1 + ",  Point2 = " + _point2);
            return;
        }

        // Recur for the two parts divided by _points[_index] 
        quickHull(_points, _size, _points[_index], _point1, -findSide(_points[_index], _point1, _point2));
        quickHull(_points, _size, _points[_index], _point2, -findSide(_points[_index], _point2, _point1));
    }

    public void Createconvexhull(List<Vector2> _points, int _size)
    {

        // Finding the point with minimum and 
        // maximum x-coordinate 
        int _min_x = 0, max_x = 0;
        for (int i = 1; i < _size; i++)
        {
            if (_points[i].x < _points[_min_x].x)
                _min_x = i;
            if (_points[i].x > _points[max_x].x)
                max_x = i;
        }

        // Recursively find convex hull _points on 
        // one _side of line joining _points[_min_x] and 
        // _points[max_x] 
        quickHull(_points, _size, _points[_min_x], _points[max_x], 1);

        // Recursively find convex hull _points on 
        // other _side of line joining _points[_min_x] and 
        // _points[max_x] 
        quickHull(_points, _size, _points[_min_x], _points[max_x], -1);


    }
    
}
