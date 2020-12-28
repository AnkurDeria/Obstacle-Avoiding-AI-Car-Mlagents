using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CurveHandler : MonoBehaviour
{
    public List<Vector2> m_convexhull = new List<Vector2>();


    /// <summary>
    /// Sorts in clockwise order around m_origin point
    /// </summary>
    public class ClockwiseComparer : IComparer<Vector2>
    {
        private Vector2 m_origin;
       
        public Vector2 Origin { get { return m_origin; } set { m_origin = value; } }

        public ClockwiseComparer(Vector2 origin)
        {
            m_origin = origin;
        }

        /// <summary>
        ///     Compares two objects and returns a value indicating whether one is less than, equal to, or greater than the other.
        /// </summary>
        public int Compare(Vector2 first, Vector2 second)
        {
            return IsClockwise(first, second, m_origin);
        }

        /// <summary>
        ///     Returns 1 if first comes before second in clockwise order.
        ///     Returns -1 if second comes before first.
        ///     Returns 0 if the points are identical.
        /// </summary>
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

    int FindSide(Vector2 _point1, Vector2 _point2, Vector2 _point_maxdist)
    {
        int _val = LineDist(_point1, _point2, _point_maxdist);

        if (_val > 0)
            return 1;
        if (_val < 0)
            return -1;
        return 0;
    }

    // returns points value proportional to the distance 
    // between the point _pointMaxDist and the line joining the 
    // points _point1 and _point2 
    int LineDist(Vector2 _point1, Vector2 _point2, Vector2 _pointMaxDist)
    {
        return (int)((_pointMaxDist.y - _point1.y) * (_point2.x - _point1.x) -
                   (_point2.y - _point1.y) * (_pointMaxDist.x - _point1.x));
    }


    void QuickHull(List<Vector2> _points, int _size, Vector2 _point1, Vector2 _point2, int _side)
    {
        int _index = -1;
        int _max_dist = 0;

        // finding the point with maximum distance 
        // from L and also on the specified side of L. 
        for (int i = 0; i < _size; i++)
        {
            int _temp = Mathf.Abs(LineDist(_point1, _point2, _points[i]));
            if (FindSide(_point1, _point2, _points[i]) == _side && _temp > _max_dist)
            {
                _index = i;
                _max_dist = _temp;
            }
        }

        // If no point is found, add the end points 
        // of L to the convex hull. 
        if (_index == -1)
        {
            if (!m_convexhull.Contains(_point1)) m_convexhull.Add(_point1);
            if (!m_convexhull.Contains(_point2)) m_convexhull.Add(_point2);
            //Debug.Log("Point 1 = " + _point1 + ",  Point2 = " + _point2);
            return;
        }

        // Recur for the two parts divided by _points[_index] 
        QuickHull(_points, _size, _points[_index], _point1, -FindSide(_points[_index], _point1, _point2));
        QuickHull(_points, _size, _points[_index], _point2, -FindSide(_points[_index], _point2, _point1));
    }

    public void CreateConvexhull(List<Vector2> _points, int _size)
    {

        // Finding the point with minimum and 
        // maximum x-coordinate 
        int _minX = 0, _maxX = 0;
        for (int i = 1; i < _size; i++)
        {
            if (_points[i].x < _points[_minX].x)
                _minX = i;
            if (_points[i].x > _points[_maxX].x)
                _maxX = i;
        }

        // Recursively find convex hull points on 
        // one side of line joining _points[_minX] and 
        // _points[_maxX] 
        QuickHull(_points, _size, _points[_minX], _points[_maxX], 1);

        // Recursively find convex hull _points on 
        // other _side of line joining _points[_minX] and 
        // _points[_maxX] 
        QuickHull(_points, _size, _points[_minX], _points[_maxX], -1);


    }
    
}
