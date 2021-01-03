
using System.Collections.Generic;
using UnityEngine;

public class CurveHandler : MonoBehaviour
{
    /// <summary>
    /// public variables
    /// </summary>

    [Header("The lower the value the more dense the mesh")]
    [Range(0.01f, 10f)] public float detailLevel;

    [Header("The higher the value the more difficult the track")]
    [Range(0.05f, 50f)] public float difficulty;

    [Header("Number of random points to be generated", order = 0)]
    [Space(-10, order = 1)]
    [Header("for the procedural track", order = 2)]
    [Range(3, 100)] public int pointDensity;

    [Header("X = min x local coordinate, Y = min y local coordinate", order = 0)]
    [Space(-10, order = 1)]
    [Header("Width = max x local coordinate, Height = max y local coordinate", order = 2)]
    public Rect roadBounds;

    public List<Vector2> m_splinePoints = new List<Vector2>();
   

    /// <summary>
    /// private variables
    /// </summary>

    [SerializeField] private List<Vector2> m_convexhull = new List<Vector2>();
    [SerializeField] private List<Vector2> m_randPoints = new List<Vector2>();
  
    /// <summary>
    /// Sorts in clockwise order around m_origin point
    /// </summary>
    public class ClockwiseComparer : IComparer<Vector2>
    {
        private Vector2 m_origin;
       
        public Vector2 Origin { get { return m_origin; } set { m_origin = value; } }

        public ClockwiseComparer(Vector2 _origin)
        {
            m_origin = _origin;
        }

        /// <summary>
        /// Compares two objects and returns a value indicating whether one is less than, equal to, or greater than the other.
        /// </summary>
        public int Compare(Vector2 _first, Vector2 _second)
        {
            return IsClockwise(_first, _second, m_origin);
        }

        /// <summary>
        /// Returns 1 if first comes before second in clockwise order.
        /// Returns -1 if second comes before first.
        /// Returns 0 if the points are identical.
        /// </summary>
        public int IsClockwise(Vector2 _first, Vector2 _second, Vector2 _origin)
        {
            if (_first == _second)
                return 0;

            Vector2 _firstOffset = _first - _origin;
            Vector2 _secondOffset = _second - _origin;

            float _angle1 = Mathf.Atan2(_firstOffset.x, _firstOffset.y);
            float _angle2 = Mathf.Atan2(_secondOffset.x, _secondOffset.y);

            if (_angle1 < _angle2)
                return -1;

            if (_angle1 > _angle2)
                return 1;

            // Check to see which point is closest
            return (_firstOffset.sqrMagnitude < _secondOffset.sqrMagnitude) ? -1 : 1;
        }
    }

    public void CreateConvexhull(List<Vector2> _points, int _size)
    {
        // Finding the point with minimum and maximum x-coordinate 
        int _minX = 0, _maxX = 0;
        for (int i = 1; i < _size; i++)
        {
            if (_points[i].x < _points[_minX].x)
                _minX = i;
            if (_points[i].x > _points[_maxX].x)
                _maxX = i;
        }

        // Recursively find convex hull points on one side of line joining _points[_minX] and _points[_maxX] 
        QuickHull(_points, _size, _points[_minX], _points[_maxX], 1);

        // Recursively find convex hull points on other side of line joining _points[_minX] and _points[_maxX] 
        QuickHull(_points, _size, _points[_minX], _points[_maxX], -1);
    }

    private int FindSide(Vector2 _point1, Vector2 _point2, Vector2 _point_maxdist)
    {
        int _val = LineDistance(_point1, _point2, _point_maxdist);

        if (_val > 0)
            return 1;
        if (_val < 0)
            return -1;
        return 0;
    }

    /// <summary>
    /// Returns points value proportional to the distance between the point _pointMaxDist
    /// and the line joining the points _point1 and _point2
    /// </summary>
    private int LineDistance(Vector2 _point1, Vector2 _point2, Vector2 _pointMaxDist)
    {
        return (int)((_pointMaxDist.y - _point1.y) * (_point2.x - _point1.x) -
                   (_point2.y - _point1.y) * (_pointMaxDist.x - _point1.x));
    }

    private void QuickHull(List<Vector2> _points, int _size, Vector2 _point1, Vector2 _point2, int _side)
    {
        int _index = -1;
        int _max_dist = 0;

        for (int i = 0; i < _size; i++)
        {
            int _temp = Mathf.Abs(LineDistance(_point1, _point2, _points[i]));
            if (FindSide(_point1, _point2, _points[i]) == _side && _temp > _max_dist)
            {
                _index = i;
                _max_dist = _temp;
            }
        }

        // If no point is found, add the end points to the convex hull. 
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

    private void GenPoints()
    {
        // Generates random points within a given area
        for (int i = 0; i < pointDensity; i++)
        {
            float _rand1 = UnityEngine.Random.Range(roadBounds.x, roadBounds.width);
            float _rand2 = UnityEngine.Random.Range(roadBounds.y, roadBounds.height);
            m_randPoints.Add(new Vector2(_rand1, _rand2));
        }
    }

    public void GenerateSpline()
    {
        m_convexhull.Clear();
        m_randPoints.Clear();

        // Generate random points
        GenPoints();

        // Get convexhull of the random points
        CreateConvexhull(m_randPoints, m_randPoints.Count);

        // Sort the convexhull in clockwise order
        m_convexhull.Sort((new CurveHandler.ClockwiseComparer(new Vector2(0f, 0f))));

        // Clear spline from previous episode if any
        m_splinePoints.Clear();

        GenerateTrackVariation();
        EvenlySpacedSpline();
    }

    private void EvenlySpacedSpline()
    {
        Vector2 _previousPoint = m_convexhull[1];
        float _dstSinceLastEvenPoint = 0;
        Vector2[] _segment = new Vector2[4];

        for (int pos = 1; pos < m_convexhull.Count - 3; pos++)
        {
            _segment[0] = m_convexhull[pos];
            _segment[1] = m_convexhull[pos + 1];
            _segment[2] = m_convexhull[pos + 2];
            _segment[3] = m_convexhull[pos + 3];

            float _netLenControl = Vector2.Distance(_segment[0], _segment[1]) + Vector2.Distance(_segment[1], _segment[2]) + Vector2.Distance(_segment[2], _segment[3]);
            float _estimatedCurveLength = Vector2.Distance(_segment[0], _segment[3]) + _netLenControl / 2f;
            int _divisions = Mathf.CeilToInt(_estimatedCurveLength * 10);
            float _t = 0;

            while (_t <= 1)
            {
                // Calculate spline position
                _t += 1f / _divisions;
                Vector2 _pointOnSpline = GetCatmullRomPosition(_t, _segment);
                _dstSinceLastEvenPoint += Vector2.Distance(_previousPoint, _pointOnSpline);

                // Check if the point on the spline is evenly spaces
                // If not then move the point towards the previous point and add it to m_splinePoints
                // Repeat the above steps till required detail level is achieved
                while (_dstSinceLastEvenPoint >= detailLevel)
                {
                    float _extraDistance = _dstSinceLastEvenPoint - detailLevel;
                    Vector2 _newPoint = _pointOnSpline + (_previousPoint - _pointOnSpline).normalized * _extraDistance;
                    m_splinePoints.Add(_newPoint);
                    _dstSinceLastEvenPoint = _extraDistance;
                    _previousPoint = _newPoint;
                }

                _previousPoint = _pointOnSpline;
            }
        }
    }

    private void GenerateTrackVariation()
    {
        // Pushes apart points that are too close
        for (int i = 0; i < 4; i++)
        {
            PushApartPoints();
        }

        DisplacePoints();

        for (int i = 0; i < 10; i++)
        {
            FixAngles();

        }

        // Sort the convexhull in clockwise order
        m_convexhull.Sort((new CurveHandler.ClockwiseComparer(new Vector2(0f, 0f))));

        // Storing the world positions in m_convexhull
        for (int i = 0; i < m_convexhull.Count; i++)
        {
            m_convexhull[i] += new Vector2(transform.position.x, transform.position.z);
        }
    }

    /// <summary>
    /// If angle formed by 3 points at the middle point is less than 80 then this function increases that angle
    /// </summary>
    private void FixAngles()
    {
        for (int i = 0; i < m_convexhull.Count; i++)
        {
            int _previousPoint = ((i - 1) < 0) ? m_convexhull.Count - 1 : i - 1;
            int _nextPoint = (i + 1) % m_convexhull.Count;

            // Normalize vectors going to the next and coming from the previous point. 
            Vector2 _prevVec = m_convexhull[i] - m_convexhull[_previousPoint];
            float _prevVecLen = _prevVec.magnitude;
            _prevVec /= _prevVecLen;

            Vector2 _nextVec = m_convexhull[_nextPoint] - m_convexhull[i];
            float _nextVecLen = _nextVec.magnitude;
            _nextVec /= _nextVecLen;

            // Only smooth those angles that are less than 60
            float _angle = Vector2.Angle(_prevVec, _nextVec);
            if (_angle >= 80f)
            {
                continue;
            }

            // Move the middle point towards the center of the 3 points
            m_convexhull[i] = Vector2.Lerp(m_convexhull[i], (m_convexhull[_previousPoint] + m_convexhull[i] + m_convexhull[_nextPoint]) / 3f, 0.5f);
        }
    }

    /// <summary>
    /// Displaces the points in a random direction with a random amount
    /// </summary>
    private void DisplacePoints()
    {
        Vector2 _displacement;
        List<Vector2> rSet = new List<Vector2>();

        // MaxDisp i.e max displacement can be changed according to one's needs
        float maxDisp = 5f; 
        
        for (int i = 0; i < m_convexhull.Count; i++)
        {
            // Max displacement value based on difficulty
            float dispLen = Mathf.Pow(difficulty, UnityEngine.Random.value) * maxDisp;

            // Selects a random displacement direction close to the perpendicular of the direction of path
            _displacement = (m_convexhull[(i + 1) % m_convexhull.Count] - m_convexhull[i]).normalized;
            _displacement = new Vector2(-_displacement.y, _displacement.x);
            _displacement = Rotate(_displacement, UnityEngine.Random.Range(-20f, 20f));
            _displacement *= dispLen;

            // Displace to the left or to the right of the track
            if (UnityEngine.Random.value > 0.5f)
            {
                _displacement = -_displacement;
            }

            rSet.Add(new Vector2(m_convexhull[i].x, m_convexhull[i].y));

            // Add the new displaced point between the 2 existing points
            rSet.Add((m_convexhull[i]+m_convexhull[(i + 1) % m_convexhull.Count] / 2f) + _displacement);
        }

        m_convexhull = new List<Vector2>(rSet);
    }

    /// <summary>
    /// Function to rotate a vector by given degrees
    /// </summary>
    private Vector2 Rotate(Vector2 _vec, float _degrees)
    {
        float _sin = Mathf.Sin(_degrees * Mathf.Deg2Rad);
        float _cos = Mathf.Cos(_degrees * Mathf.Deg2Rad);

        float _vecX = _vec.x;
        float _vecY = _vec.y;
        _vec.x = (_cos * _vecX) - (_sin * _vecY);
        _vec.y = (_sin * _vecX) + (_cos * _vecY);
        return _vec;
    }

    Vector2 GetCatmullRomPosition(float _t, Vector2[] _points)
    {
        return (0.5f * (2f * _points[1]))
                + ((0.5f * (_points[2] - _points[0])) * _t)
                + ((0.5f * (2f * _points[0] - 5f * _points[1] + 4f * _points[2] - _points[3])) * _t * _t)
                + ((0.5f * (-_points[0] + 3f * _points[1] - 3f * _points[2] + _points[3])) * _t * _t * _t);
    }
    
    private void PushApartPoints()
    {
        // Minimum distance below which the points will be pushed apart
        float _mindist = 100f; 
       
        for (int i = 0; i < m_convexhull.Count - 1; i++)  
        {
            Vector2 _vec;
            for (int j = i + 1; j < m_convexhull.Count; j++)  
             {
                if (Vector2.Distance(m_convexhull[i],m_convexhull[j]) < _mindist)  
                 {
                    _vec = m_convexhull[j] - m_convexhull[i];
                    float _dist = _vec.magnitude;
                    _vec /= _dist;
                    float _difference = _mindist - _dist;
                    _vec *= (_difference/2f);
                  
                    m_convexhull[j] += _vec;
                    m_convexhull[i] -= _vec;
                 }
             }
        }
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        int j = 0;
        foreach(var i in m_convexhull)
        {
            Gizmos.DrawWireSphere(new Vector3(i.x, 0f, i.y), 5f);
            
            j++;
        }
        Gizmos.color = Color.yellow;
        foreach(var i in m_splinePoints)
        {
            Gizmos.DrawWireSphere(new Vector3(i.x, 0f, i.y), 2f);
        }
    }
}

