
using System.Collections.Generic;
using UnityEngine;

public class CurveHandler : MonoBehaviour
{
    /// <summary>
    /// public variables
    /// </summary>


    public Rect roadBounds;
    [Range(0.01f, 10f)]
    public float detailLevel;
    [Range(0.01f, 10f)]
    public float spacing;
    [Range(3, 100)]
    public int pointDensity;
    public List<Vector2> m_splinePoints = new List<Vector2>();
    
    public List<Vector3> splineNormal = new List<Vector3>();
   
    /// <summary>
    /// private variables
    /// </summary>
    [SerializeField]
    private List<Vector2> m_convexhull = new List<Vector2>();
    [SerializeField]
    private List<Vector2> m_randPoints = new List<Vector2>();
  
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

        // Recursively find convex hull _points on other _side of line joining _points[_minX] and _points[_maxX] 
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
    /// Returns points value proportional to the distance between the point _pointMaxDist and the line joining the points _point1 and _point2
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
        //Generating random points within a given area
        for (int i = 0; i < pointDensity; i++)
        {
            float _rand1 = UnityEngine.Random.Range(-250f, 250f);
            float _rand2 = UnityEngine.Random.Range(-250f, 250f);
            m_randPoints.Add(new Vector2(_rand1, _rand2));
        }
    }
    public void GenerateSpline()
    {
        m_convexhull.Clear();
        m_randPoints.Clear();
        //Generate random points
        GenPoints();

        //Get convexhull of the random points
        CreateConvexhull(m_randPoints, m_randPoints.Count);

        //Sort the convexhull in clockwise order
        m_convexhull.Sort((new CurveHandler.ClockwiseComparer(new Vector2(0f, 0f))));

        // Clear spline and normals from previous episode if any
        m_splinePoints.Clear();
        splineNormal.Clear();
        for (int i = 0; i < 4; i++)
        {
            EqualizeDistance();
        }

        DisplacePoints();
        for (int i = 0; i < 10; i++)
        {
            FixAngles();
            EqualizeDistance();
        }
        //Sort the convexhull in clockwise order
        m_convexhull.Sort((new CurveHandler.ClockwiseComparer(new Vector2(0f, 0f))));
        for (int i = 0; i < m_convexhull.Count; i++)
        {
            m_convexhull[i] += new Vector2(transform.position.x, transform.position.z);
        }
        Vector2 previousPoint = m_convexhull[1];
        float dstSinceLastEvenPoint = 0;
        Vector2[] _segment = new Vector2[4];
        for (int pos = 1; pos < m_convexhull.Count - 3; pos++)
        {

            _segment[0] = m_convexhull[pos];
            _segment[1] = m_convexhull[pos + 1];
            _segment[2] = m_convexhull[pos + 2];
            _segment[3] = m_convexhull[pos + 3];


            float controlNetLength = Vector2.Distance(_segment[0], _segment[1]) + Vector2.Distance(_segment[1], _segment[2]) + Vector2.Distance(_segment[2], _segment[3]);
            float estimatedCurveLength = Vector2.Distance(_segment[0], _segment[3]) + controlNetLength / 2f;
            int divisions = Mathf.CeilToInt(estimatedCurveLength * detailLevel * 10);
            float t = 0;
            while (t <= 1)
            {
                t += 1f / divisions;


                Vector2 newPos = GetCatmullRomPosition(t, _segment);

                dstSinceLastEvenPoint += Vector2.Distance(previousPoint, newPos);

                while (dstSinceLastEvenPoint >= spacing)
                {
                    float overshootDst = dstSinceLastEvenPoint - spacing;
                    Vector2 newEvenlySpacedPoint = newPos + (previousPoint - newPos).normalized * overshootDst;
                    m_splinePoints.Add(newEvenlySpacedPoint);
                    dstSinceLastEvenPoint = overshootDst;
                    previousPoint = newEvenlySpacedPoint;
                }

                previousPoint = newPos;
            }

            //int loops = Mathf.FloorToInt(1f / m_detailLevel);

            //for (int i = 1; i <= loops; i++)
            //{

            //    float t = i * m_detailLevel;


            //    Vector3 newPos = GetCatmullRomPosition( t,_segment);
            //    Vector3 newTangent = GetCatmullRomTangent(t, _segment);
            //    Vector3 newNormal = GetCatmullRomNormal(newTangent);
            //    m_splinePoints.Add(new Vector2(newPos.x, newPos.z));
            //    splineNormal.Add(new Vector3(newNormal.x, 0f, newNormal.z));


            //}
        }

    }

    private void FixAngles()
    {
        for (int i = 0; i < m_convexhull.Count; ++i)
        {
            int previous = ((i - 1) < 0) ? m_convexhull.Count - 1 : i - 1;
            int next = (i + 1) % m_convexhull.Count;
            float px = m_convexhull[i].x - m_convexhull[previous].x;
            float py = m_convexhull[i].y - m_convexhull[previous].y;
            float pl = (float)Mathf.Sqrt(px * px + py * py);
            px /= pl;
            py /= pl;

            float nx = m_convexhull[i].x - m_convexhull[next].x;
            float ny = m_convexhull[i].y - m_convexhull[next].y;
            nx = -nx;
            ny = -ny;
            float nl = (float)Mathf.Sqrt(nx * nx + ny * ny);
            nx /= nl;
            ny /= nl;
            //I got a vector going to the next and to the previous points, normalised.  

            float a = (float)Mathf.Atan2(px * ny - py * nx, px * nx + py * ny); // perp dot product between the previous and next point. 

            if (Mathf.Abs(a * Mathf.Rad2Deg) <= 100) continue;

            float nA = 100 * Mathf.Sign(a) * Mathf.Deg2Rad;
            float diff = nA - a;
            float cos = (float)Mathf.Cos(diff);
            float sin = (float)Mathf.Sin(diff);
            float newX = nx * cos - ny * sin;
            float newY = nx * sin + ny * cos;
            newX *= nl;
            newY *= nl;
            m_convexhull[next] = new Vector2(m_convexhull[i].x + newX, m_convexhull[i].y + newY);
        }
    }

    private void DisplacePoints()
    {
        Vector2 disp = new Vector2();
        List<Vector2> rSet = new List<Vector2>();
        float difficulty = 1f; //the closer the value is to 0, the harder the track should be. Grows exponentially.  
        float maxDisp = 5f; // Again, this may change to fit your units.  
        for (int i = 0; i < m_convexhull.Count; i++)
        {
            float dispLen = (float)Mathf.Pow(UnityEngine.Random.value, difficulty) * maxDisp;
            disp = (m_convexhull[(i + 1) % m_convexhull.Count] - m_convexhull[i]).normalized;
            disp = new Vector2(-disp.y, disp.x);
            disp = Rotate(disp, UnityEngine.Random.Range(-30f, 30f));
            
            disp *= dispLen;
            rSet.Add(new Vector2(m_convexhull[i].x, m_convexhull[i].y));
            rSet.Add(new Vector2(m_convexhull[i].x, m_convexhull[i].y));
            rSet[rSet.Count - 1] += (new Vector2(m_convexhull[(i + 1) % m_convexhull.Count].x, m_convexhull[(i + 1) % m_convexhull.Count].y) / 2f) + new Vector2(UnityEngine.Random.Range(-disp.x,disp.x), UnityEngine.Random.Range(-disp.y, disp.y));
            //Explaining: a mid point can be found with (m_convexhull[i]+m_convexhull[i+1])/2.  
            //Then we just add the displacement.  
        }
        m_convexhull = new List<Vector2>(rSet);
    }

    private Vector2 Rotate(Vector2 v, float degrees)
    {
        float sin = Mathf.Sin(degrees * Mathf.Deg2Rad);
        float cos = Mathf.Cos(degrees * Mathf.Deg2Rad);

        float tx = v.x;
        float ty = v.y;
        v.x = (cos * tx) - (sin * ty);
        v.y = (sin * tx) + (cos * ty);
        return v;
    }
    Vector2 GetCatmullRomPosition(float t, Vector2[] points)
    {
        
        return (0.5f * (2f * points[1]))
                + ((0.5f * (points[2] - points[0])) * t)
                + ((0.5f * (2f * points[0] - 5f * points[1] + 4f * points[2] - points[3])) * t * t)
                + ((0.5f * (-points[0] + 3f * points[1] - 3f * points[2] + points[3])) * t * t * t);
    }
    
   

   
    
    private void EqualizeDistance()
    {
        float _mindist = 70; 
       
        for (int i = 0; i < m_convexhull.Count; i++)  
        {
            for (int j = i + 1; j < m_convexhull.Count; j++)  
             {
                if (Vector2.Distance(m_convexhull[i],m_convexhull[j]) < _mindist)  
                 {
                    float _x = m_convexhull[j].x - m_convexhull[i].x;
                    float _y = m_convexhull[j].y - m_convexhull[i].y;
                    float _dist = (float)Mathf.Sqrt(_x * _x + _y * _y);
                    _x /= _dist;
                    _y /= _dist;
                    float _difference = _mindist - _dist;
                    _x *= _difference;
                    _y *= _difference;
                    m_convexhull[j] += new Vector2(_x, _y);
                    m_convexhull[i] -= new Vector2(_x, _y);
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

