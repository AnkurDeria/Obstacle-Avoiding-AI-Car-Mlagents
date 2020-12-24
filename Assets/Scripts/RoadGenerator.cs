using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(MeshFilter))]
//[RequireComponent(typeof(LineRenderer))]
[RequireComponent(typeof(MeshCollider))]
public class RoadGenerator : MonoBehaviour
{
    
    [System.Serializable]
    public class RoadBounds
    { 
        public float minX, maxX, minZ, maxZ; 
    }
    [Range(3,100)]
    public uint pointDensity;
    public Transform waypointParent;
    public Transform obstacleParent;
    [Range(1, 100)]
    public float smoothness;
    [Range(3, 10)]
    public float roadOffset;
    public List<Transform> waypoints = new List<Transform>();
    public List<Transform> obstacles = new List<Transform>();
    public List<Vector3> vertices = new List<Vector3>();
    [Range(0.1f, 100f)]
    public float obstacleSpeed;


    private CurveHandler m_curves ;
    private List<Vector2> m_randPoints = new List<Vector2>();
    private List<int> m_triangles = new List<int>();
    private List<Vector3> m_normals = new List<Vector3>();
    private GameObject m_waypoint;
    public RoadBounds m_roadBounds;
    //private LineRenderer lineRenderer;
    private MeshCollider m_meshCollider;
    private Mesh m_mesh;
    private List<Vector3> m_obstacleMoveDir = new List<Vector3>();
    private List<Vector3> m_roadMid = new List<Vector3>();
    private List<float> m_obstacleRandSpeed = new List<float>();
    void Start()
    {
        m_curves = transform.GetComponent<CurveHandler>();
        m_mesh = transform.GetComponent<MeshFilter>().mesh;
        m_meshCollider = transform.GetComponent<MeshCollider>();
        //GenTrack();
    }
    private void FixedUpdate()
    {
        for (int i = 0; i < obstacles.Count; i++)
        {
            obstacles[i].GetComponent<Rigidbody>().MovePosition(transform.TransformPoint(m_roadMid[i])+ (transform.TransformDirection(m_obstacleMoveDir[i]) * Mathf.Sin(Time.time*m_obstacleRandSpeed[i]*obstacleSpeed) * (roadOffset-0.5f)) );
            
        }
    }
    public void GenTrack()
    {
        m_mesh.Clear();
        m_randPoints.Clear();
        m_curves.m_convexhull.Clear();
        vertices.Clear();
        m_triangles.Clear();
        m_normals.Clear();
        waypoints.Clear();
        obstacles.Clear();
        m_obstacleRandSpeed.Clear();
        m_roadMid.Clear();
        foreach (Transform child in waypointParent)
            GameObject.Destroy(child.gameObject);
        foreach (Transform child in obstacleParent)
            GameObject.Destroy(child.gameObject);
        GenPoints();
        //Debug.Log("Size of random points = " + m_randpoints.Count);
        m_curves.Createconvexhull(m_randPoints, m_randPoints.Count);
        //curves.m_convexhull = m_randpoints;
        //Debug.Log("Size of convex hull = " + curves.m_convexhull.Count);
        
        m_curves.m_convexhull.Sort(new CurveHandler.ClockwiseComparer(new Vector2(0, 0)));
        m_curves.m_convexhull = MakeSmoothCurve(m_curves.m_convexhull);
       
        GenMesh();
        GenWaypoints();
        m_meshCollider.sharedMesh = m_mesh;
        GenObstacles();
       
    }

    private void GenObstacles()
    {
        GameObject _obstacle = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _obstacle.tag = "Obstacle";
        _obstacle.AddComponent<Rigidbody>().mass = 500;
       
        for(int i=3;i<m_curves.m_convexhull.Count - 2;i++)
        {
            if(UnityEngine.Random.value > 0.6f)
            {
                //float _t = UnityEngine.Random.value;
                //obstacles.Add(Instantiate(_obstacle,transform.TransformPoint(new Vector3(Mathf.Lerp(vertices[2*i].x,vertices[(2*i)+1].x, _t), _obstacle.GetComponent<BoxCollider>().size.y / 2f,Mathf.Lerp(vertices[2 * i].z, vertices[(2 * i) + 1].z, _t)) ), Quaternion.Euler(0f,UnityEngine.Random.Range(-180f,180f),0f), obstacleParent).transform);
                m_roadMid.Add(new Vector3((vertices[2 * i].x + vertices[(2 * i) + 1].x) / 2f, _obstacle.GetComponent<SphereCollider>().radius, (vertices[2 * i].z + vertices[(2 * i) + 1].z) / 2f));
                obstacles.Add(Instantiate(_obstacle, transform.TransformPoint(m_roadMid[m_roadMid.Count-1]), Quaternion.Euler(0f, UnityEngine.Random.Range(-180f, 180f), 0f), obstacleParent).transform);
                m_obstacleMoveDir.Add((vertices[2 * i] - vertices[(2 * i) + 1]).normalized);
                m_obstacleRandSpeed.Add( UnityEngine.Random.value);
            }
        }
        GameObject.Destroy(_obstacle);
    }

    private void GenWaypoints()
    {
        m_waypoint = new GameObject("Waypoint");
        m_waypoint.layer = 2;
        m_waypoint.tag = "Waypoint";
        m_waypoint.AddComponent<BoxCollider>();
        BoxCollider _waypointcol = m_waypoint.GetComponent<BoxCollider>();
        _waypointcol.size = new Vector3(2*roadOffset,2* roadOffset,0.5f);
        _waypointcol.isTrigger = true;
        Vector3 _vec1, _vec2;
        for(int i=2;i<m_curves.m_convexhull.Count - 2;i+=2)
        {
            _vec1 = new Vector3(m_curves.m_convexhull[i + 2].x, 0f, m_curves.m_convexhull[i + 2].y);
            _vec2 = new Vector3(m_curves.m_convexhull[i-1].x, 0f, m_curves.m_convexhull[i-1].y);
            Quaternion _rot = Quaternion.LookRotation((_vec1 - _vec2).normalized, Vector3.up);
            waypoints.Add(Instantiate(m_waypoint, transform.TransformPoint(new Vector3(m_curves.m_convexhull[i].x, roadOffset, m_curves.m_convexhull[i].y)), _rot, waypointParent).transform);
            
        }
        GameObject.Destroy(m_waypoint);
    }

    private void GenMesh()
    {
        for (int i = 0; i < m_curves.m_convexhull.Count - 1; i++)
        {
            Vector2 vec2 = (m_curves.m_convexhull[i + 1] - m_curves.m_convexhull[i]).normalized;
            Vector3 tempvec = new Vector3(vec2.x, 0f, vec2.y);
            tempvec = Vector3.Cross(tempvec, Vector3.up);
            Vector3 vec3 = (new Vector3(m_curves.m_convexhull[i].x, 0f, m_curves.m_convexhull[i].y));
            vertices.Add(vec3 + (tempvec * roadOffset));
            m_normals.Add(Vector3.up);
            vertices.Add(vec3 + (-tempvec * roadOffset));
            m_normals.Add(Vector3.up);
        }
        for (int i = 0; i < vertices.Count - 2; i += 2)
        {
            m_triangles.Add(i);
            m_triangles.Add(i + 2);
            m_triangles.Add(i + 3);
            m_triangles.Add(i);
            m_triangles.Add(i + 3);
            m_triangles.Add(i + 1);
        }
        m_mesh.SetVertices(vertices);
        m_mesh.SetTriangles(m_triangles, 0);
        m_mesh.SetNormals(m_normals);
    }

    private void GenPoints()
    {
        for (int i = 0; i < pointDensity; i++)
        {
            float rand1 = UnityEngine.Random.value;
            float rand2 = UnityEngine.Random.value;
            m_randPoints.Add(new Vector2(Map(Mathf.PerlinNoise(rand1, rand2), 0, 1, m_roadBounds.minX, m_roadBounds.maxX), Map(Mathf.PerlinNoise(rand2, rand1), 0, 1, m_roadBounds.minZ, m_roadBounds.maxZ)));
        }
    }

    public float Map(float _value, float _from1, float _to1, float _from2, float _to2)
    {
        return (_value - _from1) / (_to1 - _from1) * (_to2 - _from2) + _from2;
    }
    
   
    // Update is called once per frame
    public List<Vector2> MakeSmoothCurve(List<Vector2> _arrayToCurve)
    {
        List<Vector2> _points;
        List<Vector2> _curvedPoints;
        int _pointsLength = 0;
        int _curvedLength = 0;

        if (smoothness < 1.0f) smoothness = 1.0f;

        _pointsLength = _arrayToCurve.Count;

        _curvedLength = (_pointsLength * Mathf.RoundToInt(smoothness)) - 1;
        _curvedPoints = new List<Vector2>(_curvedLength);

        float _temp;
        for (int pointInTimeOnCurve = 0; pointInTimeOnCurve < _curvedLength + 1; pointInTimeOnCurve++)
        {
            _temp = Mathf.InverseLerp(0, _curvedLength, pointInTimeOnCurve);

            _points = new List<Vector2>(_arrayToCurve);

            for (int j = _pointsLength - 1; j > 0; j--)
            {
                for (int i = 0; i < j; i++)
                {
                    _points[i] = (1 - _temp) * _points[i] + _temp * _points[i + 1];
                }
            }

            _curvedPoints.Add(_points[0]);
        }

        return _curvedPoints;
    }
}
