
using System.Collections.Generic;
using UnityEngine;


[RequireComponent(typeof(MeshFilter))]
[RequireComponent(typeof(MeshCollider))]

//[RequireComponent(typeof(LineRenderer))]

public class RoadGenerator : MonoBehaviour
{
    /// <summary>
    /// public variables
    /// </summary>

    
    
    [Range(3, 10)]
    public float roadOffset;
    [Range(1, 100)]
    public float smoothness;

    public bool isRoadBoundary = false;
    public List<Vector3> vertices = new List<Vector3>();
    public List<Transform> waypoints = new List<Transform>();
    
    public Transform waypointParent;


    /// <summary>
    /// private variables
    /// </summary>

    private CurveHandler m_curves;
    private GameObject m_waypoint;
    private GameObject m_roadBoundary;
    private List<int> m_triangles = new List<int>();
    private List<int> m_boundaryTriangles1 = new List<int>();
    private List<int> m_boundaryTriangles2 = new List<int>();
    
    private List<Vector3> m_normals = new List<Vector3>();
    private List<Vector3> m_boundaryVertices1 = new List<Vector3>();
    private List<Vector3> m_boundaryVertices2 = new List<Vector3>();
    private MeshCollider[] m_meshCollider = new MeshCollider[3];
    private MeshFilter[] m_mesh = new MeshFilter[3];
    private ObstacleGenerator m_obstacle;
    private List<Vector2> m_leftSpline = new List<Vector2>();
    private List<Vector2> m_rightSpline = new List<Vector2>();
    void Start()
    {
        m_curves = transform.GetComponent<CurveHandler>();
        if(isRoadBoundary)
        {
            m_roadBoundary = new GameObject("Road Boundary");
            m_roadBoundary.AddComponent<MeshCollider>();
            m_roadBoundary.AddComponent<MeshFilter>();
            m_roadBoundary.tag = "Obstacle";
            m_roadBoundary.isStatic = true;

            // Create gameobjects for boundary of road. One for left and one for right boundary
            Instantiate(m_roadBoundary, transform);
            Instantiate(m_roadBoundary, transform);

            GameObject.Destroy(m_roadBoundary);
        }
        m_mesh = transform.GetComponentsInChildren<MeshFilter>();
        m_meshCollider = transform.GetComponentsInChildren<MeshCollider>();
        m_obstacle = transform.parent.GetComponentInChildren<ObstacleGenerator>();

        //GenTrack();
    }


    /// <summary>
    /// Generate the road
    /// </summary>
    public void GenTrack()
    {
        //Clear all variables every time a new road is made
        ResetVariables();

        //Generate catmull rom spline from the convexhull points
        m_curves.GenerateSpline();
       
        GenMesh();

        GenWaypoints();
    }

    /// <summary>
    /// Remaps given value between a new range
    /// </summary>
    public float Map(float _value, float _from1, float _to1, float _from2, float _to2)
    {
        return (_value - _from1) / (_to1 - _from1) * (_to2 - _from2) + _from2;
    }

    private void ResetVariables()
    {
        m_mesh[0].mesh.Clear();
        if (isRoadBoundary)
        {
            m_mesh[1].mesh.Clear();
            m_mesh[2].mesh.Clear();
        }
      
        
        vertices.Clear();
        m_boundaryVertices1.Clear();
        m_boundaryVertices2.Clear();
        m_triangles.Clear();
        m_boundaryTriangles1.Clear();
        m_boundaryTriangles2.Clear();
        m_normals.Clear();
        waypoints.Clear();
       
        m_obstacle.waypointOnSpline.Clear();

        foreach (Transform child in waypointParent)
            GameObject.Destroy(child.gameObject);
    }

    /// <summary>
    /// Generates the checkpoints on the road
    /// </summary>
    private void GenWaypoints()
    {
        //Creating the gameobject to instantiate on road
        m_waypoint = new GameObject("Waypoint");
        m_waypoint.layer = 10;
        m_waypoint.tag = "Waypoint";
        m_waypoint.AddComponent<BoxCollider>();
        m_waypoint.isStatic = true;
        BoxCollider _waypointcol = m_waypoint.GetComponent<BoxCollider>();
        _waypointcol.size = new Vector3(roadOffset,2* roadOffset,0.1f);
        _waypointcol.isTrigger = true;
        
        //Instantiating the checkpoints along the road with proper rotation
        Vector3 _vec1, _vec2;
        for(int i=8;i< m_curves.m_splinePoints.Count - 5;i+=7)
        {
            _vec1 = new Vector3(m_curves.m_splinePoints[i + 1].x, 0f, m_curves.m_splinePoints[i + 1].y);
            _vec2 = new Vector3(m_curves.m_splinePoints[i-1].x, 0f, m_curves.m_splinePoints[i-1].y);
            Quaternion _rot = Quaternion.LookRotation((_vec1 - _vec2).normalized, Vector3.up);
            waypoints.Add(Instantiate(m_waypoint, (new Vector3(m_curves.m_splinePoints[i].x, roadOffset, m_curves.m_splinePoints[i].y)), _rot, waypointParent).transform);

            m_obstacle.waypointOnSpline.Add(i);
        }

        GameObject.Destroy(m_waypoint);
    }

    /// <summary>
    /// Generates road mesh from the generated spline points
    /// </summary>
    private void GenMesh()
    {
        //Assigning vertices and normals
        for (int i = 0; i < m_curves.m_splinePoints.Count - 1; i++)
        {
            Vector2 _vec2 = (m_curves.m_splinePoints[i + 1] - m_curves.m_splinePoints[i]).normalized;
            Vector3 _vec2tovec3 = new Vector3(-_vec2.y, 0f, _vec2.x);
           
            Vector3 _newvec3 = transform.InverseTransformPoint(new Vector3(m_curves.m_splinePoints[i].x, 0f, m_curves.m_splinePoints[i].y));
            vertices.Add(_newvec3 + (_vec2tovec3 * roadOffset));
            m_normals.Add(Vector3.up);
            vertices.Add(_newvec3 + (-_vec2tovec3 * roadOffset));
            m_normals.Add(Vector3.up);

            if(isRoadBoundary)
            {
                m_boundaryVertices1.Add(_newvec3 + (_vec2tovec3 * roadOffset));
                m_boundaryVertices1.Add((_newvec3 + (_vec2tovec3 * roadOffset)) + Vector3.up * 10f);
                m_boundaryVertices2.Add(_newvec3 + (_vec2tovec3 * roadOffset));
                m_boundaryVertices2.Add((_newvec3 + (_vec2tovec3 * roadOffset)) + Vector3.up * 10f);
            }
        }
        //Assigning triangle indices
        for (int i = 0; i < vertices.Count - 2; i += 2)
        {
            m_triangles.Add(i);
            m_triangles.Add(i + 2);
            m_triangles.Add(i + 3);
            m_triangles.Add(i);
            m_triangles.Add(i + 3);
            m_triangles.Add(i + 1);
            if (isRoadBoundary)
            {
                m_boundaryTriangles1.Add(i);
                m_boundaryTriangles1.Add(i + 3);
                m_boundaryTriangles1.Add(i + 2);
                m_boundaryTriangles1.Add(i);
                m_boundaryTriangles1.Add(i + 1);
                m_boundaryTriangles1.Add(i + 3);
                m_boundaryTriangles2.Add(i);
                m_boundaryTriangles2.Add(i + 2);
                m_boundaryTriangles2.Add(i + 3);
                m_boundaryTriangles2.Add(i);
                m_boundaryTriangles2.Add(i + 3);
                m_boundaryTriangles2.Add(i + 1);
            }
        }
        
        //Setting the values for the mesh
        m_mesh[0].mesh.SetVertices(vertices);
        m_mesh[0].mesh.SetTriangles(m_triangles, 0);
        m_mesh[0].mesh.SetNormals(m_normals);
        m_mesh[0].mesh.Optimize();

        //Use the generated mesh as mesh collider too
        m_meshCollider[0].sharedMesh = m_mesh[0].mesh;

        if (isRoadBoundary)
        {
            m_mesh[1].mesh.SetVertices(m_boundaryVertices1);
            m_mesh[1].mesh.SetTriangles(m_boundaryTriangles1, 0);
            m_mesh[2].mesh.SetVertices(m_boundaryVertices2);
            m_mesh[2].mesh.SetTriangles(m_boundaryTriangles2, 0);
            m_meshCollider[1].sharedMesh = m_mesh[1].mesh;
            m_meshCollider[2].sharedMesh = m_mesh[2].mesh;
        }
    }

    

    /// <summary>
    /// Extra function for smoothing the points
    /// </summary>
    private List<Vector2> MakeSmoothCurve(List<Vector2> _arrayToCurve)
    {
        List<Vector2> _points;
        List<Vector2> _curvedPoints;
        int _pointsLength = 0;
        int _curvedLength = 0;

        if (smoothness < 1.0f) smoothness = 1.0f;

        _pointsLength = _arrayToCurve.Count;

        _curvedLength = (_pointsLength* Mathf.RoundToInt(smoothness)) - 1;
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
    private void Update()
    {
        foreach (var i in waypoints)
        {
            Debug.DrawLine(i.position, i.position + i.forward * 10f, Color.black);
        }
    }
}

