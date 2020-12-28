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
    [Range(1, 100)]
    public float smoothness;
    [Range(3, 10)]
    public float roadOffset;
    public List<Transform> waypoints = new List<Transform>();
    public List<Vector3> vertices = new List<Vector3>();
    public RoadBounds m_roadBounds;


    private CurveHandler m_curves ;
    private List<Vector2> m_randPoints = new List<Vector2>();
    private List<int> m_triangles = new List<int>();
    private List<Vector3> m_normals = new List<Vector3>();
    private GameObject m_waypoint;
    private MeshCollider m_meshCollider;
    private Mesh m_mesh;
    
    void Start()
    {
        m_curves = transform.GetComponent<CurveHandler>();
        m_mesh = transform.GetComponent<MeshFilter>().mesh;
        m_meshCollider = transform.GetComponent<MeshCollider>();
        //GenTrack();
    }


    /// <summary>
    /// Generate the road
    /// </summary>
    public void GenTrack()
    {
        //Clear all variables every time a new road is made
        m_mesh.Clear();
        m_randPoints.Clear();
        m_curves.m_convexhull.Clear();
        vertices.Clear();
        m_triangles.Clear();
        m_normals.Clear();
        waypoints.Clear();
        foreach (Transform child in waypointParent)
            GameObject.Destroy(child.gameObject);
        
        //Generate random points
        GenPoints();
        //Get convexhull of the random points
        m_curves.CreateConvexhull(m_randPoints, m_randPoints.Count);
        //Sort the convexhull in clockwise order
        m_curves.m_convexhull.Sort(new CurveHandler.ClockwiseComparer(new Vector2(transform.localPosition.x, transform.localPosition.z)));
        //Smooth the resultant convexhull
        m_curves.m_convexhull = MakeSmoothCurve(m_curves.m_convexhull);
       
        GenMesh();
        GenWaypoints();
        //Use the generated mesh as mesh collider too
        m_meshCollider.sharedMesh = m_mesh;
    }



    /// <summary>
    /// Generates the checkpoints on the road
    /// </summary>
    private void GenWaypoints()
    {
        //Creating the gameobject to instantiate on road
        m_waypoint = new GameObject("Waypoint");
        m_waypoint.layer = 2;
        m_waypoint.tag = "Waypoint";
        m_waypoint.AddComponent<BoxCollider>();
        BoxCollider _waypointcol = m_waypoint.GetComponent<BoxCollider>();
        _waypointcol.size = new Vector3(2*roadOffset,2* roadOffset,0.5f);
        _waypointcol.isTrigger = true;

        //Instantiating the checkpoints along the road with proper rotation
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


    /// <summary>
    /// Generates road mesh from the generated convexhull list
    /// </summary>
    private void GenMesh()
    {
        //Assigning vertices and normals
        for (int i = 0; i < m_curves.m_convexhull.Count - 1; i++)
        {
            Vector2 _vec2 = (m_curves.m_convexhull[i + 1] - m_curves.m_convexhull[i]).normalized;
            Vector3 _vec2tovec3 = new Vector3(_vec2.x, 0f, _vec2.y);
            _vec2tovec3 = Vector3.Cross(_vec2tovec3, Vector3.up);
            Vector3 _newvec3 = (new Vector3(m_curves.m_convexhull[i].x, 0f, m_curves.m_convexhull[i].y));
            vertices.Add(_newvec3 + (_vec2tovec3 * roadOffset));
            m_normals.Add(Vector3.up);
            vertices.Add(_newvec3 + (-_vec2tovec3 * roadOffset));
            m_normals.Add(Vector3.up);
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
        }

        //Setting the values for the mesh
        m_mesh.SetVertices(vertices);
        m_mesh.SetTriangles(m_triangles, 0);
        m_mesh.SetNormals(m_normals);
    }

    private void GenPoints()
    {
        //Generating random points within a given area
        for (int i = 0; i < pointDensity; i++)
        {
            float _rand1 = UnityEngine.Random.value;
            float _rand2 = UnityEngine.Random.value;
            m_randPoints.Add(new Vector2(transform.localPosition.x + Map(Mathf.PerlinNoise(_rand1, _rand2), 0, 1, m_roadBounds.minX,  m_roadBounds.maxX), transform.localPosition.z + Map(Mathf.PerlinNoise(_rand2, _rand1), 0, 1,  m_roadBounds.minZ,  m_roadBounds.maxZ)));
        }
    }

    /// <summary>
    /// Remaps given value between a new range
    /// </summary>
    public float Map(float _value, float _from1, float _to1, float _from2, float _to2)
    {
        return (_value - _from1) / (_to1 - _from1) * (_to2 - _from2) + _from2;
    }



    /// <summary>
    /// Smooths the generated convexhull
    /// </summary>
    private List<Vector2> MakeSmoothCurve(List<Vector2> _arrayToCurve)
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
