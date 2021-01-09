
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

    [Range(3, 10)] public float halfRoadWidth;

    public List<Vector3> vertices = new List<Vector3>();
    public List<Transform> waypoints = new List<Transform>();


    /// <summary>
    /// private variables
    /// </summary>

    [Header("Apply a boundary around the track or not")]
    [SerializeField] private bool m_isRoadBoundary = false;

    [SerializeField] private Material m_roadMat;
    [SerializeField] private Transform m_waypointParent;

    private CurveHandler m_curves;
    private MeshCollider[] m_meshCollider = new MeshCollider[3];
    private MeshFilter[] m_mesh = new MeshFilter[3];
    private ObstacleGenerator m_obstacle;

    void Start()
    {
        m_curves = transform.GetComponent<CurveHandler>();
        if (m_isRoadBoundary)
        {
            GameObject m_roadBoundary = new GameObject("Road Boundary");
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
    public void GenTrack(int _sortOrder)
    {
        // Clear all variables every time a new road is made
        ResetVariables();

        m_curves.sortOrder = _sortOrder;
        // Generate catmull rom spline from the convexhull points
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
        if (m_isRoadBoundary)
        {
            m_mesh[1].mesh.Clear();
            m_mesh[2].mesh.Clear();
        }
        vertices.Clear();
        waypoints.Clear();
        m_obstacle.waypointOnSpline.Clear();

        foreach (Transform child in m_waypointParent)
            GameObject.Destroy(child.gameObject);
    }

    /// <summary>
    /// Generates the checkpoints on the road
    /// </summary>
    private void GenWaypoints()
    {
        // Creating the gameobject to instantiate on road
        GameObject _waypoint = new GameObject("Waypoint");
        _waypoint.layer = 10;
        _waypoint.tag = "Waypoint";
        _waypoint.AddComponent<BoxCollider>();
        _waypoint.isStatic = true;
        BoxCollider _waypointcol = _waypoint.GetComponent<BoxCollider>();
        _waypointcol.size = new Vector3(halfRoadWidth, 2 * halfRoadWidth, 0.1f);
        _waypointcol.isTrigger = true;

        // Instantiating the checkpoints along the road with proper rotation
        Vector3 _vec1;
        Vector3 _vec2;
        Vector3 _vec3;

        for (int i = 8; i < m_curves.splinePoints.Count - 5; i += 5)
        {
            _vec1 = new Vector3(m_curves.splinePoints[i + 1].x, 0f, m_curves.splinePoints[i + 1].y);
            _vec2 = new Vector3(m_curves.splinePoints[i - 1].x, 0f, m_curves.splinePoints[i - 1].y);
            _vec3 = ((new Vector3(m_curves.splinePoints[i].x, 0f, m_curves.splinePoints[i].y) - _vec2) +
                     (_vec1 - new Vector3(m_curves.splinePoints[i].x, 0f, m_curves.splinePoints[i].y))) / 2f;
            Quaternion _rot = Quaternion.LookRotation(transform.TransformDirection(_vec3.normalized), Vector3.up);
            waypoints.Add(Instantiate(_waypoint, transform.TransformPoint(new Vector3(m_curves.splinePoints[i].x, halfRoadWidth, m_curves.splinePoints[i].y)), _rot, m_waypointParent).transform);

            m_obstacle.waypointOnSpline.Add(i);
        }

        GameObject.Destroy(_waypoint);
    }

    /// <summary>
    /// Generates road mesh from the generated spline points
    /// </summary>
    private void GenMesh()
    {
        List<int> _boundaryTriangles1 = new List<int>();
        List<int> _boundaryTriangles2 = new List<int>();
        List<Vector2> _uvs = new List<Vector2>();
        List<Vector3> _normals = new List<Vector3>();
        List<Vector3> _boundaryVertices1 = new List<Vector3>();
        List<Vector3> _boundaryVertices2 = new List<Vector3>();
        List<int> _triangles = new List<int>();
        Vector2 _tangent;
        Vector3 _tangent3;
        float _roadPercent;
        m_roadMat.SetVector("Vector2_2439372E", new Vector4(2, m_curves.splinePoints.Count - 1, 0, 0));

        // Assigning vertices and normals
        for (int i = 1; i < m_curves.splinePoints.Count; i++)
        {
            _tangent = Vector2.zero;

            // Calculating normal to the direction of road
            _tangent += m_curves.splinePoints[(i + 1) % m_curves.splinePoints.Count] - m_curves.splinePoints[i];
            _tangent += m_curves.splinePoints[i] - m_curves.splinePoints[(i - 1 + m_curves.splinePoints.Count) % m_curves.splinePoints.Count];
            _tangent.Normalize();
            _tangent3 = new Vector3(-_tangent.y, 0f, _tangent.x);

            _roadPercent = (i-1f) /(float) (m_curves.splinePoints.Count - 1);
            _uvs.Add(new Vector2(0, _roadPercent));
            _uvs.Add(new Vector2(1, _roadPercent));

            Vector3 _newvec3 = new Vector3(m_curves.splinePoints[i].x, 0f, m_curves.splinePoints[i].y);
            vertices.Add(_newvec3 + (_tangent3 * halfRoadWidth));
            _normals.Add(Vector3.up);
            vertices.Add(_newvec3 + (-_tangent3 * halfRoadWidth));
            _normals.Add(Vector3.up);

            if (m_isRoadBoundary)
            {
                _boundaryVertices1.Add(_newvec3 + (_tangent3 * halfRoadWidth));
                _boundaryVertices1.Add((_newvec3 + (_tangent3 * halfRoadWidth)) + Vector3.up * 10f);
                _boundaryVertices2.Add(_newvec3 + (_tangent3 * halfRoadWidth));
                _boundaryVertices2.Add((_newvec3 + (_tangent3 * halfRoadWidth)) + Vector3.up * 10f);
            }
        }

        // Assigning triangle indices
        for (int i = 0; i < vertices.Count - 2; i += 2)
        {
            _triangles.Add(i);
            _triangles.Add(i + 2);
            _triangles.Add(i + 3);
            _triangles.Add(i);
            _triangles.Add(i + 3);
            _triangles.Add(i + 1);
            if (m_isRoadBoundary)
            {
                _boundaryTriangles1.Add(i);
                _boundaryTriangles1.Add(i + 3);
                _boundaryTriangles1.Add(i + 2);
                _boundaryTriangles1.Add(i);
                _boundaryTriangles1.Add(i + 1);
                _boundaryTriangles1.Add(i + 3);
                _boundaryTriangles2.Add(i);
                _boundaryTriangles2.Add(i + 2);
                _boundaryTriangles2.Add(i + 3);
                _boundaryTriangles2.Add(i);
                _boundaryTriangles2.Add(i + 3);
                _boundaryTriangles2.Add(i + 1);
            }
        }

        // Setting the values for the mesh
        m_mesh[0].mesh.SetVertices(vertices);
        m_mesh[0].mesh.SetTriangles(_triangles, 0);
        m_mesh[0].mesh.SetNormals(_normals);
        m_mesh[0].mesh.Optimize();

        // Use the generated mesh as mesh collider too
        m_meshCollider[0].sharedMesh = m_mesh[0].mesh;

        m_mesh[0].mesh.SetUVs(0, _uvs);

        if (m_isRoadBoundary)
        {
            m_mesh[1].mesh.SetVertices(_boundaryVertices1);
            m_mesh[1].mesh.SetTriangles(_boundaryTriangles1, 0);
            m_mesh[2].mesh.SetVertices(_boundaryVertices2);
            m_mesh[2].mesh.SetTriangles(_boundaryTriangles2, 0);
            m_meshCollider[1].sharedMesh = m_mesh[1].mesh;
            m_meshCollider[2].sharedMesh = m_mesh[2].mesh;
        }
    }

    private void Update()
    {
        foreach (var i in waypoints)
        {
            Debug.DrawLine(i.position, i.position + i.forward * 10f, Color.black);
        }
    }
}

