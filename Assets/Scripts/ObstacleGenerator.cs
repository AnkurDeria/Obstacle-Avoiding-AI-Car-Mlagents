
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class ObstacleGenerator : MonoBehaviour
{
    /// <summary>
    /// public variables
    /// </summary>

    [Range(0f, 1.1f)] public float obstacleDensity;

    [Header("0 for no obstacles, 1 for static obstacles,", order = 0)]
    [Space(-10, order = 1)]
    [Header("2 for moving obstacles", order = 2)]
    public int obstacleState = 0;
    
    public float obstacleSpeed;
    public List<int> waypointOnSpline = new List<int>();
    public List<int> obstaclesBeforeWaypoint = new List<int>();


    /// <summary>
    /// private variables
    /// </summary>

    [SerializeField] private bool m_obstacleMove = false;
    [SerializeField] private Material m_obstacleMaterial;

    private List<float> m_obstacleRandSpeed = new List<float>();
    private List<Vector3> m_obstacleMoveDir = new List<Vector3>();
    private List<Vector3> m_roadMid = new List<Vector3>();
    private List<Transform> m_obstacles = new List<Transform>();
    private RoadGenerator m_roadGen;
    
    void Start()
    {
        m_roadGen = transform.parent.GetComponentInChildren<RoadGenerator>();
    }

    /// <summary>
    /// Changes obstacle state via user input
    /// </summary>
    public void Obstacle(InputAction.CallbackContext context)
    {
        if (context.started)
        {
            obstacleState++;
            obstacleState %= 3;
            ObstacleStateChange();
        }
    }

    /// <summary>
    /// Decides if the road has moving obstacles, static obstacles or no obstacles
    /// </summary>
    public void ObstacleStateChange()
    {
        obstaclesBeforeWaypoint.Clear();
        for (int i = 0; i < m_roadGen.waypoints.Count; i++)
        {
            obstaclesBeforeWaypoint.Add(0);
        }

        if (m_obstacles.Count > 0)
        {
            m_obstacles.Clear();
            m_obstacleRandSpeed.Clear();
            m_roadMid.Clear();
            foreach (Transform child in transform)
                GameObject.Destroy(child.gameObject);
            m_obstacleMove = false;
        }

        if (obstacleState == 1)
        {
            GenObstacles(true);
        }

        else if (obstacleState == 2)
        {
            if (m_obstacles.Count == 0)
                GenObstacles(false);
            m_obstacleMove = true;
        }
    }

    private void GenObstacles(bool _kinematic)
    {
        // Create the gameobject to instantiate repeatedly
        GameObject _obstacle = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _obstacle.tag = "Obstacle";
        _obstacle.GetComponent<MeshRenderer>().material = m_obstacleMaterial;
        _obstacle.AddComponent<Rigidbody>().mass = 500;
        _obstacle.GetComponent<Rigidbody>().isKinematic = _kinematic;

        int _waypointMark = 2;
        for (int i = 10; i < (m_roadGen.vertices.Count)/2 - 6; i+=2)
        {
            if (UnityEngine.Random.value > 1f - obstacleDensity)
            {
                float _t = UnityEngine.Random.value;

                // Instantiate obstacle at random point on the road with random rotation 
                m_obstacles.Add(Instantiate(_obstacle,transform.TransformPoint(new Vector3(Mathf.Lerp(m_roadGen.vertices[2*i].x,m_roadGen.vertices[(2*i)+1].x, _t), _obstacle.GetComponent<SphereCollider>().radius,Mathf.Lerp(m_roadGen.vertices[2 * i].z, m_roadGen.vertices[(2 * i) + 1].z, _t)) ), Quaternion.Euler(0f,UnityEngine.Random.Range(-180f,180f),0f), transform).transform);

                // Store the midpoint of the road where the obstacle was instantiated
                m_roadMid.Add(new Vector3((m_roadGen.vertices[2 * i].x + m_roadGen.vertices[(2 * i) + 1].x) / 2f, _obstacle.GetComponent<SphereCollider>().radius, (m_roadGen.vertices[2 * i].z + m_roadGen.vertices[(2 * i) + 1].z) / 2f));

                // Direction of movement of the obstacle if it moves i.e. perpendicular to the direction of the road
                m_obstacleMoveDir.Add((m_roadGen.vertices[2 * i] - m_roadGen.vertices[(2 * i) + 1]).normalized);
               
                // Assign random speed to the obstacle
                m_obstacleRandSpeed.Add(_t);
                for(int j=_waypointMark;j<waypointOnSpline.Count;j++)
                {
                    if(i<waypointOnSpline[j])
                    {
                        obstaclesBeforeWaypoint[j]++;
                        break;
                    }
                    else
                    {
                        _waypointMark++;
                    }
                }
            }
        }

        GameObject.Destroy(_obstacle);
    }

    private void FixedUpdate()
    {
        if (m_obstacleMove)
        {
            // Move the obstacle perpendicular to the direction of the road
            for (int i = 0; i < m_obstacles.Count; i++)
            {
                m_obstacles[i].GetComponent<Rigidbody>().MovePosition(transform.TransformPoint(m_roadMid[i]) + (transform.TransformDirection(m_obstacleMoveDir[i]) * Mathf.Sin(Time.time * m_obstacleRandSpeed[i] * obstacleSpeed) * (m_roadGen.halfRoadWidth - 0.5f)));

            }
        }
    }

}
