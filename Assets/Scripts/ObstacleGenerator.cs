using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class ObstacleGenerator : MonoBehaviour
{
    public float obstacleSpeed;
    public List<Transform> obstacles = new List<Transform>();
    public int obstacleState = 0;
    public bool obstacleMove = false;

    private CurveHandler m_curves;
    private List<Vector3> m_obstacleMoveDir = new List<Vector3>();
    private List<Vector3> m_roadMid = new List<Vector3>();
    private List<float> m_obstacleRandSpeed = new List<float>();
    private RoadGenerator m_roadGen;
    private Transform m_road;
    // Start is called before the first frame update
    void Start()
    {
        m_road = transform.parent.Find("Road");
        m_roadGen = m_road.GetComponent<RoadGenerator>();
        m_curves = m_road.GetComponent<CurveHandler>();
    }
    private void GenObstacles()
    {
        //Create the gameobject to instantiate repeatedly
        GameObject _obstacle = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        _obstacle.tag = "Obstacle";
        _obstacle.AddComponent<Rigidbody>().mass = 500;
        
        for (int i = 3; i < m_curves.m_convexhull.Count - 2; i++)
        {
            
            if (UnityEngine.Random.value > 0.6f)
            {
                float _t = UnityEngine.Random.value;

                //Instantiate obstacle at random point on the road with random rotation 
                obstacles.Add(Instantiate(_obstacle,transform.TransformPoint(new Vector3(Mathf.Lerp(m_roadGen.vertices[2*i].x,m_roadGen.vertices[(2*i)+1].x, _t), _obstacle.GetComponent<SphereCollider>().radius / 2f,Mathf.Lerp(m_roadGen.vertices[2 * i].z, m_roadGen.vertices[(2 * i) + 1].z, _t)) ), Quaternion.Euler(0f,UnityEngine.Random.Range(-180f,180f),0f), transform).transform);

                //Store the midpoint of the road where the obstacle was instantiated
                m_roadMid.Add(new Vector3((m_roadGen.vertices[2 * i].x + m_roadGen.vertices[(2 * i) + 1].x) / 2f, _obstacle.GetComponent<SphereCollider>().radius, (m_roadGen.vertices[2 * i].z + m_roadGen.vertices[(2 * i) + 1].z) / 2f));

                //Direction of movement of the obstacle if it moves i.e. perpendicular to the direction of the road
                m_obstacleMoveDir.Add((m_roadGen.vertices[2 * i] - m_roadGen.vertices[(2 * i) + 1]).normalized);
               
                //Assign random speed to the obstacle
                m_obstacleRandSpeed.Add(_t);
            }
        }
        GameObject.Destroy(_obstacle);
    }
    private void FixedUpdate()
    {
        
        if (obstacleMove)
        {
            //Move the obstacle perpendicular to the direction of the road
            for (int i = 0; i < obstacles.Count; i++)
            {
                obstacles[i].GetComponent<Rigidbody>().MovePosition(transform.TransformPoint(m_roadMid[i]) + (transform.TransformDirection(m_obstacleMoveDir[i]) * Mathf.Sin(Time.time * m_obstacleRandSpeed[i] * obstacleSpeed) * (m_roadGen.roadOffset - 0.5f)));

            }
        }
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
        if (obstacles.Count > 0)
        {
            obstacles.Clear();
            m_obstacleRandSpeed.Clear();
            m_roadMid.Clear();
            foreach (Transform child in transform)
                GameObject.Destroy(child.gameObject);
            obstacleMove = false;
        }
        if (obstacleState == 1)
        {
            GenObstacles();
        }
        else if (obstacleState == 2)
        {
            if (obstacles.Count == 0)
                GenObstacles();
            obstacleMove = true;
        }
    }
}
