using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CarAgent : Agent
{
    /// <summary>
    /// public variables
    /// </summary>
  
    public GameObject road;
    public GameObject obstacles;
    public Transform target;



    /// <summary>
    /// private variables
    /// </summary>

    [SerializeField] private bool m_allGrounded = false;
    [SerializeField] private float m_currentReward;
    [SerializeField] private float m_distanceToTarget;
    [SerializeField] private float m_prevDistanceToTarget;
    [SerializeField] private int m_obstacleHit;
    [SerializeField] private int m_prevObstacleHit;
    [SerializeField] private int m_nextCheckpointNumber;
    [SerializeField] private Vector3 m_dirToTarget;
   
    private CarController m_carController;
    private int m_steps;
    private int m_deadCounter = 0;
    private ObstacleGenerator m_obstacleGen;
    private Rigidbody m_carRigidbody;
    private RoadGenerator m_roadGen;
    private Transform m_nextCheckpoint;
    private Vector2 m_move;
    private Vector3 m_checkpointPos;
    private WheelCollider[] m_wheelColliders;
    private WheelHit m_out;

    //private Vector3 m_prevCheckpointPos;
    //private Vector3 m_prevToNextCheckpoint;
    //private Vector3 m_prevToCar;
    //private Vector3 m_closestPoint;
    //private float m_laneOffset;
    //private RaycastHit m_rayout;

    public override void Initialize()
    {
        m_roadGen = road.GetComponent<RoadGenerator>();
        m_carController = this.transform.GetComponent<CarController>();
        m_carRigidbody = this.transform.GetComponent<Rigidbody>();
        m_wheelColliders = this.transform.GetComponentsInChildren<WheelCollider>();
        m_obstacleGen = obstacles.GetComponent<ObstacleGenerator>();
    }

    public override void OnEpisodeBegin()
    {
        RoadAndObstacleReset();
        ResetAll();
        PrivateVariableReset();
    }

    public override void OnActionReceived(ActionBuffers actions)
    {

        m_move.x = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        m_move.y = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        m_carController.AgentMove(m_move);

    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var _actionsOut = actionsOut.ContinuousActions;
        _actionsOut[0] = m_move.x;
        _actionsOut[1] = m_move.y;
        m_carController.AgentMove(m_move);

    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent velocity
        sensor.AddObservation(transform.InverseTransformDirection(m_carRigidbody.velocity)); // vec3

        // Distance to incoming checkpoint
        sensor.AddObservation(m_distanceToTarget); // float

        // Agent's normalized local position
        sensor.AddObservation(new Vector3(transform.localPosition.x / 500f, transform.localPosition.y / 20f, transform.localPosition.z / 500f)); // vec3

        // Calculate the direction to incoming checkpoint
        m_dirToTarget = (m_checkpointPos - transform.localPosition).normalized;

        // Dot product of agent forward and direction to incoming checkpoint/target
        sensor.AddObservation(Vector3.Dot(transform.forward, m_dirToTarget)); //float

        // Agent angular velocity
        sensor.AddObservation(transform.InverseTransformDirection(m_carRigidbody.angularVelocity)); // vec3
    }

    private void RoadAndObstacleReset()
    {
        //Generates the road
        m_roadGen.GenTrack();

        //Randomly sets road with obstacles or no obstacles (put range 0-3 if you want obstacles to move)
        m_obstacleGen.obstacleState = UnityEngine.Random.Range(0, 2);
        m_obstacleGen.ObstacleStateChange();
        
        //Sets random speed for obstacles
        //m_obstacleGen.obstacleSpeed = UnityEngine.Random.Range(0.1f, 2f);
    }

    private void PrivateVariableReset()
    {
        //step count of episode
        m_steps = 0;

        //movement vector of car
        m_carController.AgentMove( Vector2Int.zero);

        //variable to store the accumulated reward for each episode for debugging purposes
        m_currentReward = 0;
 
        m_obstacleHit = 0;
        m_prevObstacleHit = 0;

        //variable to count how long the agent moves with speed less than minimum threshold (1 used here)
        m_deadCounter = 0;
        
        //Current distance to target and previous recorded distance to target
        m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
        m_prevDistanceToTarget = m_distanceToTarget;
    }

    private void ResetAll()
    {
        //Resets checkpoints
        m_nextCheckpoint = m_roadGen.waypoints[0];
        m_nextCheckpointNumber = 1;

        //Resets agent velocities
        m_carRigidbody.velocity = Vector3.zero;
        m_carRigidbody.angularVelocity = Vector3.zero;

        //Stops movement of all wheel colliders
        foreach (WheelCollider tempcol in m_wheelColliders)
        {
            tempcol.brakeTorque = Mathf.Infinity;
        }

        //Assigns random start point (around the beggining of the road) for agent with random rotation
        transform.localPosition = ((m_roadGen.vertices[4] + m_roadGen.vertices[5] + m_roadGen.vertices[6] + m_roadGen.vertices[7]) / 4f);
        transform.localRotation = Quaternion.LookRotation((((m_nextCheckpoint.localPosition) - new Vector3(0, m_roadGen.roadOffset, 0)) - transform.localPosition).normalized, Vector3.up);
        transform.localPosition += new Vector3(0, 0.8f, 0);
        transform.localPosition += transform.right * UnityEngine.Random.Range((-m_roadGen.roadOffset + 2f), (m_roadGen.roadOffset - 2f));
        transform.Rotate(new Vector3(0f, UnityEngine.Random.Range(-80f, 80f), 0f));

        //Assigns starting checkpoint
        m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
        //m_prevCheckpointPos = transform.localPosition;

        //Assigns target position
        target.transform.localPosition = ((m_roadGen.vertices[m_roadGen.vertices.Count - 1] + m_roadGen.vertices[m_roadGen.vertices.Count - 2] + m_roadGen.vertices[m_roadGen.vertices.Count - 3] + m_roadGen.vertices[m_roadGen.vertices.Count - 4]) / 4f);
        target.transform.localPosition += new Vector3(0, 0.5f, 0);
    }

    
    private void OnCollisionStay(Collision collision)
    {
        //Adds negative reward if agent tries to move by pushing the obstacle
        if (collision.collider.CompareTag("Obstacle"))
        {
            m_currentReward += -0.1f;
            AddReward(-0.1f);
        }
        
    }

    /// <summary>
    /// Checks for collision with obstacles or deadzone i.e. agent fell off the track
    /// </summary>
    private void OnCollisionEnter (Collision other)
    {
        if (other.collider.CompareTag("Obstacle"))
        {
            m_currentReward += -2 * (m_obstacleHit + 1);
            AddReward(-2 * (m_obstacleHit+1));
            m_obstacleHit++;
        }
        
    }
   
    private void NextEpisode(float _reward)
    {
        AddReward(_reward);
        EndEpisode();
    }

    /// <summary>
    /// Checks for collision with final target or checkpoints
    /// </summary>
    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Finish"))
        {
            if (m_roadGen.waypoints.Count < m_nextCheckpointNumber)
            {
                m_currentReward += 5f + ((200f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
                AddReward(5f + ((200f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                EndEpisode();
            }
            // If agent reaches the final target without clearing the previous checkpoints
            else
            {
                NextEpisode(-5f);
            }
        }
        else if(other.CompareTag("Waypoint"))
        {
            //If agent collided with the right checkpoint
            if (other.gameObject.transform == m_nextCheckpoint)
            {
                //Debug.Log("Good Checkpoint " + m_nextcheckpointnumber);
                m_currentReward += (1.5f + ((50f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)) +((m_obstacleGen.obstaclesBeforeWaypoint[m_nextCheckpointNumber - 1] - (m_obstacleHit-m_prevObstacleHit))*0.5f));
                AddReward(1.5f + ((50f* m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)) + ((m_obstacleGen.obstaclesBeforeWaypoint[m_nextCheckpointNumber - 1] - (m_obstacleHit - m_prevObstacleHit)) * 0.5f));
                m_prevObstacleHit = m_obstacleHit;
                NewCheckpoint();
            }
            //If agent collided with wrong checkpoint
            else
            {
                m_currentReward += (-1f);
                AddReward(-1f );
               //Debug.Log("Bad Checkpoint");
            }
        }
        //Colliding with outermost boundary
        else if(other.CompareTag("Respawn"))
        {
            
            NextEpisode(-1f);
        }
    }

    private void NewCheckpoint()
    {
        //m_prevCheckpointPos = m_checkpointPos;

        //If there are more checkpoints to be crossed then assign the incoming checkpoint
        if (m_roadGen.waypoints.Count > m_nextCheckpointNumber)
        {
            m_nextCheckpoint = m_roadGen.waypoints[m_nextCheckpointNumber];
            m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
            m_nextCheckpointNumber++;
            m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
            m_prevDistanceToTarget = m_distanceToTarget;
        }
        //If there are no checkpoints left then assign the final target
        else
        {
            m_nextCheckpoint = target.transform;
            m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
            m_nextCheckpointNumber += 10;
            m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
            m_prevDistanceToTarget = m_distanceToTarget;
        }
    }

    private void CheckMovement()
    {
        CheckDistanceFromCheckpoint();
        if ( m_carRigidbody.velocity.magnitude > 3f)
        {
            //Reward based on how far the agent is from the middle of road
            //m_currentReward += (1f / (3000f * m_laneOffset * m_laneOffset * m_laneOffset));
            //AddReward(1f / (3000f * m_laneOffset * m_laneOffset * m_laneOffset));
      
            //Reward based on the direction of movement and speed of agent
            m_currentReward += (0.0005f * ((Vector3.Dot(m_carRigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f));
            AddReward((0.0005f * ((Vector3.Dot(m_carRigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f)));
        }

      //If agent's speed is too low and its off the track for a certain amount of time then end the episode
        if (m_carRigidbody.velocity.magnitude < 1f || !m_allGrounded)
        {
                m_deadCounter++;
        }
        if (m_carRigidbody.velocity.magnitude > 1f && m_allGrounded)
        { 
            m_deadCounter = 0;
        }
        if (m_deadCounter >= 500)
        {
            NextEpisode(-2f);
        }
    }


    /// <summary>
    /// Calculates distance of incoming checkpoint from agent and gives a reward based on the progress of the agent towards the checkpoint
    /// </summary>
    private void CheckDistanceFromCheckpoint()
    {
        m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
        // Gives reward every 50th step
        if (m_steps % 50 == 0)
        {
            m_currentReward += (m_prevDistanceToTarget - m_distanceToTarget) / 50f;
            //Debug.Log("Distance reward = " + (m_prevDistanceToTarget - m_distanceToTarget) / 50f);
            AddReward((m_prevDistanceToTarget - m_distanceToTarget) / 50f);
            m_prevDistanceToTarget = m_distanceToTarget;
        }
    }

    /// <summary>
    /// Checks if all wheel colliders are grounded and if all wheels are touching the road or not
    /// </summary>
    private void CheckGrounded()
    {
        foreach (WheelCollider tempcol in m_wheelColliders)
        {
            if (!tempcol.isGrounded)
            {
                m_allGrounded = false;
                break;
            }
            else
            {
                tempcol.GetGroundHit(out m_out);
                if (m_out.collider.CompareTag("DeadZone"))
                {
                    NextEpisode(-5f);
                    //m_currentReward += -0.5f;
                    //AddReward(-0.5f);
           
                    break;
                }
            }
        }
    }
    private void FixedUpdate()
    {
        m_steps++;
        m_allGrounded = true;

        //CalcLaneOffset();

        CheckGrounded();
        CheckMovement();
    }

    /// <summary>
    /// Calculates the shortest distance of agent from the line connecting previous checkpoint to next checkpoint
    /// </summary>
    //private void CalcLaneOffset()
    //{
    //    m_prevToNextCheckpoint = m_checkpointPos - m_prevCheckpointPos;
    //    m_prevToNextCheckpoint = Vector3.ProjectOnPlane(m_prevToNextCheckpoint, Vector3.up);
    //    m_prevToCar = transform.localPosition - m_prevCheckpointPos;
    //    m_prevToCar = Vector3.ProjectOnPlane(m_prevToCar, Vector3.up);
    //    m_closestPoint = Vector3.Project(m_prevToCar, m_prevToNextCheckpoint);
    //    m_closestPoint = m_prevCheckpointPos + m_closestPoint;
    //    m_laneOffset = Mathf.Clamp(Vector3.Distance(new Vector3(transform.localPosition.x, 0, transform.localPosition.z), m_closestPoint), 0.4f, 10f);
    //}

    private void Update()
    {
        //Debug.Log("Local position  = " + transform.localPosition);
        //Debug.Log("Obstacle hit = " + m_obstacleHit);
        //Debug.Log("Lane offset = " + m_laneOffset);
        //Debug.Log("Angular velocity = " + transform.InverseTransformDirection(m_carRigidbody.angularVelocity));
        //m_dirToTarget = (m_checkpointPos - transform.localPosition).normalized;
        //Debug.Log("Dot product (agent forward,dirToTarget) = "+Vector3.Dot(transform.forward, m_dirToTarget)); //float
        //Debug.Log("Velocity = " + transform.InverseTransformDirection(m_carRigidbody.velocity));

        //Physics.Raycast(transform.position + (transform.forward * (1.97f)), Vector3.down, out m_rayout, 10f);
        //Debug.Log("Raycast hit = " + m_rayout.collider.tag);

        //Line showing direction to incoming checking
        //Debug.DrawLine(transform.position, transform.position + m_dirToTarget * 10f, Color.red);
        //Line showing agent's forward direction
        //Debug.DrawLine(transform.position, transform.position + 10f * transform.forward, Color.blue);
    }
}
