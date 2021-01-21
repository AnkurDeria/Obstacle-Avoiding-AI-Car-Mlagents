using System;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

public class CarAgent : Agent
{
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
    [SerializeField] private GameObject m_road;
    [SerializeField] private GameObject m_obstacles;
    [SerializeField] private Transform m_target;
    [SerializeField] private Vector3 m_dirToTarget;
    [SerializeField] private Vector3 m_velocity;
    [SerializeField] private Vector3 m_angularVelocity;

    private WheelVehicle m_carController;
    private int m_steps;
    private int m_deadCounter;
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
        m_roadGen = m_road.GetComponent<RoadGenerator>();
        m_carController = GetComponent<WheelVehicle>();
        m_carRigidbody = GetComponentInChildren<Rigidbody>();
        m_wheelColliders = GetComponentsInChildren<WheelCollider>();
        m_obstacleGen = m_obstacles.GetComponent<ObstacleGenerator>();
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
        m_velocity = transform.InverseTransformDirection(m_carRigidbody.velocity) / 20f;
        sensor.AddObservation(new Vector2(m_velocity.x, m_velocity.z)); // vec2

        // Distance to incoming checkpoint
        sensor.AddObservation(m_distanceToTarget / 30f); // float

        // Agent's normalized local position
        sensor.AddObservation(new Vector2(transform.localPosition.x / 500f, transform.localPosition.z / 500f)); // vec2

        // Agent's normalized torque and steering angle
        sensor.AddObservation(m_carController.GetTorque()); //float
        sensor.AddObservation(m_carController.GetSteeringAngle()); //float

        // Calculate the direction to incoming checkpoint
        m_dirToTarget = (m_checkpointPos - transform.localPosition).normalized;

        // Dot product of agent forward and direction to incoming checkpoint/target
        sensor.AddObservation(Vector3.Dot(transform.forward, m_dirToTarget)); //float

        // Agent angular velocity
        m_angularVelocity = transform.InverseTransformDirection(m_carRigidbody.angularVelocity) / 3f;
        sensor.AddObservation(m_angularVelocity.y); // float
    }

    private void RoadAndObstacleReset()
    {
        // Randomly select clockwise or anti-clockwise sorting of control points of spline
        float _rand = UnityEngine.Random.Range(-1f, 1f);

        // Generates the road 
        m_roadGen.GenTrack((int)(Mathf.Sign(_rand) * Mathf.Ceil(Mathf.Abs(_rand))));

        // Randomly sets road with obstacles or no obstacles (0 for no obstalces
        //                                                    1 for static obstacles
        //                                                    2 for moving obstacles)
        m_obstacleGen.obstacleState = 1; //UnityEngine.Random.Range(1, 2);
        m_obstacleGen.obstacleDensity = UnityEngine.Random.Range(0.6f, 1.1f);
        m_obstacleGen.ObstacleStateChange();

        // Sets random speed for obstacles
        //m_obstacleGen.obstacleSpeed = UnityEngine.Random.Range(0.1f, 2f);
    }

    private void PrivateVariableReset()
    {
        // Step count of episode
        m_steps = 0;

        // Movement vector of car
        m_carController.AgentMove(Vector2Int.zero);

        // Variable to store the accumulated reward for debugging purposes
        m_currentReward = 0;

        m_obstacleHit = 0;
        m_prevObstacleHit = 0;

        // Variable to count how long the agent moves with speed less than minimum threshold (1 used here)
        m_deadCounter = 0;

        // Current distance to target and previous recorded distance to target
        m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
        m_prevDistanceToTarget = m_distanceToTarget;
    }

    /// <summary>
    /// Resets agent position and target position according to new track
    /// </summary>
    private void ResetAll()
    {
        // Resets checkpoints
        m_nextCheckpoint = m_roadGen.waypoints[0];
        m_nextCheckpointNumber = 1;

        // Resets agent velocities
        m_carRigidbody.velocity = Vector3.zero;
        m_carRigidbody.angularVelocity = Vector3.zero;

        // Stops movement of all wheel colliders
        foreach (WheelCollider tempcol in m_wheelColliders)
        {
            tempcol.brakeTorque = Mathf.Infinity;
        }

        // Assign random start point (around the beggining of the road) for agent with random rotation

        // Midpoint of 4 vertices of the road
        transform.localPosition = ((m_roadGen.vertices[4] + m_roadGen.vertices[5] + m_roadGen.vertices[6] + m_roadGen.vertices[7]) / 4f);

        // Rotation facing the incoming checkpoint 
        transform.localRotation = Quaternion.LookRotation((((m_nextCheckpoint.localPosition) - new Vector3(0, m_roadGen.halfRoadWidth, 0)) - transform.localPosition).normalized, Vector3.up);

        // Place the agent above the road
        transform.localPosition += new Vector3(0, 0.8f, 0);

        // Randomize the position of the agent
        transform.localPosition += transform.right * UnityEngine.Random.Range((-m_roadGen.halfRoadWidth + 2f), (m_roadGen.halfRoadWidth - 2f));

        // Randomize rotation
        transform.Rotate(new Vector3(0f, UnityEngine.Random.Range(-45f, 45f), 0f));

        // Assigns starting checkpoint
        m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
        //m_prevCheckpointPos = transform.localPosition;

        // Assigns target position
        m_target.transform.localPosition = ((m_roadGen.vertices[m_roadGen.vertices.Count - 1] + m_roadGen.vertices[m_roadGen.vertices.Count - 2] + m_roadGen.vertices[m_roadGen.vertices.Count - 3] + m_roadGen.vertices[m_roadGen.vertices.Count - 4]) / 4f);
        m_target.transform.localPosition += new Vector3(0, 0.5f, 0);
    }

    private void OnCollisionStay(Collision collision)
    {
        // Adds negative reward if agent tries to move by pushing the obstacle
        if (collision.collider.CompareTag("Obstacle"))
        {
            m_currentReward += -0.01f;
            AddReward(-0.01f);
        }
    }

    /// <summary>
    /// Checks for collision with obstacles or deadzone i.e. agent fell off the track
    /// </summary>
    private void OnCollisionEnter(Collision other)
    {
        if (other.collider.CompareTag("Obstacle"))
        {
            //Debug.Log("Collided with obstacle, negative reward = " + (-1f * (m_obstacleHit + 1) * (m_currentReward / 5f)));

            // Reset reward to -1 if agent hits obstacle
            m_currentReward += -1f * (m_obstacleHit + 1) * (m_currentReward / 10f);
            AddReward(-1f * (m_obstacleHit + 1) * (m_currentReward / 10f));

            m_obstacleHit++;
        }
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
                // A set reward for reaching the final target + extra reward based on how quickly the agent reached the target
                m_currentReward += 1f + ((30f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
                AddReward(1f + ((30f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                EndEpisode();
            }

            // If agent reaches the final target without clearing the previous checkpoints
            else
            {
                NextEpisode(-1f);
            }
        }

        else if (other.CompareTag("Waypoint"))
        {
            // If agent collided with the right checkpoint
            if (other.gameObject.transform == m_nextCheckpoint)
            {
                //Debug.Log("Good Checkpoint " + m_nextcheckpointnumber);

                // A set reward for reaching the checkpoint + extra reward based on how quickly the agent reached the checkpoint 
                // and how many obstacles it was able to avoid completely
                m_currentReward += (0.5f + ((20f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                AddReward(0.5f + ((20f * m_nextCheckpointNumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));

                m_prevObstacleHit = m_obstacleHit;
                NewCheckpoint();
            }

            // If agent collided with the wrong checkpoint
            else
            {
                //Debug.Log("Bad Checkpoint");

                m_currentReward += (-1f);
                AddReward(-1f);
            }
        }

        // Colliding with outermost boundary
        else if (other.CompareTag("Respawn"))
        {
            NextEpisode(-1f);
        }
    }

    private void NewCheckpoint()
    {
        //m_prevCheckpointPos = m_checkpointPos;

        // If there are more checkpoints to be crossed then assign the incoming checkpoint
        if (m_roadGen.waypoints.Count > m_nextCheckpointNumber)
        {
            m_nextCheckpoint = m_roadGen.waypoints[m_nextCheckpointNumber];
            m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
            m_nextCheckpointNumber++;

            // Reset the distance to the new checkpoint
            m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
            m_prevDistanceToTarget = m_distanceToTarget;
        }

        // If there are no checkpoints left then assign the final target
        else
        {
            m_nextCheckpoint = m_target.transform;
            m_checkpointPos = (new Vector3(m_nextCheckpoint.localPosition.x, transform.localPosition.y, m_nextCheckpoint.localPosition.z));
            m_nextCheckpointNumber++;

            // Reset the distance to the final target
            m_distanceToTarget = Vector3.Distance(m_checkpointPos, transform.localPosition);
            m_prevDistanceToTarget = m_distanceToTarget;
        }
    }

    private void CheckMovement()
    {
        CheckDistanceFromCheckpoint();

        if (m_carRigidbody.velocity.magnitude > 1f)
        {
            // Reward based on how far the agent is from the middle of road
            //m_currentReward += (1f / (3000f * m_laneOffset * m_laneOffset * m_laneOffset));
            //AddReward(1f / (3000f * m_laneOffset * m_laneOffset * m_laneOffset));

            // Reward based on the agent moving in the forward direction 
            m_currentReward += (0.0005f * ((Vector3.Dot(m_carRigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f));
            AddReward((0.0005f * ((Vector3.Dot(m_carRigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f)));
        }

        // If agent's speed is too low or its off the track for a certain amount of time then start giving negative rewards
        if (m_carRigidbody.velocity.magnitude < 0.5f || !m_allGrounded)
        {
            m_deadCounter++;
        }

        if (m_carRigidbody.velocity.magnitude > 0.5f && m_allGrounded)
        {
            m_deadCounter = 0;
        }

        if (m_deadCounter >= 500)
        {
            m_currentReward += -0.001f;
            AddReward(-0.001f);
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
            //Debug.Log("Distance reward = " + (m_prevDistanceToTarget - m_distanceToTarget) / 25f);

            m_currentReward += (m_prevDistanceToTarget - m_distanceToTarget) / 25f;
            AddReward((m_prevDistanceToTarget - m_distanceToTarget) / 25f);

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
            if (transform.position.y >= 1f || transform.position.y <= 0f)
            {
                NextEpisode(-1f);
            }

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
                    NextEpisode(-1f);

                    //m_currentReward += -0.5f;
                    //AddReward(-0.5f);
                }
            }
        }
    }


    /// <summary>
    /// Sets the reward to the given value and ends the episode
    /// </summary>
    private void NextEpisode(float _reward)
    {
        SetReward(_reward);
        EndEpisode();
    }

    private void InfiniteRewardCheck()
    {
        if (Mathf.Abs(m_currentReward) > 10000)
        {
            NextEpisode(0f);
        }
    }

    private void FixedUpdate()
    {
        m_steps++;

        m_allGrounded = true;

        InfiniteRewardCheck();

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
        //Debug.Log("Local position  = " + new Vector2(transform.localPosition.x / 500, transform.localPosition.z / 500));
        //Debug.Log("Lane offset = " + m_laneOffset);
        //Debug.Log("Angular velocity = " + (transform.InverseTransformDirection(m_carRigidbody.angularVelocity)/3f).y);
        //m_dirToTarget = (m_checkpointPos - transform.localPosition).normalized;
        Debug.Log("Dot product (agent forward,dirToTarget) = "+Vector3.Dot(transform.forward, m_dirToTarget)); 
        //Debug.Log("Velocity = " + transform.InverseTransformDirection(m_carRigidbody.velocity)/20f);
        //Debug.Log("Steering = " + m_carController.GetSteeringAngle());
        //Debug.Log("Torque = " + m_carController.GetTorque());

        //Physics.Raycast(transform.position + (transform.forward * (1.97f)), Vector3.down, out m_rayout, 10f);
        //Debug.Log("Raycast hit = " + m_rayout.collider.tag);

        //m_dirToTarget = (m_checkpointPos - transform.localPosition).normalized;
        // Line showing direction to incoming checking
        //Debug.DrawLine(transform.position, transform.position + m_dirToTarget * 10f, Color.red);
        // Line showing agent's forward direction
        //Debug.DrawLine(transform.position, transform.position + 10f * transform.forward, Color.blue);
    }
}
