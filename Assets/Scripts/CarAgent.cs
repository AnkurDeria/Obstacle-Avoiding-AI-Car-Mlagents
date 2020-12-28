using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

public class CarAgent : Agent
{

    public Transform target;
    public GameObject road;
    public GameObject obstacles;



    private CarController m_cc;
    private Rigidbody m_carrigidbody;
    private WheelCollider[] m_wheelcols;
    private bool m_allgrounded = false;
    private Vector3 m_prevtonextcheckpoint;
    private Vector3 m_prevtocar;
    private Vector3 m_closestpoint;
    private float m_laneOffset;
    private int m_steps;
    private float m_cuurentReward;
    private int m_obstacleHit;
    private Vector3 m_velocity = Vector3.zero;
    private Vector3 m_dirToTarget;
    private Transform m_nextcheckpoint;
    private Vector3 m_prevcheckpointpos;
    private Vector3 m_checkpointpos;
    private int m_nextcheckpointnumber;
    private ObstacleGenerator m_obstaclegen;
    private Vector2 m_move = new Vector2(0f,0f);
    private int m_deadcounter = 0;
    private RoadGenerator m_roadgen;
    private float m_distancetotarget;
    private float m_prevdistancetotarget;
    

    public override void Initialize()
    {
        m_roadgen = road.GetComponent<RoadGenerator>();
        m_cc = this.transform.GetComponent<CarController>();
        m_carrigidbody = this.transform.GetComponent<Rigidbody>();
        m_wheelcols = this.transform.GetComponentsInChildren<WheelCollider>();
        m_obstaclegen = obstacles.GetComponent<ObstacleGenerator>();
       
    }

    
    public override void OnEpisodeBegin()
    {
        RoadAndObstacleReset();
        ResetAll();
        PrivateVariableReset();
    }

    private void RoadAndObstacleReset()
    {
        m_roadgen.GenTrack();
        m_obstaclegen.obstacleState = UnityEngine.Random.Range(0, 3);
        m_obstaclegen.ObstacleStateChange();
        m_obstaclegen.obstacleSpeed = UnityEngine.Random.Range(0.1f, 2f);
    }

    private void PrivateVariableReset()
    {
        m_steps = 0;
        m_cc.AgentMove( Vector2Int.zero);
        m_cuurentReward = 0;
        m_obstacleHit = 0;
        m_deadcounter = 0;
        m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
        m_prevdistancetotarget = m_distancetotarget;
    }

    private void ResetAll()
    {
        //Resets checkpoints
        m_nextcheckpoint = m_roadgen.waypoints[0];
        m_nextcheckpointnumber = 1;

        //Resets agent velocities
        m_carrigidbody.velocity = Vector3.zero;
        m_carrigidbody.angularVelocity = Vector3.zero;

        //Stops movement of all wheel colliders
        foreach (WheelCollider tempcol in m_wheelcols)
        {
            tempcol.brakeTorque = Mathf.Infinity;
        }

        //Assigns random start point for agent with random rotation
        transform.localPosition = ((m_roadgen.vertices[1] + m_roadgen.vertices[0] + m_roadgen.vertices[2] + m_roadgen.vertices[3]) / 4f);
        transform.localRotation = Quaternion.LookRotation((((m_nextcheckpoint.localPosition) - new Vector3(0, m_roadgen.roadOffset, 0)) - transform.localPosition).normalized, Vector3.up);
        transform.localPosition += new Vector3(0, 0.8f, 0);
        transform.localPosition += transform.right * UnityEngine.Random.Range((-m_roadgen.roadOffset + 1.5f), (m_roadgen.roadOffset - 1.5f));
        transform.Rotate(new Vector3(0f, UnityEngine.Random.Range(-60f, 60f), 0f));

        //Assigns starting checkpoint
        m_checkpointpos = (new Vector3(m_nextcheckpoint.localPosition.x, transform.localPosition.y, m_nextcheckpoint.localPosition.z));
        m_prevcheckpointpos = transform.localPosition;

        //Assigns target position
        target.transform.localPosition = ((m_roadgen.vertices[m_roadgen.vertices.Count - 1] + m_roadgen.vertices[m_roadgen.vertices.Count - 2] + m_roadgen.vertices[m_roadgen.vertices.Count - 3] + m_roadgen.vertices[m_roadgen.vertices.Count - 4]) / 4f);
        target.transform.localPosition += new Vector3(0, 0.5f, 0);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        m_move.x = Mathf.Clamp(actions.ContinuousActions[0],-1f,1f);
        m_move.y = Mathf.Clamp(actions.ContinuousActions[1],-1f,1f);
        m_cc.AgentMove(m_move);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var _actionsOut = actionsOut.ContinuousActions;
        _actionsOut[0]= m_move.x;
        _actionsOut[1] = m_move.y;
        m_cc.AgentMove(m_move);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        //Calculate the direction to incoming checkpoint
        m_dirToTarget = (m_checkpointpos - transform.localPosition).normalized;
        // Agent velocity
        sensor.AddObservation(
           (m_carrigidbody.velocity).normalized); // vec 3
        //Position of incoming checkpoint
        sensor.AddObservation(m_checkpointpos); // vec 3
        // Direction to next checkpoint in agent frame
        sensor.AddObservation(m_dirToTarget); // vec 3
        // Distance of agent from middle of road
        sensor.AddObservation(m_laneOffset); // float
        // Agent local rotation
        sensor.AddObservation(transform.localRotation.normalized); // quaternion
    }

    private void OnCollisionStay(Collision collision)
    {
        //Adds negative reward if agent tries to move by pushing the obstacle
        if (collision.collider.CompareTag("Obstacle"))
        {
            m_cuurentReward += -0.3f;
            AddReward(-0.3f);
        }
    }

    /// <summary>
    /// Checks for collision with obstacles or deadzone i.e. agent fell off the track
    /// </summary>
    private void OnCollisionEnter (Collision other)
    {
        if (other.collider.CompareTag("Obstacle"))
        {
            m_cuurentReward += -2 * (m_obstacleHit + 1);
            AddReward(-2 * (m_obstacleHit+1));
        }
        if (other.collider.CompareTag("DeadZone"))
        {
            NextEpisode(-2f);
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
            m_cuurentReward += 5f + ((200f * m_nextcheckpointnumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
            AddReward(5f + ((200f * m_nextcheckpointnumber) / Mathf.Clamp(m_steps,1,Mathf.Infinity)));
            EndEpisode();
           
        }
        else if(other.CompareTag("Waypoint"))
        {
            //If agent collided with the right checkpoint
            if (other.gameObject.transform == m_nextcheckpoint)
            {
                //Debug.Log("Good Checkpoint " + m_nextcheckpointnumber);
                m_cuurentReward += (1f + ((50f * m_nextcheckpointnumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                AddReward(1f + ((50f* m_nextcheckpointnumber) / Mathf.Clamp(m_steps, 1, Mathf.Infinity)));
                NewCheckpoint();
            }
            //If agent collided with wrong checkpoint
            else
            {
                m_cuurentReward += (-1f);
                AddReward(-1f );
               //Debug.Log("Bad Checkpoint");
            }
        }
    }

    private void NewCheckpoint()
    {
        m_prevcheckpointpos = m_checkpointpos;
        //If there are more checkpoints to be crossed then assign the incoming checkpoint
        if (m_roadgen.waypoints.Count > m_nextcheckpointnumber)
        {
            m_nextcheckpoint = m_roadgen.waypoints[m_nextcheckpointnumber];
            m_checkpointpos = (new Vector3(m_nextcheckpoint.localPosition.x, transform.localPosition.y, m_nextcheckpoint.localPosition.z));
            m_nextcheckpointnumber++;
            m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
            m_prevdistancetotarget = m_distancetotarget;
        }
        //If there are no checkpoints left then assign the final target
        else
        {
            m_nextcheckpoint = target.transform;
            m_checkpointpos = (new Vector3(m_nextcheckpoint.localPosition.x, transform.localPosition.y, m_nextcheckpoint.localPosition.z));
            m_nextcheckpointnumber += 10;
            m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
            m_prevdistancetotarget = m_distancetotarget;
        }
    }
    private void Update()
    {
      /*
        Debug.Log("Distance to target = " + m_distancetotarget);
        Debug.Log("All grounded = " + m_allgrounded);
        Debug.Log("Local Rotation  = " + transform.localRotation);
      */

        Debug.Log("Current Reward = " + m_cuurentReward);
        Debug.Log("Obstacle Hit = " + m_obstacleHit);
        Debug.Log("Lane Offset = " + m_laneOffset);

        //Line showing direction to incoming checking
        Debug.DrawLine(transform.position, transform.position + m_dirToTarget * 10f, Color.red);
        //Line showing agent's forward direction
        Debug.DrawLine(transform.position, transform.position + 10f * transform.forward, Color.blue);
    }
    private void FixedUpdate()
    {
        m_steps++;
        m_allgrounded = true;
        CalcLaneOffset();
        CheckGrounded();
        CheckMovement();
    }

    private void CheckMovement()
    {
        CheckDistanceFromCheckpoint();
        if (m_allgrounded && m_carrigidbody.velocity.magnitude > 3f)
        {

            //Reward based on how far the agent is from the middle of road
            m_cuurentReward += (1 / (800f * m_laneOffset * m_laneOffset * m_laneOffset));
            AddReward(1 / (800f * m_laneOffset * m_laneOffset * m_laneOffset));
      
            //Reward based on the direction of movement and speed of agent
            m_cuurentReward += (0.0005f * ((Vector3.Dot(m_carrigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f));
            AddReward((0.0005f * ((Vector3.Dot(m_carrigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f)));

        }


      //If agent's speed is too low and its off the track for a certain amount of time then end the episode
        if (m_carrigidbody.velocity.magnitude < 1f && !m_allgrounded)
        {
                m_deadcounter++;
        }
        if (m_carrigidbody.velocity.magnitude > 1f && m_allgrounded)
        { 
            m_deadcounter = 0;
        }
        if (m_deadcounter >= 500)
        {
            NextEpisode(-2f);
        }
    }


    /// <summary>
    /// Calculates distance of incoming checkpoint from agent and gives a reward based on the progress of the agent towards the checkpoint
    /// </summary>
    private void CheckDistanceFromCheckpoint()
    {
        m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
        if (m_steps % 50 == 0)
        {
            m_cuurentReward += (m_prevdistancetotarget - m_distancetotarget) / 40f;
            AddReward((m_prevdistancetotarget - m_distancetotarget) / 40f);
            m_prevdistancetotarget = m_distancetotarget;
        }
    }

    /// <summary>
    /// Checks if all wheel colliders are grounded
    /// </summary>
    private void CheckGrounded()
    {
        foreach (WheelCollider tempcol in m_wheelcols)
        {
            if (!tempcol.isGrounded)
            {
                m_allgrounded = false;
                break;
            }
        }
    }

    /// <summary>
    /// Calculates the shortest distance of agent from the line connecting previous checkpoint to next checkpoint
    /// </summary>
    private void CalcLaneOffset()
    {
        m_prevtonextcheckpoint = m_checkpointpos - m_prevcheckpointpos;
        m_prevtonextcheckpoint = Vector3.ProjectOnPlane(m_prevtonextcheckpoint, Vector3.up);
        m_prevtocar = transform.localPosition - m_prevcheckpointpos;
        m_prevtocar = Vector3.ProjectOnPlane(m_prevtocar, Vector3.up);
        m_closestpoint = Vector3.Project(m_prevtocar, m_prevtonextcheckpoint);
        m_closestpoint = m_prevcheckpointpos + m_closestpoint;
        m_laneOffset = Mathf.Clamp(Vector3.Distance(new Vector3(transform.localPosition.x, 0, transform.localPosition.z), m_closestpoint), 0.2f, 10f);
    }
}
