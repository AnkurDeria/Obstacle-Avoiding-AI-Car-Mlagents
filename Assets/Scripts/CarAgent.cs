using System;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using UnityEngine.InputSystem;

public class CarAgent : Agent
{

    public Transform target;
    //public Vector3[] startpositions;
    public GameObject road;


    private CarController m_cc;
    private Rigidbody m_carrigidbody;
    private WheelCollider[] m_wheelcols;
    private float m_rewardfactor = 1;
    private bool m_allgrounded = false;
    private Vector3 m_prevtonextcheckpoint;
    private Vector3 m_prevtocar;
    private Vector3 m_closestpoint;
    private float m_laneOffset;
    private int m_steps;
    private float m_rewardPerEpisode;
    private int m_obstacleHit;
    private Vector3 m_velocity = Vector3.zero;
    private Vector3 m_dirToTarget;
    private Transform m_nextcheckpoint;
    private Vector3 m_prevcheckpointpos;
    private Vector3 m_checkpointpos;
    //private bool m_resettrack = true;
    private int m_nextcheckpointnumber;
    // private int m_nnoutvertical = 0, m_nnouthorizontal = 0;
    private Vector2 m_move;
    private int m_deadcounter = 0;
    //private int trigger = 0;
    private RoadGenerator m_roadgen;
    private float m_distancetotarget;
    private float m_prevdistancetotarget;
    //private bool m_frozen = true;

   
    //public Vector3 Initial_Position = new Vector3(113.85f, 0.9f, -217.65f);
    public override void Initialize()
    {
        m_roadgen = road.GetComponent<RoadGenerator>();
        m_cc = this.transform.GetComponent<CarController>();
        m_carrigidbody = this.transform.GetComponent<Rigidbody>();
        m_wheelcols = this.transform.GetComponentsInChildren<WheelCollider>();
        //m_roadgen.GenTrack();
        //transform.position = ((m_roadgen.m_vertices[1] - m_roadgen.m_vertices[0]) / 2f);
    }

    public override void OnEpisodeBegin()
    {
        //Debug.Log("On episode begin called");


        //m_distancetotarget = 0f;
        m_steps = 0;
        m_cc.movement = Vector2.zero;

        m_rewardPerEpisode = 0;
        m_obstacleHit = 0;

        m_roadgen.GenTrack();
        ResetPositions();

        m_deadcounter = 0;
        m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
        m_prevdistancetotarget = m_distancetotarget;
        //m_carrigidbody.isKinematic = true;
    }

    private void OnCollisionStay(Collision collision)
    {
        if (collision.collider.CompareTag("Obstacle"))
        {
            m_rewardPerEpisode += -1.5f * Time.deltaTime;
            AddReward(-1.5f * Time.deltaTime);
        }
    }
    private void ResetPositions()
    {
        m_nextcheckpoint = m_roadgen.waypoints[0];
        m_nextcheckpointnumber = 1;
        m_carrigidbody.velocity = Vector3.zero;
        m_carrigidbody.angularVelocity = Vector3.zero;
        foreach (WheelCollider tempcol in m_wheelcols)
        {
            tempcol.brakeTorque = Mathf.Infinity;
            // tempcol.brakeTorque = 0;
        }
        transform.localPosition = ((m_roadgen.vertices[1] + m_roadgen.vertices[0] + m_roadgen.vertices[2] + m_roadgen.vertices[3]) / 4f);

        transform.localRotation = Quaternion.LookRotation((((m_nextcheckpoint.localPosition) - new Vector3(0, m_roadgen.roadOffset, 0)) - transform.localPosition).normalized, Vector3.up);
        transform.localPosition += new Vector3(0, 0.8f, 0);
        transform.localPosition += transform.right * UnityEngine.Random.Range((-m_roadgen.roadOffset + 1.5f), (m_roadgen.roadOffset - 1.5f));
        transform.Rotate(new Vector3(0f, UnityEngine.Random.Range(-60f, 60f), 0f));
        m_checkpointpos = (new Vector3(m_nextcheckpoint.localPosition.x, transform.localPosition.y, m_nextcheckpoint.localPosition.z));
        m_prevcheckpointpos = transform.localPosition;
        target.transform.localPosition = ((m_roadgen.vertices[m_roadgen.vertices.Count - 1] + m_roadgen.vertices[m_roadgen.vertices.Count - 2] + m_roadgen.vertices[m_roadgen.vertices.Count - 3] + m_roadgen.vertices[m_roadgen.vertices.Count - 4]) / 4f);
        target.transform.localPosition += new Vector3(0, 0.5f, 0);
    }

    public override void OnActionReceived(float[] vectorAction)
    {

        //    if (!m_frozen)
        //    {
        //Debug.Log("TRIGGER VALUE =" + trigger);
        

        //m_distancetotarget = Vector3.Distance(target.position, this.transform.position);
        //Debug.Log("Distance to target = " + m_distancetotarget);
        if (vectorAction[0] == 1) m_move.y = -1;
        if (vectorAction[0] == 2) m_move.y = 1;
        if (vectorAction[1] == 1) m_move.x = -1;
        if (vectorAction[1] == 2) m_move.x = 1;
        //m_move.x = Mathf.Clamp(vectorAction[0], -1, 1);
        //m_move.y = Mathf.Clamp(vectorAction[1], -1, 1);
        m_cc.AgentMove(m_move);
        //

        //m_rewardfactor = Mathf.Clamp(m_distancetotarget / 10f, 0.1f, Mathf.Infinity);
        //m_rewardfactor = 1 / MaxStep;
        
      
        

    }
    public override void Heuristic(float[] actionsOut)
    {
       
            actionsOut[0] = m_cc.movement[0];
            actionsOut[1] = m_cc.movement[1];
        
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // Postion of next checkpoint in agent frame

        m_dirToTarget = (m_checkpointpos - transform.localPosition).normalized;
        // Agent velocity in agent frame
        sensor.AddObservation(
           (m_carrigidbody.velocity).normalized); // vec 3
        sensor.AddObservation(m_checkpointpos); // vec 3
        // Direction to next checkpoint in agent frame
        sensor.AddObservation(m_dirToTarget); // vec 3
        sensor.AddObservation(m_laneOffset); // float
        sensor.AddObservation(transform.localRotation.normalized); // quaternion
        //float velocityAlignment = Vector3.Dot(m_dirToTarget, m_carrigidbody.velocity);
        //AddReward(m_rewardfactor * velocityAlignment);
        //sensor.AddObservation(this.transform.rotation);
        //sensor.AddObservation(transform.forward);
        //sensor.AddObservation((target.position - transform.position));
        //sensor.AddObservation(m_carrigidbody.velocity.magnitude);
        //sensor.AddObservation(target.position);
        //sensor.AddObservation(transform.position);
        //sensor.AddObservation((m_carrigidbody.angularVelocity).normalized); //vec 3
        

    }
    private void OnCollisionEnter (Collision other)
    {
        if (other.collider.CompareTag("Obstacle"))
        {
            m_obstacleHit++;
            //Debug.Log("TRIGGERED Obstacle");
            m_rewardPerEpisode += -1f;
            AddReward(-3f);
        }
        if (other.collider.CompareTag("DeadZone"))
            {
            //Debug.Log("TRIGGERED Deadzone" + ++trigger);
            // m_rewardperepisode += -1f;
            EpisodeReset();

            }
    }

    private void EpisodeReset()
    {
        AddReward(-2f);
        //m_resettrack = true;
        EndEpisode();
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Finish"))
        {
            m_rewardPerEpisode += 1f * (20f / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
            AddReward(1f * (20f/Mathf.Clamp(m_steps,1,Mathf.Infinity)));
           // m_resettrack = true;
            EndEpisode();
           
        }
        else if(other.CompareTag("Waypoint"))
        {
            if (other.gameObject.transform == m_nextcheckpoint)
            {
                //Debug.Log("Good Checkpoint " + m_nextcheckpointnumber);
                m_rewardPerEpisode += (100f / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
                AddReward(100f / Mathf.Clamp(m_steps, 1, Mathf.Infinity));
                NewCheckpoint();
            }
            else
            {
                m_rewardPerEpisode += (-1f);
                AddReward(-1f );
               // Debug.Log("Bad Checkpoint ");
            }
        }
    }

    private void NewCheckpoint()
    {
        m_prevcheckpointpos = m_checkpointpos;
        if (m_roadgen.waypoints.Count > m_nextcheckpointnumber)
        {
            m_nextcheckpoint = m_roadgen.waypoints[m_nextcheckpointnumber];
            m_checkpointpos = (new Vector3(m_nextcheckpoint.localPosition.x, transform.localPosition.y, m_nextcheckpoint.localPosition.z));
            m_nextcheckpointnumber++;
            m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
            m_prevdistancetotarget = m_distancetotarget;
        }
        else
        {
            m_nextcheckpoint = target.transform;
            m_checkpointpos = (new Vector3(m_nextcheckpoint.localPosition.x, transform.localPosition.y, m_nextcheckpoint.localPosition.z));
            m_nextcheckpointnumber += 10;
            m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
            m_prevdistancetotarget = m_distancetotarget;
        }
    }


    public static float Sigmoid(float val, float scale = 1f)
    {
        val *= scale;
        return val / (1f + Mathf.Abs(val));
    }

    public static Vector3 Sigmoid(Vector3 v3, float scale = 1f)
    {
        v3.x = Sigmoid(v3.x, scale);
        v3.y = Sigmoid(v3.y, scale);
        v3.z = Sigmoid(v3.z, scale);
        return v3;
    }


    private void Update()
    {

        //Debug.Log("Distance to target = " + m_distancetotarget);
        //Debug.Log("All grounded = " + m_allgrounded);
        //Debug.Log("Lane Offset = " + m_laneoffset);
        Debug.Log("Current Reward = " + m_rewardPerEpisode);
        //Debug.Log("Local Rotation  = " + transform.localRotation);
        Debug.DrawLine(transform.position, transform.position + 10f * transform.forward, Color.blue);
        //Debug.DrawLine(transform.position, transform.position + (m_checkpointpos - m_prevcheckpointpos) * 10f, Color.green);
        Debug.Log("Obstacle Hit = " + m_obstacleHit);
        Debug.DrawLine(transform.position, transform.position + m_dirToTarget * 10f, Color.red);
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
            //AddReward(0.001f * (m_carrigidbody.velocity.magnitude/100f) );
            //Debug.Log("ALL TIRES GROUNDED");
            m_rewardPerEpisode += (1 / (1000f * m_laneOffset * m_laneOffset * m_laneOffset));
            AddReward(1 / (1000f * m_laneOffset * m_laneOffset * m_laneOffset));
            //
            m_rewardPerEpisode += (0.0005f * ((Vector3.Dot(m_carrigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f) * m_carrigidbody.velocity.magnitude);

            AddReward((0.0005f * ((Vector3.Dot(m_carrigidbody.velocity.normalized, transform.forward) / 2f) + 0.5f) * m_carrigidbody.velocity.magnitude));

        }

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
            EpisodeReset();
        }
    }

    private void CheckDistanceFromCheckpoint()
    {
        m_distancetotarget = Vector3.Distance(m_checkpointpos, transform.localPosition);
        if (m_steps % 50 == 0)
        {
            m_rewardPerEpisode += (m_prevdistancetotarget - m_distancetotarget) / 40f;
            AddReward((m_prevdistancetotarget - m_distancetotarget) / 40f);
            m_prevdistancetotarget = m_distancetotarget;
        }
    }

    private void CheckGrounded()
    {
        foreach (WheelCollider tempcol in m_wheelcols)
        {
            if (!tempcol.isGrounded)
            {

                //Debug.Log("A wheel is not grounded");
                m_allgrounded = false;
                break;
            }
        }
    }

    private void CalcLaneOffset()
    {
        m_prevtonextcheckpoint = m_checkpointpos - m_prevcheckpointpos;
        m_prevtonextcheckpoint = Vector3.ProjectOnPlane(m_prevtonextcheckpoint, Vector3.up);
        m_prevtocar = transform.localPosition - m_prevcheckpointpos;
        m_prevtocar = Vector3.ProjectOnPlane(m_prevtocar, Vector3.up);
        m_closestpoint = Vector3.Project(m_prevtocar, m_prevtonextcheckpoint);
        m_closestpoint = m_prevcheckpointpos + m_closestpoint;
        //m_closestpoint = new Vector3(m_closestpoint.x, transform.position.y, m_closestpoint.z);
        m_laneOffset = Mathf.Clamp(Vector3.Distance(new Vector3(transform.localPosition.x, 0, transform.localPosition.z), m_closestpoint), 0.4f, 5f);
    }
}
