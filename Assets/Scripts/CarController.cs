using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class CarController : MonoBehaviour
{
	public float Downforcevalue = 50;
	public Vector2 movement = new Vector2(0.0f,0.0f);
	public WheelCollider FL, FR;
	public WheelCollider BL, BR;
	public Transform FLT, FRT;
	public Transform BLT, BRT;
	public int MaxRPM = 350 ;
	public float maxSteerAngle = 35f;
	public float turnSensitivity = 1.0f;
	public float motorForce = 500f;
	public float brakeForce = 100f;
	public Vector3 com = new Vector3(0.0f, -0.05f, 0.0f);

	private float m_turnradius;
	private float m_curspeed;
	private float m_brake;
	private float m_torque = 0.0f;
	private Rigidbody m_rigidbody;
	private float m_wheelbase , m_axlelength;
	void Start()
    {
		
		m_rigidbody = GetComponent<Rigidbody>();
		m_rigidbody.centerOfMass = com;
		m_wheelbase = Vector3.Distance(FLT.localPosition,BLT.localPosition);
		m_axlelength = Vector3.Distance(FLT.localPosition, FRT.localPosition);
	}
	public void Move(InputAction.CallbackContext context)
    {
		 movement = context.ReadValue<Vector2>();
		
    }

    public void AgentMove(Vector2 _move)
    {
		movement = _move;
    }

    private void Steer()
	{
		m_turnradius = maxSteerAngle * movement.x * turnSensitivity;
		m_turnradius = m_axlelength / Mathf.Tan(m_turnradius * Mathf.Deg2Rad);
		if (movement.x > 0)
		{
			FL.steerAngle = Mathf.Lerp(FL.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelbase / (m_turnradius + (m_axlelength / 2f))) * movement.x,0.3f);
			FR.steerAngle = Mathf.Lerp(FR.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelbase / (m_turnradius - (m_axlelength / 2f))) * movement.x,0.3f);
		}
		else if (movement.x < 0)
		{
			FR.steerAngle = Mathf.Lerp(FR.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelbase / (-m_turnradius + (m_axlelength / 2f))) * movement.x,0.3f);
			FL.steerAngle = Mathf.Lerp(FL.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelbase / (-m_turnradius - (m_axlelength / 2f))) * movement.x,0.3f);
		}
		else
		{
			FL.steerAngle = Mathf.Lerp(FL.steerAngle, 0, 0.3f);
			FR.steerAngle = Mathf.Lerp(FR.steerAngle, 0, 0.3f);
		}
	}

	private void Accelerate()
	{
		m_curspeed = m_rigidbody.velocity.magnitude ;
		Debug.Log("Current speed = " + m_curspeed);
		if (m_curspeed< 150f)
		{
				BL.brakeTorque = 0;
				BR.brakeTorque = 0;
				FL.brakeTorque = 0;
				FR.brakeTorque = 0;
				m_torque = movement.y * motorForce * 500 * Time.fixedDeltaTime;
				BL.motorTorque = m_torque;
				BR.motorTorque = m_torque;
				FL.motorTorque = m_torque;
				FR.motorTorque = m_torque;
			

		}
		else
		{
			BL.brakeTorque = brakeForce;
			BR.brakeTorque = brakeForce;
			FL.brakeTorque = brakeForce;
			FR.brakeTorque = brakeForce;
		}
		//Debug.Log("Current torque = " + m_torque);
		
	}

	private void UpdateWheelPoses()
	{
		UpdateWheelPose(FL, FLT);
		UpdateWheelPose(FR, FRT);
		UpdateWheelPose(BL, BLT);
		UpdateWheelPose(BR, BRT);
	}

	private void UpdateWheelPose(WheelCollider _collider, Transform _transform)
	{
		Vector3 _pos = _transform.position;
		Quaternion _quat = _transform.rotation;

		_collider.GetWorldPose(out _pos, out _quat);

		_transform.position = _pos;
		_transform.rotation = _quat;
	}
	
	private void FixedUpdate()
	{
		//Debug.Log("accel =" + movement.y);
		//Debug.Log("turn =" + movement.x);
		AddDownforce();
		UpdateWheelPoses();
		Accelerate();
		Steer();
		
		
		
	}

    private void AddDownforce()
    {
		m_rigidbody.AddForce(-transform.up * Downforcevalue * m_rigidbody.velocity.magnitude);
    }
}
