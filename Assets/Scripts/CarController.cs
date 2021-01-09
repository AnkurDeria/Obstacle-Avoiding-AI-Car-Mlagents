
using UnityEngine;
using UnityEngine.InputSystem;

public class CarController : MonoBehaviour
{
	/// <summary>
	/// private variables
	/// </summary>
	
	[Range(0.01f,1f)] [SerializeField] private float m_turnSensitivity;

	[SerializeField] private float m_downForceValue = 50;
	[SerializeField] private float m_maxSteerAngle = 35f;
	[SerializeField] private float m_motorForce = 500f;
	[SerializeField] private float m_brakeForce = 100f;
	[SerializeField] private Transform m_frontLeftT;
	[SerializeField] private Transform m_frontRightT;
	[SerializeField] private Transform m_backLeftT;
	[SerializeField] private Transform m_backRightT;
	[SerializeField] private Vector2 m_movement = new Vector2(0f, 0f);
	[SerializeField] private Vector3 m_centerOfMass = new Vector3(0.0f, -0.05f, 0.0f);
	[SerializeField] private WheelCollider m_frontLeft;
	[SerializeField] private WheelCollider m_frontRight;
	[SerializeField] private WheelCollider m_backLeft;
	[SerializeField] private WheelCollider m_backRight;

	private float m_turnRadius;
	private float m_curSpeed;
	private float m_torque = 0.0f;
	private float m_wheelBase;
	private float m_axleLength;
	private Rigidbody m_rigidbody;

	void Start()
    {
		m_rigidbody = GetComponentInChildren<Rigidbody>();
		m_rigidbody.centerOfMass = m_centerOfMass;
		m_wheelBase = Vector3.Distance(m_frontLeftT.localPosition,m_backLeftT.localPosition);
		m_axleLength = Vector3.Distance(m_frontLeftT.localPosition, m_frontRightT.localPosition);
    }

	public float GetSteeringAngle()
    {
		return ((m_frontRight.steerAngle +m_frontLeft.steerAngle) / 2f) / 55f;
	}

	public float GetTorque()
	{
		return (m_frontRight.motorTorque / 700f);
	}

	public void Move(InputAction.CallbackContext context)
    {
		// Input from new input system
		m_movement = context.ReadValue<Vector2>();
    }

    public void AgentMove(Vector2 _move)
    {
		// Input from agent
		m_movement = _move;
    }

	/// <summary>
	/// Ackerman steering
	/// </summary>
	private void Steer()
	{
		m_turnRadius = m_maxSteerAngle * m_movement.x;
		m_turnRadius = m_axleLength / Mathf.Tan(m_turnRadius * Mathf.Deg2Rad);

		if (m_movement.x > 0)
		{
			m_frontLeft.steerAngle = Mathf.Lerp(m_frontLeft.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (m_turnRadius + (m_axleLength / 2f))) * m_movement.x, m_turnSensitivity);
			m_frontRight.steerAngle = Mathf.Lerp(m_frontRight.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (m_turnRadius - (m_axleLength / 2f))) * m_movement.x, m_turnSensitivity);
		}

		else if (m_movement.x < 0)
		{
			m_frontRight.steerAngle = Mathf.Lerp(m_frontRight.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (-m_turnRadius + (m_axleLength / 2f))) * m_movement.x, m_turnSensitivity);
			m_frontLeft.steerAngle = Mathf.Lerp(m_frontLeft.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (-m_turnRadius - (m_axleLength / 2f))) * m_movement.x, m_turnSensitivity);
		}

		else
		{
			m_frontLeft.steerAngle = Mathf.Lerp(m_frontLeft.steerAngle, 0, m_turnSensitivity);
			m_frontRight.steerAngle = Mathf.Lerp(m_frontRight.steerAngle, 0, m_turnSensitivity);
		}
	}

	private void Accelerate()
	{
		m_curSpeed = m_rigidbody.velocity.magnitude;
        //Debug.Log("Current speed = " + m_curSpeed);

        if (m_curSpeed < 17f)
        {
            m_backLeft.brakeTorque = 0;
            m_backRight.brakeTorque = 0;
            m_frontLeft.brakeTorque = 0;
            m_frontRight.brakeTorque = 0;
            m_torque = m_movement.y * m_motorForce * 500 * Time.fixedDeltaTime;
            m_backLeft.motorTorque = m_torque;
            m_backRight.motorTorque = m_torque;
            m_frontLeft.motorTorque = m_torque;
            m_frontRight.motorTorque = m_torque;
        }

        // If speed exceeds 17 apply brake
        else
		{
			m_backLeft.brakeTorque = m_brakeForce;
			m_backRight.brakeTorque = m_brakeForce;
			m_frontLeft.brakeTorque = m_brakeForce;
			m_frontRight.brakeTorque = m_brakeForce;
		}
	}

	private void UpdateWheelPoses()
	{
		UpdateWheelPose(m_frontLeft, m_frontLeftT);
		UpdateWheelPose(m_frontRight, m_frontRightT);
		UpdateWheelPose(m_backLeft, m_backLeftT);
		UpdateWheelPose(m_backRight, m_backRightT);
	}

	private void UpdateWheelPose(WheelCollider _collider, Transform _transform)
    {
        Vector3 _pos;
		Quaternion _quat;

		_collider.GetWorldPose(out _pos, out _quat);

		_transform.position = _pos;
		_transform.rotation = _quat;
	}
	
	private void FixedUpdate()
	{
		AddDownforce();
		UpdateWheelPoses();
		Accelerate();
		Steer();
        
    }

    private void AddDownforce()
    {
		m_rigidbody.AddForce(-transform.up * m_downForceValue * m_rigidbody.velocity.magnitude);
    }
}
