
using UnityEngine;
using UnityEngine.InputSystem;

public class CarController : MonoBehaviour
{
	/// <summary>
	/// public variables
	/// </summary>

	[Range(0.01f,1f)] public float turnSensitivity;

	public float downForceValue = 50;
	public float maxSteerAngle = 35f;
	public float motorForce = 500f;
	public float brakeForce = 100f;
	public Transform frontLeftT;
	public Transform frontRightT;
	public Transform backLeftT;
	public Transform backRightT;
	public Vector3 centerOfMass = new Vector3(0.0f, -0.05f, 0.0f);
	public WheelCollider frontLeft;
	public WheelCollider frontRight;
	public WheelCollider backLeft;
	public WheelCollider backRight;


	/// <summary>
	/// private variables
	/// </summary>

	[SerializeField] private Vector2 m_movement = new Vector2(0f, 0f);

	private float m_turnRadius;
	private float m_curSpeed;
	private float m_torque = 0.0f;
	private float m_wheelBase;
	private float m_axleLength;
	private Rigidbody m_rigidbody;

	void Start()
    {
		m_rigidbody = GetComponent<Rigidbody>();
		m_rigidbody.centerOfMass = centerOfMass;
		m_wheelBase = Vector3.Distance(frontLeftT.localPosition,backLeftT.localPosition);
		m_axleLength = Vector3.Distance(frontLeftT.localPosition, frontRightT.localPosition);
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
		m_turnRadius = maxSteerAngle * m_movement.x;
		m_turnRadius = m_axleLength / Mathf.Tan(m_turnRadius * Mathf.Deg2Rad);

		if (m_movement.x > 0)
		{
			frontLeft.steerAngle = Mathf.Lerp(frontLeft.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (m_turnRadius + (m_axleLength / 2f))) * m_movement.x, turnSensitivity);
			frontRight.steerAngle = Mathf.Lerp(frontRight.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (m_turnRadius - (m_axleLength / 2f))) * m_movement.x, turnSensitivity);
		}

		else if (m_movement.x < 0)
		{
			frontRight.steerAngle = Mathf.Lerp(frontRight.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (-m_turnRadius + (m_axleLength / 2f))) * m_movement.x, turnSensitivity);
			frontLeft.steerAngle = Mathf.Lerp(frontLeft.steerAngle,Mathf.Rad2Deg * Mathf.Atan(m_wheelBase / (-m_turnRadius - (m_axleLength / 2f))) * m_movement.x, turnSensitivity);
		}

		else
		{
			frontLeft.steerAngle = Mathf.Lerp(frontLeft.steerAngle, 0, turnSensitivity);
			frontRight.steerAngle = Mathf.Lerp(frontRight.steerAngle, 0, turnSensitivity);
		}
	}

	private void Accelerate()
	{
		m_curSpeed = m_rigidbody.velocity.magnitude;
        //Debug.Log("Current speed = " + m_curSpeed);

        if (m_curSpeed < 17f)
        {
            backLeft.brakeTorque = 0;
            backRight.brakeTorque = 0;
            frontLeft.brakeTorque = 0;
            frontRight.brakeTorque = 0;
            m_torque = m_movement.y * motorForce * 500 * Time.fixedDeltaTime;
            backLeft.motorTorque = m_torque;
            backRight.motorTorque = m_torque;
            frontLeft.motorTorque = m_torque;
            frontRight.motorTorque = m_torque;
        }

        // If speed exceeds 30 apply brake
        else
		{
			backLeft.brakeTorque = brakeForce;
			backRight.brakeTorque = brakeForce;
			frontLeft.brakeTorque = brakeForce;
			frontRight.brakeTorque = brakeForce;
		}
	}

	private void UpdateWheelPoses()
	{
		UpdateWheelPose(frontLeft, frontLeftT);
		UpdateWheelPose(frontRight, frontRightT);
		UpdateWheelPose(backLeft, backLeftT);
		UpdateWheelPose(backRight, backRightT);
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
		AddDownforce();
		UpdateWheelPoses();
		Accelerate();
		Steer();
	}

    private void AddDownforce()
    {
		m_rigidbody.AddForce(-transform.up * downForceValue * m_rigidbody.velocity.magnitude);
    }
}
