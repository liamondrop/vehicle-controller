using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VehicleControl
{
    [RequireComponent(typeof(Rigidbody))]
    public class VehicleController : MonoBehaviour
    {
        public float maxSteeringAngle = 25;
        public float maxBrakeTorque = 20000;
        public float maxMotorTorque = 2500;
        [SerializeField] float m_topSpeed = 100;
        public float slipLimit = 0.4f;
        [SerializeField] float m_downForce = 100;
        [Range(0, 1)] [SerializeField] float m_steerHelper = 0.774f; // 0 is raw physics, 1 the car will grip in the direction it is facing
        [Range(0, 1)] [SerializeField] float m_tractionControl = 1; // 0 is no traction control, 1 is full interference
        public AxleInfo[] axleInfos;
        public WheelInfo wheelInfo;

        private Rigidbody m_rbody;
        private float m_steering = 0;
        private float m_throttle = 0;
        private float m_brake = 0;
        private float m_currentMotorTorque;
        private float m_motorWheelCount = 0;
        private float m_lastHeading;

        public float GetSteering(bool normalized = true)
        {
            var value = axleInfos[0].GetSteeringAngle();
            return normalized ? value / maxSteeringAngle : value;
        }
        
        public void SetSteering(float value, bool normalized = true)
        {
            if (!normalized)
                value /= maxSteeringAngle;
            m_steering = Mathf.Clamp(value, -1, 1);
        }

        public float GetThrottle()
        {
            return m_throttle;
        }

        public void SetThrottle(float value)
        {
            m_throttle = Mathf.Clamp(value, 0, 1);
        }

        public float GetBrake()
        {
            return m_brake;
        }

        public void SetBrake(float value)
        {
            m_brake = Mathf.Clamp(value, 0, 1);
        }

        public float GetSteerAngle(bool normalized = false)
        {
            var steerAngle = axleInfos[0].GetSteeringAngle();
            return normalized ? steerAngle / maxSteeringAngle : steerAngle;
        }

        public float GetCurrentSpeed(bool normalized = false)
        {
            var speed = m_rbody.velocity.magnitude;
            return normalized ? speed / m_topSpeed : speed;
        }

        void Awake()
        {
            m_rbody = GetComponent<Rigidbody>();
            m_currentMotorTorque = maxMotorTorque - (m_tractionControl * maxMotorTorque);

            foreach (AxleInfo axleInfo in axleInfos)
            {
                CreateWheelCollider(axleInfo.leftWheel);
                CreateWheelCollider(axleInfo.rightWheel);
                if (axleInfo.motor)
                {
                    m_motorWheelCount += 2;
                }
            }
        }

        public void FixedUpdate()
        {
            float steerAngle = maxSteeringAngle * m_steering;
            float motorTorque = (m_currentMotorTorque / m_motorWheelCount) * m_throttle;
            float brakeTorque = maxBrakeTorque * m_brake;

            WheelHit wheelHit;
            foreach (var axleInfo in axleInfos)
            {
                var wcLeft = axleInfo.leftWheel.GetComponent<WheelCollider>();
                var wcRight = axleInfo.rightWheel.GetComponent<WheelCollider>();

                if (axleInfo.steering)
                {
                    wcLeft.steerAngle = steerAngle;
                    wcRight.steerAngle = steerAngle;
                }
                if (axleInfo.motor)
                {
                    wcLeft.motorTorque = motorTorque;
                    wcLeft.GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip);
                    wcRight.motorTorque = motorTorque;
                    wcRight.GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip);
                }
                if (axleInfo.brake)
                {
                    wcLeft.brakeTorque = brakeTorque;
                    wcRight.brakeTorque = brakeTorque;
                }
                ApplyLocalPositionToVisuals(axleInfo.leftWheel);
                ApplyLocalPositionToVisuals(axleInfo.rightWheel);
            }

            ClampSpeed();
            AdjustDownwardForce();
            AdjustSteering();
        }

        void ClampSpeed()
        {
            if (m_rbody.velocity.magnitude > m_topSpeed)
            {
                m_rbody.velocity = m_topSpeed * m_rbody.velocity.normalized;
            }
        }

        void AdjustTorque(float forwardSlip)
        {
            if (forwardSlip > slipLimit && m_currentMotorTorque >= 0)
            {
                m_currentMotorTorque -= 10 * m_tractionControl;
            }
            else
            {
                m_currentMotorTorque += 10 * m_tractionControl;
                if (m_currentMotorTorque > maxMotorTorque)
                {
                    m_currentMotorTorque = maxMotorTorque;
                }
            }
        }

        void AdjustDownwardForce()
        {
            m_rbody.AddForce(
                -this.transform.up * m_downForce * m_rbody.velocity.magnitude);
        }

        void AdjustSteering()
        {
            WheelHit wheelHit;
            bool WheelTouching(GameObject wheel) =>
                wheel.GetComponent<WheelCollider>().GetGroundHit(out wheelHit);

            foreach (var axleInfo in axleInfos)
            {
                // wheels arent on the ground so dont realign the rigidbody velocity
                if (!WheelTouching(axleInfo.leftWheel) ||
                    !WheelTouching(axleInfo.rightWheel))
                    return;
            }

            // this is needed to avoid gimbal lock problems
            // that will make the car suddenly shift direction
            var delta_heading = this.transform.eulerAngles.y - m_lastHeading;
            if (Mathf.Abs(delta_heading) < 10)
            {
                var turnadjust = delta_heading * m_steerHelper;
                var velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
                m_rbody.velocity = velRotation * m_rbody.velocity;
            }
            m_lastHeading = transform.eulerAngles.y;
        }

        // finds the corresponding visual wheel
        // correctly applies the transform
        public void ApplyLocalPositionToVisuals(GameObject wheel)
        {
            if (wheel.transform.GetChild(0) == null)
                return;

            Vector3 position;
            Quaternion rotation;
            wheel.GetComponent<WheelCollider>()
                 .GetWorldPose(out position, out rotation);

            wheel.transform.GetChild(0).position = position;
            wheel.transform.GetChild(0).rotation = rotation;
        }

        void CreateWheelCollider(GameObject wheel)
        {
            // if wheel collider already exists, exit
            if (wheel.GetComponent<WheelCollider>())
            {
                return;
            }

            WheelCollider wc = wheel.AddComponent<WheelCollider>();
            wc.mass = wheelInfo.wheelMass;
            wc.radius = wheelInfo.wheelRadius;
            wc.wheelDampingRate = wheelInfo.wheelDampingRate;
            wc.suspensionDistance = wheelInfo.suspensionDistance;

            WheelFrictionCurve fwdFriction = new WheelFrictionCurve();
            fwdFriction.extremumSlip = wheelInfo.forwardExtremumSlip;
            fwdFriction.extremumValue = wheelInfo.forwardExtremumValue;
            fwdFriction.asymptoteSlip = wheelInfo.forwardAsymptoteSlip;
            fwdFriction.asymptoteValue = wheelInfo.forwardAsymptoteValue;
            fwdFriction.stiffness = wheelInfo.forwardStiffness;
            wc.forwardFriction = fwdFriction;

            WheelFrictionCurve sideFriction = new WheelFrictionCurve();
            sideFriction.extremumSlip = wheelInfo.sidewaysExtremumSlip;
            sideFriction.extremumValue = wheelInfo.sidewaysExtremumValue;
            sideFriction.asymptoteSlip = wheelInfo.sidewaysAsymptoteSlip;
            sideFriction.asymptoteValue = wheelInfo.sidewaysAsymptoteValue;
            sideFriction.stiffness = wheelInfo.sidewaysStiffness;
            wc.sidewaysFriction = sideFriction;

            JointSpring suspSpring = new JointSpring();
            suspSpring.spring = wheelInfo.suspensionSpring;
            suspSpring.damper = wheelInfo.suspensionDamper;
            suspSpring.targetPosition = wheelInfo.suspensionTargetPos;
            wc.suspensionSpring = suspSpring;
        }
    }

    [System.Serializable]
    public class AxleInfo
    {
        public GameObject leftWheel;
        public GameObject rightWheel;
        public bool motor;
        public bool steering;
        public bool brake;

        public float GetSteeringAngle()
        {
            return 0.5f * (WCLeft.steerAngle + WCRight.steerAngle);
        }

        public WheelCollider WCLeft
        {
            get => leftWheel.GetComponent<WheelCollider>();
        }

        public WheelCollider WCRight
        {
            get => rightWheel.GetComponent<WheelCollider>();
        }
    }

    [System.Serializable]
    public class WheelInfo
    {
        public float wheelMass = 20;
        public float wheelRadius = 0.5f;
        public float wheelDampingRate = 0.25f;
        public float suspensionDistance = 0.2f;
        [Header("Suspension Spring")]
        public float suspensionSpring = 35000f;
        public float suspensionDamper = 4500f;
        public float suspensionTargetPos = 0.1f;
        [Header("Forward Friction")]
        public float forwardExtremumSlip = 0.4f;
        public float forwardExtremumValue = 1f;
        public float forwardAsymptoteSlip = 0.8f;
        public float forwardAsymptoteValue = 0.5f;
        public float forwardStiffness = 1f;
        [Header("Sideways Friction")]
        public float sidewaysExtremumSlip = 0.2f;
        public float sidewaysExtremumValue = 1f;
        public float sidewaysAsymptoteSlip = 0.5f;
        public float sidewaysAsymptoteValue = 0.75f;
        public float sidewaysStiffness = 1f;
    }
}
