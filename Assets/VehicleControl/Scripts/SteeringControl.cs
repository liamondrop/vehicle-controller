using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VehicleControl
{
    [RequireComponent(typeof(VehicleController))]
    public class SteeringControl : MonoBehaviour
    {
        [Range(0, 1)] [SerializeField] float m_controlGain = 0.5f;
        [Range(0, 20)] [SerializeField] float m_softening = 10f;
        [SerializeField] Transform m_frontAxleRef; // front axle transform
        [SerializeField] Transform m_waypointRef; // target waypoint transform
        VehicleController m_controller;
        Rigidbody m_rbody;
        const float TWOPI = 2 * Mathf.PI;
        const float TWOPI_RECIP = 1 / TWOPI;

        void Start()
        {
            m_controller = GetComponent<VehicleController>();
            m_rbody = GetComponent<Rigidbody>();
        }

        void FixedUpdate()
        {
            var steering = NormalizeAngle(HeadingError() + AxleError());
            m_controller.SetSteering(Mathf.Rad2Deg * steering, false);
        }

        float HeadingError()
        {
            var error = m_waypointRef.rotation.eulerAngles.y -
                        this.transform.rotation.eulerAngles.y;
            return Mathf.Deg2Rad * error;
        }

        float AxleError()
        {
            var fwdVelocity = Vector3.Dot(this.transform.forward, m_rbody.velocity);
            var vecToTarget = m_frontAxleRef.position - m_waypointRef.position;
            var angleBetween = Vector3.SignedAngle(
                vecToTarget, m_waypointRef.forward, Vector3.up);
            var error = angleBetween < 0 ?
                -vecToTarget.magnitude : vecToTarget.magnitude;
            return Mathf.Atan2(m_controlGain * error, m_softening + fwdVelocity);
        }

        static float NormalizeAngle(float theta)
        {
            return theta - TWOPI * Mathf.Floor((theta + Mathf.PI) * TWOPI_RECIP);
        }
    }
}
