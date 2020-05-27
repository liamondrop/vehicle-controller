using System;
using UnityEngine;

namespace VehicleControl
{
    [RequireComponent(typeof(VehicleController))]
    public class VehicleUserControl : MonoBehaviour
    {
        [SerializeField] bool m_enableSteering;
        [SerializeField] bool m_enableThrottle;
        [SerializeField] bool m_enableBrake;
        VehicleController m_controller;

        void Awake()
        {
            m_controller = GetComponent<VehicleController>();
        }

        void FixedUpdate()
        {
            if (m_enableSteering)
            {
                m_controller.SetSteering(Input.GetAxis("Horizontal"));
            }

            if (m_enableThrottle)
            {
                m_controller.SetThrottle(Input.GetAxis("Vertical"));
            }

            if (m_enableBrake)
            {
                m_controller.SetBrake(Input.GetAxis("Jump"));
            }
        }
    }
}
