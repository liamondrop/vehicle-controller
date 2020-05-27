using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace VehicleControl
{
    [RequireComponent(typeof(VehicleController))]
    public class ThrottleControl : MonoBehaviour
    {
        [Range(0, 1)] [SerializeField] float m_targetThrottle;
        [Range(0, 1)] [SerializeField] float m_incrementSize = 0.05f;
        [Range(0, 1)] [SerializeField] float m_pGain = 1;
        [Range(0, 1)] [SerializeField] float m_dGain;
        VehicleController m_controller;
        float m_lastError = 0;

        public float TargetThrottle
        {
            get => m_targetThrottle;
            set => m_targetThrottle = Mathf.Clamp(value, 0, 1);
        }

        public void IncrementTargetSpeed()
        {
            TargetThrottle = TargetThrottle + m_incrementSize;
        }

        public void DecrementTargetSpeed()
        {
            TargetThrottle = TargetThrottle - m_incrementSize;
        }

        void Start()
        {
            m_controller = GetComponent<VehicleController>();
        }

        void FixedUpdate()
        {
            var normalizedSpeed = m_controller.GetCurrentSpeed(true);
            var deltaThrottle = ThrottleError(normalizedSpeed);

            if (deltaThrottle > 0)
            {
                m_controller.SetBrake(0);
                m_controller.SetThrottle(m_controller.GetThrottle() + deltaThrottle);
            }
            else if (m_targetThrottle == 0 && normalizedSpeed < 0.025f)
            {
                m_controller.SetBrake(1);
                m_controller.SetThrottle(0);
            }
            else
            {
                m_controller.SetBrake(-deltaThrottle);
                m_controller.SetThrottle(0);
            }
        }

        float ThrottleError(float currentSpeed)
        {
            var pErr = m_targetThrottle - currentSpeed;
            var dErr = (pErr - m_lastError);
            m_lastError = pErr;
            return (m_pGain * pErr) + (m_dGain * dErr);
        }
    }
}
