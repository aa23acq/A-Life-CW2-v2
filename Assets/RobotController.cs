using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class RobotController : MonoBehaviour

{
    public enum RobotState
    {
        Normal,
        Turning,
        Climbing,
        Descending,
    }
    private RobotState currentState = RobotState.Normal;
    // naming constraints do not change
    [SerializeField] private WheelCollider frontLeftWheelCollider;
    [SerializeField] private WheelCollider frontRightWheelCollider;
    [SerializeField] private WheelCollider rearLeftWheelCollider;
    [SerializeField] private WheelCollider rearRightWheelCollider;

    [SerializeField] private Transform frontLeftWheelTransform;
    [SerializeField] private Transform frontRightWheelTransform;
    [SerializeField] private Transform rearLeftWheelTransform;
    [SerializeField] private Transform rearRightWheelTransform;

    [SerializeField] private Transform RCFR;
    [SerializeField] private Transform RCL1;
    [SerializeField] private Transform RCL2;
    [SerializeField] private Transform RCL3;
    [SerializeField] private Transform RCR1;
    [SerializeField] private Transform RCR2;
    [SerializeField] private Transform RCR3;
    [SerializeField] private Transform AGOR;

    [SerializeField] private float maxSteeringAngle = 45f;
    [SerializeField] private float motorForce = 50f;
    [SerializeField] private float brakeForce;



    private Rigidbody rb;

    [SerializeField] private float angle_x;
    [SerializeField] private float angle_z;
    [SerializeField] private float velocity;


    [SerializeField] private float maxMotorForceTurning = 200f;
    [SerializeField] private float maxMotorForceClimbing = 500f;


    [SerializeField] private float descentSlopeThreshold = 15f; 


    private float steerAngle;
    private bool isBreaking;

    private float s1dist = 5;
    private float s3dist = 4;



    private void AdjustSensors(Transform sensor, float x_angle, float y_angle, float z_angle)
    {
        sensor.transform.Rotate(x_angle, y_angle, z_angle);
    }


    private void Start()
    {
        rb = GetComponent<Rigidbody>();

        float s1x = 0; float s1y = 10; float s1z = 0;
        float s3x = 16; float s3y = 50; float s3z = 0;

        AdjustSensors(RCFR, 20, 0, 0);
        AdjustSensors(RCL1, s1x, -s1y, s1z);
        AdjustSensors(RCL3, s3x, -s3y, s3z);
        AdjustSensors(RCR1, s1x, s1y, s1z);
        AdjustSensors(RCR3, s3x, s3y, s3z);
        AdjustSensors(AGOR, 50, 180, 0);

       
    }




    private void HandleMotor()
    {
        float currentMotorForce = isBreaking ? 0f : motorForce;

        

        frontLeftWheelCollider.motorTorque = currentMotorForce;
        frontRightWheelCollider.motorTorque = currentMotorForce;
        rearLeftWheelCollider.motorTorque = currentMotorForce;
        rearRightWheelCollider.motorTorque = currentMotorForce;

        brakeForce = isBreaking ? 3000f : 0f;
        frontLeftWheelCollider.brakeTorque = brakeForce;
        frontRightWheelCollider.brakeTorque = brakeForce;
        rearLeftWheelCollider.brakeTorque = brakeForce;
        rearRightWheelCollider.brakeTorque = brakeForce;
    }
    private void UpdateWheels()
    {
        UpdateWheelPos(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelPos(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelPos(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelPos(rearRightWheelCollider, rearRightWheelTransform);

    }

    private void UpdateWheelPos(WheelCollider wheelCollider, Transform trans)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        trans.rotation = rot;
        trans.position = pos;
    }

    private void HandleSteering(float direction)
    {
        steerAngle = maxSteeringAngle * direction;
        frontLeftWheelCollider.steerAngle = steerAngle;
        frontRightWheelCollider.steerAngle = steerAngle;
    }
    private bool sense(Transform sensor, float dist)
    {
        if (Physics.Raycast(sensor.position, sensor.TransformDirection(Vector3.forward), dist))
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * dist, Color.yellow);
            return true;
        }
        else
        {
            Debug.DrawRay(sensor.position, sensor.TransformDirection(Vector3.forward) * dist, Color.white);
            return false;
        }
    }
    private void StayOnRoad()
    {
        bool leftSensorHit = sense(RCL3, s3dist);
        bool rightSensorHit = sense(RCR3, s3dist);

        if (!leftSensorHit || !rightSensorHit)
        {
            if (!leftSensorHit)
            {
                HandleSteering(1);
            }
            if (!rightSensorHit)
            {
                HandleSteering(-1);
            }
        }
        else
        {
            HandleSteering(0);
        }
        Debug.Log("steerAngle: " + steerAngle);
         Debug.Log("motorForce: " + motorForce);
        Debug.Log("state: " + currentState.ToString());
    }

    private void AdjustSpeed()
    {
        float currentMotorForce = isBreaking ? 0f : motorForce;

        switch (currentState)
        {
            case RobotState.Normal:
                AdjustSpeedNormal(ref currentMotorForce);
                break;

            case RobotState.Turning:
                AdjustSpeedTurning(ref currentMotorForce);
                break;

            case RobotState.Climbing:
                AdjustSpeedClimbing(ref currentMotorForce);
                break;

            case RobotState.Descending:
                AdjustSpeedDescending(ref currentMotorForce);
                break;
        }

 
        motorForce = currentMotorForce;
    }

    private bool IsClimbingHill()
    {

        return angle_x > 10f  angle_z > 1f;
    }

    private bool IsTurning() {
        return steerAngle != 0;
    }

    private void AdjustSpeedNormal(ref float currentMotorForce)
    {
         
        if (velocity < 2 && currentMotorForce < 50)
        {
            currentMotorForce += 5f;
        }
        if (velocity > 4 && currentMotorForce > 0)
        {
            currentMotorForce -= 5f;
        }

      
        
        if (IsClimbingHill())
        {
            currentState = RobotState.Climbing;
        }
        else if (IsTurning())
        {
            currentState = RobotState.Turning;
        }
    }

    private void AdjustSpeedTurning(ref float currentMotorForce)
    {

        currentMotorForce =  maxMotorForceTurning;

      
        if (Mathf.Approximately(steerAngle, 0f))
        {
            currentState = RobotState.Normal;
        }
    }

    private void AdjustSpeedClimbing(ref float currentMotorForce)
    {
         
        if (!IsClimbingHill())
        {
            currentState = RobotState.Normal;
        }
        else
        {
             
            float slopeMultiplier = Mathf.Clamp01(Mathf.Abs(angle_x));
            float desiredForce = Mathf.Lerp(0f, maxMotorForceClimbing, slopeMultiplier);
            float climbingForceChangeRate = 100f; 

             
            currentMotorForce = Mathf.MoveTowards(currentMotorForce, desiredForce, Time.fixedDeltaTime * climbingForceChangeRate);

            
            Debug.Log("New Force: " + currentMotorForce);
        }
    }






    private void AdjustSpeedDescending(ref float currentMotorForce)
    {
        
        currentMotorForce *= 0.8f;  

        
        if (Mathf.Abs(angle_x) <= descentSlopeThreshold)
        {
            currentState = RobotState.Normal;
        }
    }
    private void AvoidObstacles()
    {
        bool leftObstacle = sense(RCL1, s1dist);
        bool rightObstacle = sense(RCR1, s1dist);

        if (leftObstacle)
        {
            HandleSteering(1);
        }

        if (rightObstacle)
        {
            HandleSteering(-1);
        }
    }

    private void FixedUpdate()
    {
        StayOnRoad();
        AvoidObstacles();
        AdjustSpeed();
        HandleMotor();
        UpdateWheels();

         

        angle_x = AGOR.eulerAngles.x;
        angle_z = AGOR.eulerAngles.z;

        velocity = rb.velocity.magnitude;
    }

}