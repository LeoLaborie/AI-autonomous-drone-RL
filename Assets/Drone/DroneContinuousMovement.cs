using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneContinuousMovement : MonoBehaviour
{
    Rigidbody drone;

    void Awake()
    {
        drone = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        MovementUpDown();
        MovementForward();
        Rotation();
        ClampingSpeedValues();
        Swerve();

        drone.AddRelativeForce(Vector3.up * upForce);
        drone.rotation = Quaternion.Euler(
            new Vector3(tiltAmountForward, currentYRotation, tiltAmountSideways)
        );

        InputManager.ResetInput(); // Reset input after each frame
    }

    public float upForce;

    void MovementUpDown()
    {
        float ascend = InputManager.Ascend;

        if (Mathf.Abs(ascend) > 0.1f)
        {
            upForce = Mathf.Lerp(upForce, ascend * 20f, Time.deltaTime * 5f);
        }
        else
        {
            upForce = Mathf.Lerp(upForce, 0f, Time.deltaTime * 5f);
        }
    }

    private float movementForwardSpeed = 30.0f;
    private float tiltAmountForward = 0;
    private float tiltVelocityForward;

    void MovementForward()
    {
        float forwardInput = InputManager.Vertical;

        if (Mathf.Abs(forwardInput) > 0.1f)
        {
            drone.AddRelativeForce(Vector3.forward * forwardInput * movementForwardSpeed);
            tiltAmountForward = Mathf.SmoothDamp(tiltAmountForward, 20 * forwardInput, ref tiltVelocityForward, 0.1f);
        }
        else
        {
            tiltAmountForward = Mathf.SmoothDamp(tiltAmountForward, 0, ref tiltVelocityForward, 0.1f);
        }
    }

    private float wantedYRotation;
    public float currentYRotation;
    private float rotateAmountByKeys = 2.5f;
    private float rotationYVelocity;

    void Rotation()
    {
        float rotationInput = InputManager.Rotation;

        wantedYRotation += rotationInput * rotateAmountByKeys;
        currentYRotation = Mathf.SmoothDamp(currentYRotation, wantedYRotation, ref rotationYVelocity, 0.25f);
    }

    private Vector3 velocityToSmoothDampingToZero;

    void ClampingSpeedValues()
    {
        if (drone.linearVelocity.magnitude > 0.1f)
        {
            drone.linearVelocity = Vector3.ClampMagnitude(drone.linearVelocity, Mathf.Lerp(drone.linearVelocity.magnitude, 10.0f, Time.deltaTime * 5f));
        }
        else
        {
            drone.linearVelocity = Vector3.SmoothDamp(drone.linearVelocity, Vector3.zero, ref velocityToSmoothDampingToZero, 0.95f);
        }
    }

    private float sideMovementAmount = 20.0f;
    private float tiltAmountSideways;
    private float tiltAmountVelocity;

    void Swerve()
    {
        float horizontalInput = InputManager.Horizontal;

        if (Mathf.Abs(horizontalInput) > 0.1f)
        {
            drone.AddRelativeForce(Vector3.right * horizontalInput * sideMovementAmount);
            tiltAmountSideways = Mathf.SmoothDamp(tiltAmountSideways, -20 * horizontalInput, ref tiltAmountVelocity, 0.1f);
        }
        else
        {
            tiltAmountSideways = Mathf.SmoothDamp(tiltAmountSideways, 0, ref tiltAmountVelocity, 0.1f);
        }
    }
}
