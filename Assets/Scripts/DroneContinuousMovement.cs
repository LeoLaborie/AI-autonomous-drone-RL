using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DroneContinuousMovement : MonoBehaviour
{
    Rigidbody drone;

    // Entrées locales pour chaque drone
    private float vertical;
    private float horizontal;
    private float rotate;
    private float ascend;

    public void SetInput(float vertical, float horizontal, float rotate, float ascend)
    {
        this.vertical = vertical;
        this.horizontal = horizontal;
        this.rotate = rotate;
        this.ascend = ascend;
    }

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
    }

    public float upForce;

    void MovementUpDown()
    {
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
        if (Mathf.Abs(vertical) > 0.1f)
        {
            drone.AddRelativeForce(Vector3.forward * vertical * movementForwardSpeed);
            tiltAmountForward = Mathf.SmoothDamp(tiltAmountForward, 20 * vertical, ref tiltVelocityForward, 0.1f);
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
        wantedYRotation += rotate * rotateAmountByKeys;
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
        if (Mathf.Abs(horizontal) > 0.1f)
        {
            drone.AddRelativeForce(Vector3.right * horizontal * sideMovementAmount);
            tiltAmountSideways = Mathf.SmoothDamp(tiltAmountSideways, -20 * horizontal, ref tiltAmountVelocity, 0.1f);
        }
        else
        {
            tiltAmountSideways = Mathf.SmoothDamp(tiltAmountSideways, 0, ref tiltAmountVelocity, 0.1f);
        }
    }
}
