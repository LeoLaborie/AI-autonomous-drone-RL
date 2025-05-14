using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RealisticDroneController : MonoBehaviour
{
    private Rigidbody rb;

    public float liftForce = 20f;
    public float moveForce = 10f;
    public float rotationSpeed = 60f;
    public float maxTiltAngle = 25f;
    public float tiltLerpSpeed = 3f;
    public float maxSpeed = 10f;

    private float currentYaw = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
    }

    void FixedUpdate()
    {
        HandleThrottle();
        HandleMovement();
        HandleRotation();
        ApplyTilt();
        LimitSpeed();
    }

    void HandleThrottle()
    {
        float throttle = Input.GetAxis("Throttle"); // entre -1 et 1

        if (Mathf.Abs(throttle) > 0.05f)
        {
            rb.AddForce(Vector3.up * throttle * liftForce, ForceMode.Force);
        }
    }

    void HandleMovement()
    {
        float rollInput = Input.GetAxis("Roll");   // gauche/droite
        float pitchInput = Input.GetAxis("Pitch"); // avant/arrière

        Vector3 localInput = new Vector3(rollInput, 0f, pitchInput).normalized;

        if (localInput.magnitude > 0.05f)
        {
            rb.AddRelativeForce(localInput * moveForce, ForceMode.Force);
        }
    }

    void HandleRotation()
    {
        float yawInput = Input.GetAxis("Yaw"); // rotation gauche/droite
        if (Mathf.Abs(yawInput) > 0.05f)
        {
            currentYaw += yawInput * rotationSpeed * Time.fixedDeltaTime;
        }

        // Clamp Yaw dans une plage raisonnable (sinon ça dépasse les limites flottantes)
        if (currentYaw > 360f) currentYaw -= 360f;
        else if (currentYaw < -360f) currentYaw += 360f;
    }

    void ApplyTilt()
    {
        float pitchInput = Input.GetAxis("Pitch");
        float rollInput = Input.GetAxis("Roll");

        float targetPitch = Mathf.LerpAngle(transform.eulerAngles.x, pitchInput * maxTiltAngle, Time.deltaTime * tiltLerpSpeed);
        float targetRoll = Mathf.LerpAngle(transform.eulerAngles.z, -rollInput * maxTiltAngle, Time.deltaTime * tiltLerpSpeed);

        Quaternion targetRotation = Quaternion.Euler(targetPitch, currentYaw, targetRoll);
        rb.MoveRotation(Quaternion.Slerp(rb.rotation, targetRotation, Time.deltaTime * tiltLerpSpeed));
    }

    void LimitSpeed()
    {
        if (rb.linearVelocity.magnitude > maxSpeed)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * maxSpeed;
        }
    }
}
