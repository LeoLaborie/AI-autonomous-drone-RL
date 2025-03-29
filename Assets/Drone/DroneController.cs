using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    private Rigidbody rb;

    public float liftForce = 9.81f / 5;   // Force pour monter/descendre
    public float moveForce = 5f / 5;        // Force de déplacement horizontal
    public float rotationTorque = 2f / 5;   // Couple de rotation (yaw)
    public float tiltAngle = 15f;           // Angle d'inclinaison maximal en degrés pour le pitch/roll
    public float tiltSpeed = 2f;            // Vitesse d'inclinaison

    private Quaternion targetTilt;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        targetTilt = transform.rotation;
    }

    void FixedUpdate()
    {
        HandleThrottle();
        HandleMovement();
        HandleRotation();
        ApplyTilt();
    }

    // Contrôle de la montée/descente via l'axe "Throttle"
    void HandleThrottle()
    {
        float throttleInput = Input.GetAxis("Throttle"); // Valeur attendue entre -1 et 1
        if (Mathf.Abs(throttleInput) > 0.1f)
        {
            rb.AddForce(Vector3.up * throttleInput * liftForce, ForceMode.Force);
        }
    }

    // Contrôle du déplacement horizontal et calcul de l'inclinaison en fonction du pitch et roll
    void HandleMovement()
    {
        float rollInput = Input.GetAxis("Roll");   // Contrôle de l'inclinaison latérale (stick droit horizontal)
        float pitchInput = Input.GetAxis("Pitch"); // Contrôle de l'inclinaison avant/arrière (stick droit vertical)

        Vector3 moveDirection = new Vector3(rollInput, 0, pitchInput).normalized;
        rb.AddRelativeForce(moveDirection * moveForce, ForceMode.Force);

        // Calcul de l'inclinaison cible : 
        // - L'inclinaison sur l'axe Z pour le roll (négatif pour correspondre à l'effet visuel)
        // - L'inclinaison sur l'axe X pour le pitch
        float targetRoll = -rollInput * tiltAngle;
        float targetPitch = pitchInput * tiltAngle;
        targetTilt = Quaternion.Euler(targetPitch, transform.eulerAngles.y, targetRoll);
    }

    // Contrôle de la rotation (yaw) via l'axe "Yaw" (stick gauche horizontal)
    void HandleRotation()
    {
        float yawInput = Input.GetAxis("Yaw");  // Attendu entre -1 et 1
        if (Mathf.Abs(yawInput) > 0.1f)
        {
            rb.AddTorque(Vector3.up * yawInput * rotationTorque, ForceMode.Force);
        }
    }

    // Application progressive du tilt calculé vers la rotation du drone
    void ApplyTilt()
    {
        transform.rotation = Quaternion.Slerp(transform.rotation, targetTilt, Time.deltaTime * tiltSpeed);
    }
}