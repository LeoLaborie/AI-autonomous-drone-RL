using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class DroneController : MonoBehaviour
{
    private Rigidbody rb;

    public float liftForce = 9.81f / 5;   // Force pour monter
    public float moveForce = 5f / 5;      // Force de déplacement
    public float rotationTorque = 2f / 5; // Couple de rotation
    public float tiltAngle = 15f;     // Angle d'inclinaison maximal en degrés
    public float tiltSpeed = 2f;      // Vitesse d'inclinaison

    private Quaternion targetTilt;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true;
        targetTilt = transform.rotation;
    }

    void FixedUpdate()
    {
        HandleLift();
        HandleMovement();
        HandleRotation();
        ApplyTilt();
    }

    void HandleLift()
    {
        if (Input.GetKey(KeyCode.Space))
        {
            rb.AddForce(Vector3.up * liftForce, ForceMode.Force);
        }
    }

    void HandleMovement()
    {
        float moveX = Input.GetAxis("Horizontal"); // A/D ou Flèches Gauche/Droite
        float moveZ = Input.GetAxis("Vertical");   // W/S ou Flèches Haut/Bas

        Vector3 moveDirection = new Vector3(moveX, 0, moveZ).normalized;
        rb.AddRelativeForce(moveDirection * moveForce, ForceMode.Force);

        // Calcul de l'inclinaison cible
        float tiltAroundZ = -moveX * tiltAngle;
        float tiltAroundX = moveZ * tiltAngle;
        targetTilt = Quaternion.Euler(tiltAroundX, transform.eulerAngles.y, tiltAroundZ);
    }

    void HandleRotation()
    {
        if (Input.GetKey(KeyCode.Q))
            rb.AddTorque(Vector3.up * -rotationTorque, ForceMode.Force);

        if (Input.GetKey(KeyCode.E))
            rb.AddTorque(Vector3.up * rotationTorque, ForceMode.Force);
    }

    void ApplyTilt()
    {
        transform.rotation = Quaternion.Slerp(transform.rotation, targetTilt, Time.deltaTime * tiltSpeed);
    }
}
