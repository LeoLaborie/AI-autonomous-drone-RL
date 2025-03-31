using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class RealisticDroneController : MonoBehaviour
{
    private Rigidbody rb;

    private Quaternion targetTilt; // Déclaration de targetTilt
    public float liftForce = 9.81f;       // Force pour monter/descendre
    public float moveForce = 5f;          // Force de déplacement horizontal
    public float rotationTorque = 1f;     // Couple de rotation pour Yaw (rotation autour de l'axe Y)
    public float maxTiltAngle = 30f;      // Angle d'inclinaison maximal en degrés pour le pitch/roll
    public float tiltSpeed = 5f;          // Vitesse d'inclinaison
    public float maxSpeed = 10f;          // Limite de la vitesse maximale du drone

    private float currentPitch = 0f;
    private float currentRoll = 0f;
    private float currentYaw = 0f;
    private float throttleInput = 0f;     // Pour gérer la montée/descente

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true; // S'assurer que la gravité est activée sur le Rigidbody
        rb.position = new Vector3(transform.position.x, 1f, transform.position.z); // Position initiale au-dessus du sol
        targetTilt = transform.rotation;
    }

    void FixedUpdate()
    {
        HandleThrottle();
        HandleMovement();
        HandleRotation();
        ApplyTilt();
        LimitSpeed();
    }

    // Contrôle de la montée/descente via l'axe "Throttle"
    void HandleThrottle()
    {
        throttleInput = Input.GetAxis("Throttle"); // Valeur attendue entre -1 et 1

        // Appliquer la force de montée/descente en fonction de l'entrée utilisateur
        if (throttleInput > 0.1f)
        {
            rb.AddForce(Vector3.up * throttleInput * liftForce, ForceMode.Force);
        }
        else if (throttleInput < -0.1f)
        {
            rb.AddForce(Vector3.down * -throttleInput * liftForce, ForceMode.Force);
        }

        // Lorsque le throttle est à zéro, la gravité fait son travail naturellement
        // On ne rajoute plus de force manuelle vers le bas
        if (throttleInput == 0f)
        {
            // La gravité fait déjà son travail, on n'ajoute pas de force vers le bas ici
        }
    }

    // Contrôle du déplacement horizontal (avant/arrière, gauche/droite)
    void HandleMovement()
    {
        float rollInput = Input.GetAxis("Roll");   // Contrôle de l'inclinaison latérale (stick droit horizontal)
        float pitchInput = Input.GetAxis("Pitch"); // Contrôle de l'inclinaison avant/arrière (stick droit vertical)

        Vector3 moveDirection = new Vector3(rollInput, 0, pitchInput).normalized;
        rb.AddRelativeForce(moveDirection * moveForce, ForceMode.Force);

        // Calcul de l'inclinaison du drone en fonction des mouvements
        currentRoll = Mathf.Lerp(currentRoll, -rollInput * maxTiltAngle, Time.deltaTime * tiltSpeed);
        currentPitch = Mathf.Lerp(currentPitch, pitchInput * maxTiltAngle, Time.deltaTime * tiltSpeed);
    }

    // Contrôle de la rotation du drone (Yaw)
    void HandleRotation()
    {
        float yawInput = Input.GetAxis("Yaw");  // Attendu entre -1 et 1
        if (Mathf.Abs(yawInput) > 0.1f)
        {
            currentYaw += yawInput * rotationTorque * Time.deltaTime;
        }

        // Appliquer la rotation autour de l'axe Y (Yaw)
        Quaternion targetRotation = Quaternion.Euler(currentPitch, currentYaw, currentRoll);
        rb.MoveRotation(targetRotation);
    }

    // Appliquer l'inclinaison calculée pour simuler un mouvement réaliste
    void ApplyTilt()
    {
        // On applique un "tilt" progressif en fonction des mouvements du drone
        transform.rotation = Quaternion.Slerp(transform.rotation, Quaternion.Euler(currentPitch, currentYaw, currentRoll), Time.deltaTime * tiltSpeed);
    }

    // Limiter la vitesse du drone pour éviter qu'il ne se déplace trop vite
    void LimitSpeed()
    {
        if (rb.linearVelocity.magnitude > maxSpeed)
        {
            rb.linearVelocity = rb.linearVelocity.normalized * maxSpeed;
        }
    }
}
