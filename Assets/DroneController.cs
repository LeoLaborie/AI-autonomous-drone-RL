using UnityEngine;

public class DroneController : MonoBehaviour
{
    private Rigidbody rb;

    public float liftForce = 9.81f;   // Force pour monter
    public float moveForce = 5f;      // Déplacement
    public float rotationTorque = 2f; // Rotation

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        rb.useGravity = true; // Active la gravité
    }

    void FixedUpdate()
    {
        // Monte quand on appuie sur "Espace"
        if (Input.GetKey(KeyCode.Space))
        {
            rb.AddForce(Vector3.up * liftForce, ForceMode.Force);
        }

        // Déplacement WASD
        float moveX = Input.GetAxis("Horizontal"); // Gauche/Droite (A/D)
        float moveZ = Input.GetAxis("Vertical");   // Avant/Arrière (W/S)

        rb.AddForce(transform.forward * moveZ * moveForce, ForceMode.Force);
        rb.AddForce(transform.right * moveX * moveForce, ForceMode.Force);

        // Rotation avec Q et E
        if (Input.GetKey(KeyCode.Q))
            rb.AddTorque(Vector3.up * -rotationTorque, ForceMode.Force);

        if (Input.GetKey(KeyCode.E))
            rb.AddTorque(Vector3.up * rotationTorque, ForceMode.Force);
    }
}
