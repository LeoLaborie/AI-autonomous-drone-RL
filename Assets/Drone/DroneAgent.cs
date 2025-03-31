using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class DroneAgent : Agent
{
    private Rigidbody rb;

    public float moveForce = 5f;
    public float liftForce = 8f;

    // Start is called before the first frame update
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Méthode qui s'occupe de l'entrée heuristique
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;

        // Exemple : contrôler la montée/descente, le mouvement horizontal et la rotation
        continuousActions[0] = Input.GetKey(KeyCode.Space) ? 1 : 0; // 1 pour montée, 0 pour descente
        continuousActions[1] = Input.GetAxis("Horizontal"); // Déplacement gauche/droite
        continuousActions[2] = Input.GetAxis("Vertical"); // Déplacement avant/arrière
        continuousActions[3] = Input.GetAxis("Yaw"); // Rotation (yaw)
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float moveX = actions.ContinuousActions[1]; // Déplacement horizontal (gauche/droite)
        float moveZ = actions.ContinuousActions[2]; // Déplacement avant/arrière
        float yaw = actions.ContinuousActions[3];   // Rotation (yaw)

        // Appliquer la force pour déplacer le drone
        rb.AddForce(new Vector3(moveX, 0, moveZ) * moveForce);

        // Appliquer la rotation
        rb.AddTorque(Vector3.up * yaw * 0.1f);

        // Appliquer la montée/descente
        if (actions.ContinuousActions[0] > 0.1f) // Pour la montée
        {
            rb.AddForce(Vector3.up * liftForce);
        }
        else if (actions.ContinuousActions[0] < -0.1f) // Pour descendre
        {
            rb.AddForce(Vector3.down * liftForce);
        }
    }

    // Méthode pour collecter les observations du drone
    public override void CollectObservations(VectorSensor sensor)
    {
        // Ajouter les observations : la position et la vitesse
        sensor.AddObservation(transform.position); // Position du drone (x, y, z)
        sensor.AddObservation(rb.linearVelocity);        // Vitesse du drone (vx, vy, vz)
        sensor.AddObservation(transform.rotation.eulerAngles); // Rotation du drone (pitch, yaw, roll)
    }
}
