using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

[RequireComponent(typeof(DroneContinuousMovement))]
public class DroneAgent : Agent
{
    [Header("Target and environment")]
    public Transform target;
    public List<Transform> defenders;
    public Transform[] obstacles;
    public float maxStepTime = 500f;

    [Header("References")]
    private DroneContinuousMovement movement;
    private Rigidbody rb;

    [Header("Rewards")]
    public float collisionPenalty = -1f;
    public float defenderPenalty = -0.5f;
    public float proximityReward = 0.003f;
    public float goalReward = 1.5f;
    public float timePenalty = -0.001f;

    private Vector3 startingPosition;
    private Quaternion startingRotation;

    public override void Initialize()
    {
        movement = GetComponent<DroneContinuousMovement>();
        rb = GetComponent<Rigidbody>();
        startingPosition = transform.position;
        startingRotation = transform.rotation;
    }

    public TrainingArea trainingArea;

    public override void OnEpisodeBegin()
    {
        if (trainingArea != null)
        {
            trainingArea.ResetScene();
        }

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }


    public override void CollectObservations(VectorSensor sensor)
    {
        // Position relative à la cible
        Vector3 relativeTarget = target.position - transform.position;
        sensor.AddObservation(relativeTarget);

        // Vitesse du drone
        sensor.AddObservation(rb.linearVelocity);

        // Vecteur de direction (avant local du drone)
        sensor.AddObservation(transform.forward);

        // Défenseurs (positions relatives)
        foreach (Transform defender in defenders)
        {
            sensor.AddObservation(defender.position - transform.position);
        }

        // Obstacle le plus proche
        Transform closestObstacle = GetClosestObstacle();
        if (closestObstacle != null)
        {
            sensor.AddObservation(closestObstacle.position - transform.position);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Actions continues : Vertical, Horizontal, Rotation, Ascend
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        // Mapping des inputs vers ton script de mouvement
        InputManager.SetInput(vertical, horizontal, rotate, ascend);

        // Récompense basée sur la proximité de la cible
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        AddReward(proximityReward * (1f / (distanceToTarget + 1f)));

        // Pénalité de temps
        AddReward(timePenalty);

        // Limite de temps
        if (StepCount > maxStepTime)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
{
    var contActions = actionsOut.ContinuousActions;

    // Récupérer les entrées clavier
    float vertical = Input.GetAxis("Vertical");  // Avant / Arrière
    float horizontal = Input.GetAxis("Horizontal"); // Gauche / Droite
    float rotate = Input.GetKey(KeyCode.J) ? -1f : Input.GetKey(KeyCode.L) ? 1f : 0f; // Rotation
    float ascend = Input.GetKey(KeyCode.I) ? 1f : Input.GetKey(KeyCode.K) ? -1f : 0f; // Monter / Descendre

    // Mapper les entrées dans les actions
    contActions[0] = vertical;
    contActions[1] = horizontal;
    contActions[2] = rotate;
    contActions[3] = ascend;

    // Transmettre les entrées au InputManager
    InputManager.SetInput(vertical, horizontal, rotate, ascend);
}

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Target"))
        {
            AddReward(goalReward);
            EndEpisode();
        }
        else if (other.CompareTag("Defender"))
        {
            AddReward(defenderPenalty);
            EndEpisode();
        }
        else if (other.CompareTag("Obstacle"))
        {
            AddReward(collisionPenalty);
            EndEpisode();
        }
    }

    private Transform GetClosestObstacle()
    {
        Transform closest = null;
        float minDistance = float.MaxValue;

        foreach (Transform obs in obstacles)
        {
            float dist = Vector3.Distance(transform.position, obs.position);
            if (dist < minDistance)
            {
                minDistance = dist;
                closest = obs;
            }
        }

        return closest;
    }

    private void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        if (target != null)
            Gizmos.DrawLine(transform.position, target.position);

        if (defenders != null)
        {
            Gizmos.color = Color.blue;
            foreach (var def in defenders)
            {
                if (def != null)
                    Gizmos.DrawLine(transform.position, def.position);
            }
        }
    }
}
