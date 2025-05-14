using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

[RequireComponent(typeof(DroneContinuousMovement))]
public class DefenderAgent : Agent
{
    [Header("Target and enemy")]
    public Transform target; // La cible à protéger 
    public Transform enemyDrone; // Le drone attaquant

    [Header("Team and environment")]
    public List<Transform> otherDefenders; // Les autres drones défenseurs
    public Transform[] obstacles; // Les obstacles dans l'environnement

    [Header("Rewards")]
    public float collisionReward = 1f;
    public float proximityReward = 0.003f;
    public float collisionPenalty = -1f;
    public float enemyGoalPenalty = -1f;
    public float timePenalty = -0.001f;
    public float maxStepTime = 10000f;
    private DroneContinuousMovement movement;
    private Rigidbody rb;

    public override void Initialize()
    {
        movement = GetComponent<DroneContinuousMovement>();
        rb = GetComponent<Rigidbody>();
        
        // startingPosition = transform.position;
        // startingRotation = transform.rotation;
    }

    public override void OnEpisodeBegin()
    {   
        
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {   
        // Debug.Log("COllectObservations");
        // Position relative du drone ennemi
        Vector3 relativeEnemyPosition = enemyDrone.position - transform.position;
        sensor.AddObservation(relativeEnemyPosition);

        // Position relative de la cible
        Vector3 relativeTargetPosition = target.position - transform.position;
        sensor.AddObservation(relativeTargetPosition);

        // Vitesse propre du défenseur
        sensor.AddObservation(rb.linearVelocity);

        // Vecteur de direction (avant local du défenseur)
        sensor.AddObservation(transform.forward);

        // Positions, vitesses et directions des autres défenseurs
        foreach (Transform defender in otherDefenders)
        {
            if (defender != null)
            {
                Vector3 relativeDefenderPosition = defender.position - transform.position;
                Rigidbody defenderRb = defender.GetComponent<Rigidbody>();
                sensor.AddObservation(relativeDefenderPosition);
                sensor.AddObservation(defenderRb != null ? defenderRb.linearVelocity : Vector3.zero);
                sensor.AddObservation(defender.forward);
            }
        }

        // Position relative des obstacles
        var (closestPoint, distance, obs) = GetClosestObstaclePoint();

        sensor.AddObservation(closestPoint - transform.position);

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Actions continues : Vertical, Horizontal, Rotation, Ascend
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        // Mapping des inputs vers le script de mouvement
        InputManager.SetInput(vertical, horizontal, rotate, ascend);

        // Debug.Log($"[{name}] Actions => Vertical: {vertical:F2}, Horizontal: {horizontal:F2}, Rotate: {rotate:F2}, Ascend: {ascend:F2}");


        // Récompense basée sur la proximité du drone ennemi
        float distanceToEnemy = Vector3.Distance(transform.position, enemyDrone.position);
        AddReward(proximityReward * (1f / (distanceToEnemy + 1f)));

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
        if (other.CompareTag("Attacker"))
        {
            AddReward(collisionReward);
            EndEpisode();
        }
        else if (other.CompareTag("Obstacle"))
        {
            AddReward(collisionPenalty);
            EndEpisode();
        }
        else if (other.CompareTag("Target"))
        {
            AddReward(enemyGoalPenalty);
            EndEpisode();
        }
    }
    private (Vector3 point, float distance, Transform obstacle) GetClosestObstaclePoint()
{
    Vector3 closestPoint = Vector3.zero;
    float minDistance = float.MaxValue;
    Transform closestObstacle = null;

    foreach (Transform obs in obstacles)
    {
        Collider col = obs.GetComponent<Collider>();
        if (col != null)
        {
            Vector3 point = col.ClosestPoint(transform.position);
            float dist = Vector3.Distance(transform.position, point);

            if (dist < minDistance)
            {
                minDistance = dist;
                closestPoint = point;
                closestObstacle = obs;
            }
        }
    }

    return (closestPoint, minDistance, closestObstacle);
}

}


