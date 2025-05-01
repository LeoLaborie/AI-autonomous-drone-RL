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

    private DroneContinuousMovement movement;
    private Rigidbody rb;

    public override void Initialize()
    {
        movement = GetComponent<DroneContinuousMovement>();
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
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
        foreach (Transform obstacle in obstacles)
        {
            if (obstacle != null)
            {
                Vector3 relativeObstaclePosition = obstacle.position - transform.position;
                sensor.AddObservation(relativeObstaclePosition);
            }
        }
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

        // Récompense basée sur la proximité du drone ennemi
        float distanceToEnemy = Vector3.Distance(transform.position, enemyDrone.position);
        AddReward(proximityReward * (1f / (distanceToEnemy + 1f)));

        // Pénalité de temps
        AddReward(timePenalty);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var contActions = actionsOut.ContinuousActions;
        contActions[0] = Input.GetAxis("Vertical");  // Avant / Arrière
        contActions[1] = Input.GetAxis("Horizontal"); // Gauche / Droite
        contActions[2] = Input.GetKey(KeyCode.J) ? -1f : Input.GetKey(KeyCode.L) ? 1f : 0f; // Rotation
        contActions[3] = Input.GetKey(KeyCode.I) ? 1f : Input.GetKey(KeyCode.K) ? -1f : 0f; // Monter / Descendre
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
}