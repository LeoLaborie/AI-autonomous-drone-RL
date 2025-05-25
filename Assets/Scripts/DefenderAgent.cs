using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]
public class DefenderAgent : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;

    [Header("Références")]
    public Transform enemyDrone;
    public Transform target;
    public List<Transform> defenders;
    public List<Transform> obstacles;
    public Collider[] limiteTerrain;
    public Collider Ground;
    public Transform trainingArea;

    [Header("Paramètres")]
    public float maxStepTime = 10000f;

    private float previousDistanceToTarget;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        movement = GetComponent<DroneContinuousMovement>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // transform.localPosition = target.localPosition + new Vector3(Random.Range(-10, 10), Random.Range(10, 20), Random.Range(-10, 10));
        // previousDistanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity); // 3
        sensor.AddObservation(transform.forward); // 3
        sensor.AddObservation(enemyDrone.localPosition - transform.localPosition); // 3
        var (closestPoint, _, _) = GetClosestObstaclePoint();
        Vector3 localClosest = trainingArea.InverseTransformPoint(closestPoint);
        sensor.AddObservation(localClosest - transform.localPosition); // 3

        sensor.AddObservation(target.localPosition - transform.localPosition); // 3

        foreach (Transform def in defenders)
        {
            if (def != null && def != transform) {
                sensor.AddObservation(def.localPosition - transform.localPosition); // 3 * 5
            }
                
        }

        
    }

    private (Vector3 point, float distance, Collider obstacle) GetClosestObstaclePoint()
    {
        Vector3 droneLocalPos = transform.localPosition;
        Vector3 closestPoint = Vector3.zero;
        float minDistance = float.MaxValue;
        Collider closestObstacle = null;

        // Obstacles dynamiques
        foreach (Transform t in obstacles)
        {
            if (t != null)
            {
                Collider col = t.GetComponent<Collider>();
                if (col != null)
                {
                    Vector3 worldPoint = col.ClosestPoint(transform.position);
                    Vector3 localPoint = trainingArea.InverseTransformPoint(worldPoint);
                    float dist = Vector3.Distance(droneLocalPos, localPoint);

                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        closestPoint = worldPoint;
                        closestObstacle = col;
                    }
                }
            }
        }

        if (Ground != null)
        {
            Vector3 worldPoint = Ground.ClosestPoint(transform.position);
            Vector3 localPoint = trainingArea.InverseTransformPoint(worldPoint);
            float dist = Vector3.Distance(droneLocalPos, localPoint);

            if (dist < minDistance)
            {
                minDistance = dist;
                closestPoint = worldPoint;
                closestObstacle = Ground;
            }
        }

        // Limites du terrain
        // foreach (Collider col in limiteTerrain)
        // {
        //     if (col != null)
        //     {
        //         Vector3 worldPoint = col.ClosestPoint(transform.position);
        //         Vector3 localPoint = trainingArea.InverseTransformPoint(worldPoint);
        //         float dist = Vector3.Distance(droneLocalPos, localPoint);

        //         if (dist < minDistance)
        //         {
        //             minDistance = dist;
        //             closestPoint = worldPoint;
        //             closestObstacle = col;
        //         }
        //     }
        // }

        return (closestPoint, minDistance, closestObstacle);
    }
    public override void OnActionReceived(ActionBuffers actions)
    {
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        movement.SetInput(vertical, horizontal, rotate, ascend);

        // 📍 Récompense basée sur la distance à la cible (max à 20m)
        float currentDistanceTarget = Vector3.Distance(transform.localPosition, target.localPosition);
        if (currentDistanceTarget > 40) AddReward(-10f);
        // float distanceRewardTarget = 1f - Mathf.Pow(currentDistanceTarget - 20f, 2) / 100f;
        // AddReward(0.1f * distanceRewardTarget);

        // // 👥 Récompense basée sur la distance au plus proche allié (max à 5m)
        // float minAllyDistance = float.MaxValue;

        // foreach (Transform other in defenders)
        // {
        //     if (other != null && other != this.transform)
        //     {
        //         float d = Vector3.Distance(transform.localPosition, other.localPosition);
        //         if (d < minAllyDistance)
        //             minAllyDistance = d;
        //     }
        // }

        // // Même formule, max à 5m, décroît au carré
        // float distanceRewardAlly = 1f - Mathf.Pow(minAllyDistance - 10f, 2) / 100f;
        // AddReward(0.1f * distanceRewardAlly);

        float currentDistance = Vector3.Distance(transform.localPosition, enemyDrone.localPosition);
        float distanceReward = 1f - currentDistance / 100f;
        AddReward(distanceReward);
    }



    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var c = actionsOut.ContinuousActions;
        c[0] = Input.GetAxis("Vertical");
        c[1] = Input.GetAxis("Horizontal");
        c[2] = Input.GetKey(KeyCode.J) ? -1f : Input.GetKey(KeyCode.L) ? 1f : 0f;
        c[3] = Input.GetKey(KeyCode.I) ? 1f : Input.GetKey(KeyCode.K) ? -1f : 0f;

        movement.SetInput(c[0], c[1], c[2], c[3]);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other == enemyDrone.GetComponent<Collider>())
        {
            AddReward(+1000f);
            // var area = trainingArea.GetComponent<TrainingArea>();
            // if (area != null && area.dronesDefensifs != null)
            // {
            //     foreach (Transform def in area.dronesDefensifs)
            //     {
            //         if (def != null)
            //         {
            //             var agent = def.GetComponent<DefenderAgent>();
            //             if (agent != null && agent != this)
            //             {
            //                 agent.AddReward(+500f);
            //             }
            //         }
            //     }
            // }

            // var manager = trainingArea.GetComponent<TrainingArea>();
            // if (manager != null)
            // {
            //     manager.ResetScene();
            // }

        }

        if (other.CompareTag("Obstacle") || other.CompareTag("Defender") || other.CompareTag("Target"))
        {
            AddReward(-1000f);
            gameObject.SetActive(false);
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (enemyDrone != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, enemyDrone.position);
        }

        if (target != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, target.position);
        }

        if (obstacles != null && obstacles.Count > 0)
        {
            var (closestPoint, _, _) = GetClosestObstaclePoint();
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, closestPoint);
            Gizmos.DrawSphere(closestPoint, 0.3f);
        }
    }
}
