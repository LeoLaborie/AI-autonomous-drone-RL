using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]
public class DroneAgentPhase25 : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;
    // private Vector3 previousPosition;
    private float previousDistanceToTarget;

    [Header("Références")]
    public Transform target;
    public Collider[] limiteTerrain;
    public List<Transform> obstacles;
    public Collider Ground;
    public Transform trainingArea;

    [Header("Paramètres")]
    public float maxStepTime = 10000f;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        movement = GetComponent<DroneContinuousMovement>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Positionnement en coordonnées locales
        transform.localPosition = new Vector3(
            Random.Range(-50f, 50f),
            Random.Range(1f, 100f),
            Random.Range(-50f, 50f)
        );
        transform.rotation = Quaternion.identity;

        target.localPosition = new Vector3(
            Random.Range(-50f, 50f),
            Random.Range(1f, 100f),
            Random.Range(-50f, 50f)
        );

        previousDistanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);
        // previousPosition = transform.localPosition;


        PlaceObjectsScatteredWithDistanceCheck(obstacles, new Vector2(180, 180), y: 0f);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity);                              // 3
        sensor.AddObservation(transform.forward);                        // 3
        sensor.AddObservation(target.localPosition - transform.localPosition); // 3
        var (closestPoint, _, _) = GetClosestObstaclePoint();
        sensor.AddObservation(trainingArea.InverseTransformPoint(closestPoint) - transform.localPosition); // 3

        for (int i = 0; i < 18; i++)
        {
            sensor.AddObservation(Random.Range(-282f,282f));
        }
    }

    private (Vector3 point, float distance, Collider obstacle) GetClosestObstaclePoint()
    {
        Vector3 closestPoint = Vector3.zero;
        float minDistance = float.MaxValue;
        Collider closestObstacle = null;

        foreach (Transform t in obstacles)
        {
            if (t != null)
            {
                Collider col = t.GetComponent<Collider>();
                if (col != null)
                {
                    Vector3 point = col.ClosestPoint(transform.position);
                    float dist = Vector3.Distance(transform.position, point);

                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        closestPoint = point;
                        closestObstacle = col;
                    }
                }
            }
        }

        if (Ground != null)
        {
            Vector3 point = Ground.ClosestPoint(transform.position);
            float dist = Vector3.Distance(transform.position, point);

            if (dist < minDistance)
            {
                minDistance = dist;
                closestPoint = point;
                closestObstacle = Ground;
            }
        }
        // foreach (Collider col in limiteTerrain)
        // {
        //     if (col != null)
        //     {
        //         Vector3 point = col.ClosestPoint(transform.position);
        //         float dist = Vector3.Distance(transform.position, point);

        //         if (dist < minDistance)
        //         {
        //             minDistance = dist;
        //             closestPoint = point;
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

        // Récompense basée sur la variation de distance à la cible (locale)
        float currentDistance = Vector3.Distance(transform.localPosition, target.localPosition);
        float delta = previousDistanceToTarget - currentDistance;
        float proximityReward = (1f - Mathf.Pow(currentDistance, 2) / 40000);

        if (delta < 0f) AddReward(-1f);
        else AddReward(1f);
        // AddReward(-100f);

        if (currentDistance < 10) {
            AddReward(100000f);
            EndEpisode();
        }

        if (StepCount > maxStepTime)
        {
            EndEpisode();
        }

        // previousDistanceToTarget = currentDistance;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var c = actionsOut.ContinuousActions;
        c[0] = Input.GetAxis("Vertical");
        c[1] = Input.GetAxis("Horizontal");
        c[2] = Input.GetKey(KeyCode.J) ? -1f : Input.GetKey(KeyCode.L) ? 1f : 0f;
        c[3] = Input.GetKey(KeyCode.I) ? 1f : Input.GetKey(KeyCode.K) ? -1f : 0f;

        InputManager.SetInput(c[0], c[1], c[2], c[3]);
    }

    private void OnTriggerEnter(Collider other)
    {
        // Si on touche la cible
        if (other.CompareTag("Target"))
        {
            AddReward(+100000f);
            EndEpisode();
            return;
        }

        // Si on touche un obstacle (List<Transform>)
        foreach (Transform obs in obstacles)
        {
            if (obs != null && other.transform == obs)
            {
                AddReward(-100000f);
                EndEpisode();
                return;
            }
        }

        // Si on touche une limite de terrain (Collider[])
        foreach (Collider col in limiteTerrain)
        {
            if (col != null && other == col)
            {
                AddReward(-100000f);
                EndEpisode();
                return;
            }
        }
    }



    private void PlaceObjectsScatteredWithDistanceCheck(List<Transform> objects, Vector2 areaSize, float y)
    {
        float minDistance = 2f; // Distance minimale entre objets
        foreach (Transform obj in objects)
        {
            Vector3 newPosition;
            bool positionValid;
            int attempts = 0;

            do
            {
                positionValid = true;
                newPosition = new Vector3(
                    Random.Range(-areaSize.x / 2f, areaSize.x / 2f),
                    y,
                    Random.Range(-areaSize.y / 2f, areaSize.y / 2f)
                );

                foreach (Transform other in objects)
                {
                    if (other != obj && Vector3.Distance(newPosition, other.position) < minDistance)
                    {
                        positionValid = false;
                        break;
                    }
                }

                attempts++;
            } while (!positionValid && attempts < 100);

            obj.localPosition = newPosition;

            // Appliquer une taille aléatoire
            float scale = Random.Range(1f, 5f);
            obj.localScale = new Vector3(scale, scale, scale);
        }
    }
    private void OnDrawGizmosSelected()
    {
        if (target != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, target.position);
        }

        // Ligne vers l'obstacle le plus proche
        if (obstacles != null && obstacles.Count > 0)
        {
            var (closestPoint, _, _) = GetClosestObstaclePoint();
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, closestPoint);
            Gizmos.DrawSphere(closestPoint, 0.3f); // point visuel
        }
    }

}
