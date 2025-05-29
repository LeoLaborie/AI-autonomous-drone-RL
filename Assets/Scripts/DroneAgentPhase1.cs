using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]
public class DroneAgentPhase1 : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;

    [Header("Obstacles")]
    public List<Transform> obstacles;
    public Collider[] limiteTerrain;
    public Collider Ground;

    [Header("Zone")]
    public float altitudeMin = 1f;
    public float altitudeMax = 199f;
    public float zoneRadius = 100f;
    public Transform trainingArea;
    public Transform zoneCenter;
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        movement = GetComponent<DroneContinuousMovement>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        transform.localPosition = new Vector3(
            Random.Range(-zoneRadius / 2f, zoneRadius / 2f),
            Random.Range(altitudeMin + 1f, altitudeMax - 1f),
            Random.Range(-zoneRadius / 2f, zoneRadius / 2f)
        );
        transform.localRotation = Quaternion.identity;

        PlaceObjectsScatteredWithDistanceCheck(obstacles, new Vector2(180, 180), y: 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity);                    // 3
        sensor.AddObservation(transform.forward);                    // 3
        sensor.AddObservation(zoneCenter.localPosition - transform.localPosition);              // 3
        sensor.AddObservation(Vector3.zero);
        var (closestPoint, _, _) = GetClosestObstaclePoint();
        Vector3 localPoint = trainingArea.InverseTransformPoint(closestPoint);
        sensor.AddObservation(localPoint - transform.localPosition); // 3

        for (int i = 0; i < 18; i++)
        {
            sensor.AddObservation(Random.Range(-282f,282f));
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        movement.SetInput(vertical, horizontal, rotate, ascend);

        float distance = Vector3.Distance(transform.localPosition, zoneCenter.localPosition);
        float recompense = 1 - Mathf.Pow(distance, 4)/400000000;
        AddReward(recompense);

        if (StepCount > 10000)
        {
            EndEpisode();
        }
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

    private void OnTriggerEnter(Collider other)
    {
        foreach (Transform obs in obstacles)
        {
            if (obs != null && other.transform == obs)
            {
                AddReward(-1000f);
                EndEpisode();
                return;
            }
        }

        foreach (Collider col in limiteTerrain)
        {
            if (col != null && other == col)
            {
                AddReward(-1000f);
                EndEpisode();
                return;
            }
        }
    }

    private void PlaceObjectsScatteredWithDistanceCheck(List<Transform> objects, Vector2 areaSize, float y)
    {
        float minDistance = 2f;
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
                    if (other != obj && Vector3.Distance(newPosition, other.localPosition) < minDistance)
                    {
                        positionValid = false;
                        break;
                    }
                }

                attempts++;
            } while (!positionValid && attempts < 100);

            obj.localPosition = newPosition;

            float scale = Random.Range(1f, 5f);
            obj.localScale = new Vector3(scale, scale, scale);
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (obstacles != null && obstacles.Count > 0)
        {
            var (closest, _, _) = GetClosestObstaclePoint();
            Gizmos.color = Color.red;
            Gizmos.DrawLine(transform.position, closest);
            Gizmos.DrawSphere(closest, 0.3f);
        }
    }
}
