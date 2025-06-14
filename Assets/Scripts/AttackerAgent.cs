using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]
public class AttackerAgent : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;

    [Header("Références")]
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

        var manager = trainingArea.GetComponent<TrainingArea>();
        if (manager != null)
        {
            manager.ResetScene();
        }

        previousDistanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);


    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity); // 3
        sensor.AddObservation(transform.forward); // 3
        sensor.AddObservation(target.localPosition - transform.localPosition); // 3
        var (closestPoint, _, _) = GetClosestObstaclePoint();
        Vector3 localClosest = trainingArea.InverseTransformPoint(closestPoint);
        sensor.AddObservation(localClosest - transform.localPosition); // 3


        foreach (Transform defender in defenders)
        {
            if (defender != null)
                sensor.AddObservation(defender.localPosition - transform.localPosition); // 3 * N
        }

        
    }

    private (Vector3 point, float distance, Collider obstacle) GetClosestObstaclePoint()
    {
        Vector3 droneLocalPos = transform.localPosition;
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
                    Vector3 worldClosest = col.ClosestPoint(transform.position);
                    Vector3 localClosest = trainingArea.InverseTransformPoint(worldClosest);
                    float dist = Vector3.Distance(droneLocalPos, localClosest);

                    if (dist < minDistance)
                    {
                        minDistance = dist;
                        closestPoint = worldClosest;
                        closestObstacle = col;
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
        }

        return (closestPoint, minDistance, closestObstacle);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        movement.SetInput(vertical, horizontal, rotate, ascend);

        // float currentDistance = Vector3.Distance(transform.localPosition, target.localPosition);
        // float delta = previousDistanceToTarget - currentDistance;

        // if (delta < 0f) AddReward(-1f);


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

        movement.SetInput(c[0], c[1], c[2], c[3]);
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Target"))
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
            //             if (agent != null)
            //             {
            //                 agent.AddReward(-1000f);
            //             }
            //         }
            //     }
            // }


            if (Ground != null)
            {
                Renderer r = Ground.GetComponent<Renderer>();
                if (r != null)
                    r.material.color = Color.red;
            }
            
            EndEpisode();
        }

        if (other.CompareTag("Obstacle") || other.CompareTag("Defender"))
        {


            if (Ground != null)
            {

                Renderer r = Ground.GetComponent<Renderer>();
                if (r != null)
                {
                    if (other.CompareTag("Defender"))
                    {
                        AddReward(-1000f);
                        r.material.color = Color.green;
                    }
                    else if (other.CompareTag("Obstacle"))
                    {
                        AddReward(-1000f);
                        r.material.color = Color.yellow;
                    }
                }
            }
            
            EndEpisode();
        }
    }

    private void OnDrawGizmosSelected()
    {
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

        if (defenders != null && defenders.Count > 0)
        {
            Gizmos.color = Color.blue;
            foreach (var defender in defenders)
            {
                if (defender != null)
                {
                    Gizmos.DrawLine(transform.position, defender.position);
                }
            }
        }
    }

}
