using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]



public class DroneAgentPhase1 : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;

    [Header("Obstacles")]
    public Collider[] obstacles;

    [Header("Entraînement")]
    public float altitudeMin = 1f;
    public float altitudeMax = 9f;
    public float zoneRadius = 5f;
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
        transform.position = new Vector3(
            Random.Range(-zoneRadius / 2f, zoneRadius / 2f),
            Random.Range(altitudeMin + 1f, altitudeMax - 1f),
            Random.Range(-zoneRadius / 2f, zoneRadius / 2f)
        );
        transform.rotation = Quaternion.identity;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity); // 3
        sensor.AddObservation(transform.forward); // 3
        sensor.AddObservation(transform.position - zoneCenter.position); // 3
        var (closestPoint, distance, col) = GetClosestObstaclePoint();
        sensor.AddObservation(closestPoint - transform.position); // position relative
        

    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        InputManager.SetInput(vertical, horizontal, rotate, ascend);

       float y = transform.position.y;

        if (y >= 1f && y <= 9f)
        {
            float heightReward = 1f - Mathf.Pow(y - 5f, 2) / 16f; // max à y = 5
            AddReward(0.01f * heightReward);
        }
        
        // Récompense : mouvement
        AddReward(0.001f * rb.linearVelocity.magnitude);

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

        foreach (var col in obstacles)
        {
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

        return (closestPoint, minDistance, closestObstacle);
    }

    private void OnTriggerEnter(Collider other)
    {
        foreach (var obs in obstacles)
        {
            if (other == obs)
            {
                AddReward(-1f);
                EndEpisode();
                break;
            }
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (obstacles == null || obstacles.Length == 0) return;

        var (closestPoint, distance, col) = GetClosestObstaclePoint();

        if (col != null)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(transform.position, closestPoint);
            Gizmos.DrawSphere(closestPoint, 0.2f);
        }
    }


}
