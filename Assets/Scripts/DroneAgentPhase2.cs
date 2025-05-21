using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]
public class DroneAgentPhase2 : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;
    private Vector3 previousPosition;
    private float previousDistanceToTarget;

    [Header("Références")]
    public Transform target;
    public Collider[] obstacles;
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
            Random.Range(-20f, 20f),
            Random.Range(1f, 49f),
            Random.Range(-20f, 20f)
        );
        transform.rotation = Quaternion.identity;

        target.localPosition = new Vector3(
            Random.Range(-20f, 20f),
            Random.Range(1f, 49f),
            Random.Range(-20f, 20f)
        );

        previousDistanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);
        previousPosition = transform.localPosition;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity);                              // 3
        // Debug.Log("1: "+ rb.linearVelocity);
        sensor.AddObservation(transform.forward);                        // 3
        // Debug.Log("2: "+ transform.forward);
        sensor.AddObservation(target.localPosition - transform.localPosition); // 3
        // Debug.Log($"3: {target.localPosition - transform.localPosition}");
        var (closestPoint, _, _) = GetClosestObstaclePoint();
        sensor.AddObservation(trainingArea.InverseTransformPoint(closestPoint) - transform.localPosition); // 3
        // Debug.Log(closestPoint);
        // Debug.Log($"4: {trainingArea.InverseTransformPoint(closestPoint) - transform.localPosition}");
        for (int i = 0; i < 48; i++)
        {
            sensor.AddObservation(0f);
        }
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

    public override void OnActionReceived(ActionBuffers actions)
    {
        float vertical = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float horizontal = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        float rotate = Mathf.Clamp(actions.ContinuousActions[2], -1f, 1f);
        float ascend = Mathf.Clamp(actions.ContinuousActions[3], -1f, 1f);

        InputManager.SetInput(vertical, horizontal, rotate, ascend);

        // Récompense basée sur la variation de distance à la cible (locale)
        float currentDistance = Vector3.Distance(transform.localPosition, target.localPosition);
        float delta = previousDistanceToTarget - currentDistance;
        float proximityReward = 2f*(1f - Mathf.Pow(currentDistance, 2) / 4900);
        AddReward(delta > 0.03f ? proximityReward : -2f - proximityReward);


        previousDistanceToTarget = currentDistance;
        previousPosition = transform.localPosition;

        AddReward(-9f);

        if (StepCount > maxStepTime)
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

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Target"))
        {
            AddReward(+100000f);
            EndEpisode();
        }

        foreach (var obs in obstacles)
        {
            if (other == obs)
            {
                AddReward(-100000f);
                EndEpisode();
                break;
            }
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
    if (obstacles != null && obstacles.Length > 0)
    {
        var (closestPoint, _, _) = GetClosestObstaclePoint();
        Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, closestPoint);
        Gizmos.DrawSphere(closestPoint, 0.3f); // point visuel
    }
}

}
