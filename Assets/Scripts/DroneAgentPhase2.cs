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
    public Collider[] obstacles; // murs + plafond

    [Header("Paramètres")]
    public float maxStepTime = 3000f;
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        movement = GetComponent<DroneContinuousMovement>();
    }

    public override void OnEpisodeBegin()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Position initiale aléatoire
        transform.position = new Vector3(
            Random.Range(-20f, 20f),
            Random.Range(1f, 49f),
            Random.Range(-20f, 20f)
        );
        transform.rotation = Quaternion.identity;

        // Placer la cible à une autre position dans la zone
        target.position = new Vector3(
            Random.Range(-20f, 20f),
            Random.Range(1f, 49f),
            Random.Range(-20f, 20f)
        );

        previousDistanceToTarget = Vector3.Distance(transform.position, target.position);
        previousPosition = transform.position;

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity); // 3
        sensor.AddObservation(transform.forward); // 3
        sensor.AddObservation(target.position - transform.position); // 3
        var (closestPoint, distance, col) = GetClosestObstaclePoint();
        sensor.AddObservation(closestPoint - transform.position); // 3

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

        // Récompense de proximité
        float distanceToTarget = Vector3.Distance(transform.position, target.position);
        float proximityReward = (1f - (Mathf.Pow(distanceToTarget,2))/4900);
        AddReward(proximityReward);

        // float currentDistance = Vector3.Distance(transform.position, target.position);
        // float delta = previousDistanceToTarget - currentDistance;

        // if (delta > 0f)
        // {
        //     // Le drone s'est rapproché → récompense
        //     AddReward(1f/(Mathf.Pow(delta,2)+0.1f));
        //     AddReward(proximityReward);
        // }
        // else
        // {
        //     // Le drone s'est éloigné → pénalité
        //     AddReward(-1f/(Mathf.Pow(delta,2)+0.1f)); 
        //     AddReward(-1f * proximityReward);
        // }

        // previousDistanceToTarget = currentDistance;

        // Vector3 toTarget = (target.position - transform.position).normalized;
        // Vector3 droneForward = transform.forward.normalized;

        // float angle = Vector3.Angle(droneForward, toTarget); // en degrés

        // if (angle >= 45f)
        // {
        //     // Plus le drone regarde ailleurs, plus il est pénalisé (entre 0 et -0.01)
        //     float penalty = -0.01f * (angle / 180f); // facultatif : pénalité croissante
        //     AddReward(penalty);
        // }

        // Vector3 movementVector = (transform.position - previousPosition).normalized;

        // if (movementVector.magnitude > 0.01f) // éviter les divisions bizarres à l'arrêt
        // {
        //     float angle_mouvement = Vector3.Angle(movementVector, toTarget); // en degrés

        //     if (angle_mouvement >= 45f)
        //     {
        //         // pénalité croissante si le mouvement est mal dirigé
        //         float penalty = -0.01f * (angle_mouvement / 180f); 
        //         AddReward(penalty);
        //     }
        // }

        // previousPosition = transform.position;


        // //  pénalité de temps
        AddReward(-0.01f);

        if (StepCount > maxStepTime)
        {
            // AddReward(-1f);
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
    }
}
