using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using System.Collections.Generic;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(DroneContinuousMovement))]
public class DroneAgent : Agent
{
    private Rigidbody rb;
    private DroneContinuousMovement movement;
    // private Vector3 previousPosition;
    private float previousDistanceToTarget;

    [Header("Références")]
    public Transform target;
    // public Collider[] limiteTerrain;
    public List<Transform> obstacles;
    public Collider Ground;
    public Transform trainingArea;

    [Header("Paramètres")]
    public float maxStepTime = 10000f;

    public int horizontalRays = 12;  // azimut (0° → 360°)
    public int verticalRays = 6;     // élévation (-90° → 90°)
    public float rayLength = 20f;
    public LayerMask obstacleMask;


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
        // Observations standard du drone
        sensor.AddObservation(rb.linearVelocity); // 3
        sensor.AddObservation(transform.forward); // 3
        sensor.AddObservation(target.localPosition - transform.localPosition); // 3

        // Observation du LIDAR (raycasts en 3D)
        for (int i = 0; i < verticalRays; i++)
        {
            float verticalT = (float)i / (verticalRays - 1);
            float verticalAngle = Mathf.Lerp(-90f, 90f, verticalT);

            if (Mathf.Abs(verticalAngle) > 89f)
            {
                Quaternion rotation = Quaternion.Euler(verticalAngle, 0f, 0f);
                Vector3 dir = rotation * transform.forward;

                if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rayLength, obstacleMask))
                {
                    sensor.AddObservation(hit.distance / rayLength);
                }
                else
                {
                    sensor.AddObservation(1f); // Pas de hit
                }
            }
            else
            {
                for (int j = 0; j < horizontalRays; j++)
                {
                    float horizontalAngle = 360f * j / horizontalRays;
                    Quaternion rotation = Quaternion.Euler(verticalAngle, horizontalAngle, 0f);
                    Vector3 dir = rotation * transform.forward;

                    if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rayLength, obstacleMask))
                    {
                        sensor.AddObservation(hit.distance / rayLength);
                    }
                    else
                    {
                        sensor.AddObservation(1f); // Pas de hit
                    }
                }
            }
        }
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
        // float proximityReward = (1f - Mathf.Pow(currentDistance, 2) / 40000);
     
        if (delta < 0f) AddReward(-1f);
        else AddReward(1f);
        // AddReward(-300f);

        

        if (StepCount > maxStepTime)
        {
            EndEpisode();
        }

        previousDistanceToTarget = currentDistance;
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
            AddReward(+50f);
            EndEpisode();
            return;
        }

        // Si on touche un obstacle (List<Transform>)

        if (other.CompareTag("Obstacle"))
        {
            AddReward(-50);
            EndEpisode();
            return;
        }
        


    }



    private void PlaceObjectsScatteredWithDistanceCheck(List<Transform> objects, Vector2 areaSize, float y)
    {
        float minDistance = 25f; // Distance minimale entre objets
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
                    Random.Range(-areaSize.x / 2f, areaSize.x / 2f) + 100f,
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
                if (Vector3.Distance(newPosition, target.localPosition) < minDistance) positionValid = false;
                if (Vector3.Distance(newPosition, transform.localPosition) < minDistance) positionValid = false;

                attempts++;
            } while (!positionValid && attempts < 100);

            obj.localPosition = newPosition;

            // Appliquer une taille aléatoire
            float scale = Random.Range(10f, 50f);
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

        if (obstacleMask == 0) return;

        for (int i = 0; i < verticalRays; i++)
        {
            float verticalT = (float)i / (verticalRays - 1);
            float verticalAngle = Mathf.Lerp(-90f, 90f, verticalT);

            // Si verticalAngle est proche de ±90°, tirer un seul rayon droit
            if (Mathf.Abs(verticalAngle) > 89f)
            {
                Quaternion rotation = Quaternion.Euler(verticalAngle, 0f, 0f);
                Vector3 dir = rotation * transform.forward;

                if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rayLength, obstacleMask))
                {
                    Gizmos.color = Color.Lerp(Color.red, Color.yellow, 0.5f);
                    Gizmos.DrawLine(transform.position, hit.point);
                    Gizmos.DrawSphere(hit.point, 0.2f);
                }
                else
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawRay(transform.position, dir * rayLength);
                }
            }
            else
            {
                for (int j = 0; j < horizontalRays; j++)
                {
                    float horizontalAngle = 360f * j / horizontalRays;
                    Quaternion rotation = Quaternion.Euler(verticalAngle, horizontalAngle, 0f);
                    Vector3 dir = rotation * transform.forward;

                    if (Physics.Raycast(transform.position, dir, out RaycastHit hit, rayLength, obstacleMask))
                    {
                        Gizmos.color = Color.Lerp(Color.red, Color.yellow, 0.5f);
                        Gizmos.DrawLine(transform.position, hit.point);
                        Gizmos.DrawSphere(hit.point, 0.2f);
                    }
                    else
                    {
                        Gizmos.color = Color.cyan;
                        Gizmos.DrawRay(transform.position, dir * rayLength);
                    }
                }
            }
        }
    }



}
