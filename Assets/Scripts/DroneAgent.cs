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
    private float speedSum = 0f;
    private float maxSpeed = 0f;
    private int speedCount = 0;

    private int stepCount = 0;

    private float lastVertical = 0f;
    private float lastHorizontal = 0f;
    private float lastRotate = 0f;
    private float lastAscend = 0f;

    private static int totalEpisodes = 0;
    private static int successCount = 0;
    private static int timeoutCount = 0;
    private static int obstacleCrashCount = 0;


    [Header("Références")]
    public Transform target;
    // public Collider[] limiteTerrain;
    public List<Transform> obstacles;
    public Collider Ground;
    public Transform trainingArea;

    [Header("Paramètres")]
    public float maxStepTime = 4000;

    public int horizontalRays = 12;  // azimut (0° → 360°)
    public int verticalRays = 6;     // élévation (-90° → 90°)
    public float rayLength = 20f;
    public LayerMask obstacleMask;
    public LayerMask targetMask;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        movement = GetComponent<DroneContinuousMovement>();
    }

    public override void OnEpisodeBegin()
    {   
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        
        stepCount = 0;
        speedSum = 0f;
        maxSpeed = 0f;
        speedCount = 0;


        // Positionnement en coordonnées locales
        // 1. Positionner la cible de manière aléatoire dans la zone
        target.localPosition = new Vector3(
            Random.Range(-75f, 75f),
            Random.Range(1f, 199f),
            Random.Range(-75f, 75f)
        );

        // 2. Trouver une position pour le drone à au moins 100m de la cible
        Vector3 dronePos;
        do
        {
            dronePos = new Vector3(
            Random.Range(-100f, 100f),
            Random.Range(1f, 199f),
            Random.Range(-100f, 100f)
            );
        }
        while (Vector3.Distance(dronePos, target.localPosition) < 100f);

        transform.localPosition = dronePos;
        transform.rotation = Quaternion.identity;


        previousDistanceToTarget = Vector3.Distance(transform.localPosition, target.localPosition);
        // previousPosition = transform.localPosition;


        PlaceObjectsScatteredWithDistanceCheck(obstacles, new Vector2(180, 180), y: 0f);

    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(rb.linearVelocity); // 3
        sensor.AddObservation(transform.forward); // 3
        sensor.AddObservation(target.localPosition - transform.localPosition); // 3

        for (int i = 0; i < verticalRays; i++)
        {
            float verticalT = (float)i / (verticalRays - 1);
            float verticalAngle = Mathf.Lerp(-90f, 90f, verticalT);

            if (Mathf.Abs(verticalAngle) > 89f)
            {
                Quaternion rotation = Quaternion.Euler(verticalAngle, 0f, 0f);
                Vector3 dir = transform.localRotation * (rotation * Vector3.forward);
                Vector3 origin = transform.TransformPoint(Vector3.zero);

                if (Physics.Raycast(origin, dir, out RaycastHit hitTarget, rayLength, targetMask))
                {
                    sensor.AddObservation(2f);
                }
                else if (Physics.Raycast(origin, dir, out RaycastHit hitObstacle, rayLength, obstacleMask))
                {
                    sensor.AddObservation(hitObstacle.distance / rayLength);
                }
                else
                {
                    sensor.AddObservation(1f);
                }
            }
            else
            {
                for (int j = 0; j < horizontalRays; j++)
                {
                    float horizontalAngle = 360f * j / horizontalRays;
                    Quaternion rotation = Quaternion.Euler(verticalAngle, horizontalAngle, 0f);
                    Vector3 dir = transform.localRotation * (rotation * Vector3.forward);
                    Vector3 origin = transform.TransformPoint(Vector3.zero);

                    if (Physics.Raycast(origin, dir, out RaycastHit hitTarget, rayLength, targetMask))
                    {
                        sensor.AddObservation(2f);
                    }
                    else if (Physics.Raycast(origin, dir, out RaycastHit hitObstacle, rayLength, obstacleMask))
                    {
                        sensor.AddObservation(hitObstacle.distance / rayLength);
                    }
                    else
                    {
                        sensor.AddObservation(1f);
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
        // Calcul des variations
        float deltaV = Mathf.Abs(vertical - lastVertical);
        float deltaH = Mathf.Abs(horizontal - lastHorizontal);
        float deltaR = Mathf.Abs(rotate - lastRotate);
        float deltaA = Mathf.Abs(ascend - lastAscend);

        // Pénalité sur la variation des actions (valeurs à ajuster)
        float variationPenalty = -0.1f * Mathf.Abs(deltaV + deltaH + deltaR + deltaA);
        AddReward(variationPenalty);

        // Mémoriser pour le prochain step
        lastVertical = vertical;
        lastHorizontal = horizontal;
        lastRotate = rotate;
        lastAscend = ascend;


        // Récompense basée sur la variation de distance à la cible (locale)
        // float currentDistance = Vector3.Distance(transform.localPosition, target.localPosition);
        // float delta = previousDistanceToTarget - currentDistance;
        // float proximityReward = (1f - Mathf.Pow(currentDistance, 2) / 40000);

        // if (delta < 0f) AddReward(-0.1f);
        // else AddReward(0.1f);
        stepCount++;
        // AddReward(-0.00002f*stepCount);
        AddReward(-0.1f);


        if (stepCount > maxStepTime)
        {
            Debug.Log("Temps écoulé");
            timeoutCount++;
            PrintEpisodeStats();
            EndEpisode();
        }


        // previousDistanceToTarget = currentDistance;
        float speed = rb.linearVelocity.magnitude;
        speedSum += speed;
        maxSpeed = Mathf.Max(maxSpeed, speed);
        speedCount++;

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
            Debug.Log("Target touché");
            AddReward(+100f);
            successCount++;
            PrintEpisodeStats();
            EndEpisode();
            return;
        }

        if (other.CompareTag("Obstacle"))
        {
            Debug.Log("Obstacle touché");
            AddReward(-300);
            obstacleCrashCount++;
            PrintEpisodeStats();
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

    private void PrintEpisodeStats()
    {
        totalEpisodes = successCount +  timeoutCount  + obstacleCrashCount;
        float successRate = 100f * successCount / totalEpisodes;
        float timeoutRate = 100f * timeoutCount / totalEpisodes;
        float obstacleRate = 100f * obstacleCrashCount / totalEpisodes;

        if (speedCount > 0)
        {
            float averageSpeed = speedSum / speedCount;
            float averageKmH = averageSpeed * 3.6f;
            float maxKmH = maxSpeed * 3.6f;
            Debug.Log($"[Agent {totalEpisodes}] Vitesse moyenne: {averageKmH:F2} km/h | Vitesse max: {maxKmH:F2} km/h");
        }

        Debug.Log($"[Stats] Succès: {successRate:F1}%, Temps écoulé: {timeoutRate:F1}%, Collision: {obstacleRate:F1}% (sur {totalEpisodes} épisodes)");
    }


    private void OnDrawGizmosSelected()
    {
        if (target != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawLine(transform.position, target.position); // Pas besoin de changer ici car c’est visuel
        }

        if (obstacleMask == 0) return;

        Vector3 origin = transform.TransformPoint(Vector3.zero);

        for (int i = 0; i < verticalRays; i++)
        {
            float verticalT = (float)i / (verticalRays - 1);
            float verticalAngle = Mathf.Lerp(-90f, 90f, verticalT);

            if (Mathf.Abs(verticalAngle) > 89f)
            {
                Quaternion rotation = Quaternion.Euler(verticalAngle, 0f, 0f);
                Vector3 dir = transform.localRotation * (rotation * Vector3.forward);

                if (Physics.Raycast(origin, dir, out RaycastHit hitTarget, rayLength, targetMask))
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawRay(origin, dir * rayLength);
                }
                else if (Physics.Raycast(origin, dir, out RaycastHit hit, rayLength, obstacleMask))
                {
                    Gizmos.color = Color.Lerp(Color.red, Color.yellow, 0.5f);
                    Gizmos.DrawLine(origin, hit.point);
                    Gizmos.DrawSphere(hit.point, 0.2f);
                }
                else
                {
                    Gizmos.color = Color.cyan;
                    Gizmos.DrawRay(origin, dir * rayLength);
                }
            }
            else
            {
                for (int j = 0; j < horizontalRays; j++)
                {
                    float horizontalAngle = 360f * j / horizontalRays;
                    Quaternion rotation = Quaternion.Euler(verticalAngle, horizontalAngle, 0f);
                    Vector3 dir = transform.localRotation * (rotation * Vector3.forward);

                    if (Physics.Raycast(origin, dir, out RaycastHit hitTarget, rayLength, targetMask))
                    {
                        Gizmos.color = Color.cyan;
                        Gizmos.DrawRay(origin, dir * rayLength);
                    }
                    else if (Physics.Raycast(origin, dir, out RaycastHit hit, rayLength, obstacleMask))
                    {
                        Gizmos.color = Color.Lerp(Color.red, Color.yellow, 0.5f);
                        Gizmos.DrawLine(origin, hit.point);
                        Gizmos.DrawSphere(hit.point, 0.2f);
                    }
                    else
                    {
                        Gizmos.color = Color.cyan;
                        Gizmos.DrawRay(origin, dir * rayLength);
                    }
                }
            }
        }
    }




}
