using UnityEngine;

public class CameraScript2 : MonoBehaviour
{
    [Header("Cibles à suivre")]
    public Transform drone;
    public Transform target;

    [Header("Paramètres de la caméra")]
    public float smoothSpeed = 5f;
    public float minDistance = 10f;
    public float maxDistance = 50f;
    public Vector3 offset = new Vector3(0, 10, -20);
    public float padding = 2f;

    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
    }

    void LateUpdate()
    {
        if (drone == null || target == null || cam == null)
            return;

        // Calcule le point central entre drone et cible
        Vector3 centerPoint = (drone.position + target.position) / 2f;

        // Distance entre drone et cible
        float distance = Vector3.Distance(drone.position, target.position);

        // Calcule une position en arrière et en hauteur par rapport au centre
        Vector3 desiredPosition = centerPoint + offset.normalized * Mathf.Clamp(distance + padding, minDistance, maxDistance);

        // Lisse le mouvement de la caméra
        transform.position = Vector3.Lerp(transform.position, desiredPosition, Time.deltaTime * smoothSpeed);

        // Regarde toujours le centre entre drone et cible
        transform.LookAt(centerPoint);
    }
}
