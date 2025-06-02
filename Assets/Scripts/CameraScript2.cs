using UnityEngine;

public class CameraScript2 : MonoBehaviour
{
    [Header("Références")]
    public Transform drone;
    public Transform target;

    [Header("Paramètres de suivi")]
    public float distanceBehind = 10f;
    public float heightAbove = 3f;
    public float followSmoothness = 5f;
    public float lookSmoothness = 5f;

    void LateUpdate()
    {
        if (drone == null || target == null) return;

        // Direction de la cible vers le drone
        Vector3 direction = (drone.position - target.position).normalized;

        // Position désirée : derrière le drone selon cette direction + un peu en hauteur
        Vector3 desiredPosition = drone.position + direction * distanceBehind + Vector3.up * heightAbove;

        // Lisser la position de la caméra
        transform.position = Vector3.Lerp(transform.position, desiredPosition, Time.deltaTime * followSmoothness);

        // Regarder vers la cible
        Vector3 lookDir = (target.position - transform.position).normalized;
        Quaternion targetRotation = Quaternion.LookRotation(lookDir, Vector3.up);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * lookSmoothness);
    }
}
