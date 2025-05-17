using UnityEngine;

public class CameraFollowDrone : MonoBehaviour
{
    public Transform drone;

    void Update()
    {
        transform.LookAt(drone);
    }
}
