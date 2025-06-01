using UnityEngine;

public class CameraScript1 : MonoBehaviour
{
    public Transform drone;

    void Update()
    {
        transform.LookAt(drone);
    }
}
