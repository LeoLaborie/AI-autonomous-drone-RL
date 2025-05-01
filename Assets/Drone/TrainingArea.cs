using System.Collections.Generic;
using UnityEngine;

public class TrainingArea : MonoBehaviour
{
    [Header("Références")]
    public Transform cible;
    public Transform droneEnnemi;
    public List<Transform> dronesDefensifs;
    public List<Transform> obstacles;

    [Header("Paramètres de la zone")]
    public Vector3 groundSize = new Vector3(10, 0, 10); 
    public float zoneHeight = 5f; // Hauteur max pour le drone attaquant

    private Vector3 zoneCenter;

    private void Awake()
    {
        zoneCenter = transform.position;
    }

    public void ResetScene()
    {
        // Choisir coin aléatoire pour la cible (sur le sol)
        Vector3 coinCible = GetCoinPosition(onGround: true);
        cible.position = coinCible;

        // Opposé du coin pour le drone ennemi
        Vector3 coinDrone = -coinCible;
        float height = Random.Range(1f, zoneHeight);
        droneEnnemi.position = zoneCenter + new Vector3(coinDrone.x, height, coinDrone.z);

        // Générer positions intermédiaires pour défenseurs et obstacles
        PlaceObjectsBetween(droneEnnemi.position, cible.position, dronesDefensifs, yFixed: false);
        PlaceObjectsScattered(obstacles, new Vector2(groundSize.x*10, groundSize.z*10), y: 0f);
    }

    private Vector3 GetCoinPosition(bool onGround)
    {
        float halfX = groundSize.x *10/ 2f; 
        float halfZ = groundSize.z *10/ 2f;

        float distanceFactor = 0.7f; // Plus proche du bord
        float x = Random.value > 0.5f ? halfX * distanceFactor : -halfX * distanceFactor;
        float z = Random.value > 0.5f ? halfZ * distanceFactor : -halfZ * distanceFactor;
        float y = onGround ? 0.5f : Random.Range(1f, zoneHeight*10); // 10 = facteur d'échelle pour la zone

        return zoneCenter + new Vector3(x, y, z);
    }

    private void PlaceObjectsBetween(Vector3 start, Vector3 end, List<Transform> objects, bool yFixed)
    {
        foreach (Transform obj in objects)
        {
            float t = Random.Range(0.2f, 0.8f); // Entre les deux extrémités
            Vector3 pos = Vector3.Lerp(start, end, t);

            // Jitter aléatoire pour éviter le même axe exact
            float jitterX = Random.Range(-1f, 1f);
            float jitterZ = Random.Range(-1f, 1f);
            float y = yFixed ? 0f : Random.Range(0.5f, 1.5f);

            obj.position = new Vector3(pos.x + jitterX, y, pos.z + jitterZ);
        }
    }

    private void PlaceObjectsScattered(List<Transform> objects, Vector2 areaSize, float y)
{
    foreach (Transform obj in objects)
    {
        float x = Random.Range(-areaSize.x / 2f, areaSize.x / 2f);
        float z = Random.Range(-areaSize.y / 2f, areaSize.y / 2f);
        obj.position = zoneCenter + new Vector3(x, y, z);

        // Appliquer une taille aléatoire
        float scale = Random.Range(1f, 5f); // Ajuste selon la taille de tes arbres
        obj.localScale = new Vector3(scale, scale, scale);
    }
}

}
