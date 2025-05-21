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
    public float zoneHeight = 100f;

    public void ResetScene()
    {
        // Coin local pour la cible
        Vector3 coinCible = GetCoinPosition(onGround: true);
        cible.localPosition = coinCible;

        // Opposé pour le drone ennemi, avec hauteur aléatoire
        Vector3 coinDrone = -coinCible;
        float height = Random.Range(5f, zoneHeight);
        droneEnnemi.localPosition = new Vector3(coinDrone.x, height, coinDrone.z);

        // Défenseurs entre drone et cible
        PlaceObjectsBetween(droneEnnemi.localPosition, cible.localPosition, dronesDefensifs, yFixed: false);

        // Obstacles dispersés sur le sol
        PlaceObjectsScatteredWithDistanceCheck(obstacles, new Vector2(groundSize.x * 10, groundSize.z * 10), y: 0f);

        // 🔁 Fin d'épisode pour chaque agent défenseur
        foreach (Transform def in dronesDefensifs)
        {
            if (def != null)
            {
                var agent = def.GetComponent<DefenderAgent>();
                if (agent != null)
                {
                    agent.EndEpisode();
                }
            }
        }
    }


    private Vector3 GetCoinPosition(bool onGround)
    {
        float halfX = groundSize.x * 5 ;
        float halfZ = groundSize.z * 5 ;

        float distanceFactor = 0.8f;
        float x = Random.value > 0.5f ? halfX * distanceFactor : -halfX * distanceFactor;
        float z = Random.value > 0.5f ? halfZ * distanceFactor : -halfZ * distanceFactor;
        float y = onGround ? 2f : Random.Range(1f, zoneHeight);

        return new Vector3(x, y, z);
    }

    private void PlaceObjectsBetween(Vector3 start, Vector3 end, List<Transform> objects, bool yFixed)
    {
        foreach (Transform obj in objects)
        {
            float t = Random.Range(0.7f, 0.9f); 
            Vector3 pos = Vector3.Lerp(start, end, t);

            float jitterX = Random.Range(-50f, 50f);
            float jitterZ = Random.Range(-50f, 50f);
            float y = yFixed ? 0f : Random.Range(5f, 25f);

            obj.localPosition = new Vector3(pos.x + jitterX, y, pos.z + jitterZ);
        }
    }

    private void PlaceObjectsScatteredWithDistanceCheck(List<Transform> objects, Vector2 areaSize, float y)
    {
        float minDistance = 2f;
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
                    y,
                    Random.Range(-areaSize.y / 2f, areaSize.y / 2f)
                );

                foreach (Transform other in objects)
                {
                    if (other != obj && Vector3.Distance(newPosition, other.localPosition) < minDistance)
                    {
                        positionValid = false;
                        break;
                    }
                }

                attempts++;
            } while (!positionValid && attempts < 100);

            obj.localPosition = newPosition;

            float scale = Random.Range(1f, 5f);
            obj.localScale = new Vector3(scale, scale, scale);
        }
    }
}
