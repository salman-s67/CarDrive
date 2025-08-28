using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BasicResource : MonoBehaviour
{
    [SerializeField] Transform[] itemsForRandomColor;
    [SerializeField] Color[] randomColors;

    private void Start()
    {
        foreach (Transform item in itemsForRandomColor)
        {
            MeshRenderer mr = item.GetComponent<MeshRenderer>();
            if (mr != null)
            {
                if (randomColors.Length > 0)
                {
                    Color c = randomColors[Random.Range(0, randomColors.Length)];
                    mr.material.color = c;
                }
            }
        }
    }
}
