using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Suspension", menuName = "VehicleData/Suspension")]
public class SuspensionSO : ScriptableObject
{
    public string suspensionName;

    public float springStiffness;
    public float springRestLength;
    public float springTravel;

    public float damperStiffness; // (2 * sqrt(springStiffness * body.mass)) * zeta. 0.2 < zeta < 1
}
