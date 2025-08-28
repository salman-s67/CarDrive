using UnityEngine;

[CreateAssetMenu(fileName ="Tire", menuName ="VehicleData/Tire")]
public class TireSO : ScriptableObject
{
    public string tireName;

    public int tireMass; // kg
    public float coefficientOfFriction;
    public float tireGeometricRadius; // mm
    public float tireVerticalStiffness; // N/mm
    public float tireRollingResistance; // %, e.g. 2, not 0.02
}
