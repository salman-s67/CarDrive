using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName ="Transmission", menuName ="VehicleData/Transmission")]
public class TransmissionSO : ScriptableObject
{
    public string transmissionName;
    
    public List<float> gearRatios = new List<float>();
    public float differentialGearRatio;
    public float reverseGearRatio;
    public float timeToShiftGear;
}
