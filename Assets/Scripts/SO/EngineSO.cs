using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Engine", menuName = "VehicleData/Engine")]
public class EngineSO : ScriptableObject
{
    public string engineName;

    public AnimationCurve rpmTorqueCurve; // x-axis: engine RPM, y-axis: engine torque (Nm)
    public float minRPM;
    public float maxRPM;
}
