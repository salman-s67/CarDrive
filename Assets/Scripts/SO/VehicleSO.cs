using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "Vehicle", menuName = "VehicleData/Vehicle")]
public class VehicleSO : ScriptableObject
{
    public string vehicleName;

    [Header("Wheels")]
    public DrivenWheels drivenWheels;
    public int numWheelsFront;
    public int numWheelsRear;

    [Header("Mass")]
    public float totalSprungMass; // kg
    public float unsprungMassFront; // kg
    public float unsprungMassRear; // kg

    [Header("Dimensions")]
    public float vehicleLength; // mm
    public float vehicleWidth; // mm
    public float vehicleHeight; // mm
    public float trackFront; // mm
    public float trackRear; // mm
    public float wheelBase; // mm
    public float distanceLongitudinalFrontAxleToCenterOfSprungMass; // mm
    public float distanceVerticalGroundToCenterOfSprungMass; // mm
    public float groundClearance; // mm
    public float chassisDynamicMovement; // mm
    public float rollCenterVerticalDistanceFromGroundFront; // mm
    public float rollCenterVerticalDistanceFromGroundRear; // mm

    [Header("Suspension")]
    [Range(0f, 1f)] public float distributionOfRollCoupleFront; // 0-100%
    [Range(0f, 1f)] public float distributionOfRollCoupleRear; // 0-100%
    public float motionRatio;
    public float dampingRatio;
    public float magicNumberForWheelPos;

    [Header("Steering")]
    public float maxSteerAngle; // deg

    [Header("Aerodynamics")]
    public float dragCoefficient;
    public float frontalArea; // m^2
}
