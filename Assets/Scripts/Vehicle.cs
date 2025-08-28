using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

[Serializable]
public class Sensor
{
    public Transform sensorPoint;
    public float sensorLength;
    public float weight;
    public float direction;
    public RaycastHit hit;
}

[Serializable]
public class Wheel
{
    public Axle axle;
    public WheelSide wheelSide;
    public WheelPos wheelPos;
    public Transform tr;
    public Transform mesh;
    public Transform rayPoint;
    public Transform tireSmokePoint;
    public float geometricRadius; // m
    public bool isGrounded;
    public bool isDrivenWheel;

    public Vector3 sphereCastDir = Vector3.zero;
    public Vector3 sphereCastOrigin = Vector3.zero;
    public float sphereCastRadius = 0f;
    public float sphereCastDist = 0f;
    public Vector3 sphereCastEndPoint = Vector3.zero;
    public Vector3 sphereCastHitCenter = Vector3.zero;
    public Vector3 suspensionForceDir;
    public RaycastHit raycastHit;

    public Vector3 forwardVelocity;
    public Vector3 sidewaysVelocity;
    public float forwardSlip;
    public float sideSlip;
    public float skidVal;
    public ParticleSystem tireSkidSmokeVFX;

    public Vector3 rotAxis; // axis for rotation when vehicle is moving
}

public class Vehicle : MonoBehaviour
{
    // static accessor

    public static Vehicle Instance;

    // events for less frequently changed items

    public delegate void OnGearChanged(int gear);
    public event OnGearChanged onGearChanged;

    public delegate void OnClutchChanged(bool clutchVal);
    public event OnClutchChanged onClutchChanged;

    // user configurable

    [Header("AI")]
    [SerializeField] bool isAI = false;
    public bool IsAI {  get { return isAI; } }
    [SerializeField] float aiSteeringSensitivity = 0.01f;
    [SerializeField] float angleThresholdForBraking = 20f;
    [SerializeField] float checkPointDistanceThreshold = 10f;
    [SerializeField] float scoutMaxDistanceFromVehicle = 40f;
    [SerializeField] float scoutSpeed = 150f;
    [SerializeField] float sensorLength = 30f;
    [SerializeField] string ignoreSensorTag;

    [Header("Layers")]
    [SerializeField] private LayerMask drivableSurface;
    [SerializeField] private LayerMask offTrackSurface;

    [Header("References")]
    [SerializeField] VehicleSO vehicleSO;
    [SerializeField] EngineSO engineSO;
    [SerializeField] TransmissionSO transmissionSO;
    [SerializeField] SuspensionSO suspensionSO;
    [SerializeField] TireSO tireFrontSO;
    [SerializeField] TireSO tireRearSO;    
    [SerializeField] float emissionStrengthWhenDisabled = 50000;
    [SerializeField] float emissionStrengthWhenEnabled = 500000;

    [Header("Settings")]
    [SerializeField][Range(0f, 1f)] float lateralForceMult = 1f; // for dampening sideways/turning forces
    [SerializeField] float downForce = 5f; // for stabilizing the car
    [SerializeField] float velocityDampingFactor = 1f;
    [SerializeField] float groundAngularDrag = 1f; // to slow down the rotation of the car
    [SerializeField] float airAngularDrag = 0.2f;
    [SerializeField] float forwardBodyTilt = 3f; // for fake body roll
    [SerializeField] float lateralBodyRoll = 3f; // for fake body roll
    [SerializeField][Range(0f, 1f)] float forwardAccelerationMultForTilt = 0.3f;
    [SerializeField][Range(0f, 1f)] float lateralAccelerationMultForRoll = 0.2f;
    [SerializeField][Range(0f, 1f)] float timeForBodyTiltAndRoll = 0.1f;
    [SerializeField][Range(0f, 3f)] float steeringReductionFactor = 2f; // at higher speeds, how fast should the exponential function decrease

    [SerializeField] float smallestValueForValidInput = 0.1f;
    [SerializeField] float velocityReductionToZeroLerpTime = 1.0f;
    [SerializeField] float slipRatioForSkidSFXVFX = 0.4f;

    [Header("Vehicle immunity settings")]
    [SerializeField] float vehicleTurnedOverAngle = 30f;
    [SerializeField] float vehicleImmunityTimerMax = 2.5f;
    [SerializeField] float vehicleImmunityBlinkTimerMax = 0.25f;
    [SerializeField] float vehicleImmunityBlinkColorMultiplier = 10f;

    [Header("Debug information")]
    [SerializeField] bool showDebug = false;

    // components

    Rigidbody rb;
    
    // general
    
    float gravityVal;
    float airDensitySeaLevel20c = 1.204f; // kg/m^3

    Transform bumperRearRayPoint;
    Transform bumperFrontRayPoint;

    List<Sensor> sensors = new List<Sensor>();

    // initialization
    bool initDone = false;

    // input
    float accelerationInput;
    float adjustedAccelerationInput = 0f;
    float steerInput;
    float handbrakeInput;

    // vehicle immunity
    bool vehicleHasImmunity = false;
    float vehicleImmunityTimer;
    float vehicleImmunityBlinkTimer;
    bool vehicleImmunityColorToggle = false;

    // vehicle stuck in a vertical pose
    float vehicleStuckTimer;
    float vehicleStuckTimerMax = 1f;
    int frontOrRearBumperStuckCount = 0;

    // vehicle
    bool respawning = false;
    bool vehicleTurnedOver = false;

    Transform bodyMeshParent;
    MeshRenderer bodyMeshRenderer;

    Transform wheelsParent;
    Wheel rearLeftWheel;
    Wheel rearRightWheel;
    Wheel frontLeftWheel;
    Wheel frontRightWheel;
    List<Wheel> wheels = new List<Wheel>();

    Transform rayPointsParent;
    Transform tireSmokePointsParent;

    Vector3 centerOfMass = Vector3.zero;
    Transform centerOfMass_Airborne;

    float totalSprungMass; // kg
    float sprungMassFront; // kg
    float sprungMassRear; // kg
    float sprungMassPerWheelFront; // kg
    float sprungMassPerWheelRear; // kg

    float totalUnsprungMass; // kg
    float unsprungMassFront; // kg
    float unsprungMassRear; // kg
    float unsprungMassPerWheelFront; // kg
    float unsprungMassPerWheelRear; // kg

    float vehicleMass; // kg

    float vehicleSprungWeight; // N
    float vehicleUnsprungWeight; // N
    float vehicleTotalWeight; // N

    float trackFront; // mm
    float trackRear; // mm
    float wheelBase; // mm
    float distanceLongitudinalFrontAxleToCenterOfSprungMass; // mm
    float distanceVerticalGroundToCenterOfSprungMass; // mm
    float groundClearance; // mm
    float chassisDynamicMovement; // mm
    float corneringGroundClearance; // mm
    float rollCenterVerticalDistanceFromGroundFront; // mm
    float rollCenterVerticalDistanceFromGroundRear; // mm
    float distanceVerticalCenterOfSprungMassToRollAxis; // mm

    float staticWheelLoadFrontAxle; // N
    float staticWheelLoadRearAxle; // N
    float staticWheelLoadFront; // N. each wheel
    float staticWheelLoadRear; // N. each wheel

    float massDistributionFront; // %
    float massDistributionRear; // %

    // speed and acceleration

    bool canDrive = true;
    bool canAccelerate = true;

    DrivenWheels drivenWheels = DrivenWheels.Rear;
    int numDrivenWheels = 0;
    int numWheelsFront;
    int numWheelsRear;

    float vehicleMaxAcceleration; // g
    float vehicleMaxDeceleration; // g
    float vehicleAcceleration;
    float multVehicleAccelerationOnIncline = 1f;
    float vehicleSpeed; // m/s
    Vector3 localVehicleVelocity;
    Vector3 prevLocalVehicleVelocity;
    float localForwardVelocity;
    float localLateralVelocity;
    float localForwardAccel; // g
    float localLateralAccel; // g
    float vehicleMaxCorneringForce; // g


    Vector3 prevVelocity;
    float maxVehicleSpeed = -1f; // m/s

    float torqueWheel;
    float tractionForceWheel;
    List<float> angularVelocityTires = new List<float>();
    float angularVelocityTireFront;
    float angularVelocityTireRear;

    List<float> maxAngularVelocityTires = new List<float>();
    float maxAngularVelocityTireFront;
    float maxAngularVelocityTireRear;

    float frictionCoefficient = 1f;

    // engine

    float angularVelocityEngine;
    float engineCurrentRPM;
    AnimationCurve engineRPMTorqueCurve;
    float engineMinRPM;
    float engineMaxRPM;
    float engineMaxTorque; // Nm

    // transmission

    bool shiftingGear = false;
    bool shiftingGearUp = false;

    int selectedGear = 1;
    float timeToShiftGear;

    bool clutchPressed = false;

    List<float> gearRatios = new List<float>();
    float selectedGearRatio;
    float differentialGearRatio;
    float reverseGearRatio;
    float totalGearRatio;

    bool vehicleMovingBackwards = false;
    bool reverseInput = false;
    bool comingOutOfReverse = false;
    bool reverseGearSet = false;

    // input

    bool accelerationInputAndReversing;
    bool brakeInputAndMovingForward;
    bool handbrakePressed;
    bool handbrakePressedAndStopped;
    bool handbrakePressedNoAccelerationInputVehicleStopped;

    bool vehicleSpeedZeroed = true;
    bool startedVehicleZeroVelocityCoroutine = false;

    // aerodynamics
    float dragCoefficient;
    float frontalArea; // m^2
    float aerodynamicDragForce; // N
    float reductionInAccelerationFromAerodynamicDragForce; // m/s^2

    // tire

    float coefficientOfFrictionFront;
    float coefficientOfFrictionRear;
    float geometricRadiusTireFront; // mm
    float geometricRadiusTireRear; // mm
    float loadedHeightTireFront; // mm
    float loadedHeightTireRear; // mm
    float rollingRadiusTireFront; // mm
    float rollingRadiusTireRear; // mm
    float verticalStiffnessTireFront; // N/mm
    float verticalStiffnessTireRear; // N/mm
    float rollingResistanceTireFront; // %
    float rollingResistanceTireRear; // %
    float reductionInAccelerationFromRollingResistance; // m/s^2

    // spring

    float springRateFront = 8.5f; // N/mm
    float springRateRear = 19.4f; // N/mm
    float distributionOfRollCoupleFront; // 0-1f
    float distributionOfRollCoupleRear; // 0-1f
    float motionRatio;
    float dampingRatio;

    // suspension

    float springStiffness;
    float springRestLength;
    float springTravel;
    float springMaxLength;
    float damperStiffness;
    float maxSpringDistance;

    int numGroundedWheels = 0;
    bool vehicleGrounded = false;

    int suspensionPasses = 0;
    int suspensionPassesMax = 10; // give the suspension function some time to stabilize before applying some other functions
    bool suspensionInitComplete = false;

    // steering

    float turningRadius; // mm
    float maxSteerAngle;

    float steerAngleFactor = 1f; // as the vehicle speed becomes higher, the steering angle reduces

    List<float> ackermanSteeringAngles = new List<float>();
    float ackermanSteerAngleFL = 0f;
    float ackermanSteerAngleFR = 0f;

    // ui interaction
    float skidTimerMax = 0.2f;
    float skidTimer = 0f;

    // ai
    GameObject targetCheckpoint;
    GameObject scout;
    GameObject scoutTargetCheckpoint;
    float scoutMaxDistanceFromVehicleStored;

    bool sensorObstacleDetected = false;
    float sensorTurnAmount = 0f;
    float sensorObstacleAngle = 0f;
    float sensorTurnMultiplier = 0f;

    ///////////////////////////////////////////////////////////////////////
    //
    // initialization functions
    //
    ///////////////////////////////////////////////////////////////////////

    public void SetAI(bool ai)
    {
        isAI = ai;
    }

    private void Awake()
    {
        Instance = this;

        rb = GetComponent<Rigidbody>();
        if (rb == null) Debug.LogError("Vehicle - rb is null");

        prevVelocity = Vector3.zero;

        //boxCollider = transform.Find("BodyCollider").GetComponent<BoxCollider>();
        //if (boxCollider == null) Debug.LogError("Vehicle - boxCollider is null");

        //carBody = transform.Find("CarBody");
        //if (carBody == null) Debug.LogError("Vehicle - carBody is null");

        bodyMeshParent = transform.Find("BodyMeshParent");
        if (bodyMeshParent == null) Debug.LogError("Vehicle - bodyMeshParent is null");

        bodyMeshRenderer = bodyMeshParent.Find("Mesh").GetComponent<MeshRenderer>();

        rayPointsParent = transform.Find("RayPoints");
        if (rayPointsParent == null) Debug.LogError("Vehicle - rayPointsParent is null");

        tireSmokePointsParent = transform.Find("TireSmokePoints");
        if (rayPointsParent == null) Debug.LogError("Vehicle - tireSmokePointsParent is null");

        centerOfMass_Airborne = transform.Find("CenterOfMass_Air");
        if (centerOfMass_Airborne == null) Debug.LogError("Vehicle - centerOfMass_Airborne is null");

        wheelsParent = transform.Find("Wheels");
        if (wheelsParent == null) Debug.LogError("Vehicle - wheelsParent is null");

        bumperFrontRayPoint = transform.Find("BumperRayPoints").Find("Front");
        if (bumperFrontRayPoint == null) Debug.LogError("Vehicle - bumperFrontRayPoint is null");

        bumperRearRayPoint = transform.Find("BumperRayPoints").Find("Rear");
        if (bumperRearRayPoint == null) Debug.LogError("Vehicle - bumperRearRayPoint is null");

        InitializeVehicle();

        InitializeWheels();

        maxVehicleSpeed = GetMaxVehicleSpeed(); // m/s
        frictionCoefficient = (tireFrontSO.coefficientOfFriction + tireRearSO.coefficientOfFriction) / 2f; // average

        InitializeSprings();
        SetVehicleMass();

        initDone = true;
    }

    void InitializeVehicle()
    {
        gravityVal = -Physics.gravity.y;

        ///////////////////////////////////////////////////////////
        //
        // vehicle
        //
        ///////////////////////////////////////////////////////////

        drivenWheels = vehicleSO.drivenWheels;
        numWheelsFront = vehicleSO.numWheelsFront;
        numWheelsRear = vehicleSO.numWheelsRear;

        totalSprungMass = vehicleSO.totalSprungMass;
        unsprungMassFront = vehicleSO.unsprungMassFront;
        unsprungMassRear = vehicleSO.unsprungMassRear;
        totalUnsprungMass = unsprungMassFront + unsprungMassRear;
        vehicleMass = totalSprungMass + totalUnsprungMass;

        vehicleSprungWeight = vehicleSO.totalSprungMass * gravityVal;
        vehicleUnsprungWeight = totalUnsprungMass * gravityVal;
        vehicleTotalWeight = vehicleSprungWeight + vehicleUnsprungWeight;

        trackFront = vehicleSO.trackFront;
        trackRear = vehicleSO.trackRear;
        wheelBase = vehicleSO.wheelBase;
        distanceLongitudinalFrontAxleToCenterOfSprungMass = vehicleSO.distanceLongitudinalFrontAxleToCenterOfSprungMass;
        distanceVerticalGroundToCenterOfSprungMass = vehicleSO.distanceVerticalGroundToCenterOfSprungMass;
        groundClearance = vehicleSO.groundClearance;
        chassisDynamicMovement = vehicleSO.chassisDynamicMovement;
        corneringGroundClearance = groundClearance - chassisDynamicMovement;
        rollCenterVerticalDistanceFromGroundFront = vehicleSO.rollCenterVerticalDistanceFromGroundFront;
        rollCenterVerticalDistanceFromGroundRear = vehicleSO.rollCenterVerticalDistanceFromGroundRear;
        distanceVerticalCenterOfSprungMassToRollAxis = distanceVerticalGroundToCenterOfSprungMass
                                                        - rollCenterVerticalDistanceFromGroundFront
                                                        - (distanceLongitudinalFrontAxleToCenterOfSprungMass *
                                                           (rollCenterVerticalDistanceFromGroundRear - rollCenterVerticalDistanceFromGroundFront) / wheelBase);

        staticWheelLoadRearAxle = gravityVal * (unsprungMassRear + (totalSprungMass * distanceLongitudinalFrontAxleToCenterOfSprungMass / wheelBase));
        staticWheelLoadFrontAxle = gravityVal * (unsprungMassRear + unsprungMassFront + totalSprungMass) - staticWheelLoadRearAxle;

        staticWheelLoadFront = staticWheelLoadFrontAxle / numWheelsFront;
        staticWheelLoadRear = staticWheelLoadRearAxle / numWheelsRear;

        massDistributionFront = staticWheelLoadFront / (staticWheelLoadFront + staticWheelLoadRear);
        massDistributionRear = staticWheelLoadRear / (staticWheelLoadFront + staticWheelLoadRear);

        sprungMassFront = totalSprungMass * massDistributionFront;
        sprungMassRear = totalSprungMass * massDistributionRear;

        sprungMassPerWheelFront = sprungMassFront / numWheelsFront;
        sprungMassPerWheelRear = sprungMassRear / numWheelsRear;

        unsprungMassPerWheelFront = unsprungMassFront / numWheelsFront;
        unsprungMassPerWheelRear = unsprungMassRear / numWheelsRear;

        ///////////////////////////////////////////////////////////
        //
        // aerodynamics
        //
        ///////////////////////////////////////////////////////////

        dragCoefficient = vehicleSO.dragCoefficient;
        frontalArea = vehicleSO.frontalArea;

        ///////////////////////////////////////////////////////////
        //
        // tire
        //
        ///////////////////////////////////////////////////////////

        coefficientOfFrictionFront = tireFrontSO.coefficientOfFriction;
        coefficientOfFrictionRear = tireRearSO.coefficientOfFriction;

        geometricRadiusTireFront = tireFrontSO.tireGeometricRadius;
        geometricRadiusTireRear = tireRearSO.tireGeometricRadius;

        loadedHeightTireFront = geometricRadiusTireFront - staticWheelLoadFront / springRateFront;
        loadedHeightTireRear = geometricRadiusTireRear - staticWheelLoadRear / springRateRear;

        rollingRadiusTireFront = (2f / 3f) * geometricRadiusTireFront + (1f / 3f) * loadedHeightTireFront;
        rollingRadiusTireRear = (2f / 3f) * geometricRadiusTireRear + (1f / 3f) * loadedHeightTireRear;

        verticalStiffnessTireFront = tireFrontSO.tireVerticalStiffness;
        verticalStiffnessTireRear = tireRearSO.tireVerticalStiffness;

        rollingResistanceTireFront = tireFrontSO.tireRollingResistance;
        rollingResistanceTireRear = tireRearSO.tireRollingResistance;

        ///////////////////////////////////////////////////////////
        //
        // spring
        //
        ///////////////////////////////////////////////////////////

        distributionOfRollCoupleFront = vehicleSO.distributionOfRollCoupleFront;
        distributionOfRollCoupleRear = vehicleSO.distributionOfRollCoupleRear;

        motionRatio = vehicleSO.motionRatio;
        dampingRatio = vehicleSO.dampingRatio;

        springStiffness = suspensionSO.springStiffness;
        springRestLength = suspensionSO.springRestLength;
        springTravel = suspensionSO.springTravel;
        damperStiffness = suspensionSO.damperStiffness;
        springMaxLength = springRestLength + springTravel;

        ///////////////////////////////////////////////////////////
        //
        // steering
        //
        ///////////////////////////////////////////////////////////

        maxSteerAngle = vehicleSO.maxSteerAngle;
        turningRadius = wheelBase / Mathf.Tan(maxSteerAngle / Mathf.Rad2Deg); // mm

        ///////////////////////////////////////////////////////////
        //
        // engine
        //
        ///////////////////////////////////////////////////////////

        engineRPMTorqueCurve = engineSO.rpmTorqueCurve;
        engineMinRPM = engineSO.minRPM;
        engineMaxRPM = engineSO.maxRPM;
        engineCurrentRPM = engineMinRPM;

        foreach (var key in engineRPMTorqueCurve.keys)
        {
            float thisTorque = engineRPMTorqueCurve.Evaluate(key.time);
            if (thisTorque > engineMaxTorque) engineMaxTorque = thisTorque;
        }

        ///////////////////////////////////////////////////////////
        //
        // transmission
        //
        ///////////////////////////////////////////////////////////

        gearRatios = transmissionSO.gearRatios;
        differentialGearRatio = transmissionSO.differentialGearRatio;
        reverseGearRatio = transmissionSO.reverseGearRatio;
        timeToShiftGear = transmissionSO.timeToShiftGear;

        SetGear(1);

    }

    void InitializeWheels()
    {
        drivenWheels = vehicleSO.drivenWheels;
        numDrivenWheels = GetNumDrivenWheels();

        foreach (Transform w in wheelsParent)
        {
            Wheel wheel = new Wheel();

            wheel.tr = w.transform;
            wheel.isGrounded = false;
            wheel.geometricRadius = GetTireGeometricRadius(wheel.axle) / 1000; // m
            wheel.suspensionForceDir = transform.up;
            wheel.raycastHit = new RaycastHit();
            wheel.mesh = w.transform.Find("Mesh");
            wheel.rayPoint = rayPointsParent.Find(w.name);
            wheel.tireSmokePoint = tireSmokePointsParent.Find(w.name);

            if (wheel.mesh == null) Debug.LogError("Vehicle - wheel.mesh is null for " + w.name);
            if (wheel.rayPoint == null) Debug.LogError("Vehicle - wheel.rayPoint is null for " + w.name);
            if (wheel.tireSmokePoint == null) Debug.LogError("Vehicle - wheel.tireSmokePoint is null for " + w.name);

            wheel.rayPoint.localPosition = new Vector3(wheel.tr.localPosition.x, 0, wheel.tr.localPosition.z);

            wheel.forwardVelocity = Vector3.zero;
            wheel.sidewaysVelocity = Vector3.zero;
            wheel.forwardSlip = 0f;
            wheel.sideSlip = 0f;
            
            wheel.tireSkidSmokeVFX = wheel.tireSmokePoint.Find("TireSmoke").GetComponent<ParticleSystem>();
            if (wheel.tireSkidSmokeVFX == null) Debug.LogError("Vehicle - wheel.tireSkidSmokeVFX is null for " + w.name);

            if (w.position.x < transform.position.x)
            {
                wheel.wheelSide = WheelSide.LEFT;
                wheel.rotAxis = Vector3.left;
            }
            else if (w.position.x > transform.position.x)
            {
                wheel.wheelSide = WheelSide.RIGHT;
                wheel.rotAxis = Vector3.right;
            }
            else
            {
                Debug.LogError("Undetermined wheel side for " + w.name);
            }

            if (w.position.z < transform.position.z)
            {
                wheel.axle = Axle.REAR;
                wheel.isDrivenWheel = drivenWheels == DrivenWheels.Rear || drivenWheels == DrivenWheels.All;
            }
            else if (w.position.z > transform.position.z)
            {
                wheel.axle = Axle.FRONT;
                wheel.isDrivenWheel = drivenWheels == DrivenWheels.Front || drivenWheels == DrivenWheels.All;
            }
            else
            {
                Debug.LogError("Undetermined wheel axle for " + w.name);
            }

            if (wheel.wheelSide == WheelSide.LEFT && wheel.axle == Axle.FRONT)
            {
                wheel.wheelPos = WheelPos.FRONTLEFT;
                frontLeftWheel = wheel;
            }
            else if (wheel.wheelSide == WheelSide.RIGHT && wheel.axle == Axle.FRONT)
            {
                wheel.wheelPos = WheelPos.FRONTRIGHT;
                frontRightWheel = wheel;
            }
            else if (wheel.wheelSide == WheelSide.LEFT && wheel.axle == Axle.REAR)
            {
                wheel.wheelPos = WheelPos.REARLEFT;
                rearLeftWheel = wheel;
            }
            else if (wheel.wheelSide == WheelSide.RIGHT && wheel.axle == Axle.REAR)
            {
                wheel.wheelPos = WheelPos.REARRIGHT;
                rearRightWheel = wheel;
            }
            else
            {
                Debug.LogError("Unable to set wheel for " + wheel.wheelSide + " " + wheel.axle);
            }

            wheels.Add(wheel);
        }
    }

    void InitializeSprings()
    {
        maxSpringDistance = Mathf.Abs(wheels[0].tr.localPosition.y - wheels[0].rayPoint.localPosition.y) + 0.1f + wheels[0].geometricRadius;
        //Debug.Log(wheels[0].tr.localPosition.y + " radius " + wheels[0].geometricRadius + " maxSpringDistance " + maxSpringDistance);
    }

    void SetVehicleMass()
    {
        foreach (Wheel w in wheels)
        {
            centerOfMass += w.rayPoint.localPosition;
        }
        centerOfMass /= 4f;

        rb.centerOfMass = centerOfMass;

        rb.mass = GetVehicleMass();
    }

    private void Start()
    {
        if (GameManager.Instance != null)
        {
            transform.rotation = Quaternion.Euler(GameManager.Instance.StartRotation);
            GameManager.Instance.SetTargetCamera();
        }

        foreach (Wheel w in wheels)
        {
            w.tireSkidSmokeVFX.Stop();
        }

        vehicleMaxAcceleration = GetVehicleMaximumAcceleration();
        vehicleMaxDeceleration = GetVehicleMaximumDeceleration();
        vehicleMaxCorneringForce = GetVehicleMaximumCorneringForce();

        maxAngularVelocityTires = GetTireMaxAngularVelocity(showDebug);

        maxAngularVelocityTireFront = maxAngularVelocityTires[0];
        maxAngularVelocityTireRear = maxAngularVelocityTires[1];

        if (isAI)
        {
            scoutMaxDistanceFromVehicleStored = scoutMaxDistanceFromVehicle;
            
            targetCheckpoint = Track.Instance.GetNextCheckpoint();
            scoutTargetCheckpoint = Track.Instance.GetScoutNextCheckpoint();
            
            scout = GameObject.CreatePrimitive(PrimitiveType.Capsule);
            DestroyImmediate(scout.GetComponent<Collider>());
            //tracker.GetComponent<MeshRenderer>().enabled = false;
            
            scout.transform.position = transform.position;
            scout.transform.rotation = transform.rotation;

            Transform sensorsParent = transform.Find("SensorRayPoints");
            foreach(Transform child in sensorsParent)
            {
                Sensor sensor = new Sensor();
                sensor.sensorPoint = child.transform;

                if (child.transform.name=="4" || child.transform.name=="5")
                {
                    sensor.sensorLength = 0.75f * sensorLength;
                }
                else
                {
                    sensor.sensorLength = sensorLength;
                }
                sensors.Add(sensor);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // AI cars have a scout 1 checkpoint ahead and this scout is what the
    // vehicle tracks
    //
    ///////////////////////////////////////////////////////////////////////

    void UpdateScoutMovement()
    {
        Debug.DrawLine(transform.position, scout.transform.position);

        // ensure that the tracker doesn't get too far ahead of the vehicle. slow down
        if (Vector3.Distance(transform.position, scout.transform.position) > scoutMaxDistanceFromVehicle)
        {
            scoutSpeed -= 3f;
            if (scoutSpeed < 20) scoutSpeed = 20;
            return;
        }

        // speed up the scout if the vehicle is catching up
        if (Vector3.Distance(transform.position, scout.transform.position) < scoutMaxDistanceFromVehicle / 2f)
        {
            scoutSpeed += 3f;
            if (scoutSpeed > 150) scoutSpeed = 150;
        }

        scout.transform.LookAt(Track.Instance.GetScoutNextCheckpoint().transform.position);
        scout.transform.Translate(0, 0, scoutSpeed * Time.deltaTime);

        if (Vector3.Distance(scout.transform.position, Track.Instance.GetScoutNextCheckpoint().transform.position) < 10)
        {
            Track.Instance.SetTrackerCheckpointPassed(scoutTargetCheckpoint);
            scoutTargetCheckpoint = Track.Instance.GetScoutNextCheckpoint();
        }
    }

    bool IsObstacleDetected()
    {
        for (int i = 0; i < sensors.Count; i++)
        {
            if (sensors[i].weight == 1)
            {
                return true;
            }
        }
        return false;
    }

    float SensorNetValue()
    {
        float sensorValue = 0;
        for (int i = 0; i < sensors.Count; i++)
        {
            sensorValue += sensors[i].weight * sensors[i].direction;
        }
        return sensorValue;
    }

    void ProcessSensors()
    {
        foreach (Sensor sensor in sensors)
        {
            if (sensor.sensorPoint.localPosition.x == 0)
            {
                sensor.direction = 0;
            }
            else
            {
                sensor.direction = Mathf.Sign(sensor.sensorPoint.localPosition.x); // -1 for left of center. +1 for right of center
            }

            if (Physics.Raycast(sensor.sensorPoint.position, sensor.sensorPoint.forward, out sensor.hit, sensor.sensorLength))
            {
                if (sensor.hit.collider.CompareTag(ignoreSensorTag) || sensor.hit.transform.name.Contains("Checkpoint"))
                {
                    sensor.weight = 0;
                }
                else
                {
                    sensor.weight = 1;
                    Debug.DrawLine(sensor.sensorPoint.position, sensor.hit.point, Color.red);
                }

            }
            else
            {
                sensor.weight = 0;
            }
        }

        sensorObstacleDetected = IsObstacleDetected();
        sensorTurnAmount = SensorNetValue();

        if (sensorTurnAmount == 0 && sensorObstacleDetected)
        {
            sensorObstacleAngle = Vector3.Dot(sensors[1].hit.normal, transform.right);

            if (sensorObstacleAngle > 0)
            {
                sensorTurnMultiplier = -1;
            }
            if (sensorObstacleAngle < 0)
            {
                sensorTurnMultiplier = 1;
            }
        }
        else
        {
            sensorTurnMultiplier = Mathf.Sign(sensorTurnAmount);
        }

        //Debug.Log("obstacle " + sensorObstacleDetected + " angle " + sensorObstacleAngle + " sensorTurn " + sensorTurnAmount + " turnMultiplier " + sensorTurnMultiplier);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // read player acceleration/brake/steer input in Update.
    // set acceleration/brake/steer values for AI vehicles.
    //
    ///////////////////////////////////////////////////////////////////////

    float accelerationZeroTimeMax = 1f;
    float accelerationZeroTime = 0f;

    private void Update()
    {
        if (InputReader.Instance == null) return; // not yet initialized
        if (GameManager.Instance == null) return; // not yet initialized
        if (!GameManager.Instance.InitDone) return; // not yet initialized

        if (GameManager.Instance.GameState == GameState.RaceCountdown)
        {
            canAccelerate = false;
            handbrakeInput = 1;
        }
        else
        {
            if (isAI)
            {
                UpdateScoutMovement();
                ProcessSensors();
            }

            canAccelerate = true;

            if (canDrive)
            {
                if (!isAI)
                {
                    accelerationInput = canAccelerate ? InputReader.Instance.AccelerationInput : 0;
                    steerInput = InputReader.Instance.SteerInput;
                    handbrakeInput = InputReader.Instance.HandbrakeInput;
                }
                else
                {
                    //Vector3 targetLocalSpace = transform.InverseTransformPoint(targetCheckpoint.transform.position); // car is 0,0,0 when getting local space
                    //float distanceToTarget = Vector3.Distance(targetCheckpoint.transform.position, transform.position);

                    Vector3 targetLocalSpace = transform.InverseTransformPoint(scout.transform.position); // car is 0,0,0 when getting local space
                    float targetAngle = Mathf.Atan2(targetLocalSpace.x, targetLocalSpace.z) * Mathf.Rad2Deg;

                    if(Mathf.Abs(targetAngle) > 30)
                    {
                        scoutMaxDistanceFromVehicle = scoutMaxDistanceFromVehicleStored / 1.3f;
                    }
                    else
                    {
                        scoutMaxDistanceFromVehicle = scoutMaxDistanceFromVehicleStored;
                    }

                    // normalize the target angle between 0-1, where 1 = 90 degrees. this will inform braking and accelerating percentage
                    float targetAngleClamped = Mathf.Clamp(Mathf.Abs(targetAngle), 0, 90);
                    float targetAngleNormalized = targetAngleClamped / (float)90f;

                    handbrakeInput = 0;

                    // steering clockwise/anti depends on the direction in which the car is facing with respect to the target
                    //float obstacleSteerMult = Mathf.Max(Mathf.Abs(sensorObstacleAngle) * 100 * 1.25f, 75f);
                    if (sensorObstacleDetected)
                    {
                        steerInput = Mathf.Clamp(-sensorTurnMultiplier * aiSteeringSensitivity * 60f, -1f, 1f);
                    }
                    else
                    {
                        steerInput = Mathf.Clamp(targetAngle * aiSteeringSensitivity, -1f, 1f) * Mathf.Sign(rb.linearVelocity.magnitude);
                    }

                    if (!canAccelerate)
                    {
                        accelerationInput = 0f;
                    }
                    else if (targetAngleClamped > angleThresholdForBraking && rb.linearVelocity.magnitude > 10) // brake
                    {
                        accelerationInput = Mathf.Lerp(0, -1, targetAngleNormalized);
                        accelerationInput *= 1.3f;
                    }
                    else // if we are on a straight segment, targetAngleNormalized will be 0, so full acceleration = 1
                    {                        
                        accelerationInput = Mathf.Lerp(0, 1, 1 - targetAngleNormalized);
                    }

                    // the vehicle is stuck. respawn
                    if (accelerationInput == 0f && canAccelerate)
                    {
                        accelerationZeroTime += Time.deltaTime;
                        if (accelerationZeroTime >= accelerationZeroTimeMax && !respawning)
                        {
                            accelerationZeroTime = 0f;
                            RespawnVehicle();
                        }
                    }
                    else
                    {
                        accelerationZeroTime = 0f;
                    }

                    float distanceToCheckpoint = Vector3.Distance(targetCheckpoint.transform.position, transform.position);
                    
                    //Debug.Log("target " + targetCheckpoint.name + " dist " + distanceToCheckpoint + " angle " + targetAngle + " normalized " + targetAngleNormalized + " s " + steerInput + " a " + accelerationInput);

                    if (distanceToCheckpoint < checkPointDistanceThreshold)
                    {
                        Track.Instance.SetCheckpointPassed(targetCheckpoint);
                        targetCheckpoint = Track.Instance.GetNextCheckpoint();
                    }
                }
            }
            else
            {
                handbrakeInput = 1;
            }
        }

        // when reverse throttle input is received, make the brake light glow brighter. don't light it up while reversing
        if (accelerationInput < -smallestValueForValidInput && selectedGear > 0) 
        {
            bodyMeshRenderer.material.SetColor("_EmissiveColor", Color.white * emissionStrengthWhenEnabled);
        }
        else
        {
            bodyMeshRenderer.material.SetColor("_EmissiveColor", Color.white * emissionStrengthWhenDisabled);
        }

        // when respawning, trigger immunity blinking
        if (respawning)
        {
            vehicleHasImmunity = true;
            vehicleImmunityTimer = vehicleImmunityTimerMax;
            vehicleImmunityBlinkTimer = vehicleImmunityBlinkTimerMax;
            respawning = false;
        }

        if (vehicleHasImmunity)
        {
            vehicleImmunityTimer -= Time.deltaTime;

            if (vehicleImmunityTimer <= 0f)
            {
                vehicleHasImmunity = false;
                bodyMeshRenderer.material.SetColor("_BaseColor", GameConsts.materialColorWhite);
            }
            else
            {
                vehicleImmunityBlinkTimer -= Time.deltaTime;
                if (vehicleImmunityBlinkTimer <= 0f)
                {
                    vehicleImmunityColorToggle = !vehicleImmunityColorToggle;
                    vehicleImmunityBlinkTimer = vehicleImmunityBlinkTimerMax;
                }

                if (vehicleImmunityColorToggle)
                {
                    bodyMeshRenderer.material.SetColor("_BaseColor", Color.Lerp(bodyMeshRenderer.material.GetColor("_BaseColor"), GameConsts.materialColorWhite * vehicleImmunityBlinkColorMultiplier, Time.deltaTime * 10f));
                }
                else
                {
                    bodyMeshRenderer.material.SetColor("_BaseColor", Color.Lerp(bodyMeshRenderer.material.GetColor("_BaseColor"), GameConsts.materialColorWhite * 1, Time.deltaTime * 10f));
                }
            }
        }

        // check if the vehicle is stuck vertically and needs to be respawned
        vehicleStuckTimer -= Time.deltaTime;
        if (vehicleStuckTimer <= 0f)
        {
            CheckVehicleStuckVertical();
            vehicleStuckTimer = vehicleStuckTimerMax;
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // shoot rays from the front and rear bumpers to detect if the vehicle
    // is stuck in vertically. respawn if needed
    //
    ///////////////////////////////////////////////////////////////////////

    void CheckVehicleStuckVertical()
    {
        Vector3 frontRayStart = bumperFrontRayPoint.transform.position;
        Vector3 frontRayDirection = bumperFrontRayPoint.transform.forward;

        Vector3 rearRayStart = bumperRearRayPoint.transform.position;
        Vector3 rearRayDirection = -bumperRearRayPoint.transform.forward;

        float rayDistance = 0.2f;

        if (Physics.Raycast(frontRayStart, frontRayDirection, rayDistance, drivableSurface, QueryTriggerInteraction.Ignore) ||
            Physics.Raycast(rearRayStart, rearRayDirection, rayDistance, drivableSurface, QueryTriggerInteraction.Ignore))
        {
            frontOrRearBumperStuckCount++;
        }
        else
        {
            frontOrRearBumperStuckCount = 0;
        }

        if (frontOrRearBumperStuckCount > 2)
        {
            RespawnVehicle();
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // vehicle physics in FixedUpdate
    //
    ///////////////////////////////////////////////////////////////////////

    void FixedUpdate()
    {
        if (!initDone) return;
        if (GameManager.Instance == null) return; // not yet initialized
        if (!GameManager.Instance.InitDone) return; // not yet initialized

        prevLocalVehicleVelocity = localVehicleVelocity;
        localVehicleVelocity = transform.InverseTransformDirection(rb.linearVelocity); // convert world space velocity to local space

        prevSpeed = vehicleSpeed;

        ComputeSideSlip();

        if (!isAI) DrawTractionCircle();

        // ackermann steering based on player horizontal input
        SetWheelSteerRotation();

        // modified acceleration on inclines
        AdjustAcceleration();

        // use sphere cast to apply suspension forces
        ApplySuspensionForce();

        if (!suspensionInitComplete && suspensionPasses < suspensionPassesMax)
        {
            suspensionPasses++;
            suspensionInitComplete = false;
        }
        else
        {
            suspensionInitComplete = true;
        }

        if (vehicleGrounded)
        {
            GetWheelTorque(showDebug);

            // wheel torque causes an acceleration
            vehicleAcceleration = GetVehicleAcceleration(adjustedAccelerationInput, handbrakeInput, showDebug);

            // the acceleration changes the vehicle speed
            SetVehicleSpeed();

            // based on speed changes, add fake body roll
            FakeVehicleBodyTiltAndRoll();

            // the vehicle speed gives tire angular velocity
            angularVelocityTires = GetTireAngularVelocity(showDebug);
            angularVelocityTireFront = angularVelocityTires[0];
            angularVelocityTireRear = angularVelocityTires[1];

            // update engine rpm for current tire rpm
            engineCurrentRPM = GetEngineRPMFromTireAngularVelocity(showDebug);

            // keep the car balanced
            if (rb.centerOfMass != centerOfMass)
            {
                rb.centerOfMass = centerOfMass;
            }

            rb.angularDamping = groundAngularDrag;

            // further stabilize the car
            AddDownForce();
        }
        else // not grounded
        {
            if (rb.centerOfMass != centerOfMass_Airborne.localPosition)
            {
                rb.centerOfMass = centerOfMass_Airborne.localPosition;
            }

            rb.angularDamping = airAngularDrag;
        }

        vehicleTurnedOver = IsVehicleTurnedOver();
        AddLateralForce();
        
        GearChangeCheck(showDebug);
        PlayTireSkidSFXVFX();

        //if (respawning) respawning = false;

        LogVehicleMetrics(showDebug);
        distTraveled += 0.5f * (prevSpeed + vehicleSpeed) * Time.fixedDeltaTime;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // calculate forward and lateral acceleration
    // and also each wheel's sideways velocity as a fraction of its total v
    //
    ///////////////////////////////////////////////////////////////////////

    void ComputeSideSlip(bool printDebug = false)
    {
        // get each wheel's sideways velocity as a percentage of total velocity.
        // note that forwardSlip is calculated in a different function, as data is more easily accessible in that function.

        foreach (Wheel w in wheels)
        {
            w.forwardVelocity = w.tr.InverseTransformDirection(rb.GetPointVelocity(w.rayPoint.transform.position)).z * w.tr.forward;
            w.sidewaysVelocity = w.tr.InverseTransformDirection(rb.GetPointVelocity(w.rayPoint.transform.position)).x * w.tr.right;

            // forward velocity can be 0, which will cause div by 0, so clamp it
            w.sideSlip = w.sidewaysVelocity.magnitude / (w.sidewaysVelocity.magnitude + Mathf.Clamp(w.forwardVelocity.magnitude, 0.1f, w.forwardVelocity.magnitude));

            if (printDebug)
            {
                Debug.Log(w.tr.name + " fwdV " + w.forwardVelocity.magnitude + " latV " + w.sidewaysVelocity.magnitude + " slip " + w.sideSlip);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // play skid VFX/SFX based on accelerations and lateral velocity %
    //
    ///////////////////////////////////////////////////////////////////////

    void PlayTireSkidSFXVFX(bool printDebug = false)
    {
        bool playSFX = false;

        foreach (Wheel w in wheels)
        {
            if (!w.isGrounded)
            {
                w.tireSkidSmokeVFX.Stop();
                playSFX = false;
                continue;
            }

            float newSkidVal = (w.forwardSlip + w.sideSlip) / 2; // magic number to avoid continuous skidding
            w.skidVal = Mathf.MoveTowards(w.skidVal, newSkidVal, 0.05f);

            bool highForwardAccel = localForwardAccel > smallestValueForValidInput ? (localForwardAccel > slipRatioForSkidSFXVFX * vehicleMaxAcceleration) : false;
            bool highDeceleration = localForwardAccel < -smallestValueForValidInput ? (localForwardAccel < slipRatioForSkidSFXVFX * -vehicleMaxAcceleration) : false;
            bool highLateralAccel = Mathf.Abs(localLateralAccel) > slipRatioForSkidSFXVFX * vehicleMaxCorneringForce;
            bool highSkidVal = w.skidVal > slipRatioForSkidSFXVFX;

            if (highForwardAccel || highDeceleration || highLateralAccel || highSkidVal)
            {
                w.tireSkidSmokeVFX.Play();
                playSFX = true;
            }
            else
            {
                w.tireSkidSmokeVFX.Stop();
                playSFX = false;
            }

            if (printDebug)
            {
                Debug.Log(w.tr.name + " fwdAccel " + localForwardAccel + " maxAccel " + vehicleMaxAcceleration + " lateralAccel " + localLateralAccel + " maxCornerForce " + vehicleMaxCorneringForce + " highFwdAccel " + highForwardAccel + " " + " highDecel " + highDeceleration + " highLateralAccel " + highLateralAccel + " highSkid " + highSkidVal);
            }
        }

        if (playSFX)
        {

        }
        else
        {

        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // the traction circle provides accelerations in multiples of g's.
    // make the call after a few frames to improve performance
    //
    ///////////////////////////////////////////////////////////////////////

    void DrawTractionCircle()
    {
        skidTimer += Time.fixedDeltaTime;

        if (skidTimer > skidTimerMax &&
            (handbrakeInput > smallestValueForValidInput ||
             Mathf.Abs(steerInput) > smallestValueForValidInput ||
             Mathf.Abs(accelerationInput) > smallestValueForValidInput))
        {
            skidTimer = 0f;
            if (TractionUI.Instance != null)
            {
                int remapForwardAccel = Mathf.CeilToInt(GameUtils.RemapValue(-vehicleMaxDeceleration, vehicleMaxAcceleration, 0, TractionUI.Instance.TextureSize - 1, localForwardAccel));
                int remapLateralAccel = Mathf.CeilToInt(GameUtils.RemapValue(-vehicleMaxCorneringForce, vehicleMaxCorneringForce, 0, TractionUI.Instance.TextureSize - 1, localLateralAccel));

                Color dotColor = remapForwardAccel >= TractionUI.Instance.TextureSize / 2 ? Color.green : Color.red;

                TractionUI.Instance.SetValue(remapLateralAccel, remapForwardAccel, dotColor, true);
            }
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // adjust acceleration for non-zero incline
    //
    ///////////////////////////////////////////////////////////////////////

    void AdjustAcceleration()
    {
        // this is the actual intended behavior:

        // downhill
        // sin(30 deg) = 0.5
        // sin(10 deg) = 0.17
        // downhill makes accelerating easier: 1 + sin() => 1.5, 1.17
        // downhill makes braking more difficult: 1 - sin() => 0.5, 0.83

        // uphill
        // sin(-30 deg) = -0.5
        // sin(-10 deg) = -0.17
        // uphill makes accelerating more difficult: 1 + sin() => 0.5, 0.83
        // uphill makes braking easier: 1 - sin() => 1.5, 1.17

        // accelerating: 1 + sin(angle)
        // braking: 1 -sin(angle)

        // ---

        // BUT changed behavior is needed because the car is already slowing down when on an upward incline, perhaps due to Unity's internal physics calculations. negate this effect

        // downhill
        // sin(30 deg) = 0.5
        // sin(10 deg) = 0.17
        // downhill makes accelerating more difficult: 1 - sin() => 0.5, 0.83
        // downhill makes braking easier: 1 + sin() => 1.5, 1.17

        // uphill
        // sin(-30 deg) = -0.5
        // sin(-10 deg) = -0.17
        // uphill makes accelerating easier: 1 - sin() => 1.5, 1.17
        // uphill makes braking more difficult: 1 + sin() => 0.5, 0.83

        // accelerating: 1 + sin(angle)
        // braking: 1 -sin(angle)

        if (accelerationInput > 0f) // accelerating
        {
            multVehicleAccelerationOnIncline = 1 - Mathf.Sin(transform.localEulerAngles.x * Mathf.Deg2Rad);
        }
        else if (accelerationInput < 0f) // braking
        {
            multVehicleAccelerationOnIncline = 1 + Mathf.Sin(transform.localEulerAngles.x * Mathf.Deg2Rad);
        }
        else
        {
            multVehicleAccelerationOnIncline = 1f;
        }

        adjustedAccelerationInput = reverseGearSet ? accelerationInput : accelerationInput * multVehicleAccelerationOnIncline * 1.15f;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // set speed to 0 if it is near 0
    //
    ///////////////////////////////////////////////////////////////////////

    bool SetZeroVehicleSpeed(bool printDebug = false)
    {
        if (!suspensionInitComplete) return false;

        //rb.linearVelocity.sqrMagnitude > 0.1f &&
        bool speedZeroingNeeded = Mathf.Abs(vehicleSpeed) < 2f &&
                                  Mathf.Abs(vehicleSpeed) > 0.1f &&
                                  vehicleGrounded &&
                                  (Mathf.Abs(adjustedAccelerationInput) < smallestValueForValidInput || handbrakeInput > smallestValueForValidInput);

        speedZeroingNeeded |= vehicleTurnedOver || respawning;

        //bool speedLT2 = Mathf.Abs(vehicleSpeed) < 2f;
        //bool speedGT001 = Mathf.Abs(vehicleSpeed) > 0.001f;
        //bool rbLV = rb.linearVelocity.sqrMagnitude > 0.01f;
        //bool accel = Mathf.Abs(adjustedAccelerationInput) < smallestValueForValidInput;
        //bool hbrake = handbrakeInput > smallestValueForValidInput;

        bool accelInputRcvd = Mathf.Abs(adjustedAccelerationInput) >= smallestValueForValidInput;

        //Debug.Log("zeroing " + speedZeroingNeeded +
        //          " speed < 2 " + speedLT2 +
        //          " speed > 0.001 " + speedGT001 +
        //          " rbLV > 0 " + rbLV +
        //          " grounded " + isVehicleGrounded +
        //          " accelInput " + accel +
        //          " handbrakeInput " + hbrake);

        if (accelInputRcvd && startedVehicleZeroVelocityCoroutine)
        {
            StopCoroutine(SetZeroVehicleVelocityCoroutine(rb.linearVelocity, rb.angularVelocity, Vector3.zero, Vector3.zero, velocityReductionToZeroLerpTime));
            startedVehicleZeroVelocityCoroutine = false;
            return false;
        }
        else if (speedZeroingNeeded && !startedVehicleZeroVelocityCoroutine)
        {
            StartCoroutine(SetZeroVehicleVelocityCoroutine(rb.linearVelocity, rb.angularVelocity, Vector3.zero, Vector3.zero, velocityReductionToZeroLerpTime));
            startedVehicleZeroVelocityCoroutine = true;
            return true;
        }
        
        return false;
    }

    IEnumerator SetZeroVehicleVelocityCoroutine(Vector3 startingLinearVelocity, Vector3 startingAngularVelocity, Vector3 endingLinearVelocity, Vector3 endingAngularVelocity, float timeMax)
    {
        float t = 0f;
        float p = 0f;

        bool accelInputRcvd = Mathf.Abs(adjustedAccelerationInput) >= smallestValueForValidInput;

        while (t <= timeMax)
        {
            accelInputRcvd = Mathf.Abs(adjustedAccelerationInput) >= smallestValueForValidInput;
            if (accelInputRcvd)
            {
                yield break;
            }

            p = t / timeMax;

            rb.linearVelocity = Vector3.Lerp(startingLinearVelocity, endingLinearVelocity, p);
            rb.angularVelocity = Vector3.Lerp(startingAngularVelocity, endingAngularVelocity, p);

            t += Time.fixedDeltaTime;
            yield return null;
        }

        if (!accelInputRcvd)
        {
            rb.linearVelocity = endingLinearVelocity;
            rb.angularVelocity = endingAngularVelocity;
        }

        startedVehicleZeroVelocityCoroutine = false;
        
        yield return null;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // set common values for input and velocity
    //
    ///////////////////////////////////////////////////////////////////////

    void UpdateSpeedInputs()
    {
        localForwardVelocity = localVehicleVelocity.z;
        localLateralVelocity = localVehicleVelocity.x;

        // forward and lateral accelerations as multiples of g's
        localForwardAccel = Mathf.Abs(localVehicleVelocity.z) > smallestValueForValidInput ? ((localVehicleVelocity.z - prevLocalVehicleVelocity.z) / Time.fixedDeltaTime) / gravityVal : 0f;
        localLateralAccel = Mathf.Abs(localVehicleVelocity.x) > smallestValueForValidInput ? ((localVehicleVelocity.x - prevLocalVehicleVelocity.x) / Time.fixedDeltaTime) / gravityVal : 0f;

        accelerationInputAndReversing = adjustedAccelerationInput > 0 && localForwardVelocity < 0f;
        brakeInputAndMovingForward = adjustedAccelerationInput < 0 && localForwardVelocity > 0f;
        handbrakePressed = handbrakeInput > smallestValueForValidInput;// && Mathf.Abs(localForwardVelocity) > 0.1f;
        handbrakePressedAndStopped = handbrakePressed && Mathf.Abs(localForwardVelocity) < 2f;
        handbrakePressedNoAccelerationInputVehicleStopped = handbrakePressedAndStopped && Mathf.Abs(adjustedAccelerationInput) < smallestValueForValidInput;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // set speed based on acceleration. v = u + at
    //
    ///////////////////////////////////////////////////////////////////////

    void SetVehicleSpeed(bool printDebug = false)
    {
        UpdateSpeedInputs();

        //float deltaSpeed = 0f;

        // if the vehicle is stopped after pressing handbrake, don't add acceleration forces
        if (!handbrakePressedAndStopped && !handbrakePressedNoAccelerationInputVehicleStopped)
        {
            // process acceleration/deceleration if
            // handbrake is pressed and the vehicle is still moving OR
            // handbrake is not pressed and vehicle speed < max speed for this gear

            if (handbrakePressed || 
                accelerationInputAndReversing || 
                (!handbrakePressed && Mathf.Abs(localForwardVelocity) <= GetMaxVehicleSpeedForCurrentGear()))
            {
                // apply handbrake force to all wheels
                // apply driving force only to driven wheels
                float forcePerWheel = handbrakePressed ? vehicleAcceleration / (vehicleSO.numWheelsRear + vehicleSO.numWheelsFront) : vehicleAcceleration / numDrivenWheels;

                foreach (Wheel w in wheels)
                {
                    if (w.isGrounded)
                    {
                        if (handbrakePressed || brakeInputAndMovingForward)
                        {
                            Vector3 force = forcePerWheel * w.tr.forward;
                            rb.AddForceAtPosition(force, w.tr.position, ForceMode.Acceleration);
                            Debug.DrawLine(w.tr.position, w.tr.position + force, Color.magenta);
                        }
                        else if (w.isDrivenWheel)
                        {
                            Vector3 force = forcePerWheel * w.tr.forward;
                            rb.AddForceAtPosition(force, w.tr.position, ForceMode.Acceleration);
                            Debug.DrawLine(w.tr.position, w.tr.position + force, Color.green);
                        }
                    }
                }
            }
        }

        vehicleSpeed = localForwardVelocity;
        //SetVehicleSpeed(vehicleSpeed);

        vehicleMovingBackwards = vehicleSpeed < 0f;
        reverseInput = adjustedAccelerationInput < -smallestValueForValidInput;

        vehicleSpeedZeroed = SetZeroVehicleSpeed();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // EngineClusterUI will call these continuously. Using functions
    // instead of events because these are getting new values every
    // frame, and events have overhead
    //
    ///////////////////////////////////////////////////////////////////////

    public float GetVehicleSpeed()
    {
        return Mathf.Abs(vehicleSpeed) * GameConsts.msToKmph;
    }

    public float GetEngineRPM()
    {
        return engineCurrentRPM;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // - increase the gear when the engine is just below the max rpm
    // - downshift when the vehicle speed drops to a point where it is
    //   better to be in a lower gear (i.e. keep the rpm high)
    //
    ///////////////////////////////////////////////////////////////////////

    void GearChangeCheck(bool printDebug = false)
    {
        if (!shiftingGear)
        {
            if (vehicleMovingBackwards)
            {
                if (!reverseGearSet && reverseInput)
                {
                    StartCoroutine(ChangeGear(-1)); // set it to 0, assuming that it will be at 1 normally
                    reverseGearSet = true;
                }
            }
            else
            {
                comingOutOfReverse = reverseGearSet && vehicleSpeed >= 0f;

                if (comingOutOfReverse)
                {
                    StartCoroutine(ChangeGear(1));
                    reverseGearSet = false;
                }

                if (Mathf.Abs(engineCurrentRPM - engineSO.maxRPM) < 50f)
                {
                    if (GetSelectedGear() < transmissionSO.gearRatios.Count)
                    {
                        if (printDebug)
                        {
                            Debug.Log("gear up from " + GetSelectedGear() + " at " + vehicleSpeed * GameConsts.msToKmph);
                        }
                        StartCoroutine(ChangeGear(1));
                    }
                }

                if (ShouldDownShift())
                {
                    if (GetSelectedGear() > 1)
                    {
                        if (printDebug)
                        {
                            Debug.Log("gear down from " + GetSelectedGear() + " at " + vehicleSpeed * GameConsts.msToKmph);
                        }
                        StartCoroutine(ChangeGear(-1));
                    }
                }
            }
        }
    }

    IEnumerator ChangeGear(int gearChange)
    {
        shiftingGear = true;
        shiftingGearUp = gearChange > 0;

        float t = 0f;

        SetClutchPressed(true);
        onClutchChanged?.Invoke(true);

        while (t <= transmissionSO.timeToShiftGear)
        {
            t += Time.fixedDeltaTime;
            yield return null;
        }

        ShiftGear(gearChange);

        SetClutchPressed(false);
        onClutchChanged?.Invoke(false);

        onGearChanged?.Invoke(GetSelectedGear());
        shiftingGear = false;
        shiftingGearUp = false;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // if vehicle has turned over, slowly bring it back to 0 degrees.
    // note that turning over is along the z-axis
    //
    ///////////////////////////////////////////////////////////////////////

    bool IsVehicleTurnedOver()
    {
        float zRot = transform.rotation.eulerAngles.z;

        if (zRot > 180f)
        {
            zRot = zRot - 360;
        }
        else if (zRot < -180f)
        {
            zRot = 360 - zRot;
        }
       
        if (Mathf.Abs(zRot) > vehicleTurnedOverAngle)
        {
            transform.rotation = Quaternion.RotateTowards(transform.rotation, Quaternion.Euler(new Vector3(transform.rotation.eulerAngles.x, transform.rotation.eulerAngles.y, 0)), 5);
            
            if (!vehicleGrounded)
            {
                rb.angularVelocity = Vector3.zero; // to prevent spinning when airborne
            }
            
            return true;
        }

        return false;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // add down force to stabilize the vehicle
    //
    ///////////////////////////////////////////////////////////////////////

    void AddDownForce()
    {
        rb.AddForce(-transform.up * downForce * rb.mass);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // add lateral force to turn the vehicle
    //
    ///////////////////////////////////////////////////////////////////////

    void AddLateralForce(bool printDebug = false)
    {
        foreach (Wheel w in wheels)
        {
            if (!w.isGrounded) continue;

            Vector3 wheelVel = rb.GetPointVelocity(w.rayPoint.position); // wheel velocity in world coordinates
            wheelVel = w.tr.InverseTransformDirection(wheelVel); // convert wheel velocity to local coordinates
            wheelVel = wheelVel.x * w.tr.right; // get wheel velocity sideways component
            wheelVel = -Vector3.ProjectOnPlane(wheelVel, w.raycastHit.normal); // project on raycast hit normal (y-axis if flat surface)
            Vector3 wheelAccel = wheelVel / Time.fixedDeltaTime; // wheel acceleration. a = (v-u)/t
            
            Vector3 lateralForce = (rb.mass / wheels.Count) * wheelAccel * frictionCoefficient; // f = ma. divide vehicle mass equally for each wheel
            lateralForce *= lateralForceMult * steerAngleFactor; // reduce the force as the vehicle picks up speed

            rb.AddForceAtPosition(lateralForce, w.rayPoint.position);
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // reduce steering forces and visual rotation as speed increases
    //
    ///////////////////////////////////////////////////////////////////////

    void SetSteeringAngleReductionFactor()
    {
        steerAngleFactor = Mathf.Exp(-steeringReductionFactor * vehicleSpeed / maxVehicleSpeed);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // use ackermann formula to rotate the inner front wheel when turning
    //
    ///////////////////////////////////////////////////////////////////////

    void SetWheelSteerRotation()
    {
        if (steerInput > 0f) // turn right
        {
            ackermanSteeringAngles = GetAckermanSteeringAngles(TurnDirection.RIGHT, steerInput, showDebug);
        }
        else if (steerInput < 0f) // turn left
        {
            ackermanSteeringAngles = GetAckermanSteeringAngles(TurnDirection.LEFT, steerInput, showDebug);
        }
        else // 0
        {
            ackermanSteeringAngles = GetAckermanSteeringAngles(TurnDirection.NONE, steerInput, showDebug);
        }

        SetSteeringAngleReductionFactor();

        ackermanSteerAngleFL = ackermanSteeringAngles[0] * steerAngleFactor;
        ackermanSteerAngleFR = ackermanSteeringAngles[1] * steerAngleFactor;

        // visually turn the tires' parent transform (FL/FR). can't apply angles to the mesh, as the x-axis turning rotation is applied to the mesh.
        // this rotation also causes lateral force to be applied by rb in AddLateralForce()
        frontLeftWheel.tr.localRotation = Quaternion.Lerp(frontLeftWheel.tr.localRotation, Quaternion.Euler(0, ackermanSteerAngleFL, 0), 0.1f);
        frontRightWheel.tr.localRotation = Quaternion.Lerp(frontRightWheel.tr.localRotation, Quaternion.Euler(0, ackermanSteerAngleFR, 0), 0.1f);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // to save computation load, don't calculate roll and tilt forces.
    // apply rotations to fake the forces
    //
    ///////////////////////////////////////////////////////////////////////

    void FakeVehicleBodyTiltAndRoll()
    {
        Vector3 accelVec = (rb.linearVelocity - prevVelocity) / Time.fixedDeltaTime; // v = u + at => a = (v-u)/t
        accelVec = Vector3.ProjectOnPlane(accelVec, transform.up); // project the acceleration vector on local Y-axis. result is in world coordinates
        accelVec = transform.InverseTransformDirection(accelVec); // get vehicle local coordinates

        prevVelocity = rb.linearVelocity;

        // rotate along x axis to get longitudinal tilt
        // rotate along z axis to get lateral body roll

        float rotateX = Mathf.Clamp(-accelVec.z * forwardAccelerationMultForTilt, -forwardBodyTilt, forwardBodyTilt);
        if (shiftingGearUp)
        {
            rotateX = 0;
        }

        float rotateY = 0f;
        float rotateZ = Mathf.Clamp(accelVec.x * lateralAccelerationMultForRoll, -lateralBodyRoll, lateralBodyRoll);

        // rotate only the body mesh
        bodyMeshParent.localRotation = Quaternion.Lerp(bodyMeshParent.localRotation, Quaternion.Euler(rotateX, rotateY, rotateZ), timeForBodyTiltAndRoll);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // use sphere cast to apply suspension forces
    //
    ///////////////////////////////////////////////////////////////////////

    void ApplySuspensionForce(bool printDebug = false)
    {
        foreach (Wheel w in wheels)
        {
            RaycastHit hit;

            // rayPoint y = 0 during initialization
            // => spherecast origin is at y = +R, which is the tire radius
            // direction is downwards
            // maxSpringDistance = R + 0.1 + wheel_localPos.y
            // => spherecast endpoint y = 0.1 below wheel_localPos.y
            // i.e., the center point of the sphere for spherecast is at y = 0.1 below wheel_localPos.y
            // gizmos are being drawn to show that the actual spherecast at the tail end is 0.1 below the actual tire mesh

            w.sphereCastDir = -w.rayPoint.up; // downwards
            w.sphereCastOrigin = w.rayPoint.position + (transform.up * w.geometricRadius);
            w.sphereCastRadius = w.geometricRadius;
            w.sphereCastDist = maxSpringDistance;
            w.sphereCastEndPoint = w.sphereCastOrigin + w.sphereCastDist * w.sphereCastDir;
            w.sphereCastHitCenter = w.sphereCastOrigin + w.raycastHit.distance * w.sphereCastDir;

            if (Physics.SphereCast(w.sphereCastOrigin, w.sphereCastRadius, w.sphereCastDir, out hit, w.sphereCastDist, drivableSurface, QueryTriggerInteraction.Ignore))
            {
                w.isGrounded = true;

                // using the hit normal direction should have worked, but small variances in it cause the car to start sliding towards -x and -z
                // in the absence of any throttle input. on a flat plane, diffDir was varying between 1e-8 and 1e-13, although the expected
                // value was 0. small errors in floating point or force variances can result in this issue, therefore, use wheel's up direction
                // unless there is a large difference between the up and hit normal.
                Vector3 springDir = hit.normal;
                Vector3 diffDir = springDir - w.rayPoint.up;
                float diffDirSqrMag = diffDir.sqrMagnitude;
                Vector3 forceDir = (diffDirSqrMag <= 0.1f) ? w.rayPoint.up : springDir;

                w.suspensionForceDir = forceDir;
                w.raycastHit = hit;

                // the spring compensates in proportion to the current compression.
                // f = k(L0-L), where L0 is the rest length
                float springCompression = maxSpringDistance - hit.distance;

                // only apply a force if the spring is compressed
                if (springCompression > 0)
                {
                    float springForce = springStiffness * springCompression;

                    //Vector3 calcHitPoint = sphereCastOrigin + sphereCastDir * hit.distance; // this gives the center of the sphere for the hit point along the sphere surface
                    //Debug.Log(w.tr.name + " origin " + sphereCastOrigin + " hitD " + hit.distance + " calcHPoint " + calcHitPoint + " hitP " + hit.point + " compress " + springCompression);

                    // dampen the spring's movement to prevent infinite oscillation
                    Vector3 wheelVel = rb.GetPointVelocity(w.rayPoint.position); // world coordinate velocity of the wheel
                    float springVel = Vector3.Dot(forceDir, wheelVel); // wheel velocity along the hit normal direction
                    float dampForce = springVel * damperStiffness; // the higher the spring velocity, the higher the counter-force

                    float netForce = springForce - dampForce;

                    if (printDebug)
                    {
                        Debug.Log("Vehicle - ApplySuspension " + w.tr.name + " maxSpringDist " + maxSpringDistance + " hitD " + hit.distance + " r " + w.geometricRadius + " compress " + springCompression + " wheelWorldVelocity " + wheelVel + " relativeVelocity " + springVel + " damp " + dampForce + " sF " + springForce + " net " + netForce + " atPos " + w.rayPoint.position);
                    }

                    rb.AddForceAtPosition(netForce * forceDir, w.rayPoint.position);
                }
            }

            // if spherecast detects a collision with off track surface/ terrain, move the car to the last checkpoint that was passed
            else if (Physics.SphereCast(w.sphereCastOrigin, w.sphereCastRadius, w.sphereCastDir, out hit, w.sphereCastDist, offTrackSurface, QueryTriggerInteraction.Ignore))
            {
                RespawnVehicle();
            }
            else
            {
                w.isGrounded = false;
            }

            SetWheelPosition(w);
            SetWheelRotation(w);
        }

        SetVehicleGrounded();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // respawn the vehicle at the previously passed checkpoint, and 
    // rotate to face the next checkpoint
    //
    ///////////////////////////////////////////////////////////////////////
    
    void RespawnVehicle()
    {
        respawning = true;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        GameObject lastPassedCheckpoint = Track.Instance.GetLastPassedCheckpoint();
        GameObject nextCheckpoint = Track.Instance.GetNextCheckpoint();

        // the checkpoint rectangles' origin point is in the middle. get a point nearer to the road surface
        float halfColliderExtentY = lastPassedCheckpoint.GetComponent<Collider>().bounds.extents.y / 2f;
        Vector3 respawnPoint = lastPassedCheckpoint.transform.position - new Vector3(0, Mathf.Sign(lastPassedCheckpoint.transform.position.y) * halfColliderExtentY, 0);

        //Debug.Log("off track, setting to " + lastPassedCheckpoint.name + " pos " + lastPassedCheckpoint.transform.position + " half extent " + halfColliderExtentY + " respawn " + respawnPoint);

        transform.position = respawnPoint;

        Debug.Log("respawning look at " + nextCheckpoint.transform.name);
        transform.LookAt(nextCheckpoint.transform);

        SetGear(1);
        onGearChanged?.Invoke(GetSelectedGear());
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // check how many wheels are on the ground
    //
    ///////////////////////////////////////////////////////////////////////

    void SetVehicleGrounded()
    {
        numGroundedWheels = 0;

        foreach (Wheel w in wheels)
        {
            numGroundedWheels += w.isGrounded ? 1 : 0;
        }

        vehicleGrounded = numGroundedWheels > 1;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // wheel position and rotation
    //
    ///////////////////////////////////////////////////////////////////////

    void SetWheelPosition(Wheel w)
    {
        if (w.isGrounded)
        {
            //Vector3 wPos = w.rayPoint.localPosition + (transform.up * w.geometricRadius) + (w.sphereCastDir * w.raycastHit.distance);
            Vector3 wPos = w.rayPoint.localPosition + (Vector3.up * w.geometricRadius) - (Vector3.up * w.raycastHit.distance);

            // rayPoint.y = 0 (same as car body origin y)
            // the magic number of 0.12 will have to be chosen per car-tire pair based on what looks visually ok.
            // subtracting just the radius still makes the mesh intersect with the car's body mesh
            float wPosMaxY = w.rayPoint.localPosition.y - w.geometricRadius - vehicleSO.magicNumberForWheelPos;

            //Debug.Log(w.tr.name + " " + w.rayPoint.localPosition + " radius upwards " + Vector3.up * w.geometricRadius + " vectorup " + Vector3.up + " dist " + w.raycastHit.distance + " dir*dist " + Vector3.up * w.raycastHit.distance + " finalPos " + wPos + " maxY " + wPosMaxY);

            if (Mathf.Abs(wPos.y) > Mathf.Abs(wPosMaxY))
            {
                //Debug.Log(w.tr.name + " y > wPosMaxY " + wPosMaxY);
                wPos.y = wPosMaxY;
            }

            w.tr.localPosition = wPos;
        }
        else
        {
            //w.tr.localPosition = w.rayPoint.localPosition + (w.tr.up * w.geometricRadius) - w.tr.up * maxSpringDistance;
            w.tr.localPosition = w.rayPoint.localPosition + (Vector3.up * w.geometricRadius) - (Vector3.up * maxSpringDistance);
            //Debug.Log(w.tr.name + " not grounded " + w.rayPoint.localPosition + " radius upwards " + w.tr.up * w.geometricRadius + " springdown - " + w.tr.up *maxSpringDistance); 
        }
    }

    void SetWheelRotation(Wheel w)
    {
        // rotation for current speed
        float rotationFront = handbrakeInput > 0.1f ? 0 : (angularVelocityTireFront * Mathf.Rad2Deg) * Time.fixedDeltaTime;
        float rotationRear = handbrakeInput > 0.1f ? 0 : (angularVelocityTireRear * Mathf.Rad2Deg) * Time.fixedDeltaTime;

        // max rotation is for max vehicle speed
        float maxRotationFront = handbrakeInput > 0.1f ? 0 : (maxAngularVelocityTireFront * Mathf.Rad2Deg) * Time.fixedDeltaTime;
        float maxRotationRear = handbrakeInput > 0.1f ? 0 : (maxAngularVelocityTireRear * Mathf.Rad2Deg) * Time.fixedDeltaTime;

        // current rotation will pick up +/- automatically, but maxRotation is a static number calculated once, and has to get its sign set manually
        float curRotation = w.axle == Axle.FRONT ? rotationFront : rotationRear;
        float maxRotation = w.axle == Axle.FRONT ? Mathf.Sign(vehicleSpeed) * maxRotationFront : Mathf.Sign(vehicleSpeed) * maxRotationRear;

        // clamp visual mesh rotation to stop jerky movements. magic number used.
        float rotationVisual = Mathf.Clamp(curRotation, -30f, 30f);
        w.mesh.Rotate(w.rotAxis, rotationVisual);

        // to simulate tire slipping during acceleration, introduce a new variable slipRotation.
        // e.g. rotation = 30, maxRotation = 285
        // while coasting, forwardSlip is always 0
        // when the handbrake is pressed, Clamp((0-30)/285, -1,+1) = -0.1. Abs = 0.1
        // during acceleration, the function is Clamp((maxR - R)/maxR) = Clamp((285-R)/285). this is a much faster-decaying function than e^-rt

        float slipRotation = (handbrakeInput > 0.1f) ? 0 :
                             (Mathf.Abs(accelerationInput) > 0.1f) ? maxRotation :
                             curRotation;

        float forwardSlip = 0f;
        if (maxRotation != 0)
        {
            forwardSlip = Mathf.Abs(Mathf.Clamp((slipRotation - curRotation) / maxRotation, -1, 1));
        }
        w.forwardSlip = forwardSlip;

        //Debug.Log(w.tr.name + " forwardslip " + forwardSlip + " rad " + w.geometricRadius + " maxSpeed " + GetMaxVehicleSpeed() + " rot " + curRotation + " max " + maxRotation);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // verify 0-60 and distance metrics
    //
    ///////////////////////////////////////////////////////////////////////

    bool shown60 = false;
    bool shown100 = false;
    bool shown165 = false;
    bool shown200m = false;
    bool shown400m = false;
    float distTraveled = 0;
    float prevSpeed = 0f;
    float tVehicleMetricsCollection = 0;
    bool enVehicleMetricCollection = true;

    void LogVehicleMetrics(bool printDebug = false)
    {
        if (!enVehicleMetricCollection || !printDebug) return;

        tVehicleMetricsCollection += Time.fixedDeltaTime;
        if (distTraveled >= 201 && !shown200m)
        {
            Debug.Log("1/8 mile: t = " + tVehicleMetricsCollection);
            shown200m = true;
        }

        if (distTraveled >= 402 && !shown400m)
        {
            Debug.Log("1/4 mile: t = " + tVehicleMetricsCollection);
            shown400m = true;
        }

        if (GameUtils.ConvertVelocityToKmhFromMs(vehicleSpeed) >= 60f && !shown60)
        {
            Debug.Log("60kmh: t = " + tVehicleMetricsCollection);
            shown60 = true;
        }

        if (GameUtils.ConvertVelocityToKmhFromMs(vehicleSpeed) >= 100f && !shown100)
        {
            Debug.Log("100kmh/60mph: t = " + tVehicleMetricsCollection);
            shown100 = true;
        }

        if (GameUtils.ConvertVelocityToKmhFromMs(vehicleSpeed) >= 165f && !shown165)
        {
            Debug.Log("165kmh/100mph: t = " + tVehicleMetricsCollection);
            shown165 = true;
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // checkpoints
    //
    ///////////////////////////////////////////////////////////////////////

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.name.Contains("Checkpoint"))
        {
            if (isAI) return;

            if (Track.Instance != null)
            {
                Track.Instance.SetCheckpointPassed(other.gameObject);
            }
        }        
    }

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    //
    //
    // vehicle physics control
    //
    //
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    //
    // vehicle functions
    //
    ///////////////////////////////////////////////////////////

    //public float GetWheelTractionForce(bool brakePressed, bool printDebug = false)
    //{
    //    // when braking, all 4 wheels' brakes provide force
    //    // when accelerating, the force returned is per driven wheel

    //    tractionForceWheel = brakePressed ? coefficientOfFrictionRear * vehicleTotalWeight :
    //                                        coefficientOfFrictionRear * staticWheelLoadRear; // N

    //    if (printDebug)
    //    {
    //        Debug.Log("VehiclePhysicsController - coefficientOfFrictionRear " + coefficientOfFrictionRear + " staticWheelLoadRear " + staticWheelLoadRear + " tractionForceWheel " + tractionForceWheel + "N");
    //    }

    //    return tractionForceWheel;
    //}

    public float GetRollingRadiusOfDrivenWheels()
    {
        float rollingRadius = drivenWheels == DrivenWheels.Front ? rollingRadiusTireFront :
                              drivenWheels == DrivenWheels.Rear ? rollingRadiusTireRear : (rollingRadiusTireFront + rollingRadiusTireRear) / 2f; // mm

        rollingRadius /= 1000f; // m

        return rollingRadius;
    }
    public float GetGeometricRadiusOfDrivenWheels()
    {
        float geometricRadius = drivenWheels == DrivenWheels.Front ? geometricRadiusTireFront :
                              drivenWheels == DrivenWheels.Rear ? geometricRadiusTireRear : (geometricRadiusTireFront + geometricRadiusTireRear) / 2f; // mm

        geometricRadius /= 1000f; // m

        return geometricRadius;
    }

    public float GetWheelTractionForceFromWheelTorque(bool printDebug = false)
    {
        float tireRadius = GetGeometricRadiusOfDrivenWheels(); // GetRollingRadiusOfDrivenWheels();

        tractionForceWheel = torqueWheel / tireRadius; // N

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - torqueWheel " + torqueWheel + " tireRadius " + tireRadius + " tractionForceWheel " + tractionForceWheel + "N");
        }

        return tractionForceWheel;
    }

    public int GetNumDrivenWheels()
    {
        int numDrivenWheels = drivenWheels == DrivenWheels.Front ? numWheelsFront :
                                drivenWheels == DrivenWheels.Rear ? numWheelsRear : numWheelsFront + numWheelsRear;

        return numDrivenWheels;
    }

    public float GetWheelTorque(bool printDebug = false)
    {
        //float maxTorqueWheel = GetWheelTractionForce(brakePressed, printDebug) * (rollingRadiusTireRear / 1000); // Nm

        //int numDrivenWheels = GetNumDrivenWheels();
        //float torqueDivide = 1f / numDrivenWheels; // this function returns the per-wheel torque

        torqueWheel = totalGearRatio * GetEngineTorqueFromRPM(engineCurrentRPM);// * torqueDivide; // Nm. Per driven wheel

        //Debug.Log("VehiclePhysicsController - eRPM " + engineCurrentRPM + ", eTorque " + GetEngineTorqueFromRPM(engineCurrentRPM) + "Nm, wTorque " + torqueWheel + "Nm" + " maxTorque "+ maxTorqueWheel + " traction force "+ tractionForceWheel);
        //Debug.Log("speed " + vehicleSpeed*3.6f);

        //torqueWheel = Mathf.Min(torqueWheel, maxTorqueWheel);

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - eRPM " + engineCurrentRPM + ", eTorque " + GetEngineTorqueFromRPM(engineCurrentRPM) + "Nm, wTorque " + torqueWheel + "Nm");
        }

        return torqueWheel;
    }

    public float GetVehicleMaximumAcceleration()
    {
        float gearRatioFirstGear = gearRatios[0];
        float maxAcceleration = gearRatioFirstGear * differentialGearRatio * engineMaxTorque / ((vehicleMass * (geometricRadiusTireRear / 1000) * gravityVal));
        return maxAcceleration;
    }

    public float GetVehicleMaximumDeceleration()
    {
        float maxDeceleration = vehicleTotalWeight * (((coefficientOfFrictionFront + coefficientOfFrictionRear) / 2f) / vehicleMass) / gravityVal;
        return maxDeceleration;
    }

    public float GetVehicleMaximumCorneringForce()
    {
        float maxCorneringForce = vehicleTotalWeight * (((coefficientOfFrictionFront + coefficientOfFrictionRear) / 2f) / vehicleMass) / gravityVal;
        return maxCorneringForce;
    }

    public float GetVehicleAcceleration(float adjustedAccelerationInput, float handbrakeInput, bool printDebug = false)
    {
        float tireRadius = GetGeometricRadiusOfDrivenWheels(); // GetRollingRadiusOfDrivenWheels();

        //stationary tWheel = 1329, mass 1368, radius 0.342 => normalAcceleration = 2.84 vs desired 15-20 for snippy behavior

        float normalAcceleration = torqueWheel / (vehicleMass * tireRadius);
        //normalAcceleration *= 2.25f; // magic number for more arcade feel
        float brakeDeceleration = normalAcceleration;
        float handbrakeDeceleration = 1.5f * normalAcceleration;
        float coastDeceleration = 0.01f * totalGearRatio * normalAcceleration; // faster deceleration in lower gears

        bool handbrake = handbrakeInput > 0f;
        bool braking = adjustedAccelerationInput < 0f;
        bool accelerating = adjustedAccelerationInput > 0f;
        bool coasting = (clutchPressed || (!clutchPressed && !accelerating && !braking)) && (Mathf.Abs(vehicleSpeed) > 0f);

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - tWheel " + torqueWheel + " mass " + vehicleMass + " wRad " + tireRadius + " normalA " + normalAcceleration);
        }

        if (handbrake)
        {
            if (vehicleSpeed > 0.5f)
            {
                vehicleAcceleration = -handbrakeDeceleration;
            }
            else if (vehicleSpeed < -0.5f)
            {
                vehicleAcceleration = handbrakeDeceleration;
            }
        }
        else
        {
            if (!clutchPressed && accelerating)
            {
                vehicleAcceleration = normalAcceleration;
            }
            else if (braking)
            {
                vehicleAcceleration = brakeDeceleration;
            }

            vehicleAcceleration *= adjustedAccelerationInput;
        }

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - basic a " + vehicleAcceleration / gravityVal + "g, input value and incline factor " + adjustedAccelerationInput);
        }

        GetAerodynamicDragForce(printDebug);
        GetReductionInAccelerationFromAerodynamicDragForce(printDebug);
        GetReductionInAccelerationFromRollingResistance(printDebug);

        if (vehicleSpeed > 0.1f)
        {
            vehicleAcceleration -= reductionInAccelerationFromAerodynamicDragForce;
            vehicleAcceleration -= reductionInAccelerationFromRollingResistance;
        }
        else if (vehicleSpeed < -0.1f)
        {
            vehicleAcceleration += reductionInAccelerationFromAerodynamicDragForce;
            vehicleAcceleration += reductionInAccelerationFromRollingResistance;
        }

        if (coasting)
        {
            if (vehicleSpeed > 0.1f)
            {
                vehicleAcceleration -= coastDeceleration;
            }
            else if (vehicleSpeed < -0.1f)
            {
                vehicleAcceleration += coastDeceleration;
            }
        }

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - final a " + vehicleAcceleration / gravityVal + "g");
        }

        return vehicleAcceleration;
    }

    public float GetAerodynamicDragForce(bool printDebug = false)
    {
        aerodynamicDragForce = 0.5f * dragCoefficient * frontalArea * airDensitySeaLevel20c * vehicleSpeed * vehicleSpeed; // N

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - Cd " + dragCoefficient + " A " + frontalArea + " rho " + airDensitySeaLevel20c + " v " + vehicleSpeed + "m/s" + " Fd " + aerodynamicDragForce);
        }

        return aerodynamicDragForce;
    }

    public float GetReductionInAccelerationFromAerodynamicDragForce(bool printDebug = false)
    {
        reductionInAccelerationFromAerodynamicDragForce = aerodynamicDragForce / vehicleMass; // m/s^2

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - Fdrag " + aerodynamicDragForce + " a_Fdrag " + reductionInAccelerationFromAerodynamicDragForce / gravityVal + "g");
        }

        return reductionInAccelerationFromAerodynamicDragForce;
    }

    public float GetReductionInAccelerationFromRollingResistance(bool printDebug = false)
    {
        float rollingResistance = (rollingResistanceTireFront + rollingResistanceTireRear) / 2f; // average
        float rollingResistanceForce = vehicleSprungWeight * rollingResistance / 100f; // N
        reductionInAccelerationFromRollingResistance = rollingResistanceForce / vehicleMass;

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - Frolling " + rollingResistanceForce + " a_Frolling " + reductionInAccelerationFromRollingResistance / gravityVal + "g");
        }

        return reductionInAccelerationFromRollingResistance;
    }

    public float GetMaxVehicleSpeedForCurrentGear(bool printDebug = false)
    {
        float tireRadius = GetGeometricRadiusOfDrivenWheels(); // GetRollingRadiusOfDrivenWheels();

        float maxVehicleSpeed = (Mathf.PI / 30f) * tireRadius * (engineMaxRPM / totalGearRatio); // m/s
        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - r " + tireRadius + "m, vMax " + maxVehicleSpeed * GameConsts.msToKmph + "km/h for gear " + selectedGear);
        }

        return maxVehicleSpeed;
    }

    public float GetMaxVehicleSpeedForGear(int gear, bool printDebug = false)
    {
        if (gear > gearRatios.Count) return 0f;

        float tireRadius = GetGeometricRadiusOfDrivenWheels(); // GetRollingRadiusOfDrivenWheels();

        float thisGearRatio = gearRatios[gear - 1];
        float thisGearTotalRatio = thisGearRatio * differentialGearRatio;
        float maxVehicleSpeed = (Mathf.PI / 30f) * tireRadius * (engineMaxRPM / thisGearTotalRatio); // m/s

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - gear " + gear + " totalRatio " + thisGearTotalRatio + " vMax " + maxVehicleSpeed * GameConsts.msToKmph + "km/h");
        }

        return maxVehicleSpeed;
    }

    public float GetMaxVehicleSpeed(bool printDebug = false)
    {
        int maxGear = gearRatios.Count;

        float maxSpeed = GetMaxVehicleSpeedForGear(maxGear, printDebug);

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - maxGear " + maxGear + " maxSpeed " + maxSpeed * GameConsts.msToKmph + "km/h");
        }

        return maxSpeed;
    }

    float gearDownSpeedDelta = 2.8f; //10kmph

    public bool ShouldDownShift()
    {
        if (selectedGear > 1)
        {
            float maxVehicleSpeedForOneGearDown = GetMaxVehicleSpeedForGear(selectedGear - 1);

            if (vehicleSpeed < maxVehicleSpeedForOneGearDown - gearDownSpeedDelta) return true;

            return false;
        }
        return false;
    }

    public void SetVehicleSpeed(float vehicleSpeed)
    {
        this.vehicleSpeed = vehicleSpeed;
    }

    public void ComputeTireAngularVelocity(bool printDebug = false)
    {
        float tireRadiusFront = (geometricRadiusTireFront / 1000f); //(rollingRadiusTireFront / 1000f);
        float tireRadiusRear = (geometricRadiusTireRear / 1000f); //(rollingRadiusTireRear / 1000f)

        angularVelocityTireFront = vehicleSpeed / tireRadiusFront; // rad/s
        angularVelocityTireRear = vehicleSpeed / tireRadiusRear; // rad/s

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - wwr " + angularVelocityTireRear + " rad/s, wwf" + angularVelocityTireFront + " rad/s");
        }
    }

    public List<float> GetTireAngularVelocity(bool printDebug = false)
    {
        ComputeTireAngularVelocity();

        List<float> vels = new List<float>();

        vels.Add(angularVelocityTireFront);
        vels.Add(angularVelocityTireRear);

        return vels; // rad/s
    }

    public List<float> GetTireMaxAngularVelocity(bool printDebug = false)
    {
        float tireRadiusFront = (geometricRadiusTireFront / 1000f); //(rollingRadiusTireFront / 1000f);
        float tireRadiusRear = (geometricRadiusTireRear / 1000f); //(rollingRadiusTireRear / 1000f)

        float maxAngularVelocityTireFront = GetMaxVehicleSpeed() / tireRadiusFront; // rad/s
        float maxAngularVelocityTireRear = GetMaxVehicleSpeed() / tireRadiusRear; // rad/s

        List<float> vels = new List<float>();

        vels.Add(maxAngularVelocityTireFront);
        vels.Add(maxAngularVelocityTireRear);

        return vels; // rad/s
    }

    public float GetEngineRPMFromTireAngularVelocity(bool printDebug = false)
    {
        angularVelocityEngine = totalGearRatio * Mathf.Abs(angularVelocityTireRear);

        engineCurrentRPM = GameUtils.ConvertRadPerSecToRPM(angularVelocityEngine);

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - we " + angularVelocityEngine + "rad/s, eRPM " + engineCurrentRPM);
        }

        engineCurrentRPM = Mathf.Clamp(engineCurrentRPM, engineMinRPM, engineMaxRPM);

        return engineCurrentRPM;
    }

    public float GetEngineCurrentRPM()
    {
        return engineCurrentRPM;
    }

    public float GetVehicleMass()
    {
        return vehicleMass;
    }

    ///////////////////////////////////////////////////////////
    //
    // engine functions
    //
    ///////////////////////////////////////////////////////////

    public float GetEngineTorqueFromRPM(float engineRPM, bool printDebug = false)
    {
        // returns torque in Nm

        engineRPM = Mathf.Clamp(engineRPM, engineMinRPM, engineMaxRPM);

        float engineTorque = engineRPMTorqueCurve.Evaluate(engineRPM);

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - EngineRPM = " + engineRPM + ", EngineTorque = " + engineTorque + " Nm");
        }

        return engineTorque;
    }

    public float GetEngineHPFromRPM(float engineRPM, bool printDebug = false)
    {
        float engineTorque = GetEngineTorqueFromRPM(engineRPM); // Nm
        float engineHP = (engineTorque * engineRPM) * GameUtils.ConvertRadPerSecToRPMForHP();

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - EngineRPM = " + engineRPM + ", EngineTorque = " + engineTorque + "Nm, EngineHP = " + engineHP);
        }

        return engineHP;
    }

    ///////////////////////////////////////////////////////////
    //
    // transmission functions
    //
    ///////////////////////////////////////////////////////////

    public void SetGear(int selectedGear, bool printDebug = false)
    {
        if (selectedGear == GameConsts.reverseGear)
        {
            this.selectedGear = selectedGear;
            selectedGearRatio = reverseGearRatio;
        }
        else if (selectedGear <= gearRatios.Count)
        {
            this.selectedGear = selectedGear;
            selectedGearRatio = gearRatios[selectedGear - 1];
        }

        totalGearRatio = differentialGearRatio * selectedGearRatio;

        if (printDebug)
        {
            Debug.Log("\t\tVehiclePhysicsController - SelectedGear = " + selectedGear + ", SelectedGearRatio = " + selectedGearRatio + ", TotalGearRatio = " + totalGearRatio);
        }
    }

    public void ShiftGear(int gearShift, bool printDebug = false) // only +1 or -1 normally. When reversing, value coming in will be -5
    {
        if (gearShift == 1)
        {
            UpShiftGear(printDebug);
        }
        else if (gearShift == -1)
        {
            DownShiftGear(printDebug);
        }
        //else if (gearShift == -5)
        //{
        //    SetGear(-5, printDebug);
        //}
    }

    public void UpShiftGear(bool printDebug = false)
    {
        if (selectedGear < gearRatios.Count)
        {
            selectedGear++;
            SetGear(selectedGear, printDebug);
        }
    }

    public void DownShiftGear(bool printDebug = false)
    {
        if (selectedGear > GameConsts.reverseGear)
        {
            selectedGear--;
            SetGear(selectedGear, printDebug);
        }
    }

    public int GetSelectedGear()
    {
        return selectedGear;
    }

    public void SetClutchPressed(bool val)
    {
        clutchPressed = val;
    }

    ///////////////////////////////////////////////////////////
    //
    // tire functions
    //
    ///////////////////////////////////////////////////////////

    public float GetTireRollingRadius(Axle axle)
    {
        if (axle == Axle.FRONT)
        {
            return rollingRadiusTireFront; // mm
        }
        else
        {
            return rollingRadiusTireRear; // mm
        }
    }

    public float GetTireGeometricRadius(Axle axle)
    {
        if (axle == Axle.FRONT)
        {
            return geometricRadiusTireFront; // mm
        }
        else
        {
            return geometricRadiusTireRear; // mm
        }
    }

    ///////////////////////////////////////////////////////////
    //
    // steering functions
    //
    ///////////////////////////////////////////////////////////

    public List<float> GetAckermanSteeringAngles(TurnDirection turnDirection, float steerInput, bool printDebug = false)
    {
        // ackerman: larger angle for the inner tire when looking at the center of the turn radius

        List<float> ackermanAngles = new List<float>();

        if (turnDirection == TurnDirection.LEFT)
        {
            ackermanSteerAngleFL = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turningRadius - (trackRear / 2f))) * steerInput;
            ackermanSteerAngleFR = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turningRadius + (trackRear / 2f))) * steerInput;
        }
        else if (turnDirection == TurnDirection.RIGHT)
        {
            ackermanSteerAngleFL = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turningRadius + (trackRear / 2f))) * steerInput;
            ackermanSteerAngleFR = Mathf.Rad2Deg * Mathf.Atan(wheelBase / (turningRadius - (trackRear / 2f))) * steerInput;
        }
        else
        {
            ackermanSteerAngleFL = 0f;
            ackermanSteerAngleFR = 0f;
        }

        if (printDebug)
        {
            Debug.Log("VehiclePhysicsController - turnDirection " + turnDirection + " steerInput " + steerInput + " FL angle " + ackermanSteerAngleFL + " FR angle " + ackermanSteerAngleFR);
        }

        ackermanAngles.Add(ackermanSteerAngleFL);
        ackermanAngles.Add(ackermanSteerAngleFR);

        return ackermanAngles;
    }


    ///////////////////////////////////////////////////////////////////////
    //
    // debug editor gizmos
    //
    ///////////////////////////////////////////////////////////////////////

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        foreach (Wheel w in wheels)
        {
            // draw tire
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(w.tr.position, w.geometricRadius);

            // draw the raycast line
            Gizmos.color = Color.red;
            Gizmos.DrawLine(w.sphereCastOrigin, w.sphereCastOrigin + (w.sphereCastDir * w.sphereCastDist));

            // draw raycast start sphere
            Gizmos.DrawSphere(w.sphereCastOrigin, 0.02f);
            Gizmos.DrawWireSphere(w.sphereCastOrigin, w.sphereCastRadius);

            // draw raycast end sphere
            Gizmos.DrawSphere(w.sphereCastEndPoint, 0.02f);
            Gizmos.DrawWireSphere(w.sphereCastEndPoint, w.sphereCastRadius);

            // draw the raypoint
            Gizmos.color = Color.white;
            Gizmos.DrawSphere(w.rayPoint.position, 0.1f);

            //UnityEditor.Handles.color = Color.red;
            //UnityEditor.Handles.ArrowHandleCap(0, w.tr.position + transform.up * wheelRadius, w.tr.rotation * Quaternion.LookRotation(Vector3.up), springTravel, EventType.Repaint);

            // draw raycast hit sphere
            Gizmos.color = Color.yellow;
            if (w.isGrounded)
            {
                Gizmos.DrawSphere(w.sphereCastHitCenter, 0.02f);
                //Gizmos.DrawSphere(w.hit.point, 0.05f);
                Gizmos.DrawWireSphere(w.sphereCastHitCenter, w.sphereCastRadius);
            }

            Gizmos.color = Color.green;
            foreach (Sensor sensor in sensors)
            {
                if (sensor.weight == 0)
                {
                    Gizmos.DrawRay(sensor.sensorPoint.position, sensor.sensorPoint.forward * sensor.sensorLength);
                }
            }
        }
    }
#endif

}
