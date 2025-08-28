using System.Collections.Generic;
using System.Text.RegularExpressions;
using Unity.Cinemachine;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    public static GameManager Instance;

    [Header("AI settings")]
    [SerializeField] int numAIVehicles = 0;
    [SerializeField] GameObject aiVehiclePrefabs;

    [Header("Essential Settings")]

    [SerializeField] GameObject canvasWrongWay;
    [SerializeField] GameObject playerVehiclePrefab;
    [SerializeField] GameObject trackPrefab;
    [SerializeField] LayerMask drivableLayer;
    [SerializeField] CinemachineCamera playerFollowCamera;
    [SerializeField] float deltaY = 0f;

    [Header("Tree Settings")]

    [SerializeField] bool instantiateTrees = false;
    [SerializeField] GameObject[] treePrefabs;
    [SerializeField] int treeCount = 10000;
    [SerializeField] float[] treeScale = new float[] { 0.8f, 1.2f };
    [SerializeField] int[] treeRotation = new int[] { 0, 360 };

    [Header("Vegetation Settings")]

    [SerializeField] bool instantiateVegetation = false;
    [SerializeField] GameObject[] vegetationPrefabs;
    [SerializeField] int vegetationCount = 20000;
    [SerializeField] float[] vegetationScale = new float[] { 0.8f, 1.2f };
    [SerializeField] int[] vegetationRotation = new int[] { 0, 360 };
    
    GameObject playerVehicle;

    GameObject trackInst;
    Track track;
    Transform terrain;
    Mesh terrainMesh;
    Vector3[] terrainVertices;

    Transform trackStartPoint;
    Transform trackStartLookAt;
    

    Vector3 startRotation = Vector3.zero;
    public Vector3 StartRotation { get { return startRotation; } }

    bool initDone = false;
    public bool InitDone { get { return initDone; } }

    GameState gameState = GameState.RaceCountdown;
    public GameState GameState { get { return gameState; } }

    float raceCountdownTimer = 1.5f;
    float t;

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            DontDestroyOnLoad(gameObject);
        }
        else
        {
            Destroy(this);
        }
    }

    void Start()
    {
        canvasWrongWay.SetActive(false);

        gameState = GameState.RaceCountdown;
        t = raceCountdownTimer;

        trackInst = Instantiate(trackPrefab);
        //track = trackInst.GetComponent<Track>();        

        terrain = trackInst.transform.Find("Terrain");
        terrainMesh = terrain.GetComponent<MeshFilter>().mesh;
        terrainVertices = terrainMesh.vertices;

        trackStartPoint = trackInst.transform.Find("StartPoint");
        trackStartLookAt = trackInst.transform.Find("StartLookAtPoint");

        if (trackStartPoint == null) Debug.LogError("StartPoint not found");
        if (trackStartLookAt == null) Debug.LogError("StartLookAtPoint not found");

        //SetupCheckpoints();

        playerVehicle = Instantiate(playerVehiclePrefab, trackStartPoint.transform.position + deltaY * Vector3.up, Quaternion.identity);
        playerVehicle.transform.LookAt(trackStartLookAt);
        startRotation = playerVehicle.transform.rotation.eulerAngles;
        //playerVehicle.GetComponent<Vehicle>().SetAI(true);

        SetTargetCamera();

        if (instantiateTrees) InstantiateTrees();
        if (instantiateVegetation) InstantiateVegetation();
        //InstantiateText();

        initDone = true;
    }

    public void SetTargetCamera()
    {
        playerFollowCamera.Target.TrackingTarget = playerVehicle.transform.Find("CameraTrackPoint");
    }

    void InstantiateTrees()
    {
        List<int> terrainVerticesUsed = new List<int>();

        int treesInstantiated = 0;

        while (treesInstantiated < treeCount) 
        { 
            bool addTree = true;

            // choose a random vertex on the terrain mesh as a potential point to instantiate a tree at
            int testVertex = UnityEngine.Random.Range(0, terrainVertices.Length - 1);
            
            // ensure that this vertex has not been used already.
            if (!terrainVerticesUsed.Contains(testVertex))
            {
                // check distance from this vertex to other tree-instantiated vertices to ensure that there are no overlaps
                foreach (int terrainVertex in terrainVerticesUsed)
                {
                    float distance = Vector3.Distance(terrainVertices[testVertex], terrainVertices[terrainVertex]);
                    //Debug.Log("distance from " + thisVert + " " + terrainVertices[thisVert] + " to " + vert + " " + terrainVertices[vert] + " is " + distance);
                    if (distance < 5)
                    {
                        //Debug.Log("breaking " +thisVert + "should not add");
                        addTree = false;
                        break;
                    }
                }

                if (addTree)
                {
                    //Debug.Log("adding " + thisVert);

                    float xPos = terrainVertices[testVertex].x;
                    float yPos = terrainVertices[testVertex].z - 1f; // ensure on slopes that the trunk goes into the ground
                    float zPos = -terrainVertices[testVertex].y;

                    Vector3 thisPos = new Vector3(xPos, yPos, zPos);

                    // raycast to detect if chosen position is on the drivable road
                    if (!Physics.Raycast(thisPos + new Vector3(0, 50f, 0), Vector3.down, 100f, drivableLayer))
                    {
                        //Debug.Log("tree " + treesInstantiated + " at " + thisPos+", orig vert data pos " + terrainVertices[thisVert]);
                        int idxTree = UnityEngine.Random.Range(0, treePrefabs.Length - 1);
                        GameObject thisTree = Instantiate(treePrefabs[idxTree], thisPos, Quaternion.identity);

                        float scale = UnityEngine.Random.Range(treeScale[0], treeScale[1]);
                        thisTree.transform.localScale = Vector3.one * scale;

                        int yRot = UnityEngine.Random.Range(treeRotation[0], treeRotation[1]);
                        thisTree.transform.rotation = Quaternion.Euler(new Vector3(thisTree.transform.localEulerAngles.x, yRot, thisTree.transform.localEulerAngles.z));

                        thisTree.transform.name += "_" + treesInstantiated;
                        thisTree.transform.parent = terrain;
                        terrainVerticesUsed.Add(testVertex);
                        treesInstantiated++;
                    }
                }
            }
        }
    }

    void InstantiateVegetation()
    {
        List<int> terrainVerticesUsed = new List<int>();

        int vegetationInstantiated = 0;

        while (vegetationInstantiated < vegetationCount)
        {
            bool addVegetation = true;

            // choose a random vertex on the terrain mesh as a potential point to instantiate vegetation at
            int testVertex = UnityEngine.Random.Range(0, terrainVertices.Length - 1);

            // ensure that this vertex has not been used already.
            if (!terrainVerticesUsed.Contains(testVertex))
            {
                // check distance from this vertex to other vegetation-instantiated vertices to ensure that there are no overlaps
                foreach (int terrainVertex in terrainVerticesUsed)
                {
                    float distance = Vector3.Distance(terrainVertices[testVertex], terrainVertices[terrainVertex]);
                    if (distance < 2)
                    {
                        addVegetation = false;
                        break;
                    }
                }

                if (addVegetation)
                {
                    float xPos = terrainVertices[testVertex].x;
                    float yPos = terrainVertices[testVertex].z - 0.5f; // ensure on slopes that the model's base goes into the ground
                    float zPos = -terrainVertices[testVertex].y;

                    Vector3 thisPos = new Vector3(xPos, yPos, zPos);

                    // raycast to detect if chosen position is on the drivable road
                    if (!Physics.Raycast(thisPos + new Vector3(0, 50f, 0), Vector3.down, 100f, drivableLayer))
                    {
                        int idxVegetation = UnityEngine.Random.Range(0, vegetationPrefabs.Length - 1);
                        GameObject thisVegetation = Instantiate(vegetationPrefabs[idxVegetation], thisPos, Quaternion.identity);

                        float scale = UnityEngine.Random.Range(vegetationScale[0], vegetationScale[1]);
                        thisVegetation.transform.localScale = Vector3.one * scale;

                        int yRot = UnityEngine.Random.Range(vegetationRotation[0], vegetationRotation[1]);
                        thisVegetation.transform.rotation = Quaternion.Euler(new Vector3(thisVegetation.transform.localEulerAngles.x, yRot, thisVegetation.transform.localEulerAngles.z));

                        thisVegetation.transform.name += "_" + vegetationInstantiated;
                        thisVegetation.transform.parent = terrain;
                        terrainVerticesUsed.Add(testVertex);
                        vegetationInstantiated++;
                    }
                }
            }
        }
    }

    private void Update()
    {
        if (!initDone) return;

        if (gameState == GameState.RaceCountdown)
        {
            if (t >= 0)
            {
                t -= Time.deltaTime;
            }
            else
            {
                gameState = GameState.Race;
                Debug.Log("GO!");
            }
        }
    }

    public void ShowCanvasWrongWay(bool show)
    {
        canvasWrongWay.SetActive(show);
    }

}
