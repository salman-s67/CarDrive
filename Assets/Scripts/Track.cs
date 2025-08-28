using System.Collections.Generic;
using System.Text.RegularExpressions;
using UnityEngine;

public class Track : MonoBehaviour
{
    public static Track Instance;

    public delegate void OnCheckpointPassed(int checkpointNum);
    public event OnCheckpointPassed onCheckpointPassed;

    List<GameObject> checkpoints = new List<GameObject>();

    int prevCheckpoint;
    GameObject lastPassedCheckPoint;
    GameObject nextCheckpoint;

    int trackerPrevCheckpoint;
    GameObject trackerLastPassedCheckPoint;
    GameObject trackerNextCheckpoint;

    private void Awake()
    {
        Instance = this;
        SetupCheckpoints();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // checkpoint initialization
    //
    ///////////////////////////////////////////////////////////////////////

    public void SetupCheckpoints()
    {
        foreach (Transform child in transform)
        {
            if (child.name.Contains("Checkpoint"))
            {
                child.gameObject.GetComponent<MeshRenderer>().enabled = false;
                checkpoints.Add(child.gameObject);
            }
        }

        nextCheckpoint = checkpoints[0];
        trackerNextCheckpoint = checkpoints[1];
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // basic public getters
    //
    ///////////////////////////////////////////////////////////////////////

    public int GetTotalCheckpointsInLevel()
    {
        return checkpoints.Count;
    }

    public GameObject GetLastPassedCheckpoint()
    {
        return lastPassedCheckPoint;
    }

    public GameObject GetNextCheckpoint()
    {
        return nextCheckpoint;
    }

    public GameObject GetScoutNextCheckpoint()
    {
        return trackerNextCheckpoint;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // given a checkpoint game object, get its number.
    // expected name style: Checkpoint.001
    //
    ///////////////////////////////////////////////////////////////////////

    int GetCheckpointNumber(GameObject checkpoint)
    {
        if (checkpoint == null) return -1;

        string[] regexResult = Regex.Split(checkpoint.name, @"(.*)(\.)(.*)");
        if (regexResult.Length > 0)
        {
            string strPrevCheckpointNum = regexResult[3];
            return int.Parse(strPrevCheckpointNum);
        }

        return -1;
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // keep track of the checkpoints being passed, and flag wrong direction
    //
    ///////////////////////////////////////////////////////////////////////

    public void SetCheckpointPassed(GameObject checkPoint)
    {
        prevCheckpoint = GetCheckpointNumber(lastPassedCheckPoint);

        lastPassedCheckPoint = checkPoint;

        int thisCheckpointNum = GetCheckpointNumber(lastPassedCheckPoint);
        string lastPassedCheckpointName = lastPassedCheckPoint.name;

        GameManager.Instance.ShowCanvasWrongWay(thisCheckpointNum < prevCheckpoint);

        int nextCheckpointNum = thisCheckpointNum + 1;
        string sNextCheckPoint = "";
        if (nextCheckpointNum < 10)
        {
            sNextCheckPoint = "Checkpoint.00" + nextCheckpointNum.ToString();
        }
        else if (nextCheckpointNum < 100)
        {
            sNextCheckPoint = "Checkpoint.0" + nextCheckpointNum.ToString();
        }
        else
        {
            sNextCheckPoint = "Checkpoint." + nextCheckpointNum.ToString();
        }

        Transform nextCheckPointTransform = transform.Find(sNextCheckPoint);
        if (nextCheckPointTransform != null)
        {
            nextCheckpoint = nextCheckPointTransform.gameObject;
        }

        onCheckpointPassed?.Invoke(thisCheckpointNum);
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // keep track of the checkpoints being passed by the tracker
    //
    ///////////////////////////////////////////////////////////////////////

    public void SetTrackerCheckpointPassed(GameObject checkPoint)
    {
        trackerPrevCheckpoint = GetCheckpointNumber(trackerLastPassedCheckPoint);

        trackerLastPassedCheckPoint = checkPoint;

        int trackerThisCheckpointNum = GetCheckpointNumber(trackerLastPassedCheckPoint);
        string trackerLastPassedCheckPointName = trackerLastPassedCheckPoint.name;

        int trackerNextCheckpointNum = trackerThisCheckpointNum + 1;
        string sTrackerNextCheckPoint = "";
        if (trackerNextCheckpointNum < 10)
        {
            sTrackerNextCheckPoint = "Checkpoint.00" + trackerNextCheckpointNum.ToString();
        }
        else if (trackerNextCheckpointNum < 100)
        {
            sTrackerNextCheckPoint = "Checkpoint.0" + trackerNextCheckpointNum.ToString();
        }
        else
        {
            sTrackerNextCheckPoint = "Checkpoint." + trackerNextCheckpointNum.ToString();
        }

        Transform trackerNextCheckPointTransform = transform.Find(sTrackerNextCheckPoint);
        if (trackerNextCheckPointTransform != null)
        {
            trackerNextCheckpoint = trackerNextCheckPointTransform.gameObject;
        }
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // draw lines between checkpoints
    //
    ///////////////////////////////////////////////////////////////////////

    private void OnDrawGizmos()
    {
        DrawGizmos(false);
    }

    private void OnDrawGizmosSelected()
    {
        DrawGizmos(true);
    }

    void DrawGizmos(bool selected)
    {
        if (selected == false) return;
        if (checkpoints.Count > 1)
        {
            Vector3 prev = checkpoints[0].transform.position;
            for (int i = 1; i < checkpoints.Count; i++)
            {
                Vector3 next = checkpoints[i].transform.position;
                Gizmos.DrawLine(prev, next);
                prev = next;
            }
        }
    }
}
