using UnityEngine;
using UnityEngine.UI;

public class ProgressBarUI : MonoBehaviour
{
    Slider progressSlider;

    bool subscribed = false;

    private void OnEnable()
    {
        if (Track.Instance != null)
        {
            Track.Instance.onCheckpointPassed += UpdateSlider;
            subscribed = true;
        }
    }

    private void OnDisable()
    {
        if (Track.Instance != null)
        {
            Track.Instance.onCheckpointPassed -= UpdateSlider;
        }
    }

    private void Awake()
    {
        progressSlider = transform.Find("ProgressSlider").GetComponent<Slider>();
    }

    private void Update()
    {
        if (!subscribed)
        {
            if (Track.Instance != null)
            {
                Track.Instance.onCheckpointPassed += UpdateSlider;
                subscribed = true;
            }
        }
    }

    void UpdateSlider(int checkpointNum)
    {
        if (progressSlider != null)
        {
            progressSlider.value = checkpointNum / (float)Track.Instance.GetTotalCheckpointsInLevel();
        }
    }


}
