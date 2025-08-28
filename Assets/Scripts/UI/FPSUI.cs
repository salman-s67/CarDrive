using TMPro;
using UnityEngine;

public class FPSUI : MonoBehaviour
{
    TextMeshProUGUI txtFPS;

    private void Awake()
    {
        txtFPS = transform.Find("FPSText").GetComponent<TextMeshProUGUI>();
    }

    void Update()
    {
        if (txtFPS != null)
        {
            if (Time.frameCount % 30 == 0)
            {
                txtFPS.text = (1 / Time.deltaTime).ToString("n0");
            }
        }
    }
}
