using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class TractionUI : MonoBehaviour
{
    public static TractionUI Instance;

    [SerializeField] int textureSize = 256;

    Texture2D texture2D;
    int center;
    int radius;

    Transform tractionCircleImageParent;
    RawImage tractionCircle;

    Color[] textureColors;
    Color zeroColor = new Color(0, 0, 0, 0);
    int x = 0;

    public int TextureSize {  get { return textureSize; } }
    // 0,0 is the bottom left
    // (textureSize-1, textureSize-1) is the top right
    // (textureSize/2, textureSize/2) is the middle, which is the point for lateral acceleration = 0 and forward acceleration = 0
    // need to remap from incoming g values to textureSize. this is done by the caller

    private void Awake()
    {
        Instance = this;
        tractionCircleImageParent = transform.Find("TractionCircle").Find("RawImage");
        if(tractionCircleImageParent == null)
        {
            Debug.LogError("TractionUI - tractionCircleImageParent null");
        }

        tractionCircle = tractionCircleImageParent.GetComponent<RawImage>();
    }

    private void Start()
    {
        texture2D = new Texture2D(textureSize, textureSize);
        texture2D.wrapMode = TextureWrapMode.Clamp;
        tractionCircle.texture = texture2D;
        center = textureSize / 2;
        radius = Mathf.CeilToInt(textureSize / 2);

        textureColors = new Color[textureSize * textureSize];
        ClearTexture();
        //DrawGridLines();
        //DrawCircle();
    }

    void ClearTexture()
    {
        x = 0;
        for (int i = 0; i < textureSize; i++)
        {
            for (int j = 0; j < textureSize; j++)
            {
                textureColors[x] = zeroColor;
                x++;
            }
        }

        texture2D.SetPixels(textureColors);
        texture2D.Apply();
    }
    
    void DrawGridLines()
    {
        // draw x-axis
        int col = 0;
        int row = textureSize / 2;        
        for (col = 0; col < textureSize; col++)
        {
            SetValue(col, row, Color.white);
        }

        // draw y-axis
        col = textureSize / 2;
        for (row = 0; row < textureSize; row++)
        {
            SetValue(col, row, Color.white);
        }
    }

    void DrawCircle()
    {
        for (int deg = 0; deg < 360; deg++)
        {
            int x = Mathf.CeilToInt(0.5f * textureSize * Mathf.Cos(deg * Mathf.Deg2Rad) + 0.5f * textureSize);
            int y = Mathf.CeilToInt(0.5f * textureSize * Mathf.Sin(deg * Mathf.Deg2Rad) + 0.5f * textureSize);
            SetValue(x, y, Color.white);
        }
    }

    public void SetValue(int x, int y, Color color, bool checkBounds = false)
    {
        // if the coordinates are outside the circle, ignore this call.
        // determine this by calculating the length of the vector between the center and the test point.
        // if the length is more than the circle radius, reject the point
        if (checkBounds)
        {
            Vector2 vecCenterToPoint = new Vector2(x, y) - new Vector2(center, center);
            float magnitude = vecCenterToPoint.magnitude;
            if (magnitude > radius) return;
        }

        texture2D.SetPixel(x, y, color);
        texture2D.Apply();
    }
}
