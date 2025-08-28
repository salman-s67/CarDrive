using UnityEngine;
using UnityEngine.InputSystem;

public class InputReader : MonoBehaviour
{
    public static InputReader Instance { get; private set; }

    public float SteerInput { get; private set; }
    public float AccelerationInput { get; private set; }
    public float HandbrakeInput { get; private set; }

    Controls controls;

    Vector2 moveInput = Vector2.zero;

    // this frame values
    float steerCur;
    float accelerationCur;
    float handbrakeCur;

    private void Awake()
    {
        Instance = this;

        controls = new Controls();
        controls.Player.Enable();
    }

    private void OnDestroy()
    {
        controls.Dispose();
    }

    private void Update()
    {
        GetMovement();
        GetHandbrake();
    }

    public void GetMovement()
    {
        moveInput = (controls.Player.Move.ReadValue<Vector2>()).normalized;

        steerCur = moveInput.x;
        accelerationCur = moveInput.y;

        AccelerationInput = Mathf.Abs(accelerationCur) > 0 ? Mathf.Lerp(AccelerationInput, accelerationCur, 15 * Time.deltaTime) : 0; // quickly ramp up
        SteerInput = Mathf.Abs(steerCur) > 0 ? Mathf.Lerp(SteerInput, steerCur, 15 * Time.deltaTime) : Mathf.Lerp(SteerInput, steerCur, 25 * Time.deltaTime); // return to neutral position faster
    }

    public void GetHandbrake()
    {
        HandbrakeInput = controls.Player.Handbrake.ReadValue<float>();
    }
}
