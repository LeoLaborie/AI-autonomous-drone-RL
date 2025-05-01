using UnityEngine;

public static class InputManager
{
    public static float Vertical { get; private set; }
    public static float Horizontal { get; private set; }
    public static float Rotation { get; private set; }
    public static float Ascend { get; private set; }

    public static void SetInput(float vertical, float horizontal, float rotation, float ascend)
    {
        Vertical = vertical;
        Horizontal = horizontal;
        Rotation = rotation;
        Ascend = ascend;
    }

    // Optionnel : remettre à zéro à chaque frame si besoin
    public static void ResetInput()
    {
        Vertical = 0f;
        Horizontal = 0f;
        Rotation = 0f;
        Ascend = 0f;
    }
}
