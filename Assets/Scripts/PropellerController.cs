using UnityEngine;

public class PropellerController : MonoBehaviour
{
    [Header("螺旋桨引用")]
    public Transform propeller1;

    [Header("控制")]
    public bool isRotating = false;

    [Header("参数")]
    public float rotationSpeed = 1000f; // 度/秒
    private float currentAngle1 = 0f;
    private float currentAngle2 = 0f;
    void Update()
    {
        if (isRotating)
        {
            float rotationAmount = rotationSpeed * Time.deltaTime;
            currentAngle1 += rotationAmount;
            propeller1.localEulerAngles = new Vector3(currentAngle1, 0f, 0f);
        }
    }
}