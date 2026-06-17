using UnityEngine;
using Cinemachine;

public class AircraftCameraController : MonoBehaviour
{
    [Header("相机引用")]
    [SerializeField] private CinemachineVirtualCamera thirdPersonCam;
    [SerializeField] private CinemachineVirtualCamera bellyViewCam;

    [Header("固定俯瞰相机")]
    [SerializeField] private CinemachineVirtualCamera overviewVirtualCam; // 如果用方案2

    [Header("主相机")]
    [SerializeField] private Camera mainCamera;
    [SerializeField] private CinemachineBrain brain;

    private int currentCamera = 1;

    void Start()
    {
        if (mainCamera == null)
            mainCamera = Camera.main;
        if (brain == null && mainCamera != null)
            brain = mainCamera.GetComponent<CinemachineBrain>();

        SetCamera(1);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1) && currentCamera != 1)
            SetCamera(1);
        else if (Input.GetKeyDown(KeyCode.Alpha2) && currentCamera != 2)
            SetCamera(2);
        else if (Input.GetKeyDown(KeyCode.Alpha3) && currentCamera != 3)
            SetCamera(3);
    }

    void SetCamera(int index)
    {
        currentCamera = index;


        // 如果使用方案2（Virtual Camera）
        if (overviewVirtualCam != null)
        {
            thirdPersonCam.Priority = index == 1 ? 10 : 0;
            bellyViewCam.Priority = index == 2 ? 10 : 0;
            overviewVirtualCam.Priority = index == 3 ? 10 : 0;
        }
    }

    void OnGUI()
    {
        string cameraName = currentCamera switch
        {
            1 => "第三人称视角",
            2 => "机腹视角",
            3 => "全局俯瞰（固定）",
            _ => "未知"
        };

        GUI.Label(new Rect(10, 180, 300, 20), $"当前视角: {cameraName}");
        GUI.Label(new Rect(10, 200, 300, 20), "按1/2/3切换视角");
    }
}