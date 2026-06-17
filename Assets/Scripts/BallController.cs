using UnityEngine;
public class BallController : MonoBehaviour
{
    [Header("运动设置")]
    [SerializeField] private float speed = 50f;
    [SerializeField] private bool useVelocity = true;
    [SerializeField] private bool useTransform = false;
    [Header("视觉反馈")]
    [SerializeField] private Color normalColor = Color.blue;
    [SerializeField] private Color hitColor = Color.yellow;
    [SerializeField] private float colorResetTime = 1f;
    private Rigidbody rb;
    private Renderer ballRenderer;
    private Material ballMaterial;
    private float colorTimer = 0f;
    private bool hasHit = false;
    private Vector3 initialPosition;
    void Start()
    {
        // 获取组件
        rb = GetComponent<Rigidbody>();
        ballRenderer = GetComponent<Renderer>();
        // 确保有Rigidbody
        if (rb == null)
        {
            Debug.LogError("Ball需要Rigidbody组件！");
            rb = gameObject.AddComponent<Rigidbody>();
        }
        // 创建材质实例
        ballMaterial = new Material(ballRenderer.material);
        ballRenderer.material = ballMaterial;
        ballMaterial.color = normalColor;
        // 记录初始位置
        initialPosition = transform.position;
        // 配置Rigidbody
        rb.useGravity = false;
        // 冻结Y轴和所有旋转
        rb.constraints = RigidbodyConstraints.FreezePositionY | RigidbodyConstraints.FreezeRotation;
        rb.collisionDetectionMode = CollisionDetectionMode.Continuous;
        // 设置初始速度 - 向右移动
        if (useVelocity && !useTransform)
        {
            rb.velocity = Vector3.right * speed; // 改为向右
            Debug.Log($"初始化完成 - 速度: {speed} m/s, 方向: 右(+X), 模式: Velocity");
        }
    }
    void Update()
    {
        // Transform方式移动 - 向右
        if (useTransform && !useVelocity)
        {
            transform.position += Vector3.right * speed * Time.deltaTime; // 改为向右
        }
        // 颜色重置计时器
        if (hasHit)
        {
            colorTimer -= Time.deltaTime;
            if (colorTimer <= 0)
            {
                ballMaterial.color = normalColor;
                hasHit = false;
            }
        }
        // 按键控制
        if (Input.GetKeyDown(KeyCode.R))
        {
            ResetBall();
        }
        if (Input.GetKeyDown(KeyCode.Space))
        {
            speed = speed == 50f ? 100f : 50f;
            Debug.Log($"[速度切换] 新速度: {speed} m/s");
            if (useVelocity)
            {
                rb.velocity = Vector3.right * speed; // 改为向右
            }
        }
        // 显示当前状态信息
        if (Input.GetKeyDown(KeyCode.I))
        {
            Debug.Log($"[状态信息] 位置: {transform.position}, 速度: {rb.velocity}, 速度大小: {rb.velocity.magnitude}");
        }
    }
    void FixedUpdate()
    {
        // 保持速度恒定 - 向右
        if (useVelocity && !useTransform && !hasHit)
        {
            // 只在X方向保持速度
            rb.velocity = new Vector3(speed, 0, 0); // 改为X轴
        }
    }
    private void OnCollisionEnter(Collision collision)
    {
        Debug.Log($"[碰撞检测] 碰到了: {collision.gameObject.name}");
        if (collision.gameObject.name == "Wall" || collision.gameObject.tag == "Wall")
        {
            Debug.Log($"[物理碰撞] 球撞墙了！速度: {speed} m/s, 模式: {(useVelocity ? "Velocity" : "Transform")}");
            Debug.Log($"[碰撞详情] 碰撞点: {collision.contacts[0].point}, 法线: {collision.contacts[0].normal}");
            // 改变颜色
            ballMaterial.color = hitColor;
            hasHit = true;
            colorTimer = colorResetTime;
            // 停止球
            if (useVelocity)
            {
                rb.velocity = Vector3.zero;
            }
        }
    }
    private void OnCollisionStay(Collision collision)
    {
        if (collision.gameObject.name == "Wall")
        {
            Debug.Log("[持续碰撞] 球正在接触墙");
        }
    }
    private void OnCollisionExit(Collision collision)
    {
        if (collision.gameObject.name == "Wall")
        {
            Debug.Log("[碰撞结束] 球离开了墙");
        }
    }
    void ResetBall()
    {
        transform.position = initialPosition;
        rb.velocity = Vector3.zero;
        if (useVelocity)
        {
            rb.velocity = Vector3.right * speed; // 改为向右
        }
        ballMaterial.color = normalColor;
        hasHit = false;
        Debug.Log($"[重置] 球已重置到位置: {initialPosition}");
    }
    void OnDrawGizmos()
    {
        if (Application.isPlaying && rb != null)
        {
            // 绘制速度方向
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, rb.velocity.normalized * 2f);
        }
    }
}