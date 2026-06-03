import cv2
from atag import Atag

tool = Atag()

# 不同标签的 t 值字典（焦距700 × 实际边长米）
t_dict = {
    1: 112,   # 假设ID1边长0.16米 → 700×0.16
    2: 84,    # ID2边长0.12米
    3: 70,    # ID3边长0.10米
    4: 56     # ID4边长0.08米
}

cap = cv2.VideoCapture(0)
print("正在运行，按 q 键退出...")

while True:
    ret, frame = cap.read()
    if not ret:
        print("摄像头没读到画面")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = tool.detect(gray)

    # 获取画面宽高（用于判断方位）
    h, w = frame.shape[:2]

    for tag in results:                     # 改为循环，处理每个标签
        tag_id = tag.tag_id
        if tag_id not in t_dict:
            continue                        # 未预设的标签跳过

        H = tag.homography
        t = t_dict[tag_id]
        distance = tool.get_distance(H, t)

        # 画边框
        corners = tag.corners.astype(int)
        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i+1)%4]), (0,255,0), 2)

        # ---------- 方位判断 ----------
        # 计算标签中心坐标（四个角点的平均值）
        cx = int(corners[:, 0].mean())
        cy = int(corners[:, 1].mean())

        # 判断左右：画面宽三等分
        if cx < w / 3:
            pos_x = "LEFT"
        elif cx > 2 * w / 3:
            pos_x = "RIGHT"
        else:
            pos_x = "CENTER"

        # 判断上下：画面高三等分
        if cy < h / 3:
            pos_y = "TOP"
        elif cy > 2 * h / 3:
            pos_y = "BOTTOM"
        else:
            pos_y = "MIDDLE"

        pos_text = f"{pos_x}-{pos_y}"
        # -------------------------------

        # 在标签中心画一个点，并显示信息
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.putText(frame, f"ID:{tag_id} {distance:.2f}m {pos_text}",
                    (cx - 60, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("AprilTag 检测", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()