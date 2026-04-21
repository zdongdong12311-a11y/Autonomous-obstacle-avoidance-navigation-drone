# coding=utf-8
import os, sys, cv2, time, queue, threading
import numpy as np
from rknnlite.api import RKNNLite
from collections import deque
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import logging
# 修改日志，防止 rknn 与 rospy 冲突
os.environ['ROSCONSOLE_CONFIG_FILE'] = '/opt/ros/noetic/share/ros/config/rosconsole.config' 
if 'DEBUG' not in logging._nameToLevel:
    logging.addLevelName(logging.DEBUG, 'DEBUG')
if 'INFO' not in logging._nameToLevel:
    logging.addLevelName(logging.INFO, 'INFO')

# ================ 全局参数 ================
MODEL_PATH = './model/yolov8n.rknn'
IMG_SIZE = (640, 640)
OBJ_THRESH = 0.25
NMS_THRESH = 0.45
CLASSES = ("person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush")

raw_frame_queue = queue.Queue(maxsize=3)
bridge = CvBridge()

def ros_image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        if raw_frame_queue.full():
            raw_frame_queue.get_nowait()
        raw_frame_queue.put(cv_image)
    except: pass

# ================ 后处理工具函数 (接回你之前的逻辑) ================
def dfl(pos):
    n, c, h, w = pos.shape
    pos = pos.reshape(n, 4, c // 4, h, w)
    softmax = np.exp(pos) / np.sum(np.exp(pos), axis=2, keepdims=True)
    acc = np.arange(c // 4, dtype=np.float32).reshape(1, 1, -1, 1, 1)
    return np.sum(softmax * acc, axis=2)

def box_process(pos):
    gh, gw = pos.shape[2:4]
    col, row = np.meshgrid(np.arange(gw), np.arange(gh))
    grid = np.stack((col, row), 0).reshape(1, 2, gh, gw)
    stride = np.array([IMG_SIZE[1] // gw, IMG_SIZE[0] // gh]).reshape(1, 2, 1, 1)
    pos = dfl(pos)
    xy1 = grid + 0.5 - pos[:, :2]
    xy2 = grid + 0.5 + pos[:, 2:4]
    return np.concatenate((xy1 * stride, xy2 * stride), 1)

def filter_and_nms(boxes, cconf, oconf):
    cls_max = cconf.max(-1)
    cls_ids = cconf.argmax(-1)
    mask = cls_max * oconf.reshape(-1) >= OBJ_THRESH
    if not mask.any():
        return None, None, None
    boxes, scores, cls_ids = boxes[mask], (cls_max * oconf.reshape(-1))[mask], cls_ids[mask]
    
    # 使用 OpenCV 的 NMSBoxes 速度更快
    keep = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), OBJ_THRESH, NMS_THRESH)
    if len(keep) > 0:
        idx = np.array(keep).flatten()
        return boxes[idx], cls_ids[idx], scores[idx]
    return None, None, None

def letter_box(img, new_shape=IMG_SIZE):
    h, w = img.shape[:2]
    r = min(new_shape[0] / h, new_shape[1] / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    dw, dh = (new_shape[1] - nw) // 2, (new_shape[0] - nh) // 2
    img = cv2.resize(img, (nw, nh), cv2.INTER_LINEAR)
    canvas = np.zeros((new_shape[0], new_shape[1], 3), dtype=np.uint8)
    canvas[dh:dh+nh, dw:dw+nw] = img
    return canvas, r, (dw, dh)

def scale_boxes(boxes, src_shape, dw, dh, r):
    b = boxes.copy()
    b[:, [0,2]] = (b[:, [0,2]] - dw) / r
    b[:, [1,3]] = (b[:, [1,3]] - dh) / r
    return b

# ================ 推理线程 ================
class InferenceWorker(threading.Thread):
    CORE_MAP = {0: RKNNLite.NPU_CORE_0, 1: RKNNLite.NPU_CORE_1, 2: RKNNLite.NPU_CORE_2}
    def __init__(self, idx, model_path, in_q, out_q):
        super().__init__(daemon=True)
        self.in_q, self.out_q = in_q, out_q
        self.rknn = RKNNLite(verbose=False)
        self.rknn.load_rknn(model_path)
        self.rknn.init_runtime(core_mask=self.CORE_MAP[idx])

    def run(self):
        while not rospy.is_shutdown():
            item = self.in_q.get()
            if item is None: break
            fid, frame = item
            
            h0, w0 = frame.shape[:2]
            img, r, (dw, dh) = letter_box(frame)
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # 真正推理
            out = self.rknn.inference([np.expand_dims(img_rgb, 0)])

            # 解码 YOLOv8 输出
            branch, pair = 3, len(out) // 3
            boxes, cconfs, oconfs = [], [], []
            for i in range(branch):
                boxes.append(box_process(out[pair*i]))
                cconfs.append(out[pair*i+1])
                oconfs.append(np.ones_like(out[pair*i+1][:, :1, :, :], np.float32))
            
            merge = lambda xs: np.concatenate([x.transpose(0,2,3,1).reshape(-1, x.shape[1]) for x in xs])
            b, cls, s = filter_and_nms(merge(boxes), merge(cconfs), merge(oconfs))
            
            if b is not None:
                b = scale_boxes(b, (h0, w0), dw, dh, r)
            
            self.out_q.put((fid, b, cls, s))

# ================ 主程序 ================
def main():
    rospy.init_node("rknn_v8_detector")
    rospy.Subscriber("/camera/color/image_raw", Image, ros_image_callback)

    in_qs = [queue.Queue(maxsize=1) for _ in range(3)]
    out_q = queue.Queue(maxsize=12)
    workers = [InferenceWorker(i, MODEL_PATH, in_qs[i], out_q) for i in range(3)]
    for w in workers: w.start()

    cv2.namedWindow('Detection', cv2.WINDOW_AUTOSIZE)
    
    history = deque(maxlen=5)
    fid, t0 = 0, time.time()

    while not rospy.is_shutdown():
        try:
            frame = raw_frame_queue.get(timeout=1.0)
        except queue.Empty: continue

        # 异步推理分发
        target = fid % 3
        if in_qs[target].empty():
            in_qs[target].put((fid, frame.copy()))
        fid += 1

        # 获取最新推理结果
        while not out_q.empty():
            history.append(out_q.get_nowait())

        # 绘制
        if history:
            _, boxes, cls_ids, scores = history[-1]
            if boxes is not None:
                for box, cls, sc in zip(boxes, cls_ids, scores):
                    x1, y1, x2, y2 = box.astype(int)
                    cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                    cv2.putText(frame, f'{CLASSES[cls]} {sc:.2f}', (x1, y1-5), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        # FPS 统计
        if fid % 30 == 0:
            fps = 30 / (time.time() - t0)
            print(f"FPS: {fps:.2f}")
            t0 = time.time()

        cv2.imshow('Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
