#!/usr/bin/env python3
# yolo_depth.py depthai 3.x compatible
# perception pipeline with OSHA waste colour coding
# outputs: class, category, 3D position, principal axis, grasp confidence

import cv2
import depthai as dai
import numpy as np
from pathlib import Path

BLOB_PATH   = Path("/home/johnpork/Documents/binx_vision/yolov8n_6shaves.blob")
CONF_THRESH = 0.30

# OAK-D calibration
FX = 381.9
FY = 381.6
CX = 322.5
CY = 231.0

COCO_CLASSES = [
    'person','bicycle','car','motorcycle','airplane','bus','train',
    'truck','boat','traffic light','fire hydrant','stop sign',
    'parking meter','bench','bird','cat','dog','horse','sheep','cow',
    'elephant','bear','zebra','giraffe','backpack','umbrella','handbag',
    'tie','suitcase','frisbee','skis','snowboard','sports ball','kite',
    'baseball bat','baseball glove','skateboard','surfboard',
    'tennis racket','bottle','wine glass','cup','fork','knife','spoon',
    'bowl','banana','apple','sandwich','orange','broccoli','carrot',
    'hot dog','pizza','donut','cake','chair','couch','potted plant',
    'bed','dining table','toilet','tv','laptop','mouse','remote',
    'keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear',
    'hair drier','toothbrush'
]

#-------------------------------------------------for osha waste categories-----------------------------------------------

WASTE_MAP = {
    "bottle":     ("Plastic",          "Green Bin",    (0, 200, 0)),
    "cup":        ("Plastic",          "Green Bin",    (0, 200, 0)),
    "bowl":       ("Plastic",          "Green Bin",    (0, 200, 0)),
    "wine glass": ("Glass",            "White Bin",    (230, 230, 230)),
    "vase":       ("Glass",            "White Bin",    (230, 230, 230)),
    "book":       ("Paper/Cardboard",  "Blue Bin",     (200, 50, 0)),
    "scissors":   ("Metal",            "Yellow Bin",   (0, 220, 220)),
    "knife":      ("Metal",            "Yellow Bin",   (0, 220, 220)),
    "cell phone": ("E-Waste",          "Yellow Bin",   (0, 220, 220)),
    "laptop":     ("E-Waste",          "Yellow Bin",   (0, 220, 220)),
    "keyboard":   ("E-Waste",          "Yellow Bin",   (0, 220, 220)),
    "mouse":      ("E-Waste",          "Yellow Bin",   (0, 220, 220)),
    "remote":     ("E-Waste",          "Yellow Bin",   (0, 220, 220)),
    "banana":     ("Organic",          "Brown Bin",    (42, 80, 130)),
    "apple":      ("Organic",          "Brown Bin",    (42, 80, 130)),
    "orange":     ("Organic",          "Brown Bin",    (42, 80, 130)),
    "sandwich":   ("Organic",          "Brown Bin",    (42, 80, 130)),
    "carrot":     ("Organic",          "Brown Bin",    (42, 80, 130)),
    "broccoli":   ("Organic",          "Brown Bin",    (42, 80, 130)),
}

def get_waste_category(class_name):
    return WASTE_MAP.get(
        class_name,
        ("Unknown", "Manual Check", (128, 128, 128))
    )

#-------------------------------------------------depthai pipeline-----------------------------------------------

pipeline = dai.Pipeline()

camRgb    = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft  = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

stereo = pipeline.create(dai.node.StereoDepth)
stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
stereo.setRectification(True)
stereo.setExtendedDisparity(False)
stereo.setOutputSize(640, 480)

monoLeft.requestOutput((640, 400)).link(stereo.left)
monoRight.requestOutput((640, 400)).link(stereo.right)

spatCalc = pipeline.create(dai.node.SpatialLocationCalculator)
spatCalc.inputConfig.setWaitForMessage(False)

cfg_data = dai.SpatialLocationCalculatorConfigData()
cfg_data.depthThresholds.lowerThreshold = 50
cfg_data.depthThresholds.upperThreshold = 10000
cfg_data.calculationAlgorithm = dai.SpatialLocationCalculatorAlgorithm.MEDIAN
cfg_data.roi = dai.Rect(dai.Point2f(0.4, 0.4), dai.Point2f(0.6, 0.6))
spatCalc.initialConfig.addROI(cfg_data)
stereo.depth.link(spatCalc.inputDepth)

q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

nn = pipeline.create(dai.node.NeuralNetwork)
nn.setBlobPath(BLOB_PATH)
nn.setNumInferenceThreads(2)

rgbOut = camRgb.requestOutput((640, 480))
manip  = pipeline.create(dai.node.ImageManip)
manip.initialConfig.setOutputSize(416, 416, dai.ImageManipConfig.ResizeMode.STRETCH)
manip.initialConfig.setFrameType(dai.ImgFrame.Type.RGB888p)
rgbOut.link(manip.inputImage)
manip.out.link(nn.input)

q_rgb    = rgbOut.createOutputQueue(maxSize=4, blocking=False)
q_nn     = nn.out.createOutputQueue(maxSize=4, blocking=False)
q_spat   = spatCalc.out.createOutputQueue(maxSize=4, blocking=False)
q_config = spatCalc.inputConfig.createInputQueue(maxSize=4, blocking=False)


#-------------------------------------------------Additional functions-----------------------------------------------
def decode_yolo(output, conf_thresh=0.3):
    data = np.array(output.getFirstTensor())
    try:
        data = data.reshape((84, -1))
    except Exception as e:
        print(f"Reshape error: {e}")
        return []

    class_probs = data[4:, :]
    class_ids   = np.argmax(class_probs, axis=0)
    confidences = class_probs[class_ids, np.arange(class_probs.shape[1])]

    keep = confidences > conf_thresh
    if not np.any(keep):
        return []

    boxes_raw   = data[:4, keep]
    class_ids   = class_ids[keep]
    confidences = confidences[keep]

    cx, cy, w, h = boxes_raw
    x1 = (cx - w / 2).astype(int)
    y1 = (cy - h / 2).astype(int)
    x2 = (cx + w / 2).astype(int)
    y2 = (cy + h / 2).astype(int)

    boxes_xyxy = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)
    indices    = cv2.dnn.NMSBoxes(
        boxes_xyxy.tolist(), confidences.tolist(), conf_thresh, 0.45
    )

    detections = []
    for i in (indices.flatten() if len(indices) else []):
        detections.append({
            "box":        boxes_xyxy[i].astype(int).tolist(),
            "class_name": COCO_CLASSES[int(class_ids[i])],
            "confidence": float(confidences[i]),
        })
    return detections


def fast_mask(frame_shape, x1, y1, x2, y2, inward=0.15):
    h, w     = frame_shape[:2]
    margin_x = int((x2 - x1) * inward)
    margin_y = int((y2 - y1) * inward)
    ix1 = max(0,   x1 + margin_x)
    iy1 = max(0,   y1 + margin_y)
    ix2 = min(w-1, x2 - margin_x)
    iy2 = min(h-1, y2 - margin_y)
    mask = np.zeros(frame_shape[:2], np.uint8)
    mask[iy1:iy2, ix1:ix2] = 1
    return mask


def get_principal_axis(mask, depth_frame, fx, fy, cx_cam, cy_cam):
    ys, xs = np.where(mask == 1)
    if len(xs) < 5:
        return None, 0.0
    zs    = depth_frame[ys, xs].astype(float)
    valid = zs > 0
    xs, ys, zs = xs[valid], ys[valid], zs[valid]
    coverage = len(zs) / max(len(mask[mask == 1]), 1)
    if len(zs) < 10:
        return None, coverage
    Z = zs / 1000.0
    X = (xs - cx_cam) * Z / fx
    Y = (ys - cy_cam) * Z / fy
    points         = np.stack([X, Y, Z], axis=1)
    points_centred = points - points.mean(axis=0)
    _, _, Vt = np.linalg.svd(points_centred, full_matrices=False)
    return Vt[0], coverage


def grasp_confidence(det_conf, depth_coverage):
    score = (det_conf * 0.5) + (depth_coverage * 0.5)
    return round(min(score, 1.0), 2)


def draw_osha_legend(display, H, W):
    legend = [
        ("Plastics",       (0, 200, 0)),
        ("Paper/Cardboard",(200, 50, 0)),
        ("Metals/E-Waste", (0, 220, 220)),
        ("Organic",        (42, 80, 130)),
        ("Glass",          (230, 230, 230)),
    ]
    lx, ly = 10, H - 105
    cv2.rectangle(display, (lx-5, ly-18),
                  (lx+155, ly + len(legend)*18 + 5),
                  (30, 30, 30), -1)
    cv2.putText(display, "OSHA Waste Standard:",
                (lx, ly - 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.33, (200,200,200), 1)
    for i, (label, col) in enumerate(legend):
        y_pos = ly + 14 + i * 17
        cv2.rectangle(display,
                      (lx, y_pos - 10), (lx+12, y_pos+2),
                      col, -1)
        cv2.putText(display, label,
                    (lx+16, y_pos),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200,200,200), 1)


#-------------------------------------------------Main loop-----------------------------------------------
last_rect = None

with pipeline:
    pipeline.start()
    cv2.namedWindow("BINX : Waste Perception", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("BINX : Waste Perception", 640, 480)

    while pipeline.isRunning():

        rgb_data   = q_rgb.get()
        nn_data    = q_nn.get()
        sp_data    = q_spat.get()
        depth_data = q_depth.get()

        frame       = rgb_data.getCvFrame()
        H, W        = frame.shape[:2]
        depth_frame = cv2.resize(
            depth_data.getFrame(), (W, H),
            interpolation=cv2.INTER_NEAREST
        )

        display = frame.copy()

        detections = decode_yolo(nn_data, CONF_THRESH)

        best = None
        for det in detections:
            if best is None or det["confidence"] > best["confidence"]:
                best = det

        if best is not None:
            x1, y1, x2, y2 = best["box"]

            x1 = max(0,   int(x1 * W / 416))
            y1 = max(0,   int(y1 * H / 416))
            x2 = min(W-1, int(x2 * W / 416))
            y2 = min(H-1, int(y2 * H / 416))

            send_new = last_rect is None
            if not send_new:
                lx1, ly1, lx2, ly2 = last_rect
                if (abs(x1-lx1)>10 or abs(y1-ly1)>10 or
                        abs(x2-lx2)>10 or abs(y2-ly2)>10):
                    send_new = True
            if send_new:
                new_cfg = dai.SpatialLocationCalculatorConfigData()
                new_cfg.depthThresholds.lowerThreshold = 50
                new_cfg.depthThresholds.upperThreshold = 10000
                new_cfg.calculationAlgorithm = (
                    dai.SpatialLocationCalculatorAlgorithm.MEDIAN)
                new_cfg.roi = dai.Rect(
                    dai.Point2f(max(0.0, x1/W), max(0.0, y1/H)),
                    dai.Point2f(min(1.0, x2/W), min(1.0, y2/H))
                )
                cfg_msg = dai.SpatialLocationCalculatorConfig()
                cfg_msg.addROI(new_cfg)
                q_config.send(cfg_msg)
                last_rect = (x1, y1, x2, y2)

            z_m = 0.0
            for loc in sp_data.getSpatialLocations():
                z_m = loc.spatialCoordinates.z / 1000.0

            mask = fast_mask(frame.shape, x1, y1, x2, y2)
            M    = cv2.moments(mask)
            cx   = int(M["m10"]/M["m00"]) if M["m00"]>0 else (x1+x2)//2
            cy   = int(M["m01"]/M["m00"]) if M["m00"]>0 else (y1+y2)//2

            axis, depth_coverage = get_principal_axis(
                mask, depth_frame, FX, FY, CX, CY
            )

            conf   = best["confidence"]
            g_conf = grasp_confidence(conf, depth_coverage)

            X_m = (cx - CX) * z_m / FX if z_m > 0 else 0.0
            Y_m = (cy - CY) * z_m / FY if z_m > 0 else 0.0

            category, bin_label, osha_colour = get_waste_category(
                best["class_name"]
            )

            if g_conf > 0.7 and conf > 0.5:
                decision = f"ROUTE to {bin_label}"
            elif g_conf > 0.4:
                decision = "CAUTION"
            else:
                decision = "reposition required"

            # Mask overlay
            overlay = np.zeros((H, W, 3), dtype=np.uint8)
            overlay[mask == 1] = osha_colour
            display = cv2.addWeighted(display, 1.0, overlay, 0.25, 0)

            # Bounding box
            thickness = 3 if g_conf > 0.7 else 2 if g_conf > 0.4 else 1
            cv2.rectangle(display, (x1,y1), (x2,y2), osha_colour, thickness)

            # Centroid
            cv2.circle(display, (cx,cy), 7, (255,255,255), -1)
            cv2.circle(display, (cx,cy), 5, osha_colour, -1)

            # Principal axis
            if axis is not None:
                scale = 60
                cv2.line(display,
                    (int(cx - axis[0]*scale), int(cy - axis[1]*scale)),
                    (int(cx + axis[0]*scale), int(cy + axis[1]*scale)),
                    (255,255,255), 2)

            # Text background
            text_y = max(y1 - 95, 5)
            cv2.rectangle(display,
                (x1, text_y), (x1+290, text_y+92),
                (20,20,20), -1)
            cv2.rectangle(display,
                (x1, text_y), (x1+290, text_y+92),
                osha_colour, 1)

            # Text
            cv2.putText(display,
                f"{best['class_name'].upper()}  {conf:.0%}",
                (x1+6, text_y+18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, osha_colour, 2)
            cv2.putText(display,
                f"Category: {category}",
                (x1+6, text_y+36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, osha_colour, 1)
            cv2.putText(display,
                f"Route to: {bin_label}",
                (x1+6, text_y+52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255,255,255), 1)
            cv2.putText(display,
                f"X={X_m:+.3f}m  Y={Y_m:+.3f}m  Z={z_m:.3f}m",
                (x1+6, text_y+68),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180,180,180), 1)
            cv2.putText(display,
                f"Grasp={g_conf:.2f}  | {decision}",
                (x1+6, text_y+84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180,180,180), 1)

            axis_str = (f"({axis[0]:+.2f},{axis[1]:+.2f},{axis[2]:+.2f})"
                        if axis is not None else "N/A")
            print(f"\r{best['class_name']:12s} "
                  f"→ {category:18s} "
                  f"det={conf:.2f} "
                  f"X={X_m:+.3f} Y={Y_m:+.3f} Z={z_m:.3f} "
                  f"cov={depth_coverage:.2f} "
                  f"GRASP={g_conf:.2f} "
                  f"| {decision}  ",
                  end="", flush=True)

        else:
            print("\rNo detection                           ",
                  end="", flush=True)

        draw_osha_legend(display, H, W)
        cv2.imshow("BINX : Waste Perception", display)

        key = cv2.waitKey(30)
        if key == ord("q"):
            break
        elif key == ord("s"):
            if best is not None:
                fname = f"BINX_{category.replace(' ','_')}_{best['class_name']}.png"
            else:
                fname = "BINX_no_detection.png"
            cv2.imwrite(fname, display)
            print(f"\nScreenshot saved: {fname}")

cv2.destroyAllWindows()
print()