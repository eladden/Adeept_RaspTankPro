#!/usr/bin/env python3
"""
calibrate_camera.py - measure the robot camera's focal length and centre.

Follows the method of OpenCV's calibration sample (samples/python/calibrate.py):
find the corners of a printed checkerboard in several photos, then let
cv2.calibrateCamera work out the camera matrix and lens distortion. Only the
chessboard path is used, with calls that also exist in the OpenCV 3.2 that
Buster ships.

WHAT YOU NEED
    A printed checkerboard, stuck flat on something rigid. Measure one square.
    The default board has 9 x 6 INNER corners (a board of 10 x 7 squares).

ON THE ROBOT (stop the web server first, like sandbox.py does)
    sudo pkill -f webServer.py
    python3 calibrate_camera.py --square-mm 25
    Hold the board in front of the camera and move it between frames: close
    and far, tilted left/right/up/down, in the corners of the picture. A frame
    is kept whenever the whole board is seen, at most one per second, until
    --frames frames are collected. Raw frames are saved in --save-dir so you
    can calibrate again later without the robot.

FROM SAVED IMAGES (any computer)
    python3 calibrate_camera.py --from-dir calib_frames --square-mm 25

OUTPUT
    camera_calibration.json (next to this script). robot.start_odometry() picks it
    up by itself when called without focal_length / pp, as long as the image size
    matches. Delete the file to go back to the built-in defaults. To pass the
    numbers by hand instead, the script also prints:
        robot.start_odometry(focal_length=..., pp=(..., ...))

The result is only valid for the image size and camera stack it was taken with
(OpenCV VideoCapture on Buster, picamera2 on Bookworm), so calibrate once per
stack. The camera is opened exactly as the visual odometry opens it.
"""

import argparse
import datetime
import glob
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


# ---------------------------------------------------------------------------
# Camera - same open sequence as Robot.start_odometry() in robot_api.py
# ---------------------------------------------------------------------------
def open_camera():
    """Return (capture, stack_name). stack_name is 'VideoCapture' or 'picamera2'."""
    cap = None
    for _ in range(5):
        try:
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        except TypeError:                      # OpenCV 3.2 has no 2-argument form
            cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            cap = cv2.VideoCapture(0)
        if cap.isOpened():
            break
        cap.release()
        time.sleep(1)
    if cap is not None and cap.isOpened():
        for _ in range(5):
            ok, _ = cap.read()
            if ok:
                return cap, 'VideoCapture'
            time.sleep(0.05)
        cap.release()
    # Bookworm / Trixie: V4L2 opens but delivers no frames, use picamera2
    sys.path.insert(0, HERE)
    try:
        from robot_api import _Picam2Capture
        return _Picam2Capture(width=640, height=480), 'picamera2'
    except Exception as e:
        raise SystemExit("Cannot open the camera: %s\n"
                         "Is the web server still running? (sudo pkill -f webServer.py)" % e)


# ---------------------------------------------------------------------------
# Corner finding
# ---------------------------------------------------------------------------
CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
FIND_FLAGS = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE


def find_corners(gray, pattern):
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=FIND_FLAGS)
    if not found:
        return None
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)


def capture_frames(pattern, n_frames, save_dir, show):
    cap, stack = open_camera()
    os.makedirs(save_dir, exist_ok=True)
    print("Camera opened with %s. Move the board between frames. Ctrl+C to stop early." % stack)
    frames, last = [], 0.0
    try:
        while len(frames) < n_frames:
            ok, img = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            corners = find_corners(gray, pattern)
            now = time.time()
            if corners is not None and now - last >= 1.0:
                last = now
                frames.append((gray, corners))
                cv2.imwrite(os.path.join(save_dir, "frame_%02d.png" % len(frames)), img)
                print("  kept frame %d of %d" % (len(frames), n_frames))
            if show:
                vis = img.copy()
                if corners is not None:
                    cv2.drawChessboardCorners(vis, pattern, corners, True)
                cv2.imshow("calibration (q = stop)", vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if show:
            cv2.destroyAllWindows()
    return frames, stack


def load_frames(folder, pattern):
    frames, size = [], None
    for path in sorted(glob.glob(os.path.join(folder, "*"))):
        img = cv2.imread(path)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = find_corners(gray, pattern)
        print("  %-30s %s" % (os.path.basename(path), "board found" if corners is not None else "no board"))
        if corners is not None:
            frames.append((gray, corners))
    return frames


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------
def calibrate(frames, pattern, square_mm):
    cols, rows = pattern
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    h, w = frames[0][0].shape[:2]
    if any(f[0].shape[:2] != (h, w) for f in frames):
        raise SystemExit("The frames have different sizes; calibrate each size separately.")
    obj_pts = [objp] * len(frames)
    img_pts = [c for _, c in frames]
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)

    per_frame = []
    for i in range(len(frames)):
        proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], mtx, dist)
        err = cv2.norm(img_pts[i], proj, cv2.NORM_L2) / np.sqrt(len(proj))
        per_frame.append(float(err))
    return rms, mtx, dist, (w, h), per_frame


def main():
    ap = argparse.ArgumentParser(description="Calibrate the robot camera from checkerboard photos.")
    ap.add_argument("--cols", type=int, default=9, help="inner corners along the long side (default 9)")
    ap.add_argument("--rows", type=int, default=6, help="inner corners along the short side (default 6)")
    ap.add_argument("--square-mm", type=float, default=25.0, help="side of one printed square, mm")
    ap.add_argument("--frames", type=int, default=20, help="frames to collect on the robot")
    ap.add_argument("--from-dir", help="calibrate from saved images in this folder (no camera)")
    ap.add_argument("--save-dir", default=os.path.join(HERE, "calib_frames"),
                    help="where captured frames are saved")
    ap.add_argument("--out", default=os.path.join(HERE, "camera_calibration.json"))
    ap.add_argument("--show", action="store_true", help="show a live window (needs a monitor)")
    args = ap.parse_args()
    pattern = (args.cols, args.rows)

    if args.from_dir:
        print("Reading images from %s" % args.from_dir)
        frames, stack = load_frames(args.from_dir, pattern), "from saved images"
    else:
        frames, stack = capture_frames(pattern, args.frames, args.save_dir, args.show)

    if len(frames) < 8:
        raise SystemExit("Only %d usable frames; at least 8 are needed (15-20 is better)." % len(frames))

    rms, mtx, dist, (w, h), per_frame = calibrate(frames, pattern, args.square_mm)
    fx, fy, cx, cy = mtx[0, 0], mtx[1, 1], mtx[0, 2], mtx[1, 2]

    print("\nFrames used     : %d" % len(frames))
    print("Image size      : %d x %d" % (w, h))
    print("Reprojection RMS: %.3f px   (below about 0.5 is good, above 1 is poor)" % rms)
    print("fx, fy          : %.1f, %.1f   (should be close to each other)" % (fx, fy))
    print("cx, cy          : %.1f, %.1f   (image centre would be %.1f, %.1f)" % (cx, cy, w / 2.0, h / 2.0))
    print("distortion      : %s" % np.round(dist.ravel(), 4).tolist())
    worst = int(np.argmax(per_frame))
    print("worst frame     : #%d (%.2f px) - delete it and recalibrate if far above the rest" % (worst + 1, per_frame[worst]))
    if (w, h) != (640, 480):
        print("NOTE: the image is %dx%d, not 640x480. The numbers are valid only at this size." % (w, h))

    result = {
        "image_width": w, "image_height": h,
        "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
        "distortion": dist.ravel().tolist(),
        "rms_px": float(rms),
        "frames_used": len(frames),
        "square_mm": args.square_mm,
        "pattern_inner_corners": list(pattern),
        "camera_stack": stack,
        "opencv_version": cv2.__version__,
        "date": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
    }
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)
    print("\nSaved %s" % args.out)
    print("start_odometry() will use this file automatically. By hand it would be:")
    print("  robot.start_odometry(focal_length=%.1f, pp=(%.1f, %.1f))" % ((fx + fy) / 2.0, cx, cy))


if __name__ == "__main__":
    main()
