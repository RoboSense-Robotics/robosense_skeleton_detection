import numpy as np
import cv2
import argparse
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root_path", default="", type=str)
    parser.add_argument("--dst_path", default="", type=str)
    parser.add_argument("--record_start_idx", default=0, type=int)
    parser.add_argument("--frame_start_idx", default=0, type=int)
    parser.add_argument("--mode", default="video", type=str)
    args = parser.parse_args()
    return args

def read_images(image_dir):
    imgs = os.listdir(image_dir)
    imgs = sorted(imgs)
    imgs = [os.path.join(image_dir, img) for img in imgs]
    return imgs

if __name__ == "__main__":
    args = parse_args()
    root_path = args.root_path
    left_camera = os.path.join(root_path, "left_camera")
    right_camera = os.path.join(root_path, "right_camera")
    frames = os.path.join(root_path,"frames")
    # markers_path = os.path.join(root_path,"marker_frames")

    left_imgs = read_images(left_camera)
    right_imgs = read_images(right_camera)
    frame_imgs = read_images(frames)
    # marker_imgs = read_images(markers_path)
    assert len(left_imgs) == len(right_imgs)

    # 2286
    # record_start_idx = 209
    # frame_start_idx = 229
    # markers_start_idx = 209
    #
    # # 2288
    # record_start_idx = 155-9+4+2
    # frame_start_idx = 52
    #
    # # 2298
    # record_start_idx = 31
    # frame_start_idx = 46
    #
    # # 2506
    # record_start_idx = 194-10+2
    # frame_start_idx = 296
    #
    # # 2831
    # record_start_idx = 19
    # frame_start_idx = 26

    record_start_idx = args.record_start_idx
    frame_start_idx = args.frame_start_idx

    # seq = min([len(left_imgs)-record_start_idx,len(frame_imgs)-frame_start_idx, len(marker_imgs)-markers_start_idx])
    seq = min([len(left_imgs)-record_start_idx,len(frame_imgs)-frame_start_idx])
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = None

    for i in range(seq):
        left_img = cv2.imread(left_imgs[i+record_start_idx])
        right_img = cv2.imread(right_imgs[i+record_start_idx])
        frame_img = cv2.imread(frame_imgs[i+frame_start_idx])
        # marker_img = cv2.imread(marker_imgs[i+markers_start_idx])
        # mh, mw = marker_img.shape[0], marker_img.shape[1]
        lh, lw = left_img.shape[0],left_img.shape[1]
        left_resize_img = cv2.resize(left_img,(lw//2, lh//2))
        right_resize_img = cv2.resize(right_img,(lw//2, lh//2))
        col0: np.ndarray = np.concatenate([left_resize_img,right_resize_img],axis=0)
        frame_h, frame_w = frame_img.shape[0],frame_img.shape[1]
        res_h = col0.shape[0]
        col1_w = int(frame_w * res_h / frame_h)
        # col2_w = int(mw * res_h / mh)
        frame_resize_img = cv2.resize(frame_img, (col1_w, res_h))
        # marker_resize_img = cv2.resize(marker_img, (col2_w, res_h))
        # res_img = np.concatenate([col0, frame_resize_img,marker_resize_img],axis=1)
        res_img = np.concatenate([col0, frame_resize_img],axis=1)
        if args.mode == "video":
            if i == 0:
                out = cv2.VideoWriter(args.dst_path, fourcc, 20, (res_img.shape[1],res_img.shape[0]), True)
            assert out is not None
            out.write(res_img)
        else:
            cv2.imshow("peek",res_img)
            cv2.waitKey(0)
    if args.mode == "video" and out is not None:
        out.release()

        # cv2.imshow("res",res_img)
        # cv2.waitKey(0)

