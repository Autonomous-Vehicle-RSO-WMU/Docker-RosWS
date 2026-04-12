import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import torch
import numpy as np
import torchvision.transforms as transforms

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from lib.config import cfg
from lib.models import get_net
from lib.utils.utils import create_logger, select_device
from lib.dataset import LoadImages, LoadStreams

# -----------------------------
# Preprocessing (same as YOLOP)
# -----------------------------
normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406],
    std=[0.229, 0.224, 0.225]
)

transform = transforms.Compose([
    transforms.ToTensor(),
    normalize,
])


def detect(opt):
    logger, _, _ = create_logger(cfg, cfg.LOG_DIR, "demo")

    device = select_device(logger, opt.device)
    half = device.type != "cpu"

    # Load model
    model = get_net(cfg)
    ckpt = torch.load(opt.weights, map_location=device)
    model.load_state_dict(ckpt["state_dict"])
    model.to(device).eval()

    if half:
        model.half()

    # Load input
    if opt.source.isnumeric():
        dataset = LoadStreams(opt.source, img_size=opt.img_size)
    else:
        dataset = LoadImages(opt.source, img_size=opt.img_size)

    save_dir = Path(opt.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    for path, img, img0, cap, shapes in dataset:
        img = transform(img).to(device)
        img = img.half() if half else img.float()
        img = img.unsqueeze(0)

        # -------- Inference --------
        _, _, ll_seg = model(img)

        # -------- Lane mask --------
        prob = torch.sigmoid(ll_seg)
        mask = (prob > opt.lane_thresh).float()
        mask = mask.squeeze().cpu().numpy().astype(np.uint8) * 255

        # Remove padding / resize back
        h0, w0, _ = img0.shape
        mask = cv2.resize(mask, (w0, h0), interpolation=cv2.INTER_NEAREST)

        # -------- Overlay --------
        overlay = img0.copy()
        overlay[mask == 255] = (0, 255, 0)

        # Save
        out_path = save_dir / Path(path).name
        cv2.imwrite(str(out_path), overlay)

        print(f"Saved: {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", default="weights/End-to-end.pth")
    parser.add_argument("--source", default="inference/images")
    parser.add_argument("--img-size", type=int, default=640)
    parser.add_argument("--lane-thresh", type=float, default=0.5)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--save-dir", default="inference/output")
    opt = parser.parse_args()

    with torch.no_grad():
        detect(opt)

