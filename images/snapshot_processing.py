# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.


import argparse
import os
from collections import defaultdict

import cv2


def get_args():
    parser = argparse.ArgumentParser(
        description="Process images with cyan or red objects into monochromatic. Merge any pairs of files that have the same filename, except 2 letters in suffix, e.g. fileAA.png fileZZ.png ."
    )
    parser.add_argument("indir", type=str)
    parser.add_argument("--outdir", default="/tmp/", type=str)
    parser.add_argument("--preview", action="store_true")
    return parser.parse_args()


args = get_args()


def increase_contrast(rgb_image):
    lab = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB)
    l_channel, alpha, beta = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(2, 2))
    clahe_l = clahe.apply(l_channel)
    l_frame = cv2.merge((clahe_l, alpha, beta))
    enhanced_rgb = cv2.cvtColor(l_frame, cv2.COLOR_LAB2RGB)
    return enhanced_rgb


def extract(
    rgb_image,
):
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cyan_mask = cv2.inRange(hsv_image, (80, 0, 0), (100, 255, 255))
    red_mask_low = cv2.inRange(hsv_image, (0, 200, 200), (10, 255, 255))
    red_mask_high = cv2.inRange(hsv_image, (170, 200, 200), (180, 255, 255))
    red_mask = cv2.bitwise_or(red_mask_low, red_mask_high)
    mask = cv2.bitwise_or(red_mask, cyan_mask)
    rgb_image_copy = rgb_image.copy()
    rgb_image_copy[mask == 0] = (255, 255, 255)

    return rgb_image_copy, mask


def resize(image, larger_shape=None):
    if larger_shape is None:
        larger_shape = (
            (image.shape[0] // 100 + 1) * 100,
            (image.shape[1] // 100 + 1) * 100,
        )
    vertical = (larger_shape[0] - image.shape[0]) // 2
    horizontal = (larger_shape[1] - image.shape[1]) // 2
    image = cv2.copyMakeBorder(
        image,
        vertical,
        larger_shape[0] - image.shape[0] - vertical,
        horizontal,
        larger_shape[1] - image.shape[1] - horizontal,
        cv2.BORDER_REPLICATE,
    )
    return image


def process_file(
    filename_in,
    preview=False,
):
    rgb_image = cv2.imread(filename_in, cv2.COLOR_BGR2RGB)
    extracted_rgb, mask = extract(rgb_image)
    contrast_rgb_image = increase_contrast(extracted_rgb)
    extracted_grey = cv2.cvtColor(contrast_rgb_image, cv2.COLOR_RGB2GRAY)
    normalized = cv2.normalize(
        src=extracted_grey,
        dst=None,
        alpha=0,
        beta=200,
        mask=mask,
        norm_type=cv2.NORM_MINMAX,
    )
    normalized = cv2.bitwise_or(normalized, cv2.bitwise_not(mask))

    if preview:
        cv2.imshow("original", rgb_image)
        cv2.imshow("extracted_rgb", extracted_rgb)
        cv2.imshow("contrast_rgb_image", contrast_rgb_image)
        cv2.imshow("extracted_grey", extracted_grey)
        cv2.imshow("normalized", normalized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return normalized


def merge_pair(pair, basename):
    if len(pair) != 2:
        print(f"The not able to find pairs of images for {basename}.")
        return None
    larger_shape = (
        max(pair[0].shape[0], pair[1].shape[0]),
        max(pair[0].shape[1], pair[1].shape[1]),
    )
    pair = [resize(image, larger_shape) for image in pair]

    merged = cv2.hconcat(pair)
    merged = resize(merged)  # round up shape
    return merged


def merge_images(images, preview=False):
    pairs = defaultdict(list)
    for fname, image in sorted(images.items()):
        basename = fname.split("/")[-1][:-6] + "__.png"
        pairs[basename].append(image)

    for basename, pair in pairs.items():
        merged = merge_pair(pair, basename)
        if merged is None:
            continue
        if preview:
            cv2.imshow("merged", merged)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        yield ("merged_" + basename, merged)


assert os.path.isdir(args.indir)
assert os.path.isdir(args.outdir)
processed_images = {}
for fname in sorted(os.listdir(args.indir)):
    if fname.endswith(".png"):
        filename_in = args.indir + fname
        filename_out = args.outdir + "/processed_" + fname
        processed_image = process_file(
            filename_in=filename_in,
            preview=args.preview,
        )
        print(f"Writing to {filename_out}")
        cv2.imwrite(filename_out, processed_image)
        processed_images[filename_in.split("/")[-1]] = processed_image

for fname, merged_image in merge_images(processed_images, preview=args.preview):
    filename_out = args.outdir + "/" + fname
    print(f"Writing to {filename_out}")
    cv2.imwrite(filename_out, merged_image)
