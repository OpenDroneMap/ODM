import numpy as np

# Based on Fast Guided Filter
# Kaiming He, Jian Sun
# https://arxiv.org/abs/1505.00996

def box(img, radius):
    dst = np.zeros_like(img)
    (r, c) = img.shape

    s = [radius, 1]
    c_sum = np.cumsum(img, 0)
    dst[0:radius+1, :, ...] = c_sum[radius:2*radius+1, :, ...]
    dst[radius+1:r-radius, :, ...] = c_sum[2*radius+1:r, :, ...] - c_sum[0:r-2*radius-1, :, ...]
    dst[r-radius:r, :, ...] = np.tile(c_sum[r-1:r, :, ...], s) - c_sum[r-2*radius-1:r-radius-1, :, ...]

    s = [1, radius]
    c_sum = np.cumsum(dst, 1)
    dst[:, 0:radius+1, ...] = c_sum[:, radius:2*radius+1, ...]
    dst[:, radius+1:c-radius, ...] = c_sum[:, 2*radius+1 : c, ...] - c_sum[:, 0 : c-2*radius-1, ...]
    dst[:, c-radius: c, ...] = np.tile(c_sum[:, c-1:c, ...], s) - c_sum[:, c-2*radius-1 : c-radius-1, ...]

    return dst


def guided_filter(img, guide, radius, eps):
    (r, c) = img.shape

    CNT = box(np.ones([r, c]), radius)

    mean_img = box(img, radius) / CNT
    mean_guide = box(guide, radius) / CNT

    a = ((box(img * guide, radius) / CNT) - mean_img * mean_guide) / (((box(img * img, radius) / CNT) - mean_img * mean_img) + eps)
    b = mean_guide - a * mean_img

    return (box(a, radius) / CNT) * img + (box(b, radius) / CNT)
