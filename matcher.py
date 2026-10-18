#!/usr/bin/env python3
#
# To use with rtabmap:
#   --Vis/FeatureType 15 --Kp/DetectorStrategy 15 --PyDetector/Path "~/SuperPointPretrainedNetwork/rtabmap_superpoint.py"
#

from lightglue.utils import numpy_image_to_torch
import random
import numpy as np
import torch
import os
from lightglue import LightGlue
from time import perf_counter
import sys

torch.set_grad_enabled(False)

use_cuda = False
matcher = None

def init(descriptorDim, matchThreshold, iterations, cuda, model):
    global use_cuda, matcher
    use_cuda = torch.cuda.is_available() and cuda

    matcher = LightGlue(features="disk").eval()
    if use_cuda:
        matcher = matcher.cuda()
    matcher.compile()

def to_dev(tensor: torch.Tensor):
    global use_cuda
    if use_cuda:
        return tensor.cuda()
    return tensor

def match(kptsFrom, kptsTo, scoresFrom, scoresTo, descriptorsFrom, descriptorsTo, imageWidth, imageHeight):
    start = perf_counter()
    global matcher
    assert matcher

    image0 = {
        "keypoints": to_dev(torch.from_numpy(kptsFrom[np.newaxis, :, 0:2])),
        "keypoint_scores": to_dev(torch.from_numpy(scoresFrom[np.newaxis, :])),
        "descriptors": to_dev(torch.from_numpy(descriptorsFrom[np.newaxis, :])),
        "image_size": to_dev(torch.from_numpy(np.array([[imageWidth, imageHeight]])))
    }
    image1 = {
        "keypoints": to_dev(torch.from_numpy(kptsTo[np.newaxis, :, 0:2])),
        "keypoint_scores": to_dev(torch.from_numpy(scoresTo[np.newaxis, :])),
        "descriptors": to_dev(torch.from_numpy(descriptorsTo[np.newaxis, :])),
        "image_size": to_dev(torch.from_numpy(np.array([[imageWidth, imageHeight]])))
    }

    matches = matcher({'image0': image0, 'image1': image1})
    
    # rtabmap expects format:
    #   matches: array Nx2 (type=9 or uint64)
    end = perf_counter()
    print(f"Matching took: {end - start} seconds", file=sys.stderr)
    return matches['matches'][0].numpy()
