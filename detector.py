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
from lightglue import DISK

torch.set_grad_enabled(False)

use_cuda = False
detector = None

def init(cuda):
    global use_cuda, detector
    use_cuda = torch.cuda.is_available() and cuda

    detector = DISK(max_num_keypoints=2048).eval()
    if use_cuda:
        detector = detector.cuda()

def detect(imageBuffer):
    global use_cuda, detector
    image = numpy_image_to_torch(np.asarray(imageBuffer))
    if use_cuda:
        image = image.cuda()

    assert detector
    feats = detector.extract(image)

    points = feats['keypoints'].numpy()[0]
    scores = feats['keypoint_scores'].numpy()[0]
    points = np.hstack((points, scores[:, np.newaxis]))

    descs = feats['descriptors'].numpy()[0]
    
    # rtabmap expects format:
    #   pts:         array Nx3 (type=11 or float)
    #   descriptors: array NxDIM 35x256 (type=11 or float)
    return points, descs
