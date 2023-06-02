# Main and helper function
import time

from PIL import Image
import numpy as np
from RRT import RRT
import tqdm
from PRM import PRM
import time
import torch
from torch.autograd import Variable
import numpy as np
from FCN import FCNs
from FCN import VGGNet 
import cv2
from matplotlib import pyplot as plt
from torchvision import transforms

import matplotlib.pyplot as plt


def load_map(file_path, resolution_scale):
    ''' Load map from an image and return a 2D binary numpy array
        where 0 represents obstacles and 1 represents free space
    '''
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')
    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y  = int(size_x*resolution_scale), int(size_y*resolution_scale)
    img = img.resize((new_x, new_y), Image.ANTIALIAS)

    map_array = np.asarray(img, dtype='uint8')
    map_array =  np.transpose(map_array)
    # Get bianry image
    threshold = 127
    map_array = 1 * (map_array > threshold)

    # Result 2D numpy array
    return map_array


if __name__ == "__main__":
    # Load the map
    # data1 original

    points = np.load('demo/train_point.npy', allow_pickle=True).item()
    node = []
    path_len = []
    un_success = []
    success = []
    time1 = []
    time2 = []
    model = torch.load('checkpoints/fcn_model_115.pt')
    model = model.cuda()
    model.eval()

    for i in tqdm.tqdm(range(0,2)):
        image_path = 'demo/train/data/{}.png'.format(i)
        x = cv2.imread(image_path)
        transform = transforms.Compose([transforms.ToTensor()])
        x = transform(x)
        x = x.unsqueeze(0).cuda()
        t0 = time.time()
        y = model(x)
        y = y.cpu().data.numpy().copy()
        prob_map = np.squeeze(1-np.argmin(y, axis=1))
        t00 = time.time()
        time1.append(t00-t0)

        # prob_map = (np.load('demo/prob_map/{}.npy'.format(i), allow_pickle=True))
        time.sleep(1)
        start = (points[i][0], points[i][1])
        goal = (points[i][2], points[i][3])
        map_array = load_map("demo/maps/{}.png".format(i), 1)

        RRT_planner = RRT(map_array, start, goal, prob_map)
        RRT_planner.set_save_path("demo/output/{}.png".format(i))

        # Search with RRT and RRT*
        t1 = time.time()
        re = RRT_planner.RRT(n_pts=2000)
        t2 = time.time()
        time2.append(t2-t1)
        if not re:
            un_success.append(i)
        else:
            success.append(i)
            node.append(re[0])
            path_len.append(re[1])
 
    print(len(success))
    print(len(un_success))
    print(np.mean(node))
    print(np.mean(path_len))
    print(np.mean(time1))
    print(np.mean(time2))