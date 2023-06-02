import torch
from torch.autograd import Variable
import numpy as np
from FCN import FCNs
from FCN import VGGNet 
import cv2
from matplotlib import pyplot as plt
from torchvision import transforms

# the input to the model is of shape N*3*160*160 tensor, N presents the number of samples
x = cv2.imread('/home/huangzj/Sampling_based_path_planning_algorithms/demo/train/data/4.png')
transform = transforms.Compose([transforms.ToTensor()])
x = transform(x)
x = x.unsqueeze(0).cuda()
model = torch.load('checkpoints/fcn_model_115.pt')
model = model.cuda()
model.eval()
# y is the output, of shape N*2*160*160, 2 present the class, [1 0] for background [0 1] for handbag
y = model(x)

y = y.cpu().data.numpy().copy()

y = np.argmin(y, axis=1)
plt.subplot(1, 1, 1) 

plt.subplot(1, 1, 1) 

plt.imshow(np.squeeze(y[0, :, :]), 'gray')
plt.pause(0.5)
plt.savefig('images/re.png')