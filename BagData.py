import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
import os
import cv2
import pdb
from onehot import onehot
import torch

# transform = transforms.Compose([transforms.ToTensor(), transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])])
transform = transforms.Compose([transforms.ToTensor()])

class BagDataset(Dataset):

    def __init__(self, transform=None):
       self.transform = transform 
    def __len__(self):
       return len(os.listdir('demo/train/data'))

    def __getitem__(self, idx):
        img_name = os.listdir('demo/train/data')[idx]
        imgA = cv2.imread('demo/train/data/'+img_name)
        imgA = cv2.resize(imgA, (256, 256))
        imgB = cv2.imread('demo/train/label/'+img_name, 0)
        imgB = cv2.resize(imgB, (256, 256))
        imgB = imgB/255
        imgB = imgB.astype('uint8')
        imgB = onehot(imgB, 2)
        imgB = imgB.swapaxes(0, 2).swapaxes(1, 2)
        imgB = torch.FloatTensor(imgB)
        #print(imgB.shape)
        if self.transform:
            imgA = self.transform(imgA)    
        item = {'A':imgA, 'B':imgB}
        return item

bag = BagDataset(transform)
dataloader = DataLoader(bag, batch_size=4, shuffle=True, num_workers=4)
if __name__ =='__main__':
    for batch in dataloader:
        break






