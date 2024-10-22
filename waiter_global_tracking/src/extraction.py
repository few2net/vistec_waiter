# -*- coding: utf-8 -*-

# test module
import cv2
import time
###

import os
import torch
import torch.onnx
import torch.nn as nn
import torch.backends.cudnn as cudnn
import numpy as np
from torch.autograd import Variable
from torchvision import datasets, transforms
from extraction_model.model import ft_net
from PIL import Image

from onnx_helper import convert_onnx_to_engine, ONNXClassifierWrapper
from torchvision.transforms.functional import InterpolationMode

class FtNetExtractor:
    def __init__(self, nclasses=751, USE_FP16=False):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cudnn.benchmark = True if self.device!= "cpu" else False
        self.USE_FP16 = USE_FP16

        print("*********************")
        print(f"Device: {self.device}")
        print(f"Cudnn: {cudnn.benchmark}")
        print(f"FP16 mode: {self.USE_FP16}")
        print("*********************")
            
        root_path = os.path.dirname(os.path.realpath(__file__))
        save_path = os.path.join(root_path,'extraction_model/net_last.pth')
        self.model = ft_net(nclasses)
        self.model.load_state_dict(torch.load(save_path))
        
        # Remove the final fc layer and classifier layer
        self.model.classifier.classifier = nn.Sequential()
        
        # Change to test mode
        self.model = self.model.to(self.device)
        self.model = self.model.eval()
        
        if(self.USE_FP16):
            self.model = self.model.half()

        self.preprocess = transforms.Compose([
                            transforms.Resize((256,128), interpolation=InterpolationMode.BICUBIC),
                            transforms.ToTensor(),
                            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                            ])

    def torch_to_trt(self, BATCH_SIZE=16):
        dummy_input = torch.randn(BATCH_SIZE, 3, 256, 128).to(self.device)
        if(self.USE_FP16):
            dummy_input = dummy_input.half()

        # export the model to ONNX
        name = "FtNet16_"+str(BATCH_SIZE) if self.USE_FP16 else "FtNet32_"+str(BATCH_SIZE) 
        torch.onnx.export(self.model, dummy_input, name+".onnx", verbose=False)
        convert_onnx_to_engine(name+".onnx", name+".trt", max_batch_size=1, fp16_mode=self.USE_FP16)

    def extract(self, rgb_img):
        # Input rgb_img is cv2 type, change to PIL.
        rgb_img = Image.fromarray(rgb_img)
        
        # Preprocess
        input_data = self.preprocess(rgb_img)
        input_data = input_data.to(self.device)

        # Transform data to 1 batch size
        input_data = torch.unsqueeze(input_data, 0)

        if(self.USE_FP16):
            input_data = input_data.half()

        with torch.no_grad():
            feature = self.model(input_data)

        return feature.cpu().numpy()[0]

    def extract_batch(self, list_rgb):
        input_batch = torch.tensor([])
        for i in list_rgb:
            i = self.preprocess(Image.fromarray(i))
            i = torch.unsqueeze(i, 0)
            input_batch = torch.cat((input_batch,i),dim=0)

        input_batch = input_batch.to(self.device)

        if(self.USE_FP16):
            input_batch = input_batch.half()

        with torch.no_grad():
            feature = self.model(input_batch)

        return feature.cpu().numpy()
        
class FtNetExtractor_TRT:
    def __init__(self, USE_FP16=False, BATCH_SIZE=16):
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        cudnn.benchmark = True if self.device!= "cpu" else False
        self.USE_FP16 = USE_FP16

        root_path = os.path.dirname(os.path.realpath(__file__))
        if(self.USE_FP16):
            trt_file_path = os.path.join(root_path,'FtNet16_'+str(BATCH_SIZE)+'.trt')
        else:
            trt_file_path = os.path.join(root_path,'FtNet32_'+str(BATCH_SIZE)+'.trt')

        OUTPUT_SIZE = 512  # output features dimention from ft_net
        self.PRECISION = np.float16 if self.USE_FP16 else np.float32
        self.trt_model = ONNXClassifierWrapper(trt_file_path, [BATCH_SIZE, OUTPUT_SIZE], target_dtype = self.PRECISION)

        # First initialize with dummy
        dummy_input_batch = np.zeros((BATCH_SIZE, 3, 256, 128), dtype=self.PRECISION)
        self.trt_model.predict(dummy_input_batch)
        
        self.preprocess = transforms.Compose([
                            transforms.Resize((256,128), interpolation=InterpolationMode.BICUBIC),
                            transforms.ToTensor(),
                            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                            ])
        
    def extract(self, rgb_img):
        # Input rgb_img is cv2 type, change to PIL.
        rgb_img = Image.fromarray(rgb_img)
        # Preprocess
        input_data = self.preprocess(rgb_img).numpy().astype(self.PRECISION)
        return self.trt_model.predict(input_data)[0]

    def extract_batch(self, list_rgb):
        input_batch = torch.tensor([])
        for i in list_rgb:
            i = self.preprocess(Image.fromarray(i))
            i = torch.unsqueeze(i, 0)
            input_batch = torch.cat((input_batch,i),dim=0)

        input_batch = input_batch.numpy().astype(self.PRECISION)
        return self.trt_model.predict(input_batch)

if __name__=='__main__':
    extractor = FtNetExtractor(USE_FP16=True)
    img = cv2.imread("test1.jpg")
    rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    features = extractor.extract_batch([rgb_img])
    print(features[:10,:5])
    print(features[:10,:5])
    #extractor.torch_to_trt(BATCH_SIZE=16)
    extractor_trt = FtNetExtractor_TRT(USE_FP16=True, BATCH_SIZE=16)
    features = extractor_trt.extract_batch([rgb_img])
    print("===============================================")
    print(features[:10,:5])
    print(features[:10,:5])