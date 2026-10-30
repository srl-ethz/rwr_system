from PIL import Image
import numpy as np 
import time
import os

import torch
import torchvision
from torchvision import models, transforms
from torch.utils.data import DataLoader, Dataset

import pytorch_lightning as pl
from einops import rearrange
from einops.layers.torch import Rearrange

import fire 

import torch.nn as nn 
import torch.nn.functional as F

import kornia

LR = 1e-4
softmax_temp = 1

import cv2 
import kornia as K
   
from torchvision.models.detection.mask_rcnn import MaskRCNN
from torchvision.models.detection.backbone_utils import resnet_fpn_backbone
from torchvision.models.detection.rpn import AnchorGenerator
from torchvision.ops import MultiScaleRoIAlign

import yaml 

import torchvision
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from torchvision.models.detection.keypoint_rcnn import KeypointRCNNPredictor, KeypointRCNNHeads

from pathlib import Path

class DetectorMaskRCNN(MaskRCNN):
    def __init__(self, input_resize=(240, 320), n_classes=2,
                 backbone_str='resnet50-fpn',
                 anchor_sizes=((32, ), (64, ), (128, ), (256, ), (512, ))):

        assert backbone_str == 'resnet50-fpn'
        backbone = resnet_fpn_backbone('resnet50', pretrained=False)

        aspect_ratios = ((0.5, 1.0, 2.0),) * len(anchor_sizes)
        rpn_anchor_generator = AnchorGenerator(anchor_sizes, aspect_ratios)

        super().__init__(backbone=backbone, num_classes=n_classes,
                         rpn_anchor_generator=rpn_anchor_generator,
                         max_size=max(input_resize), min_size=min(input_resize))


def check_update_config(cfg):
    return cfg


def create_model_detector(cfg, n_classes):
    # import ipdb; ipdb.set_trace()

    model = DetectorMaskRCNN(input_resize=cfg.input_resize,
                             n_classes=n_classes,
                             backbone_str=cfg.backbone_str,
                             anchor_sizes=cfg.anchor_sizes)
    return model
                

def load_detector(model_path=None):
    if model_path is None:
        model_path = os.path.join(os.path.dirname(__file__),
                                  './pre_trained_cosypose_models/detector-bop-ycbv-synt+real--292971/')

    if not os.path.exists(os.path.join(model_path, 'checkpoint.pth.tar')):
        raise FileNotFoundError(f"{model_path}/checkpoint.pth.tar not found: run download_models.sh in "
                                f"{os.path.dirname(os.path.abspath(__file__))} first")
        
    run_dir = Path(model_path)
    # run_dir = Path('./pre_trained_cosypose_models/detector-bop-ycbv-synt+real--292971/')
    print(run_dir)
    print(run_dir / 'config.yaml')
    print(run_dir / 'checkpoint.pth.tar')
    print(run_dir / 'checkpoint.pth.tar')

    cfg_path = run_dir / 'config.yaml'
    cfg = yaml.load(cfg_path.read_text(), Loader=yaml.Loader)
    
    label_to_category_id = cfg.label_to_category_id
    model = create_model_detector(cfg, len(label_to_category_id))

    ckpt = torch.load(run_dir / 'checkpoint.pth.tar')
    ckpt = ckpt['state_dict']

    model.load_state_dict(ckpt)

    # get the number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features

    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes=2)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256

    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(in_features_mask,
                                                        hidden_layer,
                                                        num_classes=2)

    
    # add keypoint related heads
    model.roi_heads.keypoint_roi_pool = torchvision.ops.MultiScaleRoIAlign(featmap_names=['0', '1', '2', '3'],
                                                                        output_size=14,
                                                                        sampling_ratio=2)

    # add keypoint head 
    num_layers = 8 # not related to number of keypoints
    model.roi_heads.keypoint_head = KeypointRCNNHeads(in_features_mask, [512]*num_layers)

    # add keypoint RCNN predictor 
    num_keypoints = 8
    model.roi_heads.keypoint_predictor = KeypointRCNNPredictor(512, num_keypoints)
    

    return model


# pytorch lightning module
# training maskrcnn from scratch https://bjornkhansen95.medium.com/mask-r-cnn-for-segmentation-using-pytorch-8bbfa8511883

class maskrcnn_detection(pl.LightningModule):
    def __init__(self, **kwargs):
        super(maskrcnn_detection, self).__init__()
        # self.save_hyperparameters()

        self.lr = LR
        self.batch_iter = 0 
        self.learner = load_detector()

        self.keypoints = False
        self.cur_device = None 


    @torch.no_grad()
    def get_detections(self, images):
        return self.learner(images)
    
    def forward(self, images, target):
        loss = self.learner(images, target)
        return loss
            
    def training_step(self, batch, batch_idx):

        if len(batch.keys()) == 0:
            return {'loss': torch.tensor(0.0, device=self.cur_device, requires_grad=True)}

        rgb = batch["img_rgb"]
        segmn = batch["masks"]

        self.cur_device = rgb['camera_0'].device

        if "keypoints_2d" in batch:
            self.keypoints = True
            kpts = batch["keypoints_2d"]
            keypoints = torch.cat([kpts['camera_0'], kpts['camera_1'], kpts['camera_2']], dim=0).float()
            keypoints = torch.cat((keypoints, 
                               torch.ones((keypoints.shape[0], keypoints.shape[1], 1), 
                               device=keypoints.device, dtype=keypoints.dtype)), dim=-1)
        
        source_views = torch.cat([rgb['camera_0'], rgb['camera_1'], rgb['camera_2']], dim=0).float()
        segmentation_mask = torch.cat([segmn['camera_0'], segmn['camera_1'], segmn['camera_2']], dim=0).float()
        bboxes = torch.cat([batch['boxes']['camera_0'], batch['boxes']['camera_1'], 
                            batch['boxes']['camera_2']], dim=0).float()

        num_cam_batch_size = source_views.shape[0]
        labels = torch.ones((num_cam_batch_size, ), dtype=torch.int64, device=source_views.device)
        
        targets = [{'boxes': bboxes[i].unsqueeze(0), 
                    'labels': labels[i].unsqueeze(0), 
                    'masks': segmentation_mask[i].unsqueeze(0),
                    'keypoints': keypoints[i].unsqueeze(0)} for i in range(num_cam_batch_size)]

        images = [source_views[i] for i in range(num_cam_batch_size)]

        # Compute the model loss
        loss_dict = self.forward(images, targets)

        loss = sum(loss for loss in loss_dict.values())

        for k, v in loss_dict.items():
            self.log(k, v)

        # log the loss and other variables to tensorboard
        self.log('loss_train', loss)
        self.log('epoch', self.current_epoch)

        return {'loss': loss}

   
    # How to choose scheduler https://github.com/PyTorchLightning/pytorch-lightning/issues/328
    def configure_optimizers(self):
        optimiser = torch.optim.Adam(self.parameters(), lr=self.lr)

        return optimiser #[optimiser] , [lr_scheduler]


if __name__ == "__main__":

    model = load_detector('./pre_trained_cosypose_models/detector-bop-ycbv-synt+real--292971/')
    import ipdb; ipdb.set_trace()

    def find_min_max(keypts2d):
    
        min_x = np.min(keypts2d[:, 0])
        max_x = np.max(keypts2d[:, 0])
        min_y = np.min(keypts2d[:, 1])
        max_y = np.max(keypts2d[:, 1])

        min_x = np.maximum(min_x-10, 0)
        max_x = np.minimum(max_x+10, 319)

        min_y = np.maximum(min_y-10, 0)
        max_y = np.minimum(max_y+10, 239)

        return min_x, max_x, min_y, max_y

    import webdataset as wds
    import numpy as np
    from PIL import Image

    def keys_transform(data):

            result = dict()
            keypoints_2d_dict = dict()

            keypoints_2d_dict['camera_0'] = np.array(data['cameras']["Camera0"]['keypoints_2d'])
            keypoints_2d_dict['camera_1'] = np.array(data['cameras']["Camera1"]['keypoints_2d'])
            keypoints_2d_dict['camera_2'] = np.array(data['cameras']["Camera2"]['keypoints_2d'])

            result['keypoints_2d'] = keypoints_2d_dict

            return result

    my_transform = transforms.ToTensor()

    def rgb_transform(data):
        return my_transform(np.array(data))
    
    def numpy2tensor_depth(data):
        # data = data.convert('L')
        return torch.zeros([])

    def numpy2tensor_segmn(data):
        data = data.convert('L')
        data = np.array(data)
        data[data==1] = 0
        data[data==2] = 1
        return torch.from_numpy(data)


    dataset = (
                wds.WebDataset('../data/allegro-cube-dataset-000000.tar')
                .shuffle(1000)
                .decode("pilrgb")
                .to_tuple("rgb.png depth.png segmentation.png json")
                .map_tuple(rgb_transform, numpy2tensor_depth, numpy2tensor_segmn, keys_transform)
            )

    dataloader = torch.utils.data.DataLoader(dataset, num_workers=4, batch_size=1)
    import time as time
    start = time.time()
    rgb, dpt, segmn, _json = next(iter(dataloader))
    end = time.time()
    print("time to load image", end - start)

    import cv2 
    # import ipdb; ipdb.set_trace()
    imgs = rgb[0].numpy().transpose(1,2,0)
    from einops import rearrange
    imgs = rearrange(imgs, 'h (k w) c -> h k w c',k=3)

    # import ipdb; ipdb.set_trace()
    imgs = rearrange(segmn[0], 'h (k w) -> k h w', k=3)

    img_0 = np.ascontiguousarray(imgs[0])*255
    img_1 = np.ascontiguousarray(imgs[1])*255
    img_2 = np.ascontiguousarray(imgs[2])*255

    # img_0 = np.ascontiguousarray(imgs[:, 0, :, :])
    # img_1 = np.ascontiguousarray(imgs[:, 1, :, :])
    # img_2 = np.ascontiguousarray(imgs[:, 2, :, :])

    min_x, max_x, min_y, max_y = find_min_max(_json['keypoints_2d']['camera_0'].numpy()[0])
    cv2.rectangle(img_0, (int(min_x), int(min_y)), (int(max_x), int(max_y)), (0, 255, 0), 2)

    min_x, max_x, min_y, max_y = find_min_max(_json['keypoints_2d']['camera_1'].numpy()[0])
    cv2.rectangle(img_1, (int(min_x), int(min_y)), (int(max_x), int(max_y)), (0, 255, 0), 2)

    min_x, max_x, min_y, max_y = find_min_max(_json['keypoints_2d']['camera_2'].numpy()[0])
    cv2.rectangle(img_2, (int(min_x), int(min_y)), (int(max_x), int(max_y)), (0, 255, 0), 2)

    img = np.hstack((img_0, img_1, img_2))
    cv2.imshow('img', img[...,::-1])

    while True:
        k = cv2.waitKey(10)
        # Press q to break
        if k == ord('q'):
            break
