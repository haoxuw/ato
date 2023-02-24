# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.


import logging
import os

import numpy as np
import pytorch_lightning as pl
import torch
from torch import nn
from torch.nn import functional as F
from torch.utils.data import DataLoader, Dataset, TensorDataset, random_split
from torchvision import transforms

from learning.learn_kinematics import forward_kinematics, pl_module_wrapper

# pylint: disable=unused-argument,unused-variable


class InverseKinematics(pl_module_wrapper.PlModuleWrapper):
    def __init__(
        self,
        depth,
        width,
        data_x_filepath=None,
        data_y_filepath=None,
        batch_size=32,
        rotation_ranges=((0, 270), (10, 170), (0, 270), (10, 170), (0, 270), (10, 170)),
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.depth = depth
        self.width = width
        self.batch_size = batch_size
        self.in_features = self.num_actuators + 12
        self._trainer = None
        self.save_hyperparameters()
        self.ik_dense = self._dense_network(
            depth=depth,
            input_width=self.in_features,
            hidden_width=width,
            output_width=6,
        )
        self.ik_conv = self._conv_1d_network(
            kernel_size=2,
            input_width=self.in_features,
            input_channel=1,
            hidden_channel=width,
            output_channel=6,
            tanh_in_every=-1,
        )  # input (b, 12+self.num_actuators, 1) output (b, 1, 6)
        self.ik_last_activation = nn.ReLU6()

        rotation_ranges = np.array(rotation_ranges)
        self.rotation_ranges_min_np = rotation_ranges[:, 0]
        self.rotation_ranges_min = torch.tensor(self.rotation_ranges_min_np).to(
            torch.device(self.device_id)
        )
        self.rotation_ranges_max_np = rotation_ranges[:, 1]
        self.rotation_ranges_max = torch.tensor(self.rotation_ranges_min_np).to(
            torch.device(self.device_id)
        )
        self._setup_datasets(
            data_x_filepath=data_x_filepath,
            data_y_filepath=data_y_filepath,
            batch_size=batch_size,
        )

    # input: x.shape == (18,) , i.e. current_actuator_positions+current_pose+target_pose
    # output: y.shape == (6,) , i.e. target_actuator_positions
    def forward(self, x):
        x_batch = x.view(-1, 1, self.in_features)  # 12 <- 6 joint
        conv = self.ik_conv(x_batch).view(-1, self.num_actuators)
        dense = self.ik_dense(x_batch).view(-1, self.num_actuators)

        inferenced_actuator_positions = self.ik_last_activation(dense + conv)
        return inferenced_actuator_positions.view(-1, self.num_actuators)

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-3)
        return optimizer

    def _get_batch_loss(self, batch, batch_idx, logging_interval=1000):
        x_batch, y_batch = batch
        assert np.array(x_batch.shape)[1:] == (self.in_features,)
        x_batch_2D = x_batch.view(-1, 1, self.in_features)
        conv = self.ik_conv(x_batch_2D).view(-1, self.num_actuators)
        dense = self.ik_dense(x_batch_2D).view(-1, self.num_actuators)

        y_batch_hat = self.ik_last_activation(dense + conv)
        loss = F.mse_loss(y_batch_hat, y_batch)

        if logging_interval > 0 and (batch_idx % logging_interval) == 0:
            # current_pose is used as auxiliary feature
            x_positions = x_batch.cpu().detach().numpy()[:, :6]
            denormalized_x_positions = self.denormalize_relu6(x_positions)
            denormalized_y_ref = self.denormalize_relu6(y_batch.cpu().detach().numpy())
            denormalized_y_hat = self.denormalize_relu6(
                y_batch_hat.cpu().detach().numpy()
            )

            # actual difference
            denormalized_infer_delta = denormalized_x_positions - denormalized_y_hat

            # baseline difference, i.e. the model could just output denormalized_x_positions, since it's part of the input
            denormalized_ref_delta = denormalized_x_positions - denormalized_y_ref

            # relative difference, 0 is baseline , 100 is perfect, 200 and -200 are both bad
            delta_percentage = (
                (denormalized_ref_delta - denormalized_infer_delta)
                / denormalized_ref_delta
                * 100
            )
            logging.info(
                f"\n x_batch[0]: {x_batch[0]} \n y_batch    [0]: {y_batch[0]} \n y_batch_hat[0]: {y_batch_hat[0]} \n denormalized_infer_delta[0]: {denormalized_infer_delta[0]} \n denormalized_ref_delta[0]: {denormalized_ref_delta[0]} \n delta_percentage[0]: {delta_percentage[0]}"
            )
        return loss

    def denormalize_relu6(self, np_array):
        np_array = np_array.astype(np.float32)
        return self.rotation_ranges_min_np + np.multiply(
            (self.rotation_ranges_max_np - self.rotation_ranges_min_np), np_array / 6.0
        )

    def training_step(self, train_batch, batch_idx):
        loss = self._get_batch_loss(
            batch=train_batch, batch_idx=batch_idx, logging_interval=1000
        )
        self.log("train_loss", loss)
        return loss

    def validation_step(self, val_batch, batch_idx):
        loss = self._get_batch_loss(
            batch=val_batch, batch_idx=batch_idx, logging_interval=10
        )
        self.log("val_loss", loss)

    def test_step(self, batch, batch_idx):
        return self.validation_step(batch, batch_idx)

    def _get_trainer(self, new_trainer=False, max_epochs=-1, ckpt_path=None):
        if self._trainer is None or new_trainer:
            self._trainer = pl.Trainer(
                gpus=1,
                num_nodes=1,
                precision=32,
                max_epochs=max_epochs,
                limit_train_batches=0.5,
                enable_checkpointing=True,
                default_root_dir=self.default_root_dir,
                resume_from_checkpoint=ckpt_path,
            )
        return self._trainer
