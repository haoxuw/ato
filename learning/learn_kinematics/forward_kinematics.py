# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.

import logging

import pytorch_lightning as pl
import torch
from learn_kinematics import pl_module_wrapper


# pylint: disable=unused-argument,unused-variable
class ForwardKinematics(pl_module_wrapper.PlModuleWrapper):
    def __init__(
        self, depth, width, data_x_filepath, data_y_filepath, batch_size=32, **kwargs
    ):
        # calls setup_datasets inside
        super().__init__(**kwargs)
        self.batch_size = batch_size
        self._trainer = None
        self.fk_dense = self._dense_network(
            depth=depth,
            input_width=6,
            hidden_width=width,
            output_width=6,
            tanh_in_every=-1,
        )
        self.fk_conv = self._conv_1d_network(
            kernel_size=2,
            input_width=6,
            input_channel=1,
            hidden_channel=width,
            output_channel=6,
            tanh_in_every=-1,
        )
        self._setup_datasets(
            data_x_filepath=data_x_filepath,
            data_y_filepath=data_y_filepath,
            batch_size=batch_size,
        )

    def forward(self, x):
        x_batch = x.view(-1, 1, 6)
        conv = self.fk_conv(x_batch).view(-1, self.num_actuators)
        dense = self.fk_dense(x_batch).view(-1, self.num_actuators)

        pose = dense + conv
        return pose.view(-1, 6)

    def configure_optimizers(self):
        optimizer = torch.optim.Adam(self.parameters(), lr=1e-3)
        return optimizer

    def _get_batch_loss(self, train_batch, batch_idx, logging_interval=1000):
        x_batch, y_batch = train_batch
        x_batch = x_batch.view(-1, 1, 6)
        conv = self.fk_conv(x_batch).view(-1, self.num_actuators)
        dense = self.fk_dense(x_batch).view(-1, self.num_actuators)

        pose = dense + conv
        loss = torch.nn.functional.mse_loss(pose, y_batch)
        if logging_interval > 0 and (batch_idx % logging_interval) == 0:
            logging.info(
                f"x_batch[0]: {x_batch[0]} \n y_batch    [0]: {y_batch[0]} \n y_batch_hat[0]: {pose[0]}"
            )
        return loss

    def training_step(self, train_batch, batch_idx):
        loss = self._get_batch_loss(
            train_batch=train_batch, batch_idx=batch_idx, logging_interval=1000
        )
        self.log("train_loss", loss)
        return loss

    def validation_step(self, val_batch, batch_idx):
        loss = self._get_batch_loss(
            train_batch=val_batch, batch_idx=batch_idx, logging_interval=100
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
