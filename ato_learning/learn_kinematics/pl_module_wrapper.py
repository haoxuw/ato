# This software is forever free for personal and educational usage,

# Copyright (c) 2023, ATOBOT CORP. (1000399600) and/or its affiliates,
# distributed under a multi-licensing model, available for Proprietary and GPL.

# The GPL distribution of this software hopes that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See more details in the LICENSE folder.


import logging
import os
from abc import abstractmethod

import numpy as np
import pytorch_lightning as pl
import torch
import torchsummary
from pytorch_lightning.loggers import TensorBoardLogger
from torch import nn
from torch.utils.data import DataLoader, TensorDataset, random_split


class PlModuleWrapper(pl.LightningModule):
    default_root_dir = "~/.ato/artifacts/"

    def __init__(self, num_actuators=6, **kwargs) -> None:
        super().__init__(**kwargs)
        self.num_actuators = num_actuators
        assert (
            torch.cuda.is_available()
        ), "GPU not available, assuming this is not expected."
        model_name = self.get_cls_name()
        self.tb_logger = TensorBoardLogger(
            os.path.join(self.default_root_dir, "lightning_logs/"),
            name=model_name,
            log_graph=True,
        )
        self.save_hyperparameters()
        self._train_loader = None
        self._val_loader = None
        self._test_loader = None

    def _setup_datasets(self, data_x_filepath, data_y_filepath, batch_size):
        dataset = self._get_dataset(
            data_x_filepath=data_x_filepath,
            data_y_filepath=data_y_filepath,
        )
        self._split_then_setup_datasets(dataset=dataset, batch_size=batch_size)
        index = 0
        self.example_input_array = dataset[index][0]
        logging.info(
            f"example_input_array {self.example_input_array}, {self.example_input_array.shape}"
        )

    @property
    def device_id(self):
        return "cuda:0"

    @abstractmethod
    def _get_trainer(self, new_trainer=False, max_epochs=-1, ckpt_path=None):
        pass

    def _get_dataset(self, data_x_filepath, data_y_filepath):

        data_x = np.load(os.path.expanduser(data_x_filepath))
        data_y = np.load(os.path.expanduser(data_y_filepath))
        logging.info(
            f"Training on {data_x.shape, data_y.shape} {data_x.dtype, data_y.dtype}"
        )
        for index in range(10):
            logging.debug(
                f"Example data_x[{index}]: {data_x[index]} , data_y[{index}]: {data_y[index]}"
            )
        assert data_x.shape[0] == data_y.shape[0]
        dataset = TensorDataset(torch.from_numpy(data_x), torch.from_numpy(data_y))
        return dataset

    def _dense_network(
        self, depth, input_width, hidden_width, output_width, tanh_in_every=-1
    ):
        depth = int(depth)
        assert depth > 1
        seq = [nn.Linear(in_features=input_width, out_features=hidden_width)]
        current_width = hidden_width
        delta_width = (output_width - hidden_width) // depth

        for index in range(depth):
            if index == depth - 1:
                next_width = output_width
            else:
                next_width = current_width + delta_width
            seq += [
                nn.Linear(current_width, next_width),
            ]
            if index != depth - 1:
                if (index % tanh_in_every) == (tanh_in_every - 1):
                    seq += [nn.Tanh()]
                else:
                    seq += [nn.ReLU6()]
            current_width = next_width
        return nn.Sequential(*seq)

    # takes a sequence of (batch, input_width, 1), reduce into an embedding of (batch, output_channel)
    def _conv_1d_network(
        self,
        kernel_size,
        input_width,
        input_channel,
        hidden_channel,
        output_channel,
        tanh_in_every=-1,
    ):
        depth = input_width - 1 - (kernel_size - 1)  # output width is 1
        assert depth > 1
        current_channel = hidden_channel
        seq = [
            nn.Conv1d(
                in_channels=input_channel,
                out_channels=current_channel,
                padding=0,
                kernel_size=kernel_size,
            ),
            nn.ReLU(),
        ]
        delta_channel = (hidden_channel - (output_channel * 2)) // depth
        for index in range(depth):
            next_channel = current_channel - delta_channel
            seq += [
                nn.Conv1d(
                    in_channels=current_channel,
                    out_channels=next_channel,
                    padding=0,
                    kernel_size=kernel_size,
                ),
            ]
            if (index % tanh_in_every) == (tanh_in_every - 1):
                seq += [nn.Tanh()]
            else:
                seq += [nn.ReLU()]
            current_channel = next_channel
        seq += [
            nn.Flatten(),
            nn.Linear(
                in_features=current_channel,
                out_features=output_channel,
            ),
        ]
        return nn.Sequential(*seq)

    def _split_then_setup_datasets(self, dataset, batch_size, split_factor=0.9):
        dataset_size = len(dataset)
        train_size = int(dataset_size * split_factor)
        val_size = int((dataset_size - train_size) * split_factor)
        test_size = dataset_size - train_size - val_size

        logging.info(f"(train, val, test) sizes: {train_size,val_size,test_size}")

        train, val, test = random_split(dataset, [train_size, val_size, test_size])
        self._train_loader = DataLoader(train, batch_size=batch_size)
        self._val_loader = DataLoader(val, batch_size=batch_size)
        self._test_loader = DataLoader(test, batch_size=batch_size)

    @classmethod
    def get_cls_name(cls):
        return cls.__name__

    @classmethod
    def get_ckpt_path(cls, ckpt_version="latest"):
        checkpoint_path = os.path.expanduser(
            os.path.join(
                cls.default_root_dir,
                "lightning_logs",
                cls.get_cls_name(),
                f"{cls.get_cls_name()}_{ckpt_version}.ckpt",
            )
        )
        return checkpoint_path

    @classmethod
    def load_from_checkpoint(cls, ckpt_version="latest"):
        file_path = cls.get_ckpt_path(ckpt_version=ckpt_version)
        if not os.path.isfile(file_path):
            return None
        model = super().load_from_checkpoint(file_path)
        return model

    def fit(self, max_epochs=-1, load_ckpt=True, ckpt_version="latest"):
        logging.info(
            f"Fitting pl model:\n{torchsummary.summary(model = self, input_data = self.example_input_array.shape, depth = 3)}"
        )
        ckpt_path = self.get_ckpt_path(ckpt_version=ckpt_version) if load_ckpt else None
        self._get_trainer(
            new_trainer=True, max_epochs=max_epochs, ckpt_path=ckpt_path
        ).fit(model=self, ckpt_path=ckpt_path)
        self._get_trainer().save_checkpoint(self.get_ckpt_path())

    def test(self):
        self._get_trainer().test(self)

    def train_dataloader(self):
        return self._train_loader

    def val_dataloader(self):
        return self._val_loader

    def test_dataloader(self):
        return self._test_loader

    def on_fit_start(self):
        self.tb_logger.experiment.add_graph(
            self, self.example_input_array.to(self.device_id)
        )

    def on_train_epoch_end(self):
        self._get_trainer().save_checkpoint(self.get_ckpt_path(self.current_epoch))

    def inference_one(self, features, output_shape=(6,)):
        tensor = torch.FloatTensor(features).float()
        output_tensor = self.forward(tensor)
        if output_shape is not None:
            output_tensor = output_tensor.view(output_shape)
        return output_tensor.cpu().detach().numpy()

    def inference_batch(self, features_batch):
        for features in features_batch:
            yield self.inference_one(features)
