'''
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import PIL.Image
import time
import math
import numpy

from engine.pyalice import *


class PyImageProducer(Codelet):
    def start(self):
        self.tx = self.isaac_proto_tx("ColorCameraProto", "color")
        self.tick_periodically(0.1)
        self.log_info("pyimage start tick over")
        self.cnt = 0

        image_filename = self.config.color_filename
        self.log_info("color_filename {}".format(image_filename))
        self.image = numpy.array(PIL.Image.open(image_filename).convert(mode="RGB"))

    def tick(self):
        # get isaac parameters. Both ways are equivalent
        message = self.tx.init()
        message.colorSpace = 'rgb'

        focal = self.config.focal_length
        message.proto.pinhole.focal.x = focal[0]
        message.proto.pinhole.focal.y = focal[1]

        center = self.config.optical_center
        message.proto.pinhole.center.x = center[0]
        message.proto.pinhole.center.y = center[1]

        message.buffers = [numpy.array(self.image)]
        message.proto.image.dataBufferIndex = 0

        shape = self.image.shape
        message.proto.image.rows = shape[0]
        message.proto.image.cols = shape[1]
        message.proto.image.channels = shape[2]

        message.proto.image.elementType = 'uint8'
        self.tx.publish()

        self.cnt += 1

    def stop(self):
        assert self.cnt > 0, "ticking count {}".format(self.cnt)


class PyImageReader(Codelet):
    def start(self):
        self.rx1 = self.isaac_proto_rx("ColorCameraProto", "rgb_image_1")
        self.rx2 = self.isaac_proto_rx("ColorCameraProto", "rgb_image_2")
        self.tick_on_message(self.rx1)
        self.cnt = 0

    def tick(self):
        img_msg1 = self.rx1.message
        img_msg2 = self.rx2.message
        if img_msg1 is None:
            return

        assert img_msg1.proto.image.rows == img_msg2.proto.image.rows, "Incorrect image rows {} {}".format(
            img_msg1.proto.image.rows, img_msg1.proto.image.rows)
        assert img_msg1.proto.image.cols == img_msg2.proto.image.cols, "Incorrect image cols {} {}".format(
            img_msg1.proto.image.cols, img_msg2.proto.image.cols)
        assert img_msg1.proto.image.channels == img_msg2.proto.image.channels, "Incorrect image cols {} {}".format(
            img_msg1.proto.image.channels, img_msg2.proto.image.channels)
        assert img_msg1.proto.image.elementType == img_msg2.proto.image.elementType, "Incorrect element type {} {}".format(
            img_msg1.proto.image.elementType, img_msg2.proto.image.elementType)

        buffer_data_1 = img_msg1.buffers[img_msg1.proto.image.dataBufferIndex]
        buffer_data_2 = img_msg1.buffers[img_msg1.proto.image.dataBufferIndex]

        expected_bytes_length = img_msg1.proto.image.rows * img_msg1.proto.image.rows * img_msg1.proto.image.channels
        assert len(buffer_data_1) == expected_bytes_length, "Incorrect buffer size {}".format(
            len(buffer_data_1))
        assert buffer_data_1 == buffer_data_2, "Inconsistent buffer data"
        self.cnt += 1

    def stop(self):
        assert self.cnt > 0, "ticking count {}".format(self.cnt)


def main():
    app = Application("engine/pyalice/tests/buffer_test.app.json")
    app.nodes['py_image'].add(PyImageProducer)
    app.nodes['py_reader'].add(PyImageReader)
    app.start_wait_stop(2.0)

    reader_instances = list(app._pycodelet_frontends.items())
    assert len(reader_instances) == 2

    read_cnt = 0
    reader = reader_instances[0][1]
    if isinstance(reader, PyImageReader):
        read_cnt = reader.cnt
    reader = reader_instances[1][1]
    if isinstance(reader, PyImageReader):
        read_cnt = reader.cnt
    assert read_cnt > 0


if __name__ == '__main__':
    main()
