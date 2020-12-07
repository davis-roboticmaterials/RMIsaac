"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load(
    "//engine/build:isaac.bzl",
    "isaac_http_archive",
    "isaac_new_local_repository",
)

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for various modules
def isaac_packages_workspace():
    isaac_http_archive(
        name = "robotis",
        build_file = clean_dep("//third_party:dynamixel.BUILD"),
        sha256 = "1233525218b59ee9b923124ca688feab7014362c1c9c7ad4a844927f8ec3dba5",
        url = "https://developer.nvidia.com/isaac/download/third_party/robotis_dynamixel_sdk-3-6-2-tar-gz",
        type = "tar.gz",
        strip_prefix = "DynamixelSDK-3.6.2",
        licenses = ["@robotis//:LICENSE"],
    )

    isaac_http_archive(
        name = "assimp",
        build_file = clean_dep("//third_party:assimp.BUILD"),
        sha256 = "60080d8ab4daaab309f65b3cffd99f19eb1af8d05623fff469b9b652818e286e",
        url = "https://developer.nvidia.com/isaac/download/third_party/assimp-4-0-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "assimp-4.0.1",
        licenses = ["@assimp//:LICENSE"],
    )

    isaac_http_archive(
        name = "nvstereomatcher",
        build_file = clean_dep("//third_party:nvstereomatcher.BUILD"),
        sha256 = "c4db7e3641d32f370394570181c5f85fc33667a195c5de7f6bef8d4194e315af",
        url = "https://developer.nvidia.com/isaac/download/third_party/libnvstereomatcher_v5-tar-gz",
        type = "tar.gz",
        strip_prefix = "libnvstereomatcher_v5",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "apriltags",
        build_file = clean_dep("//third_party:apriltags.BUILD"),
        sha256 = "5cfc4bc73fccc4d737685e345060e98e2ce067ac41026f4a1e212b402e6c33c0",
        url = "https://developer.nvidia.com/isaac/download/third_party/april_tag_v5-tar-gz",
        type = "tar.gz",
        strip_prefix = "libapriltagging_v5",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "gmapping_repo",
        url = "https://developer.nvidia.com/isaac/download/third_party/gmapping-6f2ac5a2a2a8637ee844b4096f288f50d27a24cb-tar-gz",
        type = "tar.gz",
        sha256 = "6744340ce03976dc74a41d715a9087bccb4b273aecfb1f9aba18d370f8e635a0",
        licenses = ["https://openslam-org.github.io/gmapping.html"],
    )

    native.bind(
        name = "gmapping",
        actual = "@gmapping_repo//:gmapping",
    )

    isaac_http_archive(
        name = "audio_assets",
        sha256 = "3915240ad6c5fe50f50a84204b6d9b602505f2558fad4c14b27187266f458b25",
        url = "https://developer.nvidia.com/isaac/download/third_party/alsa_audio_assets-v2-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "torch_inference_test_assets",
        build_file = clean_dep("//third_party:torch_inference_test_assets.BUILD"),
        sha256 = "24e10fbb2aae938b9dcfbaa8ceb1fc65b31de33733159c23a1e1d3c545cb8322",
        url = "https://developer.nvidia.com/isaac/download/third_party/torch_inference_test_assets-v2-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "voice_command_detection_model_carter",
        build_file = clean_dep("//third_party:voice_command_detection_model_carter.BUILD"),
        sha256 = "57e1b0f70136f7008b467d02eb97d8f09da45e85ca6a8cb442aca9ea2f3d7b55",
        url = "https://developer.nvidia.com/isaac/download/third_party/vcd_model_carter_v1-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "voice_command_detection_model_kaya",
        build_file = clean_dep("//third_party:voice_command_detection_model_kaya.BUILD"),
        sha256 = "80a8251c81735c88573e17933f553da2aead04771fea2dee76348eddc85d426d",
        url = "https://developer.nvidia.com/isaac/download/third_party/vcd_model_kaya_v1-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "resnet_object_detection_model",
        build_file = clean_dep("//third_party:resnet_object_detection_model.BUILD"),
        sha256 = "c15f5536062a755ffe8dd5ee7425c07f382849c2d5d0d5f1a6505d0904730473",
        url = "https://developer.nvidia.com/isaac/download/third_party/resnet18_detector_dolly_20191122-zip",
        type = "zip",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "tennis_ball_resnet_object_detection_model",
        build_file = clean_dep("//third_party:tennis_ball_resnet_object_detection_model.BUILD"),
        sha256 = "baf27155dcfad51ddf892dc83005c5e592e2cf3f10e927c7429d7045b736c5ea",
        url = "https://developer.nvidia.com/isaac/download/third_party/kaya_tennis_ball_resnet18_v3-tar-gz",
        type = "tar.gz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "dolly_pose_estimation_model",
        build_file = clean_dep("//third_party:dolly_pose_estimation_model.BUILD"),
        sha256 = "40cf8edf6e15dc6544b3c12a151d68391038f6bf18a163049803ecef59d3a660",
        url = "https://developer.nvidia.com/isaac/download/third_party/dolly_pose_estimation_model_v3-tar-gz",
        type = "tar.gz",
        strip_prefix = "dolly_pose_estimation_model_v2",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "dolly_detection_pose_estimation_data",
        build_file = clean_dep("//third_party:dolly_detection_pose_estimation_data.BUILD"),
        sha256 = "02c48f49ae364ec92e022a8bd03db1b0da034ca0fc9001cabff7294ba89c4582",
        url = "https://developer.nvidia.com/isaac/download/third_party/dolly_detection_pose_estimation_data_v3-tar-gz",
        type = "tar.gz",
        strip_prefix = "dolly_detection_pose_estimation_data_v3",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_estimation_cnn_model",
        build_file = clean_dep("//third_party:industrial_dolly_pose_estimation_cnn_model.BUILD"),
        sha256 = "727ccbd88377733d7f4656afba68477268ac35ba5acb3d04d2a062f12cdf45f6",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_perception_models_bldgK_fof_v6-tar-gz",
        type = "tar.gz",
        strip_prefix = "industrial_dolly_perception_models_bldgK_fof_v6",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_estimation_data",
        build_file = clean_dep("//third_party:industrial_dolly_pose_estimation_data.BUILD"),
        sha256 = "85313d10443a81e25f4d9f04f7a86fbaa0962a7a7f2bdb4a3f98127157d51036",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_pose_estimation_data_v3-tar-xz",
        type = "tar.xz",
        strip_prefix = "industrial_dolly_pose_estimation_data_v3",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "industrial_dolly_pose_refinement_data",
        build_file = clean_dep("//third_party:industrial_dolly_pose_refinement_data.BUILD"),
        sha256 = "d5fe51c9867076a356180356566b2022be545240945107bbdbc6e538433dc1e7",
        url = "https://developer.nvidia.com/isaac/download/third_party/industrial_dolly_pose_refinement_data_v1-tar-xz",
        type = "tar.xz",
        strip_prefix = "industrial_dolly_pose_refinement_data_v1",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "sortbot_pose_estimation_models",
        build_file = clean_dep("//third_party:sortbot_pose_estimation_models.BUILD"),
        sha256 = "635d3ab33619c61b530555876e5ac6e1be0043b0ae91ba3a4b7359cac348e519",
        url = "https://developer.nvidia.com/isaac/download/third_party/sortbot_perception_models_v1-tar-gz",
        type = "tar.gz",
        strip_prefix = "sortbot_perception_models_v1",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "sortbot_pose_estimation_data",
        build_file = clean_dep("//third_party:sortbot_pose_estimation_data.BUILD"),
        sha256 = "b5948dfac7655069d108e05a74849ce213f78e365615fc395c209db9c77578f8",
        url = "https://developer.nvidia.com/isaac/download/third_party/sortbot_pose_estimation_data-tar-xz",
        type = "tar.xz",
        strip_prefix = "sortbot_pose_estimation_data",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "openpose_model",
        build_file = clean_dep("//third_party:openpose_model.BUILD"),
        sha256 = "134ac4553d34edf61b5cb91c5db4c124f87ce7462de3a08823c7c4fbee21ce1d",
        url = "https://developer.nvidia.com/isaac/download/third_party/ix-networks-openpose-20190815-tar-xz",
        type = "tar.xz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "openpose_trt_pose_model",
        build_file = clean_dep("//third_party:openpose_model.BUILD"),
        sha256 = "da2e33a16a5d792e0d2983ed26a78be86909d8673e0b9b2035bf5bc2841c59f6",
        url = "https://developer.nvidia.com/isaac/download/third_party/trt-pose-20191107-tar-xz",
        type = "tar.xz",
        licenses = ["https://github.com/NVIDIA-AI-IOT/trt_pose/blob/master/LICENSE.md"],
    )

    isaac_http_archive(
        name = "alsa",
        build_file = clean_dep("//third_party:alsa.BUILD"),
        sha256 = "938832b91e5ac8c4aee9847561f680814d199ba5ad9fb795c5a699075a19fd61",
        url = "https://developer.nvidia.com/isaac/download/third_party/alsa-x86_64-tar-xz",
        type = "tar.xz",
        licenses = ["https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html"],
    )

    # Used for both TX2 and Xavier
    isaac_http_archive(
        name = "alsa_aarch64",
        build_file = clean_dep("//third_party:alsa.BUILD"),
        sha256 = "8b0b1f65bc7fbdf45c30389457c530c423518dd12b32cdddca704bfd0daf0ec9",
        url = "https://developer.nvidia.com/isaac/download/third_party/alsa-aarch64-tar-xz",
        type = "tar.xz",
        licenses = ["https://www.gnu.org/licenses/old-licenses/lgpl-2.1.en.html"],
    )

    isaac_http_archive(
        name = "libi2c_aarch64",
        build_file = clean_dep("//third_party:libi2c.BUILD"),
        sha256 = "0371eb3a1f60a5515f5571e5fc07711eb5d82f575060096fae09dcd0821b7d39",
        url = "https://developer.nvidia.com/isaac/download/third_party/libi2c-0-aarch64_xavier-tar-xz",
        type = "tar.xz",
        strip_prefix = "libi2c",
        licenses = ["https://raw.githubusercontent.com/amaork/libi2c/master/LICENSE"],
    )

    isaac_http_archive(
        name = "vrworks_warp360",
        build_file = clean_dep("//third_party:warp360.BUILD"),
        sha256 = "48225cc6bae5a50f342998cd7bde5015f3402f7371d3c3c8deda23921171d532",
        url = "https://developer.nvidia.com/isaac/download/third_party/vrworks_warp360-3-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "libargus",
        build_file = clean_dep("//third_party:libargus.BUILD"),
        sha256 = "8db8df094efb31a6c945e69001d6f9de3ccdbe8752d4069ef05c79c14ec0af5b",
        url = "https://developer.nvidia.com/isaac/download/third_party/libargus-2019-02-tar-gz",
        type = "tar.gz",
        strip_prefix = "libargus",
        licenses = ["https://raw.githubusercontent.com/pauldotknopf/JetsonTX1Drivers/master/nv_tegra/LICENSE.libargus"],
    )

    isaac_http_archive(
        name = "vicon_datastream",
        build_file = clean_dep("//third_party:vicon_datastream.BUILD"),
        sha256 = "f8e0d88ad53a99e3ef4de21891781c664fb333a7e656967fd1d4230d7538371e",
        url = "https://developer.nvidia.com/isaac/download/third_party/vicon-datastream-sdk-tar-gz",
        type = "tar.gz",
        licenses = ["https://www.vicon.com/products/software/datastream-sdk"],
    )

    isaac_http_archive(
        name = "elbrus_vo",
        build_file = clean_dep("//third_party:elbrus_vo.BUILD"),
        sha256 = "af3d230644c76e59188d4ed2cd69fa97f4bbe8f73afde7ed52231325b7b34164",
        url = "https://developer.nvidia.com/isaac/download/third_party/elbrus_v6_4-tar-xz",
        type = "tar.xz",
        strip_prefix = "elbrus",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "kinova_jaco",
        build_file = clean_dep("//third_party:kinova_jaco.BUILD"),
        sha256 = "a8fa1a09ec98a69ab508176c35e582ed41abb63da430c73b8371940c68739fdd",
        url = "https://developer.nvidia.com/isaac/download/third_party/kinova_ros-1-2-1-tar-gz",
        type = "tar.gz",
        strip_prefix = "kinova-ros-1.2.1",
        licenses = ["https://raw.githubusercontent.com/Kinovarobotics/kinova-ros/master/LICENSE"],
    )

    isaac_http_archive(
        name = "realsense",
        build_file = clean_dep("//third_party:realsense.BUILD"),
        sha256 = "5dafabd13fe3ed23ae6c1f6c7f0c902de580f3a60a8b646e9868f7edc962abf2",
        url = "https://developer.nvidia.com/isaac/download/third_party/librealsense-v2-29-0-tar-gz",
        type = "tar.gz",
        strip_prefix = "librealsense-2.29.0",
        licenses = ["@realsense//:LICENSE"],
    )

    isaac_http_archive(
        name = "opencv_x86_64",
        build_file = clean_dep("//third_party:opencv.BUILD"),
        sha256 = "364b6004167c9ac614fc4051a768777cad3fbaf71cbd486a31540d8195db6a98",
        url = "https://developer.nvidia.com/isaac/download/third_party/opencv-3-3-1-x86_64-18-20190327c-tgz",
        type = "tgz",
        licenses = ["https://opencv.org/license.html"],
    )

    isaac_http_archive(
        name = "opencv_aarch64_jetpack42",
        build_file = clean_dep("//third_party:opencv_jetpack42.BUILD"),
        sha256 = "f50be71f870a5e064b859fd110a73eb075b08b6365847d2fc19c59c2bdebef91",
        url = "https://developer.nvidia.com/isaac/download/third_party/opencv_jetpack_4_2_b150_aarch64_nano-tar-xz",
        type = "tar.xz",
        licenses = ["https://opencv.org/license.html"],
    )

    isaac_http_archive(
        name = "libtensorflow_x86_64",
        build_file = clean_dep("//third_party:libtensorflow_x86_64.BUILD"),
        sha256 = "0f8072830081192ccb71ce5c80b43ea516d4a31be4aefe6fdc03a46a96d266fa",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtensorflow_1_15_0-tar-gz",
        type = "tar.gz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/tensorflow/master/LICENSE"],
    )

    isaac_http_archive(
        name = "libtensorflow_aarch64_jetpack43",
        build_file = clean_dep("//third_party:libtensorflow_aarch64_jetpack43.BUILD"),
        sha256 = "14b38de6fdb024185ec27f61bc6e7ef94c70ce677c6f93b53b8557e16a8c7b2d",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtensorflow_1_15_0_jp43b79-tar-gz",
        type = "tar.gz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/tensorflow/master/LICENSE"],
    )

    # libtorch for x86_64
    isaac_http_archive(
        name = "libtorch_x86_64",
        build_file = clean_dep("//third_party:libtorch_x86_64.BUILD"),
        sha256 = "203ffa86773e5e061ff249345012a23c8acc3feb3f68b93bd2aecbd9ba41c4ae",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtorch_x86_64_1-1-0-v2-tar-xz",
        type = "tar.xz",
        licenses = ["https://github.com/pytorch/pytorch/blob/master/LICENSE"],
    )

    # libtorch for aarch64_jetpack42
    isaac_http_archive(
        name = "libtorch_aarch64_jetpack42",
        build_file = clean_dep("//third_party:libtorch_aarch64_jetpack42.BUILD"),
        sha256 = "3a66d995cd0b7254e82549edc1c09d2a5562f0fe186bb69c5855e3da7ab9f7d0",
        url = "https://developer.nvidia.com/isaac/download/third_party/libtorch_aarch64_jetpack42_1-1-0-v0-tar-gz",
        type = "tar.gz",
        licenses = ["https://github.com/pytorch/pytorch/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "mobilenetv2",
        build_file = clean_dep("//third_party:mobilenetv2.BUILD"),
        sha256 = "a20d0c8d698502dc6a620528871c97a588885df7737556243a3412b39fce85e0",
        url = "https://developer.nvidia.com/isaac/download/third_party/mobilenetv2-1-4-224-tgz",
        type = "tgz",
        licenses = ["https://raw.githubusercontent.com/tensorflow/models/master/research/slim/nets/mobilenet/mobilenet.py"],
    )

    isaac_http_archive(
        name = "mobilenetv2_onnx",
        build_file = clean_dep("//third_party:mobilenetv2_onnx.BUILD"),
        sha256 = "8ce2930074b6025c141fcfee9e2c63bb7183f5f19e27695931ce763956cab098",
        url = "https://rdk-public.s3.amazonaws.com/test_data/mobilenetv2-1_0_onnx.tar.xz",
        type = "tar.xz",
        licenses = ["https://raw.githubusercontent.com/onnx/models/master/models/image_classification/mobilenet/README.md"],
    )

    isaac_http_archive(
        name = "ml_test_data",
        build_file = clean_dep("//third_party:ml_test_data.BUILD"),
        sha256 = "2916fe0330ed1c2392148fe1ba8f8353ae3b694aa1c50d28d8f3df8f318ad57e",
        url = "https://developer.nvidia.com/isaac/download/third_party/ml_test_data_1_3-tar-xz",
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    # Source: TensorRT/v6.0/6.0.1.5-cl27267773-eed615fe/10.0-r400/Ubuntu18_04-x64/deb
    isaac_http_archive(
        name = "tensorrt_x86_64",
        build_file = clean_dep("//third_party:tensorrt_x86_64.BUILD"),
        sha256 = "e781ccb0bbe6dae9c73b67b225e1657770f908fb7aa9a044be929ab40652cfe1",
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_6-0-1-5-1+cuda10-0_amd64-ubuntu18_04-x64-deb-tar-xz",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    # Source: SDKManager/JetPack_SDKs/4.3/L4T/78_19316_27599411/JETPACK_43_b78/NoDLA
    isaac_http_archive(
        name = "tensorrt_aarch64_jetpack43",
        build_file = clean_dep("//third_party:tensorrt_jetpack43.BUILD"),
        sha256 = "b40a63a14084acc6602925bf361d979615c5c975823c24ff1be94a9227631546",
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_6-0-1-9-1+cuda10-0_arm64-jetpack_43_b78-nodla-tar-xz",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    # Source: SDKManager/JetPack_SDKs/4.3/L4T/78_19316_27599411/JETPACK_43_b78/DLA
    isaac_http_archive(
        name = "tensorrt_aarch64_jetpack42_dla",
        build_file = clean_dep("//third_party:tensorrt_jetpack42.BUILD"),
        sha256 = "da01870cb65b4c50e07049749bf5b04456f087755ea673e12df12fc796b49132",
        url = "https://developer.nvidia.com/isaac/download/third_party/tensorrt_6-0-1-9-1+cuda10-0_arm64-jetpack_43_b78-dla-tar-xz",
        type = "tar.xz",
        licenses = ["https://docs.nvidia.com/deeplearning/sdk/tensorrt-sla/index.html"],
    )

    isaac_http_archive(
        name = "redtail",
        build_file = clean_dep("//third_party:redtail.BUILD"),
        sha256 = "a25fa2b181606f781220fcc22945ddb483d9d00fe093f113c4e79abb3e556013",
        url = "https://developer.nvidia.com/isaac/download/third_party/redtail-20190625-cc2745047cf5a0964bdd3a38fc8e851491e48e75-zip",
        type = "zip",
        strip_prefix = "redtail-cc2745047cf5a0964bdd3a38fc8e851491e48e75",
        licenses = ["@redtail//:LICENSE.md"],
    )

    isaac_http_archive(
        name = "tacotron2_model",
        build_file = clean_dep("//third_party:tacotron2_model.BUILD"),
        sha256 = "ffb88e4734700521925fec5926a4b29336b261368f1b02d2d61f5bb3f6d95d40",
        url = "https://developer.nvidia.com/isaac/download/third_party/tacotron2_streaming_fp32-v1-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "waveglow_model",
        build_file = clean_dep("//third_party:waveglow_model.BUILD"),
        sha256 = "a3a08e91470f8870a56e4fc4ff6fe479c31797f8d846200958f77733fa1d6cbb",
        url = "https://developer.nvidia.com/isaac/download/third_party/waveglow_randVect_noiseTrunc_fp16-v0-tar-gz",
        type = "tar.gz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "libargus_aarch64_nano",
        build_file = clean_dep("//third_party:libargus_nano.BUILD"),
        sha256 = "2a618b61387c19ff02003b5915b9f5390f6cd007243dcac7dce381333b5d32dd",
        url = "https://developer.nvidia.com/isaac/download/third_party/libargus-jetpack43-tar-gz",
        type = "tgz",
        strip_prefix = "libargus",
        licenses = ["https://raw.githubusercontent.com/pauldotknopf/JetsonTX1Drivers/master/nv_tegra/LICENSE.libargus"],
    )

    isaac_http_archive(
        name = "hgmm_impl",
        sha256 = "c5243683e164eb84da59702f4ac9f06794f795c79d3c335a174abfb7318af8b6",
        url = "https://developer.nvidia.com/isaac/download/third_party/libhgmm_impl_bionic_08_09_2019-tar-xz",
        build_file = clean_dep("//third_party:libhgmm_impl.BUILD"),
        type = "tar.xz",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "livox_sdk",
        sha256 = "1c62b3f85a548183100cc94730926b64df2feeb10883f7d3bd245708ef0340a5",
        url = "https://developer.nvidia.com/isaac/download/third_party/Livox-SDK-1-0-0-tar-gz",
        build_file = clean_dep("//third_party:livox_sdk.BUILD"),
        type = "tar.gz",
        strip_prefix = "Livox-SDK-1.0.0",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "lfll",
        url = "https://developer.nvidia.com/isaac/download/third_party/lfll-9d29453368c432e373acf712d51515505ef057b0-tar-gz",
        type = "tar.gz",
        sha256 = "716a1a786612cbce8b3e7ef69b42d7f96a071a0f9974392c193875ea1621eff3",
        build_file = clean_dep("//third_party:lfll.BUILD"),
        patches = [clean_dep("//third_party:lfll.patch")],
        licenses = ["https://github.com/nicopauss/LFLL/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "efll",
        url = "https://developer.nvidia.com/isaac/download/third_party/efll-640b8680b6535768f318172b0a28a5e4091d8f60-tar-gz",
        type = "tar.gz",
        sha256 = "0e4ab56295c3efb4ddd3611505d1a63c067dd79da8f41d71a9b5017275dbb924",
        build_file = clean_dep("//third_party:efll.BUILD"),
        licenses = ["https://github.com/zerokol/eFLL/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "object_pose_estimation_aae",
        build_file = clean_dep("//third_party:object_pose_estimation_aae.BUILD"),
        sha256 = "c967602105524862f2e075112d751a64dd61ad433ba8ed21905d36c91ae769a8",
        url = "https://developer.nvidia.com/isaac/download/third_party/object_pose_estimation_aae_v2-tar-gz",
        type = "tar.gz",
        strip_prefix = "AugmentedAutoencoder-master",
        licenses = ["https://github.com/DLR-RM/AugmentedAutoencoder/blob/master/LICENSE"],
    )

    isaac_http_archive(
        name = "path_segmentation_images",
        build_file = clean_dep("//third_party:path_segmentation_images.BUILD"),
        sha256 = "da9e7e16613bd480290c7491373922e8dde247f255eee201f583c82c601a453c",
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_images_2019_11_14-tar-xz",
        type = "tar.xz",
        strip_prefix = "path_segmentation_images_2019_11_14",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "path_segmentation_logs",
        build_file = clean_dep("//third_party:path_segmentation_logs.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_logs_2019_11_14-tar-xz",
        sha256 = "2e16fcf7bf922f933aec23ae879f59ed51f88b29f34d7316b663ee470bf7b94e",
        type = "tar.xz",
        strip_prefix = "path_segmentation_logs_2019_11_14",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "path_segmentation_pretrained_models",
        build_file = clean_dep("//third_party:path_segmentation_pretrained_models.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/path_segmentation_pretrained_models_2_2019_11_14-tar-xz",
        sha256 = "0278861657c710d48a9a1ae75378e67612e9f0cae1e1ef49d4c422593f5f3c96",
        type = "tar.xz",
        strip_prefix = "path_segmentation_pretrained_models",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "dolly_docking_reinforcement_learning_policy",
        build_file = clean_dep("//third_party:dolly_docking_reinforcement_learning_policy.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/dolly_docking_frozen_v0-tar-xz",
        sha256 = "36211abde5948ecc94b1d3412b264d64376cf5f919cb6ee12f7fd5a58f35a4a6",
        type = "tar.xz",
        licenses = ["https://creativecommons.org/publicdomain/zero/1.0/legalcode"],
    )

    isaac_http_archive(
        name = "dolly_detection_test_data",
        build_file = clean_dep("//third_party:dolly_detection_test_data.BUILD"),
        sha256 = "e245dc734dd8ad609d4159b2db915a4d78941d2a9d3bd2a3b8947008b2e948fa",
        url = "https://developer.nvidia.com/isaac/download/third_party/dolly_detection_test_data_v1-tar-xz",
        type = "tar.xz",
        strip_prefix = "dolly_detection_test_data",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "sidewalk_segmentation_test_data",
        build_file = clean_dep("//third_party:sidewalk_segmentation_test_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/sidewalk_segmentation_test_data_20191018-tar-xz",
        sha256 = "1d9c1f268d6b779d23251aea8ccadba4cf1882d4a4370edc01f1c478988ca888",
        type = "tar.xz",
        strip_prefix = "sidewalk_segmentation_test_data",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "tlt_parser",
        build_file = clean_dep("//third_party:tlt_parser.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/tlt_parser_20191121-zip",
        sha256 = "1205669b58d78a93b9f5da5656682ffd2b7146014d7501448638d5c1f08ab1a7",
        type = "zip",
        strip_prefix = "tlt_parser",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "crop_and_downsample_test_data",
        build_file = clean_dep("//third_party:crop_and_downsample_test_data.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/crop_and_downsample_test_images_2019_11_20-tar-xz",
        sha256 = "0b7f834fd3be3ed93a823d0c6b480d75482eb3213d8d8b20c0d3776559cc3d91",
        type = "tar.xz",
        strip_prefix = "crop_and_downsample_test_images_2019_11_20",
        licenses = ["//:LICENSE"],
    )

    # GStreamer is a pipeline-based multimedia framework that links together a wide variety of
    # media processing systems. It is a common Linux system component. The host system provides
    # access to the libraries with a variety of licenses depending on your usage. You should
    # review licenses per your usage of GStreamer components.
    isaac_http_archive(
        name = "gstreamer",
        build_file = clean_dep("//third_party:gstreamer.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/gstreamer-1-14-5-tar-xz",
        sha256 = "de5e0686aa3129167681a154aa11b5c5ca1aaec2e61853e82695f510f65052d1",
        type = "tar.xz",
        strip_prefix = "gstreamer-1.14.5",
        licenses = [
            "@gstreamer//:src/gstreamer/COPYING",
            "@gstreamer//:src/gst-plugins-base/COPYING",
        ],
    )

    # GLib is a set of low-level libraries useful for providing data structure handling for C,
    # portability wrappers, execution loops, and interfaces. It is a common Linux system
    # component.
    isaac_http_archive(
        name = "glib",
        build_file = clean_dep("//third_party:glib.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/glib-2-56-4-tar-xz",
        sha256 = "53afb9b9426374ab45fd3508aa765042fd8325df2e11bfb6bc0cd1cd08fe9121",
        type = "tar.xz",
        strip_prefix = "glib-2.56.4",
        licenses = ["@glib//:src/COPYING"],
    )

    # Point Cloud Library http://pointclouds.org/ (BSD License)
    isaac_http_archive(
        name = "pcl",
        build_file = clean_dep("//third_party:pcl.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/pcl-1-9-1-tar-xz",
        sha256 = "dc78b9561aa4ba315cb37ad4e855fc3349ad8746b2e9cbecb7f15a2b03901584",
        type = "tar.xz",
        strip_prefix = "pcl-pcl-1.9.1",
        licenses = ["@pcl//:LICENSE.txt"],
    )

    isaac_http_archive(
        name = "flann",
        build_file = clean_dep("//third_party:flann.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/flann-06a49513138009d19a1f4e0ace67fbff13270c69-tar-gz",
        type = "tar.gz",
        sha256 = "a0da614ba918dc536f84173e7427420cb229a47144e695c47132d5202a9c9bf0",
        patches = [clean_dep("//third_party:flann.patch")],
        licenses = ["//:COPYING"],
    )

    # Deps for USD
    isaac_http_archive(
        name = "open_subdiv",
        build_file = clean_dep("//third_party:open_subdiv.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/OpenSubdiv-3_4_0-tar-gz",
        sha256 = "d932b292f83371c7518960b2135c7a5b931efb43cdd8720e0b27268a698973e4",
        type = "tar.gz",
        licenses = ["@open_subdiv//:LICENSE.txt"],
    )

    isaac_http_archive(
        name = "lcm",
        build_file = clean_dep("//third_party:lcm.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/lcm-1-4-0-tar-xz",
        sha256 = "2a96753c31b77b9d0afbaa40863974e641223505cd704449e8470b1dc4167b8d",
        type = "tar.xz",
        strip_prefix = "lcm-1.4.0",
        licenses = ["//:COPYING"],
    )

    isaac_http_archive(
        name = "laikago_sdk",
        build_file = clean_dep("//third_party:laikago_sdk.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/laikago_sdk_V1-1-3-tar-xz",
        sha256 = "2e7d3c15262c7b72f5ea75ed66e7b224cc2c373986ea37d730d1a90cdf5eebd3",
        type = "tar.xz",
        strip_prefix = "laikago_sdk",
        licenses = ["//:LICENSE"],
    )

    isaac_http_archive(
        name = "pose2_grid_graphs_factory",
        build_file = clean_dep("//third_party:pose2_grid_graphs_factory.BUILD"),
        url = "https://developer.nvidia.com/isaac/download/third_party/pose2_grid_graphs_factory-tar-xz",
        sha256 = "4106ed2637389f6fb0071c5b3586be960a556a520becdecf47f089fd507a559c",
        type = "tar.xz",
        strip_prefix = "pose2_grid_graphs_factory",
        licenses = ["//:LICENSE"],
    )