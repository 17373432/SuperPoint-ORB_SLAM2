# SuperPoint-SLAM

​	该仓库引用自https://github.com/KinglittleQ/SuperPoint_SLAM，比较了该算法与 [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) 算法，在具体修改的地方添加了注释。

​	使用`Pytorch C ++ API`来实现`SuperPoint`模型，该模型用于特征点和描述子提取，所以需要修改原有代码中提取`FAST`特征点以及计算`brief`描述子部分代码。

- ### 新建`include/SuperPoint.h`

  - ####  `SuperPoint`结构体

    ​	函数有构造函数、`forward()`函数。

    ​	属性有模型使用的卷积层。

  - #### `SPDetector`类

    ​	函数有构造函数、`detect()`函数、`getKeyPoints()`函数、`computeDescriptors()`函数。

    ​	属性有网络模型、网络输出的每个像素为特征点的概率大小以及描述信息。

- ### 新建`src/SuperPoint.cc`

  - #### `forward()`函数

    ​	正向传播函数，网络结构如下，前四层为公共层，每层两次卷积一次池化，激活函数选用`ReLU`，公共层通过特征点检测层得到每个像素为特征点的概率大小，通过描述子检测层得到描述信息。

  ```c++
  x = torch::relu(conv1a->forward(x));
  x = torch::relu(conv1b->forward(x));
  x = torch::max_pool2d(x, 2, 2);
  
  x = torch::relu(conv2a->forward(x));
  x = torch::relu(conv2b->forward(x));
  x = torch::max_pool2d(x, 2, 2);
  
  x = torch::relu(conv3a->forward(x));
  x = torch::relu(conv3b->forward(x));
  x = torch::max_pool2d(x, 2, 2);
  
  x = torch::relu(conv4a->forward(x));
  x = torch::relu(conv4b->forward(x));
  
  auto cPa = torch::relu(convPa->forward(x));
  auto semi = convPb->forward(cPa);  // [B, 65, H/8, W/8]
  
  auto cDa = torch::relu(convDa->forward(x));
  auto desc = convDb->forward(cDa);  // [B, d1, H/8, W/8] 
  ```
  - #### `detect()`函数

    ​	先使用`torch::from_blob()`函数将输入图片转换为张量，并进行归一化。之后选择使用`GPU`还是`CPU`，并将模型加载到相应的设备上。将输入通过`forward()`函数得到两层输出，分别为各点为特征点的概率与描述信息。

  - #### `getKeyPoints()`函数

    ​	将各点为特征点的概率与设置的阈值比较，提取特征点，之后根据输入信息选择是否使用非极大值抑制。

  - #### `computeDescriptors()`函数

    ​	对输入的描述信息使用`torch::grid_sampler()`函数进行双线性插值得到完整描述子，之后使用`torch::norm()`函数进行`L2`标准化得到单位长度的描述。

  - #### `SPdetect()`函数

    ​	上述三个函数的结合。

- ### 将`src/ORBextractor.cc`修改为`src/SPextractor.cc`

  - #### 将`ORBextractor()`函数修改为`SPextractor()`函数

    ​	添加`torch::load()`导入训练好的模型，并删除计算特征点方向部分代码（用于计算之后的`brief`描述子）。

  - #### 修改`ComputeKeyPointsOctTree()`函数

    ​	删去原先使用的提取`FAST`角点算法以及计算特征点方向信息部分代码，使用`src/SuperPoint.cc`中的`detect()`函数，得到各点为特征点的概率与描述信息，之后使用`getKeyPoints()`函数来提取特征点,并使用`computeDescriptors()`函数来计算描述信息。

- ### 将`src/ORBmatcher.cc`修改为`src/SPmatcher.cc` 

  - #### 修改`DescriptorDistance()`函数

    ​	将汉明距离修改为`(float)cv::norm(a, b, cv::NORM_L2)`，即`L2`范数。同时将该文件中其他距离的数据类型从`int`改为`float`。

- ### 修改上述文件相关的`.h`文件以及`CMakeLists.txt`文件。

#### 使用自带摄像头方法

​	`myslam.yaml`中存放自带摄像头参数，`myslam.cpp`中`cv::VideoCapture cap()`函数指定使用的摄像头设备号，成功编译后执行生成的`myslam`即可使用自带摄像头。

# SuperPoint-SLAM

**UPDATE: Add citation**

This repository was forked from ORB-SLAM2 https://github.com/raulmur/ORB_SLAM2.  SuperPoint-SLAM is a modified version of ORB-SLAM2 which use SuperPoint as its feature detector and descriptor. The pre-trained model of SuperPoint  come from https://github.com/MagicLeapResearch/SuperPointPretrainedNetwork.

**NOTE: SuperPoint-SLAM is not guaranteed to outperform ORB-SLAM. It's just a trial combination of SuperPoint and ORB-SLAM. I release the code for people who wish to do some research about neural feature based SLAM.**

![overview](pic/overview.png)

![traj](pic/traj.png)

## 1. License (inherited from ORB-SLAM2)

See LICENSE file.

## 2. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

### C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### DBoW3 and g2o (Included in Thirdparty folder)
We use modified versions of [DBoW3](https://github.com/rmsalinas/DBow3) (instead of DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

### Libtorch

We use Pytorch C++ API to implement SuperPoint model. It can be built as follows:

``` shell
git clone --recursive -b v1.0.1 https://github.com/pytorch/pytorch
cd pytorch && mkdir build && cd build
python ../tools/build_libtorch.py
```

It may take quite a long time to download and build. Please wait with patience.

**NOTE**: Do not use the pre-built package in the official website, it would cause some errors.

## 3. Building SuperPoint-SLAM library and examples

Clone the repository:
```
git clone https://github.com/KinglittleQ/SuperPoint_SLAM.git SuperPoint_SLAM
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *SuperPoint_SLAM*. Please make sure you have **installed all required dependencies** (see section 2). Execute:
```
cd SuperPoint_SLAM
chmod +x build.sh
./build.sh
```

This will create **libSuerPoint_SLAM.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **mono_euroc** in *Examples* folder.

**TIPS:**

If cmake cannot find some package such as OpenCV or EIgen3, try to set XX_DIR which contain XXConfig.cmake manually. Add the following statement into `CMakeLists.txt`  before `find_package(XX)`:

``` cmake
set(XX_DIR "your_path")
# set(OpenCV_DIR "usr/share/OpenCV")
# set(Eigen3_DIR "usr/share/Eigen3")
```

## 4. Download Vocabulary

You can download the vocabulary from [google drive](https://drive.google.com/file/d/1p1QEXTDYsbpid5ELp3IApQ8PGgm_vguC/view?usp=sharing) or [BaiduYun](https://pan.baidu.com/s/1fygQil78GpoPm0zoi6BMng) (code: de3g). And then put it into `Vocabulary` directory. The vocabulary was trained on [Bovisa_2008-09-01](http://www.rawseeds.org/rs/datasets/view//7) using DBoW3 library. Branching factor k and depth levels L are set to 5 and 10 respectively.

## 5. Monocular Examples

### KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## 6. Evaluation Results on KITTI

Here are the evaluation results of monocular benchmark on KITTI using RMSE(m) as metric.

| Seq. |  Dimension  |    ORB    | SuperPoint |
| :--: | :---------: | :-------: | :--------: |
|  00  |  564 x 496  | **5.33**  |     X      |
|  01  | 1157 × 1827 |     X     |     X      |
|  02  |  599 × 946  | **21.28** |     X      |
|  03  |  471 × 199  |   1.51    |  **1.04**  |
|  04  |  0.5 × 394  |   1.62    |  **0.35**  |
|  05  |  479 × 426  |   4.85    |  **3.73**  |
|  06  |  23 × 457   | **12.34** |   14.27    |
|  07  |  191 × 209  | **2.26**  |    3.02    |
|  08  |  808 × 391  |   46.68   | **39.63**  |
|  09  |  465 × 568  | **6.62**  |     X      |
|  10  |  671 × 177  |   8.80    |  **5.31**  |

## Citation

If you find this useful, please cite our paper.
```
@inproceedings{deng2019comparative,
  title={Comparative Study of Deep Learning Based Features in SLAM},
  author={Deng, Chengqi and Qiu, Kaitao and Xiong, Rong and Zhou, Chunlin},
  booktitle={2019 4th Asia-Pacific Conference on Intelligent Robot Systems (ACIRS)},
  pages={250--254},
  year={2019},
  organization={IEEE}
}
```

