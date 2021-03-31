#ifndef SUPERPOINT_H
#define SUPERPOINT_H


#include <torch/torch.h>
#include <opencv2/opencv.hpp>

#include <vector>

#ifdef EIGEN_MPL2_ONLY
#undef EIGEN_MPL2_ONLY
#endif


namespace ORB_SLAM2
{

	struct SuperPoint : torch::nn::Module {
		// 构造函数
		SuperPoint();

		// 正向传播函数
		std::vector<torch::Tensor> forward(torch::Tensor x);

		// SuperPoint使用的卷积层
		torch::nn::Conv2d conv1a;
		torch::nn::Conv2d conv1b;

		torch::nn::Conv2d conv2a;
		torch::nn::Conv2d conv2b;

		torch::nn::Conv2d conv3a;
		torch::nn::Conv2d conv3b;

		torch::nn::Conv2d conv4a;
		torch::nn::Conv2d conv4b;

		torch::nn::Conv2d convPa;
		torch::nn::Conv2d convPb;

		// descriptor
		torch::nn::Conv2d convDa;
		torch::nn::Conv2d convDb;

	};

	// detect()、getKeyPoints()、computeDescriptors()结合版
	cv::Mat SPdetect(std::shared_ptr<SuperPoint> model, cv::Mat img, std::vector<cv::KeyPoint>& keypoints, double threshold, bool nms, bool cuda);
	// torch::Tensor NMS(torch::Tensor kpts);

	class SPDetector {
	public:
		// 构造函数
		SPDetector(std::shared_ptr<SuperPoint> _model);
		// 将输入图片转换为各像素为特征点的概率与描述信息
		void detect(cv::Mat& image, bool cuda);
		// 将各点为特征点的概率与设置的阈值比较，提取特征点
		void getKeyPoints(float threshold, int iniX, int maxX, int iniY, int maxY, std::vector<cv::KeyPoint>& keypoints, bool nms);
		// 计算描述子
		void computeDescriptors(const std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

	private:
		// 网络模型
		std::shared_ptr<SuperPoint> model;
		// 各像素为特征点的概率
		torch::Tensor mProb;
		// 各像素描述信息
		torch::Tensor mDesc;
	};

}  // ORB_SLAM

#endif
