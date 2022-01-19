#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>// measure time // https://qiita.com/yukiB/items/01f8e276d906bf443356
 
int main (int argc, char* argv[])
{
	// if (argc < 2){
	// std::cout << "Usage Example: ./build/main input/json/fountain.json\n";
	// 	std::cerr << "argc: " << argc << "should be 2\n";
	// 	return 1;
	// }	
	std::cout << "Usage Example: ./build/main\n";
	
	const std::string dir_img = "fountain/urd/";
	const int 
		width = 768, 
		height = 512;
	const float 
		width_init = 3072.0,
		height_init = 2048.0;
	const float
		fx_init = 2759.48,
		fy_init = 2764.16,
		cx_init = 1520.69,
		cy_init = 1006.81;
	const float 
		fx = fx_init * width/width_init,
		fy = fy_init * height/height_init,
		cx = cx_init * width/width_init,
		cy = cy_init * height/height_init;

	const cv::Mat K_ = (cv::Mat_<float>(3,3) << fx, 0.0, cx,
											0.0, fy, cy,
											0.0, 0.0, 1.0);
	std::cout << "K: " << K_ << std::endl;
	
	const std::string imgl_path = dir_img + "0006.png";
	const std::string imgr_path = dir_img + "0007.png";
	std::cout << "read img left name: " << imgl_path << std::endl;
	std::cout << "read img right name: " << imgr_path << std::endl;
	cv::Mat srcl, srcr, frame0gray, frame1gray;
	srcl = cv::imread(imgl_path, cv::IMREAD_COLOR);
	srcr = cv::imread(imgr_path, cv::IMREAD_COLOR);
	if (srcl.empty() || srcr.empty()) {
		std::cerr << "failed to load image. check path: " << imgl_path << std::endl;
		std::cerr << "failed to load image. check path: " << imgr_path << std::endl;
		return 1;
	}
	cvtColor(srcl,frame0gray,CV_BGR2GRAY);
	cvtColor(srcr,frame1gray,CV_BGR2GRAY); 
	int ndisparities = 16*5;   /**< Range of disparity */
	int SADWindowSize = 7; /**< Size of the block window. Must be odd */
	cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparities, SADWindowSize);
	cv::Mat dispbm;
	sbm->compute(frame0gray, frame1gray, dispbm);
	double minVal, maxVal;
	// double max_dist = 0;
	// double min_dist = 100;
	cv::minMaxLoc(dispbm, &minVal, &maxVal);
	std::cout << "depth min: " << 1./maxVal << std::endl;
	std::cout << "depth max: " << 1./minVal << std::endl;
    cv::Mat dispnorm_bm;
    dispbm.convertTo(dispnorm_bm, CV_8UC1, 255/(maxVal - minVal));
    cv::imwrite("disparity.png", dispnorm_bm);
	return 0;
}

