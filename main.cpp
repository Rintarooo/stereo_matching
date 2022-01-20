#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>// measure time // https://qiita.com/yukiB/items/01f8e276d906bf443356
 
 void getCW_fromWC(const cv::Mat &Rwc, const cv::Mat &twc, cv::Mat &Rcw, cv::Mat &tcw) {
	/*
	Rwc is rotation matrix from (c)amera frame to (w)orld frame.
	twc is translation vector from (c)amera frame to (w)orld frame.
	*/
	Rcw =  Rwc.t();
	tcw = -Rwc.t()*twc;
}

void file_loader_fou (const std::string &filename, 
	std::vector<std::string> &vec_imgname,  
	std::vector<cv::Mat> &vec_R, 
	std::vector<cv::Mat> &vec_t)
{
	// https://qiita.com/Reed_X1319RAY/items/098596cda78e9c1a6bad
	std::ifstream ifs(filename, std::ios::in);
	if(!ifs.is_open()){
		std::cerr << "Error, cannot open file, check argv: " << filename << std::endl;
		std::exit(1); 
	}
   std::string line;
   // skip 8 line
   for(int i = 0; i < 8; i++){
   	std::getline(ifs, line);
   }
   while (std::getline(ifs, line)){
   	std::stringstream ss(line);// ss << line;
   	std::string imgname;
	    float R00, R01, R02, R10, R11, R12, R20, R21, R22, tx, ty, tz;
	  	ss >> imgname >> R00 >> R01 >> R02 >> R10 >> R11 >> R12 >> R20 >> R21 >> R22 >> tx >> ty >> tz;
      	cv::Mat Rwc = (cv::Mat_<float>(3,3) <<
				   R00, R01, R02,
				   R10, R11, R12,
				   R20, R21, R22);
		cv::Mat twc = (cv::Mat_<float>(3,1) <<
				   tx, ty, tz);
		vec_R.push_back(Rwc);
		vec_t.push_back(twc);
		vec_imgname.push_back(imgname);
	}
	ifs.close();
}


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
	cv::Mat srcl, srcr;
	srcl = cv::imread(imgl_path, cv::IMREAD_COLOR);
	srcr = cv::imread(imgr_path, cv::IMREAD_COLOR);
	if (srcl.empty() || srcr.empty()) {
		std::cerr << "failed to load image. check path: " << imgl_path << std::endl;
		std::cerr << "failed to load image. check path: " << imgr_path << std::endl;
		return 1;
	}
	cvtColor(srcl,srcl,CV_BGR2GRAY);
	cvtColor(srcr,srcr,CV_BGR2GRAY); 

/*// stereo rectification
	const std::string filename = "fountain/p11.txt";
	std::vector<std::string> vec_imgname;
	std::vector<cv::Mat> vec_R, vec_t;
	file_loader_fou (filename, vec_imgname, vec_R, vec_t);
	cv::Mat Rwr, twr, Twr;
	Rwr = vec_R[0];
	twr = vec_R[0];	 
	std::cout << "left R: " << Rwr << std::endl;
	Twr = (cv::Mat_<float>(4,4) <<
		Rwr.at<float>(0,0), Rwr.at<float>(0,1), Rwr.at<float>(0,2), twr.at<float>(0),
		Rwr.at<float>(1,0), Rwr.at<float>(1,1), Rwr.at<float>(1,2), twr.at<float>(1),
		Rwr.at<float>(2,0), Rwr.at<float>(2,1), Rwr.at<float>(2,2), twr.at<float>(2),
		0,					0,					 0,				      1);

	cv::Mat Rwm, twm;// from (w)orld to (m)
	Rwm = vec_R[1];
	twm = vec_R[1];	 
	std::cout << "right R: " << Rwm << std::endl;
	cv::Mat Rmw, tmw, Tmw;// from (w)orld to (m)
	getCW_fromWC(Rwm, twm, Rmw, tmw);
	Tmw = (cv::Mat_<float>(4,4) <<
		Rmw.at<float>(0,0), Rmw.at<float>(0,1), Rmw.at<float>(0,2), tmw.at<float>(0),
		Rmw.at<float>(1,0), Rmw.at<float>(1,1), Rmw.at<float>(1,2), tmw.at<float>(1),
		Rmw.at<float>(2,0), Rmw.at<float>(2,1), Rmw.at<float>(2,2), tmw.at<float>(2),
		0,					0,					 0,				      1);

	cv::Mat Tmr;
	Tmr = Tmw * Twr;// Tmr = Tmw * Trw.inv();
	
	cv::Mat R_relative, t_relative;
	R_relative = (cv::Mat_<float>(3,3) <<
		Tmr.at<float>(0,0), Tmr.at<float>(0,1), Tmr.at<float>(0,2),
		Tmr.at<float>(1,0), Tmr.at<float>(1,1), Tmr.at<float>(1,2),
		Tmr.at<float>(2,0), Tmr.at<float>(2,1), Tmr.at<float>(2,2));
	t_relative = (cv::Mat_<float>(3,1) <<
				   Tmr.at<float>(0,3), Tmr.at<float>(1,3), Tmr.at<float>(2,3));
	R_relative.convertTo(R_relative, CV_64F);
	t_relative.convertTo(t_relative, CV_64F);

	cv::Mat D1, D2;// distortion
	cv::Mat R1, R2, P1, P2, Q;
	// 平行化後のパラメータ(R1,R2,P1,P2,Q)やら計算
	cv::stereoRectify(K_,D1,K_,D2, srcl.size(), R_relative, t_relative, R1, R2, P1, P2, Q);
	cv::Mat K1_new, K2_new;
	cv::Mat left_map1, left_map2, right_map1, right_map2;
	// 平行化のための画素対応を求める
	cv::initUndistortRectifyMap(K_, D1, R1, K1_new, srcl.size(),CV_32FC1, left_map1, left_map2);
	cv::initUndistortRectifyMap(K_, D2, R2, K2_new, srcr.size(),CV_32FC1, right_map1, right_map2);
	cv::Mat srcl_rec, srcr_rec;
	cv::remap(srcl, srcl_rec, left_map1, left_map2, cv::INTER_LINEAR);
	cv::remap(srcr, srcr_rec, right_map1, right_map2, cv::INTER_LINEAR);
	cv::imwrite("srcl_rec.png", srcl_rec);
	cv::imwrite("srcr_rec.png", srcr_rec);
*/// stereo rectification

	int ndisparities = 16*5;   /**< Range of disparity */
	int SADWindowSize = 7; /**< Size of the block window. Must be odd */
	// cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparities, SADWindowSize);
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0,    //int minDisparity
                                    96,     //int numDisparities
                                    5,      //int SADWindowSize
                                    600,    //int P1 = 0
                                    2400,   //int P2 = 0
                                    10,     //int disp12MaxDiff = 0
                                    16,     //int preFilterCap = 0
                                    2,      //int uniquenessRatio = 0
                                    20,    //int speckleWindowSize = 0
                                    30,     //int speckleRange = 0
                                    true);  //bool fullDP = false
	cv::Mat dispbm;
	// sbm->compute(srcl_rec, srcr_rec, dispbm);
	// sbm->compute(srcl, srcr, dispbm);
	sgbm->compute(srcl, srcr, dispbm);
	double minVal, maxVal;
	// double max_dist = 0;
	// double min_dist = 100;
	cv::minMaxLoc(dispbm, &minVal, &maxVal);
	// std::cout << "depth min: " << 1./maxVal << std::endl;
	// std::cout << "depth max: " << 1./minVal << std::endl;
    std::cout << "min: " << minVal << std::endl;
	std::cout << "max: " << maxVal << std::endl;
    cv::Mat dispnorm_bm;
    dispbm.convertTo(dispnorm_bm, CV_8UC1, 255/(maxVal - minVal));
    cv::imwrite("disparity.png", dispnorm_bm);
	return 0;
}

