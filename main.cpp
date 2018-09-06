#include <iostream>
#include <opencv2/opencv.hpp>

#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>

#include <svo/feature.h>
#include <svo/frame.h>
#include <svo/feature_detection.h>

#include <svo/initialization.h>
#include <svo/sparse_img_align.h>

using namespace std;

int main()
{
     cout<<"mytest!!!"<<endl;

    //test1 : feature detection
    // cv::Mat img(cv::imread("/home/server01/Datasets/sin2_tex2_h1_v8_d/img/frame_000002_0.png", 0));
    // vk::AbstractCamera* cam = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
    // svo::FramePtr frame(new svo::Frame(cam, img, 0.0));
    // svo::Features fts;
    
    // svo::feature_detection::FastDetector fast_detector(img.cols, img.rows, 25, 3);
    // fast_detector.detect(frame.get(), frame->img_pyr_, 20, fts);

	// cv::Mat img_rgb = cv::Mat(img.size(), CV_8UC3);
	// cv::cvtColor(img, img_rgb, CV_GRAY2RGB);
    // std::for_each(fts.begin(), fts.end(), [&](svo::Feature* i){
    //     cv::circle(img_rgb, cv::Point2f(i->px[0], i->px[1]), 4*(i->level+1), cv::Scalar(0,255,0), 1);
    // });
    // cv::imshow("ref_img", img_rgb);
    // cv::waitKey(0);

    //test 2 and 3: 
    cv::Mat first_img(cv::imread("/home/server01/Datasets/sin2_tex2_h1_v8_d/img/frame_000002_0.png", 0));
    cv::Mat second_img(cv::imread("/home/server01/Datasets/sin2_tex2_h1_v8_d/img/frame_000004_0.png", 0));
    cv::Mat third_img(cv::imread("/home/server01/Datasets/sin2_tex2_h1_v8_d/img/frame_000005_0.png", 0));

    vk::AbstractCamera* cam = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
    svo::FramePtr first_frame(new svo::Frame(cam, first_img, 0.0));
    svo::FramePtr second_frame(new svo::Frame(cam, second_img, 0.0));
    svo::FramePtr third_frame(new svo::Frame(cam, third_img, 0.0));


    svo::initialization::KltHomographyInit init;
    svo::initialization::InitResult result;
    result = init.addFirstFrame(first_frame);
    result = init.addSecondFrame(second_frame);
    if ( result == svo::initialization::InitResult::FAILURE ) 
        cout<<"FAILURE "<<endl;
    else if (result == svo::initialization::InitResult::NO_KEYFRAME)
        cout<<"NO_KEYFRAME "<<endl;
    else 
        cout<<"SUCCESS "<<endl;


    cout<<first_frame->T_f_w_<<endl;
    cout<<second_frame->T_f_w_<<endl;

    svo::SparseImgAlign img_align(4, 0, 30, svo::SparseImgAlign::GaussNewton, true, true);
    size_t img_align_n_tracked = img_align.run(second_frame, third_frame);

	std::cout << "Img Align:\t Tracked = " << img_align_n_tracked << std::endl;


    cout<<third_frame->T_f_w_<<endl;










    return 0;
}
