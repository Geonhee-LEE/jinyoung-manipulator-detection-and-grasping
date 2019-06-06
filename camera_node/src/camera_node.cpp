#include "../include/camera_node.hpp"


camera _camera;
// kalman filter

bool cal_complete = false;
cv::Mat image, image_hsv, image_hue, image_hist, roi, roi_hsv, roi_hist, mask, backproj, histimg = cv::Mat::zeros(200, 320, CV_8UC3);
bool mode = false;
bool selectObject = false;
int trackObject = 0;
cv::Rect selection;
cv::Point2i origin;
int vmin=10, vmax=256, smin = 30, thresh = 100, end_canny_lowThreshold = 50,end_canny_highThreshold = 100;
float hranges[] = {0, 180};
int hsize = 16;
const float* phranges = hranges;
cv::Rect trackWindow;

uint16_t _objectTopLeft_depth, _objectTopRight_depth, _objectBottomLeft_depth ,_objectBottomRight_depth;
uint16_t _depth_1, _depth_2, _depth_3, _depth_4;
cv::KalmanFilter KF(4,2,0); // state 4, measure 2;
std::vector<cv::Point> fixedcamera_Object,kalmanv;
cv::Mat_<float> measurement(2,1);

void multi_publish()
{

  ros::NodeHandle nh_pub;
  ros::Publisher pub = nh_pub.advertise<std_msgs::Float32MultiArray>("FixedCamera_Object",100);

  std_msgs::Float32MultiArray msg_array;
  ros::Rate loop_rate(100);


  while(ros::ok())
  {
  //  double error_x = abs(_camera._current_x - _camera.calculate_x);
  //  double error_y = abs(_camera._current_y - _camera.calculate_y);

  //  if(error_x > 10 || error_y > 10)
    {
      msg_array.data.clear();
      msg_array.data.push_back(_camera.calculate_x);
      msg_array.data.push_back(_camera.calculate_y);
      msg_array.data.push_back(_camera.Object_angle);
      msg_array.data.push_back(_camera.enable_gripper);
      msg_array.data.push_back(_camera.statePt.x);
      msg_array.data.push_back(_camera.statePt.y);
      msg_array.data.push_back(_camera.enable_depth);
      msg_array.data.push_back(_camera.NKalman_calculate_x);
      msg_array.data.push_back(_camera.NKalman_calculate_y);

      pub.publish(msg_array);

    }
    loop_rate.sleep();
  }

}


//unused function
void getObjectHistogram(cv::Mat &frame, cv::Rect object_region, cv::Mat &globalHistogram, cv::Mat &objectHistogram)
{
  const int channels[] = { 0, 1 };
  const int histSize[] = { 64, 64 };
  float range[] = { 0, 256 };
  const float *ranges[] = { range, range };

  // Histogram in object region
  cv::Mat objectROI = frame(object_region);
  cv::calcHist(&objectROI, 1, channels, cv::noArray(), objectHistogram, 2, histSize, ranges, true, false);


  // A priori color distribution with cumulative histogram
  cv::calcHist(&frame, 1, channels, cv::noArray(), globalHistogram, 2, histSize, ranges, true, true);


  // Boosting: Divide conditional probabilities in object area by a priori probabilities of colors
  for (int y = 0; y < objectHistogram.rows; y++) {
    for (int x = 0; x < objectHistogram.cols; x++) {
      objectHistogram.at<float>(y, x) /= globalHistogram.at<float>(y, x);
    }
  }

  cv::normalize(objectHistogram, objectHistogram, 0, 255, cv::NORM_MINMAX);
}


//unused function
void backProjection(const cv::Mat &frame, const cv::Mat &objectHistogram, cv::Mat &bp) {
  const int channels[] = { 0, 1 };
  float range[] = { 0, 256 };
  const float *ranges[] = { range, range };
  cv::calcBackProject(&frame, 1, channels, objectHistogram, bp, ranges);
}



void select_target(int event, int x, int y, int flags, void* userdata)//camshift
{
  switch (event) {
  case cv::EVENT_LBUTTONDOWN: // left button push -> select

    mode = true;
    selectObject = false;
    origin = cv::Point2i(x,y);
    selection.x = cv::min(x, origin.x);//
    selection.y = cv::min(y, origin.y);//

    break;
  case cv::EVENT_MOUSEMOVE: // mouse move -> draw rectangle
    if(mode)
    {
      cv::rectangle(image,origin,cv::Point(x,y),cv::Scalar(255,0,0),1);
      cv::imshow("image",image);
    }

    break;
  case cv::EVENT_LBUTTONUP: //Left button pull
    if(mode)
    {
      if(origin.x <= x || origin.y <= x)
      {
      cv::rectangle(image,cv::Point(x,y),origin,cv::Scalar(255,0,0),1);
      selection.width = cv::abs(x-origin.x);
      selection.height = cv::abs(y-origin.y);

      roi = image(selection); // roi <- region of interest
      mode = false;
      selectObject = true;
      trackObject = -1;
      cv::imshow("select",roi);
      }

    }
    break;
  }



}

//unused function
void plot_histogram(cv::Mat srcImage, int histSize, float valueRange[], const float*ranges[], int channels, int dims)
{
    if (srcImage.empty())
        return;
    cv::Mat hist;
    cv::calcHist(&srcImage, 1, &channels, cv::Mat(), hist, 1, &histSize, ranges);

    cv::Mat histImage(640, 480, CV_8U);
    cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, CV_32F);

    histImage = cv::Scalar(255);
    int binW = cvRound((double)histImage.cols / histSize);
    int x1, y1, x2, y2;
    for (int i = 0; i < histSize; i++)
    {
        x1 = i*binW;
        y1 = histImage.rows;
        x2 = (i + 1)*binW;
        y2 = histImage.rows - cvRound(hist.at<float>(i));
        cv::rectangle(histImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0), -1);
    }

    //cv::imshow("histImage", histImage);
}



void fixed_imageCallback(const sensor_msgs::ImageConstPtr& msg_image)//fixed camera
{
  try
  {
    cv_bridge::CvImagePtr cv_prt;
    cv_prt = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::RGB8);

    cv::cvtColor(cv_prt->image,image,cv::COLOR_BGR2RGB);// bgr to rgb cv_prt -> image
    cv::Mat check_hsv, img_hsv, img_HSV;
    cv::cvtColor(image,check_hsv,cv::COLOR_RGB2HSV);// (image) RGB to (check_hsv) HSV
    cv::cvtColor(image,img_hsv,cv::COLOR_RGB2HSV);// (image) RGB to (img_hsv) HSV
    cv::cvtColor(cv_prt->image,img_HSV,CV_BGR2HSV);//(cv_prt) BGR to (img_HSV) HSV

    cv::Mat hsv[3];
    cv::split(img_HSV,hsv); // HSV -> H, S, V
//    cv::pyrMeanShiftFiltering(img_HSV, img_HSV, 30, 45, 3);


    // cam shift algorithm

    // target image detect
     cv::setMouseCallback("image",select_target, 0);//Region of interest find ,used mouse event

     if(selectObject) // if find roi
     {
       cv::cvtColor(roi,roi_hsv,cv::COLOR_RGB2HSV); // roi -> hsv
       cv::cvtColor(image,image_hsv,cv::COLOR_RGB2HSV); // image -> hsv

       int _vmin = vmin, _vmax = vmax;

       cv::inRange(image_hsv, cv::Scalar(0, smin, MIN(_vmin,_vmax)),
               cv::Scalar(180, 256, MAX(_vmin, _vmax)), mask);// inrange -> binary

       int ch[] = {0, 0};
       image_hue.create(image_hsv.size(), image_hsv.depth());// hsv -> hue value extraction

       cv::mixChannels(&image_hsv, 1, &image_hue, 1, ch, 1);


      //cv::imshow("HSV",image_hue);

       //cv::Mat image_binary;
       //cv::threshold(image_hue,image_binary,50,255,thresh);
       //cv::imshow("binaryImage",image_binary);

     /*  int histsize = 256;
       float valueRange[] = {0, 256};
       const float*ranges[] = { valueRange };
       int channels = 0;
       int dims = 1;
       plot_histogram(image_hue,histsize,valueRange,ranges,channels,dims);*/


       if(trackObject <0) //make roi to histogram and backproject // roi find -> trackObject = -1 // used kalman filter.
       {
         // Object has been selected by user, set up CAMShift search properties once
        cv::Mat roi(image_hue, selection), maskroi(mask, selection);
        cv::calcHist(&roi, 1, 0, maskroi, image_hist, 1, &hsize, &phranges);
        cv::normalize(image_hist, image_hist, 0, 255, cv::NORM_MINMAX);

        trackWindow = selection;
        trackObject = 1; // Don't set up again, unless user selects new ROI

        histimg = cv::Scalar::all(0);
        int binW = histimg.cols / hsize;
        cv::Mat buf(1, hsize, CV_8UC3);

        for( int i = 0; i < hsize; i++ )
            buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./hsize), 255, 255);
        cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);

        for( int i = 0; i < hsize; i++ )
        {
            int val = cv::saturate_cast<int>(image_hist.at<float>(i)*histimg.rows/255);
            cv::rectangle( histimg, cv::Point(i*binW,histimg.rows),
                       cv::Point((i+1)*binW,histimg.rows - val),
                        cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
        }
       }

         cv::calcBackProject(&image_hue, 1, 0, image_hist, backproj, &phranges);
         backproj &= mask;

         cv::RotatedRect trackBox = cv::CamShift(backproj, trackWindow,
                             cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1 ));

         if( trackWindow.area() <= 1 )
         {
             int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
             trackWindow = cv::Rect(trackWindow.x - r, trackWindow.y - r,
                                trackWindow.x + r, trackWindow.y + r) &
                           cv::Rect(0, 0, cols, rows);
         }

         cv::ellipse(image,trackBox,cv::Scalar(0,0,255), 3, cv::LINE_AA);
         ROS_INFO("size x: %d y: %d",(int)trackBox.size.width,(int)trackBox.size.height);


         if(trackBox.size.width > 70 && trackBox.size.height > 370 )
         {

             //this value used in kalmanfilter
           if(trackBox.center.x < 640 && trackBox.center.y < 480)
           {
             _camera.Object_u = (int)trackBox.center.x;
             _camera.Object_v = (int)trackBox.center.y;
             _camera.Object_angle = (float)trackBox.angle;
             _camera.NKalman_Object_u = (int)trackBox.center.x;
             _camera.NKalman_Object_v = (int)trackBox.center.y;
           }
          }

         else
         {
           if(_camera.Object_u < 640 && _camera.Object_v < 480)
           {
            // _camera.Object_u = _camera.previous_u + _camera.distance_u;
            // _camera.Object_v = _camera.previous_v + _camera.distance_v;
             _camera.Object_u = _camera.previous_u + 2;
             _camera.Object_v = 150;

           }

         }

        //_camera.distance_u = abs(_camera.Object_u - _camera.previous_u);
        // _camera.distance_v = (int)0.1*abs(_camera.Object_v - _camera.previous_v);
        // ROS_INFO("dis x: %d y: %d",_camera.distance_u,_camera.distance_v);
        /*if(_camera.distance_u > 10 || _camera.distance_v > 10)
         {
           _camera.distance_u = 10;
           _camera.distance_v = 0;
         }
        */
         _camera.previous_u = _camera.Object_u;
         _camera.previous_v = _camera.Object_v;
         _camera.previous_angle = (float)trackBox.angle;

         //kalman filter
        cv::Mat prediction = KF.predict();
        cv::Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
        measurement(0) = _camera.Object_u;
        measurement(1) = _camera.Object_v;
        cv::Mat estimated = KF.correct((measurement));//???

        //cv::Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        _camera.statePt.x = estimated.at<float>(0);//??
        _camera.statePt.y = estimated.at<float>(1);//??


       // cv::Point measPt(measurement(0),measurement(1));

        _camera.measPt.x = measurement(0);
        _camera.measPt.y = measurement(1);

        fixedcamera_Object.push_back(_camera.measPt);
        kalmanv.push_back(_camera.statePt);


     }

     cv::line(image, cv::Point( _camera.statePt.x - 5, _camera.statePt.y - 5 ), cv::Point( _camera.statePt.x + 5, _camera.statePt.y + 5 ),
              cv::Scalar(255,255,255), 2, CV_AA, 0);
     cv::line(image, cv::Point( _camera.statePt.x + 5, _camera.statePt.y - 5 ), cv::Point( _camera.statePt.x - 5, _camera.statePt.y + 5 ),
              cv::Scalar(255,255,255), 2, CV_AA, 0 );

     // calculate distance

     if(_camera.statePt.x < 640 && _camera.statePt.x > 300 && _camera.statePt.y <480)
     //if(_camera.statePt.x < 640  && _camera.statePt.y <480)
     {
      // cv::line(image, cv::Point( _camera.measPt.x - 5, _camera.measPt.y - 5 ), cv::Point( _camera.measPt.x + 5, _camera.measPt.y + 5 ), cv::Scalar(0,0,255), 2, CV_AA, 0);
      // cv::line(image, cv::Point( _camera.measPt.x + 5, _camera.measPt.y - 5 ), cv::Point( _camera.measPt.x - 5, _camera.measPt.y + 5 ), cv::Scalar(0,0,255), 2, CV_AA, 0 );
       _camera.dist_pixel_u = 1.923; //500/(460-200)
       //dist_pixel_v = 1.904; //400/(374-164);
       _camera.dist_pixel_v = 1.681;

       //_camera.calculate_x = 457 - (_camera.Object_v - 480/2) * _camera.dist_pixel_v; //camera coordinate 457, 471;
      // _camera.calculate_y = 471 - (_camera.Object_u - 640/2) * _camera.dist_pixel_u;
       _camera.calculate_x = 457 - (_camera.statePt.y - 480/2) * _camera.dist_pixel_v; //camera coordinate 457, 471;
       _camera.calculate_y = 471 - (_camera.statePt.x - 640/2) * _camera.dist_pixel_u;
       _camera.NKalman_calculate_x = 457 - (_camera.NKalman_Object_v - 480/2) * _camera.dist_pixel_v;
       _camera.NKalman_calculate_y = 471 - (_camera.NKalman_Object_u - 640/2) * _camera.dist_pixel_u;
      //  cv::circle(image,cv::Point(320,240),50,cv::Scalar(0,0,255),3);

       ROS_INFO("statePT x: %d y %d",_camera.statePt.x, _camera.statePt.y);
     }


     cv::imshow("image",image);
     cv::waitKey(30);


  //   cv::namedWindow("fixed_HSV");
  //   cv::imshow("fixed_HSV",hsv[0]); //hue


  }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
      return;
    }

}


void EndEffector_imageCallback(const sensor_msgs::ImageConstPtr& msg_image)//end-effector camera
{

  cv_bridge::CvImagePtr img_prt;
  try
  {
    img_prt = cv_bridge::toCvCopy(msg_image, sensor_msgs::image_encodings::RGB8);
    cv::Mat end_image,img_gray, img_binary, img_color, image_hsv, image_hue;

    cv::cvtColor(img_prt->image,end_image,cv::COLOR_BGR2RGB);
    cv::cvtColor(img_prt->image,img_color,cv::COLOR_BGR2RGB);
    cv::cvtColor(end_image,img_gray,CV_RGB2GRAY);
    cv::cvtColor(img_prt->image,image_hsv,CV_BGR2HSV);
    //circle detection

    //cv::threshold(img_gray,img_binary,50,200,thresh);

    cv::Mat hsv[3];
    cv::split(image_hsv,hsv);


    cv::threshold(img_gray, img_binary, thresh, 200, CV_THRESH_BINARY);
  //   cv::cvtColor( img_gray, img_color, cv::COLOR_GRAY2BGR );
  /*  cv::Mat img_labels,stats, centroids;
    int numOfLables = cv::connectedComponentsWithStats(img_binary, img_labels,
                                                   stats, centroids, 8,CV_32S);

     //라벨링 된 이미지에 각각 직사각형으로 둘러싸기
     for (int j = 1; j < numOfLables; j++)
     {
       int area = stats.at<int>(j, cv::CC_STAT_AREA);
       int left = stats.at<int>(j, cv::CC_STAT_LEFT);
       int top  = stats.at<int>(j, cv::CC_STAT_TOP);
       int width = stats.at<int>(j, cv::CC_STAT_WIDTH);
       int height  = stats.at<int>(j, cv::CC_STAT_HEIGHT);
       cv::rectangle( img_binary, cv::Point(left,top), cv::Point(left+width,top+height),
                  cv::Scalar(0,0,255),1 );

       cv::putText(img_binary, std::to_string(j), cv::Point(left+20,top+20),
                         cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0), 2);
      }
*/
     //circle detection
     // Reduce the noise so we avoid false circle detection
      // cv::GaussianBlur( hsv[2], hsv[2], cv::Size(9, 9), 2, 2 );

       std::vector<cv::Vec3f> circles;

       // Apply the Hough Transform to find the circles
       cv::HoughCircles( img_binary, circles, CV_HOUGH_GRADIENT, 1, 20, 200, 50, 0, 0 ); // circle of incluser detection used houghcircle

       // Draw the circles detected
       for( size_t i = 0; i < circles.size(); i++ )
       {
           cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
           int radius = cvRound(circles[i][2]);
           cv::circle( img_binary, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );// circle center
           cv::circle( img_binary, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );// circle outline

        }
      cv::Mat img_canny;
      //canny edge input image, output image, low threshold, high threshold, size
      cv::Canny(img_binary,img_canny,end_canny_lowThreshold,end_canny_highThreshold,3);


     // cv::pyrMeanShiftFiltering(end_image, image_hsv, 30, 45, 1); segmentation

     // cv::imshow("EndEffector_image",end_image);

     // cv::namedWindow("value");
     // cv::imshow("value",hsv[2]);

   //   cv::namedWindow("hue");
   //   cv::imshow("hue",hsv[0]);

   //   cv::namedWindow("canny");
   //   cv::createTrackbar("Low","canny",&end_canny_lowThreshold, 256,0);
   //   cv::createTrackbar("High","canny",&end_canny_highThreshold, 256,0);
   //   cv::imshow("canny",img_canny);

  }
  catch(cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception: %s",e.what());
      return;
  }


}

// detectenabledepth & detectenablegripper in main_window file
void depthCallback(const sensor_msgs::ImageConstPtr& msg_depth)
  {
    cv_bridge::CvImagePtr img_prt_depth;
    try
    {
        img_prt_depth = cv_bridge::toCvCopy(msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
        double min;
        double max;
        cv::minMaxIdx(img_prt_depth->image, &min, &max);
        cv::Mat adjMap;
        //cv::convertScaleAbs(img_prt_depth->image,adjMap, 255 / max);
        img_prt_depth->image.convertTo(adjMap,CV_8UC1, 255 / (max - min), -min);
        cv::Mat falseColorsMap;
        cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_AUTUMN);


   //     cv::namedWindow("depth",CV_WINDOW_AUTOSIZE);
       // cv::imshow("depth", falseColorsMap);
   //     cv::imshow("depth",adjMap);

    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
    }
    cv::Mat mat_depth = img_prt_depth->image;


    //point of interest
    //uint16_t _depth, _objectTopLeft_depth, _objectTopRight_depth, _objectBottomLeft_depth ,_objectBottomRight_depth;
   // _depth = mat_depth.at<uint16_t>(cv::Point((_detect._objectTopLeft_x+_detect._objectBottomRight_x)/2,(_detect._objectTopLeft_y+_detect._objectBottomRight_y)/2));
    _depth_1 = mat_depth.at<uint16_t>(cv::Point(190,270));
    _depth_2 = mat_depth.at<uint16_t>(cv::Point(250,270));
    _depth_3 = mat_depth.at<uint16_t>(cv::Point(190,310));
    _depth_4 = mat_depth.at<uint16_t>(cv::Point(250,310));
    _objectTopLeft_depth = mat_depth.at<uint16_t>(cv::Point(360,195));
    _objectTopRight_depth = mat_depth.at<uint16_t>(cv::Point(430,195));
    _objectBottomLeft_depth = mat_depth.at<uint16_t>(cv::Point(360,210));
    _objectBottomRight_depth = mat_depth.at<uint16_t>(cv::Point(430,210));

    //depth calculate

    if(_depth_1 < 380 && _depth_2 < 380 && _depth_3 < 370 && _depth_4 < 370)
    {
      _camera.enable_depth = 1;
    }
    else
    {
      _camera.enable_depth = 0;
    }

    // gripper calculate
    if(_objectTopLeft_depth < 355 && _objectTopRight_depth < 355 && _objectBottomLeft_depth < 340 && _objectBottomRight_depth < 345 && _camera.enable_depth == 1)
    {
      _camera.enable_gripper = 1;
    }
    else
    {
      _camera.enable_gripper = 0;
    }

    ROS_INFO("Detect gripper depth: %d, %d, %d, %d, %f",
             _objectTopLeft_depth,_objectTopRight_depth,_objectBottomLeft_depth,_objectBottomRight_depth,_camera.enable_gripper );
    ROS_INFO("Detect depth: %d, %d, %d, %d, %f",_depth_1,_depth_2,_depth_3,_depth_4,_camera.enable_depth);
    ROS_INFO("enable_depth: %f, enable_gripper: %f",_camera.enable_depth, _camera.enable_gripper);

  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fixedcamera");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera1/color/image_raw", 1, fixed_imageCallback);

  ros::NodeHandle nh_depth;
  image_transport::ImageTransport it_depth(nh_depth);
  image_transport::Subscriber sub_depth = it_depth.subscribe("/camera2/depth/image_rect_raw/", 10,depthCallback);

  ros::NodeHandle nh2;
  image_transport::ImageTransport it2(nh2);
  image_transport::Subscriber sub2 = it2.subscribe("/camera2/color/image_raw", 10, EndEffector_imageCallback);



  // used kalman filter
  KF.transitionMatrix = (cv::Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
  measurement.setTo(cv::Scalar(0));
  KF.statePre.at<float>(0) = _camera.Object_u;
  KF.statePre.at<float>(1) =_camera.Object_v;
  KF.statePre.at<float>(2) = 0;
  KF.statePre.at<float>(3) = 0;

  cv::setIdentity(KF.measurementMatrix);
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-3));
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(10));
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(.1));

  cv::Mat KF_img(640, 480, CV_8UC3);
  fixedcamera_Object.clear();
  kalmanv.clear();

  cv::namedWindow("image");
  cv::createTrackbar("Vmin","image",&vmin, 256,0);
  cv::createTrackbar("Vmax","image",&vmax, 256,0);
  cv::createTrackbar("Smin","image",&smin, 256,0);

 // cv::createTrackbar("thresh","image",&thresh, 256,0);
  cv::namedWindow("select");

  cv::namedWindow("EndEffector_image");
  cv::createTrackbar("thresh","EndEffector_image",&thresh, 256,0);

  boost::thread th1(boost::bind(&multi_publish));

  ros::spin();
  cv::startWindowThread();
  cv::destroyAllWindows();


}




