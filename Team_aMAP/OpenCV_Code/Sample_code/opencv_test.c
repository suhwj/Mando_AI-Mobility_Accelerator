
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std; 

#define IMG_Width     640
#define IMG_Height    480

#define USE_DEBUG  0;   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용


int main(void)
{
    /////////////////////////////////   영상 변수 선언  ////////////////////////////////////
    int img_width, img_height;
  
    Mat mat_image_org_color(IMG_Height,IMG_Width,CV_8UC3);
    
    mat_image_org_color = imread("./img/line_1.jpg", IMREAD_COLOR); 
    
    img_height = mat_image_org_color.rows;
	  img_width  = mat_image_org_color.cols;
    
    if(mat_image_org_color.empty())
    {
       cerr << "image file error!";
    }
	
    Scalar GREEN(0,255,0);
    Scalar RED(0,0,255);
    Scalar BLUE(255,0,0);
    Scalar YELLOW(0,255,255);
    //////////////////////////////////////////////////////////////////////////////////////

    	
    printf("Image size[%3d,%3d]\n", img_width,img_height);
    
    namedWindow("Display window", cv::WINDOW_NORMAL);
    resizeWindow("Display window", img_width,img_height);
    moveWindow("Display window", 100, 100);
   
	
    while(1)
    {
             		  	     
      imshow("Display window", mat_image_org_color);
   
      // ESC 키를 입력하면 루프가 종료됩니다.   
      if (waitKey(25) >= 0)
      {
            break;
      }
     }		             	
        
    destroyWindow("Display Window");
  
   return 0;	
}
