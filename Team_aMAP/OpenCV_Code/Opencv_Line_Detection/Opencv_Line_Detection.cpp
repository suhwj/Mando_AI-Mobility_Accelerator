
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std; 

#define IMG_Width     1280
#define IMG_Height    720

#define USE_DEBUG  1   // 1 Debug  사용
#define USE_CAMERA 0   // 1 CAMERA 사용  0 CAMERA 미사용

#define ROI_CENTER_Y  100
#define ROI_WIDTH     100

#define NO_LINE 20

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));                 // ³ëÀÌÁî Á¦°Å	
   Canny(mat_blur_img,mat_canny_img, 100,200,3);        // canney edge ¿¬»ê
	
   return mat_canny_img;	
}


Mat Region_of_Interest(Mat image, Point *points)
{
  Mat img_mask =Mat::zeros(image.rows,image.cols,CV_8UC1);	 
  
  Scalar mask_color = Scalar(255,255,255);
  const Point* pt[1]={ points };	    
  int npt[] = { 4 };
  fillPoly(img_mask,pt,npt,1,Scalar(255,255,255),LINE_8);
  Mat masked_img;	
  bitwise_and(image,img_mask,masked_img);
  
  return masked_img;
}

Mat Region_of_Interest_crop(Mat image, Point *points)
{
   Mat img_roi_crop;	

   Rect bounds(0,0,image.cols,image.rows);	 
   Rect r(points[0].x,points[0].y,image.cols, points[2].y-points[0].y);  
   //printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x, points[2].y-points[0].y);
   //printf("%d  %d\n", image.cols, points[2].y-points[0].y);

   img_roi_crop = image(r & bounds);
   
   return img_roi_crop;
}




int main(void)
{
    /////////////////////////////////   영상 변수 선언  ////////////////////////////////////
    int img_width, img_height;
  
    Mat mat_image_org_color(IMG_Height,IMG_Width,CV_8UC3);
    Mat mat_image_org_gray;
    Mat mat_image_roi;
    Mat mat_image_canny_edge;
    
    Point points[4];
    
    int capture_width = 1280 ;
    int capture_height = 720 ;
    int display_width = 640 ;
    int display_height = 360 ;
    int framerate = 60 ;
    int flip_method = 2 ;
    
    img_width  = 640;
    img_height = 360;
	
    if(USE_CAMERA == 0) img_height = 480;
    
    
	float  c[NO_LINE] = {0.0, };
	float  d[NO_LINE] = {0.0, };
	float  line_center_x = 0.0; 
	
	
	std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
    cv::VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
    
   // cap.set(CV_CAP_PROP_FRAME_WIDTH, img_width);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT, img_height);
	
    
	
	if(!cap.isOpened())
	{
		cerr <<"Error , 카메라를 열 수 없습니다. \n";
		mat_image_org_color = imread("./img/line_1.jpg", IMREAD_COLOR); 
		img_height = mat_image_org_color.rows;
	    img_width  = mat_image_org_color.cols;
		//return -1;  
	}
	else
	{
		 printf("카메라가 잘 작동 됩니다.\n"); 
		 cap.read(mat_image_org_color);
	}
	
	
	if(USE_CAMERA == 0)  mat_image_org_color = imread("./img/line_2.jpg", IMREAD_COLOR); 
     
    
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
    
    namedWindow("Display Window", cv::WINDOW_NORMAL);
    resizeWindow("Display Window", img_width,img_height);
    moveWindow("Display Window", 10, 10);
    
    namedWindow("Gray Image Window", cv::WINDOW_NORMAL);
    resizeWindow("Gray Image Window", img_width,img_height);
    moveWindow("Gray Image Window", 700, 10);
    
    namedWindow("Gray ROI Image Window", cv::WINDOW_AUTOSIZE);   
    moveWindow("Gray ROI Image Window", 10, 600);
    
    namedWindow("Canny Edge Image Window", cv::WINDOW_AUTOSIZE);   
    moveWindow("Canny Edge Image Window", 700, 600);
    
   
   
    points[0] =  Point(0,ROI_CENTER_Y-ROI_WIDTH);
	points[1] =  Point(0,ROI_CENTER_Y+ROI_WIDTH);
	points[2] =  Point(img_width,ROI_CENTER_Y+ROI_WIDTH);
	points[3] =  Point(img_width,ROI_CENTER_Y-ROI_WIDTH);
   // imshow("Display Window", mat_image_org_color);
	  

    while(1)
    {
      
      
      if(USE_CAMERA == 1)  cap.read(mat_image_org_color);
      else                 mat_image_org_color = imread("./img/line_1.jpg", IMREAD_COLOR);    
      cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY);        // color to gray conversion  
      mat_image_roi = Region_of_Interest_crop(mat_image_org_gray,points);    // ROI ¿µ¿ªÀ» ÃßÃâÇÔ      
      mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);
      
      vector<Vec4i> linesP;
	  HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/180,30,30,40);
	  printf("Line Number : %3d\n", linesP.size());
	  
	  line_center_x = 0.0;
	  
	  for(int i=0; i<linesP.size();i++)
	  {
		
		float intersect = 0.0;
		
		if(i>=NO_LINE) break;
		Vec4i L= linesP[i];
		
		//int cx1 = linesP[i][0];
		//int cy1 = linesP[i][1];
		//int cx2 = linesP[i][2];
		//int cy2 = linesP[i][3];
		
		c[i] =  (L[2]-L[0])/(L[3]-L[1]);
		d[i] = L[0]-c[i] *L[1] ;
		
		intersect = c[i]*(float)ROI_CENTER_Y +d[i];
		line_center_x += intersect;
		
		line(mat_image_org_color,Point(L[0],L[1]+ROI_CENTER_Y-ROI_WIDTH),Point(L[2],L[3]+ROI_CENTER_Y-ROI_WIDTH), Scalar(0,0,255), 3, LINE_AA);		   
		
		if(USE_DEBUG ==1)
		{
		  printf("L[%d] :[%3d, %3d] , [%3d , %3d] \n",i,  L[0],L[1], L[2],L[3]); 
		 //printf("x[%d] = [%6.3f] *y + [%6.3f] \n",i, c[i],d[i]); 
		  printf("x[%d] = [%f] *y + [%f] \n", i,c[i],d[i]); 
		  printf("intersect =[%f] [%f]\n", intersect, line_center_x);
		//printf("H :[%3d , %3d] , [%3d , %3d] \n", cx1,cy1, cx2,cy2);
	    }
	   } 
	   
	  line_center_x = line_center_x / (float)linesP.size() - img_width/2;
	  if(USE_DEBUG ==1)
	  {
		 printf("Line Center=[%lf]\n",line_center_x);
	     printf("\n\n\n");
	  }
	  line(mat_image_org_color,Point(0,ROI_CENTER_Y),Point(640,ROI_CENTER_Y), Scalar(0,255,0), 1, LINE_AA);	
	  line(mat_image_org_color,Point((int)line_center_x+img_width/2,ROI_CENTER_Y-ROI_WIDTH),Point((int)line_center_x+img_width/2,ROI_CENTER_Y+ROI_WIDTH), Scalar(255,255,0), 1, LINE_AA);	
	   
      
      imshow("Display Window", mat_image_org_color);
      imshow("Gray Image Window", mat_image_org_gray);
      imshow("Gray ROI Image Window",mat_image_roi);  
      imshow("Canny Edge Image Window",mat_image_canny_edge);
          
      // ESC 키를 입력하면 루프가 종료됩니다.   
      if (waitKey(25) >= 0)
      {
            break;
      }
     }		             	
    
    if(USE_CAMERA == 1)  cap.release();    
    destroyAllWindows();

  
   return 0;	
}
