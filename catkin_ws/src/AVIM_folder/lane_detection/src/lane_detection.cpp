#include<iostream>
#include<math.h>
#include<string>
#include<ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
/*Using image transport for publish and subscribing to image in ros,
allows subscriber to compressed image stream*/
#include <image_transport/image_transport.h>
/*Using the headers for CvBridge, to image encondings*/
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
/*Include the headers for opencv image processing and GUI modules */
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//Library to use time of computer
#include <chrono>
#include <ctime>
using namespace cv;
using namespace std;

//static const std::string OPENCV_WINDOW = "Original";
static const std::string OPENCV_WINDOW = "LANE DETECTION";
class LineDetection{
    private:
        ros::NodeHandle nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        ros::Publisher center_pub;
		ros::Publisher angle_line_pub;
        std_msgs::Int16 center_message;
		std_msgs::UInt8 angle_line_message;
        std::chrono::time_point<std::chrono::system_clock> start,end;
        std::chrono::duration<double> elapsed_seconds;
        stringstream ss;
        /*variable image*/
        Mat frame;
        Mat img_edges;
        Mat img_lines;
        Mat img_perspective;
		vector<Point> left_points,right_points; 
        vector<Point> left_line,right_line,center_line;
        float polyleft[3],polyright[3];
 	    float polyleft_last[3],polyright_last[3];
        Point2f Source[4];
        Point2f Destination[4];
        //vector<Point> Source_points;
		int center_cam;
    	int center_lines;
		int distance_center;
		int angle;
        int fps;

    public:
        //Constructor
        LineDetection():it_(nh_){
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/app/camera/rgb/image_raw",1,&LineDetection::LineDetectionCb, this);
        center_pub = nh_.advertise<std_msgs::Int16>("/distance_center_line",1000);
	    angle_line_pub = nh_.advertise<std_msgs::UInt8>("/angle_line_now",1000);
        cv::namedWindow(OPENCV_WINDOW, WINDOW_KEEPRATIO);
 
        Source[0] = Point2f(140 * 0.5, 315 * 0.5);
        Source[1] = Point2f(500 * 0.5, 315 * 0.5);
        Source[2] = Point2f(0, 400 * 0.5);
        Source[3] = Point2f(640 * 0.5, 400 * 0.5);
        //init points to desinations
        Destination[0]=Point2f(140 * 0.5, 0);
        Destination[1]=Point2f(500 * 0.5, 0);
        Destination[2]=Point2f(140 * 0.5, 480 * 0.5);
        Destination[3]=Point2f(500 * 0.5, 480 * 0.5); 
	   	distance_center = 0.0;
	    center_cam = 0;
        center_lines = 0;
		angle = 0;
        }
        //Destructor
        ~LineDetection(){
          	cv::destroyWindow(OPENCV_WINDOW);
        }
        //Callback funtion
        void LineDetectionCb(const sensor_msgs::ImageConstPtr& msg)
        {
            set_start_time();
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                copyImageRGB(cv_ptr->image,frame);
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
	       	resize_image(frame,0.5);
            Mat copy_frame = frame.clone(); 
            Mat empty_frame = Mat::zeros( frame.rows, frame.cols, frame.type());
            //fillConvexPoly(copy_frame, Source_points, Scalar(0,0,0),8,0);
            Mat invertedPerspectiveMatrix;
            //draw_bird_eye_line(frame, Source, Destination); 
            Perspective(frame, invertedPerspectiveMatrix);
            img_edges = Threshold(frame);
            locate_lanes(img_edges, frame);
            draw_lines(frame);   
            warpPerspective(frame,empty_frame,invertedPerspectiveMatrix,Size(frame.cols,frame.rows));
            bitwise_xor(copy_frame , empty_frame, frame);
            ss.str(" ");
    	    ss.clear();
            ss << "[ANG]:" <<angle << " [FPS]:" << fps;
            putText(frame, ss.str(), Point2f(2,20), 0,1, Scalar(255, 0, 0), 2);
	    /*
            resizeWindow(OPENCV_WINDOW,frame.cols, frame.rows);
            cv::imshow(OPENCV_WINDOW,frame);
            cv::waitKey(10);
            */
	    center_message.data = distance_center;
            center_pub.publish(center_message);
	    	angle_line_message.data = angle;
	    	angle_line_pub.publish(angle_line_message);
            set_end_time();
            fps = FPS_subscriber();
            ROS_INFO("[FPS]: %i ",fps);
        }

        void set_start_time()
        {
            start = std::chrono::system_clock::now();
        }
        void set_end_time()
        {
            end = std::chrono::system_clock::now();
        }
        int FPS_subscriber()
        {
            elapsed_seconds = end-start;
            float t = elapsed_seconds.count();
            return 1/t;
        }
        //Method to change image from BGR to RGB
        void copyImageRGB(Mat & image_BGR,Mat &image_RGB)
        {
            cvtColor(image_BGR, image_RGB, COLOR_BGR2RGB);
        }
        //Method to rotate image 
        void rotate(Mat &src, double angle=180.0)
        {
            Point2f pt(src.cols/2., src.rows/2.);    
            Mat r = getRotationMatrix2D(pt, angle, 1.0);
            warpAffine(src, src, r, Size(src.cols, src.rows));
        }
        void Perspective(Mat &frame,Mat &invertedPerspectiveMatrix)
        {
 	        Mat matrixPerspective( 2, 4, CV_32FC1 );
	        matrixPerspective = Mat::zeros( frame.rows, frame.cols, frame.type() );
            matrixPerspective = getPerspectiveTransform(Source,Destination); 
			warpPerspective(frame,frame,matrixPerspective,Size(frame.cols,frame.rows));
            invert(matrixPerspective, invertedPerspectiveMatrix);
        }
        
        void draw_bird_eye_line(Mat &frame,Point2f *Source,Point2f *Destination)
        {
	        line(frame,Source[0],Source[1],Scalar(0,0,255),2);
            line(frame,Source[1],Source[3],Scalar(0,0,255),2);
            line(frame,Source[2],Source[3],Scalar(0,0,255),2);
            line(frame,Source[2],Source[0],Scalar(0,0,255),2);
            line(frame,Destination[0],Destination[1],Scalar(0,255,0),2);
            line(frame,Destination[1],Destination[3],Scalar(0,255,0),2);
            line(frame,Destination[2],Destination[3],Scalar(0,255,0),2);
            line(frame,Destination[2],Destination[0],Scalar(0,255,0),2);
        }

        Mat Threshold(Mat frame)
        {
	        Mat frameGray;
            Mat sobelx;    
            Mat draw;
            double minVal, maxVal;
	        cvtColor(frame, frameGray, COLOR_BGR2GRAY);
            medianBlur(frameGray, frameGray, 5);
	        inRange(frameGray, 120, 255, frameGray);
	        return frameGray;
        }

	    void resize_image(Mat &input,float alpha=0.15)
        {
		    cv::resize(input,input,Size(input.cols*alpha,input.rows*alpha));
	    }

	    int *Histogram(Mat &img)
        {
            //Create histogram with the length of the width of the frame 
	        vector<int> histogramLane;
	        static int LanePosition[2];
			int init_row,end_row;
	        Mat ROILane;
	        Mat frame;  
		    init_row = img.rows*2/3;	
		    end_row = img.rows/3-1;
	    	img.copyTo(frame);
            for(int i=0;i<img.cols;i++)
            {
                //Region interest
                ROILane=frame(Rect(i,init_row ,1,end_row));
                //Normal values 
                divide(255,ROILane,ROILane);
                //add the value 
                histogramLane.push_back((int)(sum(ROILane)[0]));
            } 
            //Find line left
            vector<int>:: iterator LeftPtr;
            LeftPtr = max_element(histogramLane.begin(),histogramLane.begin()+img.cols/2);
            LanePosition[0] = distance(histogramLane.begin(),LeftPtr);
            //find line right
            vector<int>:: iterator RightPtr;
            RightPtr = max_element(histogramLane.begin()+(img.cols/2)+1,histogramLane.end());
            LanePosition[1] = distance(histogramLane.begin(),RightPtr);
	        return  LanePosition;
        }

		void locate_lanes(Mat &img,Mat &out_img){
	        Mat region_interest;
	        left_points.clear();
	        right_points.clear();
	        int nwindows , margin,minpix;
	        int win_y_low = 0, win_y_high = 0, win_xleft_low = 0, win_xleft_high = 0, win_xright_low = 0, win_xright_high = 0;
	        int leftx_current,rightx_current;
	        int mean_leftx,mean_rightx,count_left,count_right;
	        int *locate_histogram;
		    uchar now_left_point;
		    uchar now_right_point;
            bool end_left = false;
            bool end_right = false;
            Point add_left_point,add_right_point;
	        nwindows = 9;
	        int window_height = img.rows/nwindows;
	        locate_histogram = Histogram(img);
	        leftx_current = locate_histogram[0];
	        rightx_current = locate_histogram[1];
	        // Set the width of the windows +/- margin
	        margin = 20;
	        minpix = 40;
	        // Set minimum number of pixels found to recenter window
	        for(int window=0;window<nwindows;window++)
		    {
	            mean_leftx = 0;
	            mean_rightx = 0;
	            count_left = 0;
	            count_right = 0;
	            win_y_low = img.rows - (window+1)*window_height;
    	        win_y_high = img.rows - window*window_height;
   	            win_xleft_low = leftx_current - margin;
    	        win_xleft_high = leftx_current + margin;
    	        win_xright_low = rightx_current - margin;
    	        win_xright_high = rightx_current + margin;
	            if( win_xleft_low<0)
		            win_xleft_low = 0+1;
	            if(win_xright_high>= img.cols-1)
	    	        win_xright_high =  img.cols-1;
	            if(win_y_high>=img.rows-1)
		            win_y_high = img.rows-1;
	            if(win_y_low<=0)
		            win_y_low = 1;
	            for(int r = win_y_low;r<win_y_high;r++)
			    {
		            for(int cl = win_xleft_low+1;cl<win_xleft_high;cl++)
				    {
					    now_left_point = img.at<uchar>(r,cl);
		                if(now_left_point >0)
					    {
                        	add_left_point.x = r;
                        	add_left_point.y = cl; 
							left_points.push_back(add_left_point);
		    				mean_leftx += add_left_point.y; 
	                		count_left++; 
    		        }		
		        }
	            for(int cr = win_xright_low+1;cr<win_xright_high;cr++)
				{
					now_right_point = img.at<uchar>(r,cr);
		            if(now_right_point >0)
					{
                        add_right_point.x = r;
                        add_right_point.y = cr; 
						right_points.push_back(add_right_point);
			    		mean_rightx += add_right_point.y ;
			    		count_right++;
    		   	    }		
		    	}
	    	}
	        if(count_left>=minpix)
		    {
	    	    mean_leftx /=  count_left;
			    leftx_current = mean_leftx;
   	        }
	        if(count_right>=minpix)
		    {
	    	    mean_rightx /=  count_right;
			    rightx_current = mean_rightx;
   	        }
	        //rectangle(out_img,cv::Point(win_xleft_low,win_y_low),cv::Point(win_xleft_high,win_y_high),cv::Scalar(0,255,0));
  	        //rectangle(out_img,cv::Point(win_xright_low,win_y_low),cv::Point(win_xright_high,win_y_high),cv::Scalar(0,255,0));
	        }
	    }
	bool regression_left()
    {
        if(left_points.size()<75)
            return false;
	    long sumX[5] = {0,0,0,0,0};
	    long sumY[3] = {0,0,0};
	    long pow2 = 0;
	    for(auto point=left_points.begin();point!=left_points.end();point++)
        {
		    pow2 = (point->x)*(point->x);
	        sumX[0]++;
		    sumX[1]+= (point->x);
		    sumX[2]+= pow2;
		    sumX[3]+= pow2*(point->x);
		    sumX[4]+= pow2*pow2;
		    sumY[0]+= (point->y);
		    sumY[1]+= (point->y)*(point->x);
		    sumY[2]+= (point->y)*pow2;
	    }	
        solve_system(sumX,sumY,polyleft);
        return true;
	}

    bool regression_right()
    {
        if(right_points.size()<75)
            return false;
	    long sumX[5] = {0,0,0,0,0};
	    long  sumY[3] = {0,0,0};
	    long  pow2 = 0;
	    for(auto point=right_points.begin();point!=right_points.end();point++)
        {
		    pow2 = (point->x)*(point->x);
	        sumX[0]++;
		    sumX[1]+= (point->x);
		    sumX[2]+= pow2;
		    sumX[3]+= pow2*(point->x);
		    sumX[4]+= pow2*pow2;
		    sumY[0]+= (point->y);
		    sumY[1]+= (point->y)*(point->x);
		    sumY[2]+= (point->y)*pow2;
	    }	
        solve_system(sumX,sumY,polyright);
        return true;
	}
 	void solve_system(long *sX,long *sY,float *x)
     {
	    int n,i,j,k;
        n=3;
    	float a[n][n+1];
        //declare an array to store the elements of augmented-matrix    
    	//"Enter the elements of the augmented-matrix row-wise
        a[0][0]=sX[0];   
        a[0][1]=sX[1];   
        a[0][2]=sX[2];   
        a[0][3]=sY[0];
        ////////////////   
        a[1][0]=sX[1];   
        a[1][1]=sX[2];   
        a[1][2]=sX[3];   
        a[1][3]=sY[1];  
        ////////////////   
        a[2][0]=sX[2];   
        a[2][1]=sX[3];   
        a[2][2]=sX[4];   
        a[2][3]=sY[2];   
        ////////////////
        for (i=0;i<n;i++)                    //Pivotisation
            for (k=i+1;k<n;k++)
                if (abs(a[i][i])<abs(a[k][i]))
                    for (j=0;j<=n;j++){
                        double temp=a[i][j];
                        a[i][j]=a[k][j];
                        a[k][j]=temp;
                    }
        for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
            for (k=i+1;k<n;k++){
                double t=a[k][i]/a[i][i];
                for (j=0;j<=n;j++)
                    a[k][j]=a[k][j]-t*a[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
            }
        for (i=n-1;i>=0;i--)                //back-substitution
        {                        //x is an array whose values correspond to the values of x,y,z..
            x[i]=a[i][n];                //make the variable to be calculated equal to the rhs of the last equation
            for (j=i+1;j<n;j++)
                if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                    x[i]=x[i]-a[i][j]*x[j];
            x[i]=x[i]/a[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
        } 
	}

    void draw_lines(Mat &img)
    {
	    float columnL,columnL_aux;
		float columnR,columnR_aux;
	    float row;
	    float m = 0.0,b = 0.0,c=0.0;
		bool find_line_left;
		bool find_line_right;
		float angle_to_mid_radian;
		find_line_right = regression_right();
		find_line_left = regression_left();
		right_points.clear();
	    left_points.clear();
		center_cam =(img.cols/2) - 5;
        if(find_line_left && find_line_right)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnR = polyright[0] + polyright[1]*(row)+polyright[2]*(row*row);
                circle(img,cv::Point(columnR,row),cvRound((double)4/ 2), Scalar(0, 255, 0), 2);
                columnL = polyleft[0] + polyleft[1]*(row)+polyleft[2]*(row*row);
                circle(img,cv::Point(columnL,row),cvRound((double)4/ 2), Scalar(0, 255, 0), 2);
            }
            center_lines = (columnR + columnL)/2;
             if (columnR == columnL){
                if (center_lines < center_cam)
                    distance_center = center_cam - 0;
                else
                    distance_center = center_cam - img.cols;
                return;   
            }
            distance_center = center_cam - center_lines;
		    if(distance_center==0)
			    angle = 90;
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(center_lines - center_cam));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
		    }
            line(img,Point(center_lines,0),Point(center_cam,(img.rows-1)),Scalar(0, 255, 0),2); 
            for(int k = 0;k < 3;k++)
            {
                polyleft_last[k] = polyleft[k]; 
                polyright_last[k] =polyright[k];
            }
        }
        else if(find_line_left)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnL = polyleft[0] + polyleft[1]*(row)+polyleft[2]*(row*row);
                 circle(img,cv::Point(columnL,row),cvRound((double)4/ 2), Scalar(0, 255, 0),2);
            }
            //columnL = polyleft[0] + polyleft[1]*(0.0)+polyleft[2]*(0.0);
            columnL = polyleft[0];
            columnL_aux =  polyleft[0] + polyleft[1]*static_cast<float>(img.rows-1)+polyleft[2]*((img.rows-1)*(img.rows-1));
            if(columnL_aux == columnL)
            {
                angle = 90;
                center_lines = columnL_aux;
            }
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(columnL - columnL_aux ));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
                if(angle<90)
                {
                    angle_to_mid_radian =  (float)(angle)*0.0174533;
                    center_lines = center_cam+(int)(360.0*cos(angle_to_mid_radian));
                }
                else
                {
                    angle_to_mid_radian =  (float)(180-angle)*0.0174533;
                    center_lines = center_cam-(int)(360.0*cos(angle_to_mid_radian));
                }
		    }
            distance_center = center_cam - center_lines;
            
            for(int k = 0;k < 3;k++)
                polyleft_last[k] = polyleft[k]; 

            if (center_lines <= center_cam)
                distance_center = center_cam - 0;
            else
                distance_center = center_cam - img.cols;
            
        }
       else if(find_line_right)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnR = polyright[0] + polyright[1]*(row)+polyright[2]*(row*row);
                circle(img,cv::Point(columnR,row),cvRound((double)4/ 2), Scalar(0, 255, 0), 2);
            }
            //columnR = polyright[0] + polyright[1]*(0.0)+polyright[2]*(0.0);
            columnR = polyright[0];
            columnR_aux = polyright[0] + polyright[1]*static_cast<float>(img.rows-1)+polyright[2]*static_cast<float>((img.rows-1)*(img.rows-1));
            if(columnR_aux == columnR)
            {
                angle = 90;
                center_lines = columnR_aux;
            }
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(columnR - columnR_aux ));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
                if(angle<90)
                {
                    angle_to_mid_radian =  angle*0.0174533;
                    center_lines = center_cam+(int)(360.0*cos(angle_to_mid_radian));
                }
                else
                {
                 angle_to_mid_radian =  (float)(180-angle)*0.0174533;
                    center_lines = center_cam-(int)(360.0*cos(angle_to_mid_radian));
                }
		    }
            distance_center = center_cam - center_lines;
             
            for(int k = 0;k < 3;k++)
                polyright[k] = polyright_last[k];
            
            if (center_lines <= center_cam)
                distance_center = center_cam - 0;
            else
                distance_center = center_cam - img.cols;
        }
        if(!find_line_left && !find_line_right)
        {
            for(row = img.rows-1;row>=0;row-=8)
            {
                columnR = polyright_last[0] + polyright_last[1]*(row)+polyright_last[2]*(row*row);
                circle(img,cv::Point(columnR,row),cvRound((double)4/ 2), Scalar(0, 255, 0), 2);
                columnL = polyleft_last[0] + polyleft_last[1]*(row)+polyleft_last[2]*(row*row);
                circle(img,cv::Point(columnL,row),cvRound((double)4/ 2), Scalar(0, 255, 0), 2);
            }
            center_lines = (columnR + columnL)/2;
             if (columnR == columnL){
                if (center_lines < center_cam)
                    distance_center = center_cam - 0;
                else
                    distance_center = center_cam - img.cols;
                return;   
            }
            distance_center = center_cam - center_lines;
		    if(distance_center==0)
			    angle = 90;
		    else
            {
			    angle_to_mid_radian = atan(static_cast<float>(0-(img.rows-1))/static_cast<float>(center_lines - center_cam));
                angle  = static_cast<int>(angle_to_mid_radian * 57.295779);  
                if(angle <0 && angle >(0-90))
                    angle = (0-1)*(angle);
                else if(angle>0 && angle<90 )
                    angle = 180 - angle; 
		    }
            
        }
        // line(img,Point(center_cam,(img.rows/4)),Point(center_cam,(img.rows*3/4)),Scalar(0,255,0),2); 
        line(img,Point(center_lines,0),Point(center_cam,(img.rows-1)),Scalar(0,0,255),2);       
        }
};
int main(int argc,char **argv){
    ros::init(argc, argv, "line_detection");
    LineDetection *ic= new  LineDetection;
    ros::spin();
    return 0;
}
