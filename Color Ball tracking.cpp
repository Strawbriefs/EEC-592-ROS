#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

static const std::string OPENCV_WINDOW = "Image window";

class BallFollower
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	
	image_transport::Subscriber depth_sub;
  	image_transport::Subscriber image_sub;
  	image_transport::Publisher image_pub;
	ros::Publisher turtle_pub;

	int iLatitude;
	int iLongitude;
	int currentRow, currentColumn;
  
	public: BallFollower(): it_(nh_)
  	{
		iLatitude = 0;
		iLongitude = 0;

  	  	// Subscribe to input video feed.
  	  	image_sub = it_.subscribe("/usb_cam/image_raw", 10, &BallFollower::rgbCb, this);
		depth_sub = it_.subscribe("camera/depth/image", 10, &BallFollower::depthCb, this);

		// Publish output video feed and turtlebot movement.
    		image_pub = it_.advertise("/image_converter/output_video", 10);
		turtle_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);

    		cv::namedWindow(OPENCV_WINDOW);
  	}

  	~BallFollower()
  	{
   		 cv::destroyWindow(OPENCV_WINDOW);
  	}

  	void rgbCb(const sensor_msgs::ImageConstPtr& msg)
  	{
    		cv_bridge::CvImagePtr cv_ptr;
    		try
    		{
     		 	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    		}
    		catch (cv_bridge::Exception& e)
    		{
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}

    		int row = 0; 
    		int col = 0; 
    		int dis_min = 1000000;
		int leftLimit, rightLimit; 
    		for (int i=0; i < cv_ptr->image.rows; i++)
 		{
			for (int j=0; j < cv_ptr->image.cols; j++) 
			{
			    int b = cv_ptr->image.at<cv::Vec3b>(i, j).val[0];
			    int g = cv_ptr->image.at<cv::Vec3b>(i, j).val[1];
			    int r = cv_ptr->image.at<cv::Vec3b>(i, j).val[2]; 
			    // int dis = pow((r-255)*(r-255) + g*g + b*b, 0.5); 
			    int dis = (r-255)*(r-255) + g*g + b*b; 
			    // ROS_INFO("b: %d", b); 
			    // ROS_INFO("g: %d", g); 
			    // ROS_INFO("r: %d", r); 
			    // ROS_INFO("%d", dis); 
			    if (dis < dis_min) 
			    {
				dis_min = dis; 
				row = i; 
				col = j; 
			    }
        		}
    		}	

	    // Prior to drawing the circle, determine if we should move left or right.
	    // We are always trying to center the circle in the middle of the feed.
	    currentRow = row;
	    currentColumn = col;
	    leftLimit = msg->width * .4;
	    rightLimit = msg->width * .6;
	    if (col < leftLimit)
		iLatitude = -1;
	    else if (col > rightLimit)
		iLatitude = 1;
	    else
		iLatitude = 0;

    	    cv::circle(cv_ptr->image, cv::Point(col, row), 10, CV_RGB(255,0,0));

	    // Update GUI Window
	    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	    cv::waitKey(3);
	    
    	     // Output modified video stream
    	     image_pub.publish(cv_ptr->toImageMsg());

	   updateTurtleBot ();
  	}
	
	void depthCb (const sensor_msgs::Image::ConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
    		{	
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
			float distance = cv_ptr->image.at<float>(currentRow, currentColumn);
			ROS_INFO("%f", distance);


			if (distance < 1.0)
				iLongitude = -1;
			else if (distance > 1.25)
				iLongitude = 1;
			else
				iLongitude = 0;

			updateTurtleBot ();
    		}
    		catch (cv_bridge::Exception& e)
    		{
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}
	}

	void updateTurtleBot ()
	{
		geometry_msgs::Twist move_msg;
		std::string output = "LATITUDE_";

		move_msg.linear.x = 0;
		move_msg.linear.y = 0;
		move_msg.linear.z = 0;
		move_msg.angular.x = 0;
		move_msg.angular.y = 0;
		move_msg.angular.z = 0;

		switch (iLatitude)
		{
			case 0:
			output += "STABLE";
				break;
			case -1:
			output += "LEFT";
			move_msg.angular.z = .25;
				break;;
				break;
			case 1:
			output += "RIGHT";
			move_msg.angular.z = -.25;
				break;
		}
		output += " / LONGITUDE ";
		switch (iLongitude)
		{
			case 0:
			output += "STABLE";
				break;
			case -1:
			output += "BACKWARDS";
			move_msg.linear.x = -.125;
				break;
			case 1:
			output += "FORWARDS";
			move_msg.linear.x = .125;
				break;
		}
		
		turtle_pub.publish (move_msg);
		ROS_INFO("%s", output.c_str());
	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  BallFollower bf;
  ros::spin();
  return 0;
}
