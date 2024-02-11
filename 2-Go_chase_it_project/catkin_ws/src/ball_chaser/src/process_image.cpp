#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <vector>

// Define a global client that can request services
ros::ServiceClient client;

int get_max_index(const std::vector<unsigned int>& v);

double get_ratio_target_pixel(const std::vector<unsigned char>& v, const unsigned int target_pixel);

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Moving the robot into the white ball.");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested commands movements
    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot.");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    // Pixels are arranged in RGB RGB RGB fashion, meaning pixel_1{RGB}, pixel_2{RGB}, ...
    const unsigned int rgb_step{3};
    const double maxRatioWhitePixels{0.2};
    const float linear_x_movement_scalar{0.25}, angular_z_movement_scalar{0.25}, still{0.0};
    const unsigned int matrix_data_size{img.step*img.width};
    std::vector<unsigned int> white_pixels_side_tracker{0, 0, 0}; // [left, center, right] counters
    int current_step, white_pixels_counter{0};

    for(unsigned int i = 0; i < matrix_data_size; i+=rgb_step)
    {
    	// check if we have found a white pixel, meaning: R=255, G=255, B=255
    	if(img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel)
    	{
    		// if we get here then we found a white pixel
    		++white_pixels_counter;
    		current_step = i % img.step;
    		if(current_step < img.step/3)
    		{
    			// pixel is on the left side
    			++white_pixels_side_tracker[0];
        } else if(current_step < 2*img.step/3)
    		{
    			// pixel is on the center
          ++white_pixels_side_tracker[1];
    		} else
    		{
    			// pixel is on the right side
          ++white_pixels_side_tracker[2];
    		}
    	}
    }

    if(white_pixels_counter == 0)
    {
	     drive_robot(still, still);
    } else if ( get_ratio_target_pixel(img.data, white_pixel) > maxRatioWhitePixels) {
	     drive_robot(still, still);
    } else {
    	const int left{0}, forward{1}, right{2};
    	switch( get_max_index(white_pixels_side_tracker) )
    	{
    		case left:
    			drive_robot(linear_x_movement_scalar, angular_z_movement_scalar);
    			break;
    		case forward:
          drive_robot(linear_x_movement_scalar, still);
          break;
    		case right:
          drive_robot(linear_x_movement_scalar, -angular_z_movement_scalar);
          break;
    		default:
    			drive_robot(still, still);
    	}
    }
}

int get_max_index(const std::vector<unsigned int>& v)
{
  if (v.size() == 0) {
	   return -1;
  }
  int max_counter{0}, max_index{0};
  const int size = v.size();
  for(int i = 0 ; i < size; ++i)
  {
	   if(v[i] > max_counter)
     {
       max_counter = v[i];
		   max_index = i;
     }
  }
  return max_index;
}

double get_ratio_target_pixel(const std::vector<unsigned char>& v, const unsigned int target_pixel)
{
	const std::size_t size = v.size();
	int target_count{0};
	for(std::size_t i = 0; i < size; ++i)
	{
		if(v[i] == target_pixel)
			++target_count;
	}
	return static_cast<double>(target_count)/size;
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
