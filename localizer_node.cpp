#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tuple>
#include <random>
#include <math.h>
#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.1
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0
#define PARTICLE_GRAPHIC_RADIUS 10.0
#define PARTICLE_GRAPHIC_LENGTH 25.0
#define NUM_PARTICLES 10000
#define ROTATION_MEAN_ERR 0
#define ROTATION_STDEV M_PI/32
#define TRANSLATION_STDEV 0.2
#define TRANSLATION_MEAN_ERR -0.2
#define ROTATION_COEFFICIENT 0.5
#define GRID_SIZE 1
#define CAMERA_OFFSET -0.32
#define POSITION_NOISE 0.0
#define SCALE 1.22 // Scale from the robot image to the map image.



class Localizer
{
public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  // image_transport::Publisher robot_grid_pub;
  image_transport::Subscriber gt_img_sub;
  image_transport::Subscriber robot_img_sub;

  ros::Subscriber motion_command_sub;
  

  geometry_msgs::PoseStamped estimated_location;

  cv::Mat map_image;
  cv::Mat ground_truth_image;
  cv::Mat localization_result_image;
  cv::Mat particle_vis_image;
  cv::Mat robot_truth_image;
  std::default_random_engine generator;
  double particleProb [NUM_PARTICLES][2]; //weight of each particle to draw from and the sum of weights seen so far
  float particlePose [NUM_PARTICLES][3]; // particlePose [particleNumber][x_pos, y_pos, yaw];
  float prevYaw =0.0;
  double target_yaw;
  double estimated_yaw = 0.0;
  Localizer( int argc, char** argv )
  {     

    image_transport::ImageTransport it(nh);
    pub = it.advertise("/assign2/localization_result_image", 1);
    map_image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    estimated_location.pose.position.x = 0;
    estimated_location.pose.position.y = 0;
    // target_yaw = 0.0;
    init(true);

    localization_result_image = map_image.clone();
    ground_truth_image = map_image.clone();
    particle_vis_image = map_image.clone();

    gt_img_sub = it.subscribe("/assign2/ground_truth_image", 1, &Localizer::groundTruthImageCallback, this);
    robot_img_sub = it.subscribe("/aqua/back_down/image_raw", 1, &Localizer::robotImageCallback, this);
    motion_command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1, &Localizer::motionCommandCallback, this);
    ROS_INFO( "localizer node constructed and subscribed." );

  }
  void init(bool kidnapped)
  {

  	double uniformProb = 1.0/NUM_PARTICLES;
    for(int i=0; i<NUM_PARTICLES;i++)
    {
    	particleProb[i][0]= uniformProb;
    }
    //Robot is Kidnapped, We don't know the starting point of the robot.
  	if(kidnapped)
  	{
  		//Distribute the particles uniformly across all possible points.
  		for(int j=0; j<NUM_PARTICLES;j++)
  		{
  			
  			std::uniform_real_distribution<double> randomX(-5.0,5.0); //x pos
  			std::uniform_real_distribution<double> randomY(-20.0,20.0); // ypos
  			std::uniform_real_distribution<double> randomA(0,2*M_PI);  //angle
  			particlePose[j][0] = randomX(generator);
  			particlePose[j][1] = randomY(generator);
  			particlePose[j][2] = randomA(generator);
 			particleProb[j][0] = 1.0/NUM_PARTICLES;
 			//ROS_INFO("Particle %d , X pos %lf , Y pos , %lf , Angle %lf ", j, particlePose[j][0], particlePose[j][1],particlePose[j][2]);
 			if(j>0)
 			{
 				particleProb[j][1] = particleProb[j-1][1]+ particleProb[j-1][0];
 			}
 			else 
 			{
 				particleProb[j][1] = 0;
 			}
  		} 
  	}
  	else{
 		for(int j=0; j<NUM_PARTICLES;j++)
 		{
 			particlePose[j][0] = 0.0; //estimated_location.pose.position.x;
 			particlePose[j][1] = 0.0; //estimated_location.pose.position.y;
 			particlePose[j][2] = 0.0;
 			particleProb[j][0] = 1.0/NUM_PARTICLES;
 			if(j>0)
 			{
 				particleProb[j][1] = particleProb[j-1][1]+ particleProb[j-1][0];
 			}
 			else 
 			{
 				particleProb[j][1] = 0;
 			}
 		} 		
  	}
  }

//Observation Model. Robot receives an image from camera, update the beliefs of the robot's position.
  // Compares the view(according to our map) from each particle to the robot's view and update.
  //Chooses the highest probability particle as our estimated location.
  void robotImageCallback( const sensor_msgs::ImageConstPtr& robot_img )
  {
 	//  	float x = estimated_location.pose.position.x;
	// float y = estimated_location.pose.position.y;
	// unsigned char mapBGR[3];
	// unsigned char estimatedBGR[3];
	//Stores the 3*3 grid of pixels.
	double robotViewBGR[GRID_SIZE][GRID_SIZE][3];
	double mapBGR[GRID_SIZE][GRID_SIZE][3];
	// ROS_INFO("Updating Pixel Value. Checking at Poistion [%f,%f]",x,y);
	ROS_INFO("UPDATING");
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(robot_img, sensor_msgs::image_encodings::BGR8);
      robot_truth_image = cv::Mat(cv_ptr->image); 
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //Building the grid that the robot sees.
    for(int xOffset=0;xOffset<GRID_SIZE;xOffset++)
    {
    	for(int yOffset=0;yOffset<GRID_SIZE;yOffset++)
    	{
    		robotViewBGR[xOffset][yOffset][0] = (double)robot_truth_image.at<cv::Vec3b>(200+(yOffset-(GRID_SIZE/2))*50,200+(xOffset-(GRID_SIZE/2))*50)[0];
			robotViewBGR[xOffset][yOffset][1] = (double)robot_truth_image.at<cv::Vec3b>(200+(yOffset-(GRID_SIZE/2))*50,200+(xOffset-(GRID_SIZE/2))*50)[1];
			robotViewBGR[xOffset][yOffset][2] = (double)robot_truth_image.at<cv::Vec3b>(200+(yOffset-(GRID_SIZE/2))*50,200+(xOffset-(GRID_SIZE/2))*50)[2];
			// cv::circle( robot_truth_image, cv::Point(200+(yOffset-(GRID_SIZE/2))*50,200+(xOffset-(GRID_SIZE/2))*50), POSITION_GRAPHIC_RADIUS, CV_RGB(0,250,0), -1);

    	}
    }
    // localization_result_image = ground_truth_image.clone();

	//Update Probabilities of every particle
	double sumOfPixelErrors = 0.0;
	double x;
	double y;
	double theta;
	double mapX;
	double mapY;
	double stdDev = 3*GRID_SIZE*GRID_SIZE*30; 
	//Compare the view from each particle to the view the robot actually receives. Updare the probabilities based on simmilarity
	for(int i=0; i<NUM_PARTICLES;i++)
	{
		//Calculate the mapRGB 
		x = particlePose[i][0];
		y = particlePose[i][1];
		theta = particlePose[i][2];
		//Check a grid of 3 pixels, each seperated by 50 pixels. center pixel is at offset 0,0 range from off set -50 to 50 on x and y axis

		sumOfPixelErrors = 0.0;
		for(int xOffset=0;xOffset<GRID_SIZE;xOffset++)
		{
			for(int yOffset=0;yOffset<GRID_SIZE;yOffset++)
			{
				//Map X position  in the middle of the grid is the same as the X_position in the middle of the robot image + the offset of the camera (i.e. slightly behind the position of the robot)
				//Any other position in the grid 50 pixels are added in the direction of the grid(i.e offset x=max, y= origin then the x offset on the map is determined by the cos of the yaw and the y offset is the sin of the yaw)
				mapX = localization_result_image.size().width/2 + (x+ CAMERA_OFFSET*cos(-theta))*METRE_TO_PIXEL_SCALE  + ((xOffset-GRID_SIZE/2)*50*sin(-theta) + (yOffset-GRID_SIZE/2)*50*cos(-theta))*SCALE;
				mapY = localization_result_image.size().height/2 + (y+ CAMERA_OFFSET*-sin(-theta))*METRE_TO_PIXEL_SCALE +  ((xOffset-GRID_SIZE/2)*50*-cos(-theta) + (yOffset-GRID_SIZE/2)*50*sin(-theta))*SCALE;
				// compute mapRGB 
				//ROS_INFO("MAP_X = %f MAP Y = %f", mapX, mapY);
				for(int k=0;k<3;k++)
				{
					mapBGR[xOffset][yOffset][k] = (double)map_image.at<cv::Vec3b>(mapY,mapX)[k];
					// cv::circle( localization_result_image, cv::Point(mapX,mapY), PARTICLE_GRAPHIC_RADIUS, CV_RGB(0,0,250), -1);
					//sumOfPixelErrors += pow(robotViewBGR[xOffset][yOffset][k] - mapBGR[xOffset][yOffset][k],2);
					sumOfPixelErrors += fabs(robotViewBGR[xOffset][yOffset][k] - mapBGR[xOffset][yOffset][k]);
				}

				 
			}
		}
		//calculates the weight of each particle by the probability of having x=sum of Pixel errors according to a half-gaussian
		double particleWeight = (sqrt(2)/ (stdDev*sqrt(M_PI) ) ) * exp( -0.5 *pow( (sumOfPixelErrors)/stdDev, 2.0 ) ); 
		particleProb[i][0] = particleWeight;
		if(i>0)
		{
			particleProb[i][1] = particleProb[i-1][0] + particleProb[i-1][1];
		}
		else
		{
			particleProb[i][1] = 0.0;
		}
		
	}
	//ROS_INFO("ALL PROBABILITIES UPDATED");

	//Visualize after observation model instead of motion model.
	//Choose pose of highest probability as our estimated location.
    double maxProb = 0.0;
    int maxIndex = -1;
    for(int j=0; j<NUM_PARTICLES;j++)
    {
    	if(particleProb[j][0]>maxProb)
   	{
    		maxIndex = j;
    		maxProb = particleProb[j][0];
    	}
    }
    //ROS_INFO("CHOOSE PARTICLE %d , has Prob %lf", maxIndex, maxProb);
    estimated_location.pose.position.x = particlePose[maxIndex][0];
    estimated_location.pose.position.y = particlePose[maxIndex][1];
    estimated_yaw = particlePose[maxIndex][2];
  
  }


  //Update the position of each particle ( and thus our estimated location) using the robot's odometry as well as adding random noise. 
 //Params : motion_command : The robot's current motion
  //		position.x = forward swim effort
  //		pose.orienatation = quaternion that contains our rotation speeds (only concerned with yaw)
  //This callback also prints all particles to our map
  void motionCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& motion_command )
  {
  	resample();
  	
  	ROS_INFO("MOVING");
    //Convert motion command into easy to manipulate variables
    geometry_msgs::PoseStamped command = *motion_command;
    double target_roll, target_pitch;
    tf::Quaternion target_orientation;
    tf::quaternionMsgToTF(command.pose.orientation, target_orientation);
    tf::Matrix3x3(target_orientation).getEulerYPR( target_yaw, target_pitch, target_roll ); //convert quaternion into euler coordinates
    // Change in Speed and Rotation to be used for updates
    float deltaS = command.pose.position.x;
    float deltaYaw = target_yaw-prevYaw;
   	prevYaw = target_yaw;
   	float move_x = 0.0;
   	float move_y = 0.0;
   	float rotate = 0.0;
    //std::default_random_engine generator;
    // Distributions for noise
    std::normal_distribution<double> translationDistribution(TRANSLATION_MEAN_ERR*deltaS,TRANSLATION_STDEV*deltaS);
    std::normal_distribution<double> rotationDistribution(ROTATION_MEAN_ERR*deltaYaw, ROTATION_STDEV*deltaYaw);
    ROS_INFO("MOVING");
   	//Update each particle based on noise and odometry 
    for(int i=0; i<NUM_PARTICLES;i++)
    {

    	move_x = (deltaS + translationDistribution(generator)) * FORWARD_SWIM_SPEED_SCALING* cos(-particlePose[i][2]);
    	move_y = (deltaS + translationDistribution(generator)) * FORWARD_SWIM_SPEED_SCALING* sin(-particlePose[i][2]);
    	deltaYaw = target_yaw -particlePose[i][2];
    	rotate = ROTATION_COEFFICIENT*deltaYaw + rotationDistribution(generator);

    	particlePose[i][0] = particlePose[i][0] + move_x  + POSITION_NOISE; // x_k = x_k-1 + (speed + noise) *cos(-yaw)
    	particlePose[i][1] = particlePose[i][1] + move_y  + POSITION_NOISE; //y_k = y_k-1 + (speed + noise) *sin(-yaw)
    	particlePose[i][2] = particlePose[i][2] + rotate;	//yaw_k = yaw_k-1 + err*delta_yaw + noise
    	
    	// ROS_INFO("MOVING X_AMT %f Y_AMT %f ROTATE_AMT %f DELTA YAW %f, target_yaw %f",move_x, move_y, rotate, deltaYaw, target_yaw);
    	// ROS_INFO("%d MOVE AMOUNT %lf", i, move);



    }
 	localization_result_image = ground_truth_image.clone();
  	//Paint the particles on to the localization image
    for(int p=0;p<NUM_PARTICLES;p++)
    {
	    int particle_robo_image_x = localization_result_image.size().width/2 + METRE_TO_PIXEL_SCALE * particlePose[p][0];
		int particle_robo_image_y = localization_result_image.size().height/2 + METRE_TO_PIXEL_SCALE * particlePose[p][1];

		int particle_heading_image_x = particle_robo_image_x + PARTICLE_GRAPHIC_LENGTH * cos(-particlePose[p][2]);
		int particle_heading_image_y = particle_robo_image_y + PARTICLE_GRAPHIC_LENGTH * sin(-particlePose[p][2]);

    	cv::circle( localization_result_image, cv::Point(particle_robo_image_x, particle_robo_image_y),PARTICLE_GRAPHIC_RADIUS, CV_RGB(0,250,0), -1);
    	cv::line( localization_result_image, cv::Point(particle_robo_image_x, particle_robo_image_y), cv::Point(particle_heading_image_x, particle_heading_image_y), CV_RGB(0,250,0), 4);
    }
  }
  //Resample according to the weights computed in the observation step.
  void resample()
  {
  	ROS_INFO("RESAMPLING");
  	double maxWeight = particleProb[NUM_PARTICLES-1][1]+particleProb[NUM_PARTICLES-1][0]; 
  	//std::default_random_engine generator;
  	std::uniform_real_distribution<double> distribution(0.0,maxWeight);
  	//std::uniform_real_distribution<double> a(0.0,1.0);
  	double ranNum;
  	double tempPose [NUM_PARTICLES][3]; 
  	for(int i=0;i<NUM_PARTICLES;i++)
  	{

  		ranNum = distribution(generator);
  		//Find the particle which the ran number falls into. particleProb[x][1] is the sum of all probabilities before node x, so if ran number is bigger than that sum, then we swap particle i with x.
  		for(int j=NUM_PARTICLES-1;j>=0;j--)
  		{
  			if(ranNum>particleProb[j][1])
  			{
  				for(int k=0;k<3;k++)
  				{
  					tempPose[i][k] = particlePose[j][k];
  					//particlePose[i][k] = particlePose[j][k];
  				}
  				break;
  			}
  		}
  	
  	}
  	for(int i=0;i<NUM_PARTICLES;i++)
  	{
  		for(int k=0;k<3;k++)
  		{
  			particlePose[i][k]=tempPose[i][k];
  		}
  	}
  	//Update the weight of all particles to be 1.
  	for(int i=0;i<NUM_PARTICLES;i++)

  	{
  		particleProb[i][0] = 1.0;
  	}

  }

  // Compares Prediction to ground truth
  void groundTruthImageCallback( const sensor_msgs::ImageConstPtr& gt_img )
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(gt_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    ground_truth_image = cv::Mat(cv_ptr->image);
  }

  // Publish localization result and spin to wait for callback.
  void spin()
  {
    ros::Rate loop_rate(30);
    while (nh.ok()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", localization_result_image).toImageMsg();
      pub.publish(msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer");
  Localizer my_loc(argc, argv);
  my_loc.spin();
}
