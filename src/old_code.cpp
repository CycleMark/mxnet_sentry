
// https://answers.ros.org/question/361940/ros-robot-moves-to-certain-coordinate-point-but-how-to-stop-and-set-multiple-goals/
// http://wiki.ros.org/p2os-purdue/Tutorials/C%2B%2B%20Velocity%20Controller%20for%20P2OS%20Robots
// https://www.programcreek.com/python/example/70235/geometry_msgs.msg.Point

/*bool toGoal(ros::Publisher vel_pub, geometry_msgs::Point goal_point, double tolerance)
{

	geometry_msgs::Twist vel_msg;

	tf::TransformListener localListener;
	tf::StampedTransform localTransform;
	geometry_msgs::Point currentLocation;
	geometry_msgs::Point orginalLocation;

	double currentRotation;
	double lDistanceToGoal = 0.0, angle_to_goal, theta = 0.0, lLastDistanceToGoal = 0.0;
	bool lWaitingForTransform = true;

	ros::Rate loop_rate(10);

	while (lWaitingForTransform == true)
	{
		try
		{
			ROS_INFO("Waiting For Transform...");
			localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);
			lWaitingForTransform = false;

		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	currentLocation.x = localTransform.getOrigin().x();
	currentLocation.y = localTransform.getOrigin().y();
	orginalLocation.x = currentLocation.x;
	orginalLocation.y = currentLocation.y;

	currentRotation = tf::getYaw(localTransform.getRotation());
	

	double xDifference = goal_point.x - currentLocation.x;
	double yDiffernece = goal_point.y - currentLocation.y;

	angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

	lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);

	lWaitingForTransform = true;
	ROS_INFO("Distance to Goal - abs(): %d", fabs(lDistanceToGoal));
	ROS_INFO("Distance to Goal : %d", (lDistanceToGoal));

	ROS_INFO("Pausing...");
	sleep(10);

	if (!ros::ok())
	{
		exit(1);
	}

	while (lWaitingForTransform == true)
	{
		try
		{
			lLastDistanceToGoal = lDistanceToGoal;
			
			while ((fabs(lDistanceToGoal) >= 0.1 && ros::ok()))
			{
				localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);

				currentLocation.x = localTransform.getOrigin().x();
				currentLocation.y = localTransform.getOrigin().y();
				currentRotation = tf::getYaw(localTransform.getRotation());

				xDifference = goal_point.x - currentLocation.x;
				yDiffernece = goal_point.y - currentLocation.y;

				angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

				lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);
				lWaitingForTransform = false;
					
				ROS_INFO("******Charging Reversing LOCATION... ******\r\n Goal PosX: %f\r\n Goal PosY: %f\r\n \r\n Orginal PosX: %f\r\n Orginal PosY: %f\r\n AngleToGoal: %f\r\n Current PosX: %f\r\n Current PosY: %f\r\n X Diff: %f\r\n Y Diff: %f\r\n Distance To Goal: %f\r\n *******************************\r\n", 
						goal_point.x, goal_point.y, orginalLocation.x, orginalLocation.y, angle_to_goal, currentLocation.x, currentLocation.y, xDifference, yDiffernece, lDistanceToGoal);

				ROS_INFO("Distance to Goal - fabs(): %f", lDistanceToGoal);
				std::cout << "cout Distance To Goal: " << lDistanceToGoal << std::endl;

				if (lDistanceToGoal >= 0.1)
				{
					ROS_INFO("Distance to Goal - fabs()...NOT met:");
				}
				else
				{
					ROS_INFO("Distance to Goal - fabs()...MET:");
				}

				if (lDistanceToGoal >= 0.1)
				{
					/*if (abs(angle_to_goal - theta) > 0.1)
					{
						vel_msg.linear.x = 0.0;
						vel_msg.angular.z = 0.1;
					}
					else 
					{
						vel_msg.linear.x = -0.1;
						vel_msg.linear.y = 0;
						vel_msg.linear.z = 0;
						vel_msg.angular.x = 0;
						vel_msg.angular.y = 0;
						//vel_msg.angular.z = Kh * (atan2(goal_pose.y - turtlesim_pose.y, goal_pose.x - turtlesim_pose.x) - turtlesim_pose.theta);
						vel_msg.angular.z = 0.0;
						ROS_INFO("Reversing...\r\n");
						vel_pub.publish(vel_msg);
					}
				}
				if (lDistanceToGoal > lLastDistanceToGoal)
				{
					ROS_INFO("DISTANCE MEASUREMENT ERROR...\r\n");
					break;					
				}
					
				lLastDistanceToGoal = lDistanceToGoal;

				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	return false;
}
*/

// Test Procedure - Initial purpose to work out why, when backing out of the charging station it keeps miss calcuating 
// the distance travelled - and missing the point.
bool testMethod()
{

	geometry_msgs::Twist vel_msg;

	tf::TransformListener localListener;
	tf::StampedTransform localTransform;
	geometry_msgs::Point currentLocation;
	geometry_msgs::Point orginalLocation;

	double currentRotation;
	double lDistanceToGoal, angle_to_goal, theta = 0.0;
	bool lWaitingForTransform = true;

	ros::Rate loop_rate(10);

	while (lWaitingForTransform == true)
	{
		try
		{
			ROS_INFO("Waiting For Transform...");
			localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);
			lWaitingForTransform = false;

			currentLocation.x = localTransform.getOrigin().x();
			currentLocation.y = localTransform.getOrigin().y();

			// Record Starting Position so we can calculate how far the robot has travelled.
			orginalLocation.x = currentLocation.x;
			orginalLocation.y = currentLocation.y;

			currentRotation = tf::getYaw(localTransform.getRotation());
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	double xDifference = currentLocation.x - orginalLocation.x;
	double yDiffernece = currentLocation.y - orginalLocation.y;

	angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

	lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);

	lWaitingForTransform = true;
	ROS_INFO("Distance to Goal - abs(): %d", fabs(lDistanceToGoal));
	ROS_INFO("Distance to Goal : %d", (lDistanceToGoal));

	while (lWaitingForTransform == true)
	{
		try
		{
			while ((cmdString != "STOP" && ros::ok()))
			{
				localListener.lookupTransform("/map", "/base_footprint", ros::Time(0), localTransform);

				currentLocation.x = localTransform.getOrigin().x();
				currentLocation.y = localTransform.getOrigin().y();
				currentRotation = tf::getYaw(localTransform.getRotation());

				xDifference = orginalLocation.x - currentLocation.x;
				yDiffernece = orginalLocation.y- currentLocation.y;

				angle_to_goal = atan2(yDiffernece, xDifference); // this is our "bearing to goal" as I can guess

				lDistanceToGoal = sqrt(xDifference * xDifference + yDiffernece * yDiffernece);
				lWaitingForTransform = false;
					
				ROS_INFO("******Data Points... ****** Orginal PosX: %f\r\n Orginal PosY: %f\r\n AngleToGoal: %f\r\n Current PosX: %f\r\n Current PosY: %f\r\n X Diff: %f\r\n Y Diff: %f\r\n Distance To Goal: %f\r\n *******************************\r\n", 
						orginalLocation.x, orginalLocation.y, angle_to_goal, currentLocation.x, currentLocation.y, xDifference, yDiffernece, lDistanceToGoal);

				ROS_INFO("Distance Travelled - fabs(): %f", lDistanceToGoal);
				std::cout << "cout Distance To Goal: " << lDistanceToGoal << std::endl;

				sleep(1);
				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
	}

	return false;
}

