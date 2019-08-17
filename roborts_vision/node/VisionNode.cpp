/**
 * Robomaster Vision program of RM2019
 * Copyright (c) 2019, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */


//opencv
#include <opencv2/opencv.hpp>
#include "VisionNode.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "roborts_vision");
    
    VisionNode node;

    ros::AsyncSpinner async_spinner(4);
    async_spinner.start();
    ros::waitForShutdown();
 	
	return 0;
}