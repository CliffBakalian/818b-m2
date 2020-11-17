// Desc: Bigbob2 - multiple robots
// Author:  Kevin Nickels 1 July 2015

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <libplayerc/playerc.h>
#include "sharedstruct.h"

struct Item
{
	char name[16];
	double x;
	double y;
}typedef item_t;

struct Robot
{
	playerc_client_t *robot;
	playerc_position2d_t *p2dProxy;
	playerc_ranger_t *sonarProxy;
	playerc_blobfinder_t *blobProxy;
	playerc_ranger_t *laserProxy;
	double forwardSpeed;
	double turnSpeed;
}typedef our_robot_t;

our_robot_t robots[5];
/**
Randomly assigns new speeds into the given addresses. 
This function will always write to the given addresses.
@param *forwardSpeed the address of the forward speed 
variable you want this function to change.
@param *turnSpeed the address of the turn speed variable 
you want this function to change.
*/
void Wander(double *forwardSpeed, double *turnSpeed)
{
      int maxSpeed = 1;
      int maxTurn = 90;
      double fspeed, tspeed;
	
      //fspeed is between 0 and 10
      fspeed = rand()%11;
      //(fspeed/10) is between 0 and 1
      fspeed = (fspeed/10)*maxSpeed;
	
      tspeed = rand()%(2*maxTurn);
      tspeed = tspeed-maxTurn;
	
      *forwardSpeed = fspeed;
      *turnSpeed = tspeed;
} 

/**
Checks sonars for obstacles and updates the given addresses 
with wheel speeds. This function will write to the addresses 
only if there is an obstacle present. Very basic obstacle avoidance only.
@param *forwardSpeed the address of the forward speed variable 
you want this function to change.
@param *turnSpeed the address of the turn speed variable you 
want this function to change.
@param &sp The sonar proxy that you want this function to monitor.
*/
void AvoidObstacles(double *forwardSpeed, double *turnSpeed, \
      playerc_ranger_t *sproxy)
{
	  double *sp = sproxy->ranges; // pointer to range array

      //will avoid obstacles closer than 40cm
      double avoidDistance = 0.4;
      //will turn away at 60 degrees/sec
      int avoidTurnSpeed = 60;
      
      //left corner is sonar no. 2
      //right corner is sonar no. 3
      if(sp[2] < avoidDistance)
      {
            *forwardSpeed = 0;
            //turn right
            *turnSpeed = (-1)*avoidTurnSpeed;
            printf("avoiding obstacle\n");
            return;
      }
      else if(sp[3] < avoidDistance)
      {
            *forwardSpeed = 0;
            //turn left
            *turnSpeed = avoidTurnSpeed;
            printf("avoiding obstacle\n");
            return;
      }
      else if( (sp[0] < avoidDistance) && \
               (sp[1] < avoidDistance))
      {
            //back off a little bit
            *forwardSpeed = -0.2;
            *turnSpeed = avoidTurnSpeed;  
            printf("avoiding obstacle\n");
            return;
      }
      
      return; //do nothing
}


/**
If blobs have been detected this function will turn the robot 
towards the largest blob. This will be the closest blob (hopefully!). 
If called this function will always overwrite information in the 
given addresses.
@param *forwardSpeed the address of the forward speed variable 
you want this function to change.
@param *turnSpeed the address of the turn speed variable you 
want this function to change.
@param &bfp The blobfinder proxy that you want this function 
to monitor.
*/
void MoveToItem(double *forwardSpeed, double *turnSpeed, 
      playerc_blobfinder_t *bfp)
{
      int i, centre;
      int noBlobs = bfp->blobs_count;
      playerc_blobfinder_blob_t blob;
      int turningSpeed = 5; // in deg/s

      /*number of pixels away from the image centre a blob
      can be to be in front of the robot*/
      int margin = 10;
      
      int biggestBlobArea = 0;
      int biggestBlob = 0;
      
      //find the largest blob
      for(i=0; i<noBlobs; i++)
      {
            //get blob from proxy
            playerc_blobfinder_blob_t currBlob = bfp->blobs[i];
            
            // (.area is a negative cast into an unsigned int! oops.)
            if( abs((int)currBlob.area) > biggestBlobArea)
            {
                  biggestBlob = i;
                  biggestBlobArea = abs((int)currBlob.area);
            }
      }
      blob = bfp->blobs[biggestBlob];
      //printf("biggest blob is %i with area %d\n",biggestBlob,biggestBlobArea);
            
      // find centre of image
      centre = bfp->width/2;
      
      //adjust turn to centre the blob in image
      /*if the blob's centre is within some margin of the image 
      centre then move forwards, otherwise turn so that it is 
      centred. */
      //printf("blob.x=%d, c=%d\n",blob.x,centre);

      //blob to the left of centre
      if(blob.x < centre-margin)
      {
            *forwardSpeed = 0;
            //turn left
            *turnSpeed = turningSpeed;
            //printf("turning left\n");
      }
      //blob to the right of centre
      else if(blob.x > centre+margin)
      {
            *forwardSpeed = 0;
            //turn right
            *turnSpeed = -turningSpeed;
            //printf("turning right\n");
      }
      //otherwise go straight ahead
      else
      {
            *forwardSpeed = 0.1;
            *turnSpeed = 0;      
            //printf("straight on\n");
      }
      
      return;
}

/**
Fills the item list array with the names and positions of items 
in the simulation
@param itemList this is the item list which contains the names 
and positions of all the items in the simulation.
@param simProxy the simulation proxy for the Player/Stage simulation.
*/
void RefreshItemList(item_t *itemList, playerc_simulation_t *simProxy)
{
      int i;
	
	//get the poses of the oranges
	for(i=0;i<4;i++)
	{
	      char orangeStr[] = "orange%d";
	      sprintf(itemList[i].name, orangeStr, i+1);
	      double dummy;  //dummy variable, don't need yaws.
	      playerc_simulation_get_pose2d(simProxy,itemList[i].name, \
	            &(itemList[i].x), &(itemList[i].y), &dummy);
	}
	
	//get the poses of the cartons
	for(i=4;i<8;i++)
	{
	      char cartonStr[] = "carton%d";
	      sprintf(itemList[i].name, cartonStr, i-3);
	      double dummy;  //dummy variable, don't need yaws.
	      playerc_simulation_get_pose2d(simProxy,itemList[i].name, \
	            &(itemList[i].x), &(itemList[i].y), &dummy);
	}
	
	return;
}






/**
Finds an item in the simulation which is near the robot's teeth. 
@param itemList this is the item list which contains the names and 
positions of all the items in the simulation.
@param listLength The number of items in the simulation
@param sim the simulation proxy for the Player/Stage simulation.
@return returns the index of the item in the array which is within 
the robot's teeth. If no item is found then this will return -1.
*/
int FindItem(item_t *itemList, int listLength, playerc_simulation_t *sim, char *robot_name)
{
	/*
		This function works by creating a search area just
		in front of the robot's teeth. The search circle is a
		fixed distance in front of the robot, and has a 
		fixed radius.
		This function finds objects within this search circle
		and then deletes the closest one.
	*/
	
	//radius of the search circle
      double radius = 0.375;
      
      //The distance from the centre of the robot to 
      //the centre of the search circle
      double distBotToCircle = 0.625;
      double robotX, robotY, robotYaw;
      double circleX, circleY;
      
      //find the robot...
	  playerc_simulation_get_pose2d(sim,robot_name, &robotX, &robotY, &robotYaw);
      
      /*now we find the centre of the search circle. 
      this is distBotToCircle metres from the robot's origin 
      along its yaw*/
           
      /*horizontal offset from robot origin*/
      circleX = distBotToCircle*cos(robotYaw);
           
      /*vertical offset from robot origin*/
      circleY = distBotToCircle*sin(robotYaw);
           
      //find actual centre relative to simulation.
      circleX = robotX + circleX;
      circleY = robotY + circleY;
           
      /* to find which items are within this circle we
      find their Euclidian distance to the circle centre.
      Find the closest one and if it's distance is smaller than
      the circle radius then return its index */
      
      double smallestDist = 1000000;
      int closestItem = 0; 
      int i;
      
      for(i=0; i<listLength; i++)
      {
            double x, y, dist; 
            
            // get manhattan distance from circle centre to item
            x = circleX - itemList[i].x;
            y = circleY - itemList[i].y;
            
            //find euclidian distance from circle centre to item
            dist = (x*x) + (y*y);
            dist = sqrt(dist);
                        
            if(dist < smallestDist)
            {
                  smallestDist = dist;
                  closestItem = i;
            }
      }
 
      if(smallestDist > (radius + distBotToCircle))
      {
      	printf("no objects were close enough, false alarm!\n");
      	return -1;
      }   
        
      return closestItem;
}



int main(int argc, char *argv[])
{	
	  playerc_simulation_t *simProxy;

		playerc_client_t *op;		

		for(int i = 0; i< 5; i++){
			robots[i].robot = playerc_client_create(NULL, "localhost", 6665+i);
			if (0 != playerc_client_connect(robots[i].robot)) return -1;
			robots[i].p2dProxy = playerc_position2d_create(robots[i].robot, 0);
			if (playerc_position2d_subscribe(robots[i].p2dProxy, PLAYER_OPEN_MODE)) return -1;
			robots[i].sonarProxy = playerc_ranger_create(robots[i].robot, 0);
			if (playerc_ranger_subscribe(robots[i].sonarProxy, PLAYER_OPEN_MODE)) return -1;
			robots[i].blobProxy = playerc_blobfinder_create(robots[i].robot, 0);
			if (playerc_blobfinder_subscribe(robots[i].blobProxy, PLAYER_OPEN_MODE)) return -1;
			robots[i].laserProxy = playerc_ranger_create(robots[i].robot, 1);
			if (playerc_ranger_subscribe(robots[i].laserProxy, PLAYER_OPEN_MODE)) return -1;

			playerc_position2d_enable(robots[i].p2dProxy,1);

			playerc_position2d_get_geom(robots[i].p2dProxy);
			playerc_ranger_get_geom(robots[i].sonarProxy);
			playerc_ranger_get_geom(robots[i].laserProxy);
	
		}
		simProxy = playerc_simulation_create(robots[0].robot, 0);
		if (playerc_simulation_subscribe(simProxy, PLAYER_OPEN_MODE)) return -1;

		op = playerc_client_create(NULL, "localhost", 6670);
		if (playerc_client_connect(op) != 0){
			puts( "Failed. Quitting." );
			return -1;
		}

		playerc_opaque_t *audio = playerc_opaque_create(op, 0);
		if (playerc_opaque_subscribe(audio, PLAYER_OPEN_MODE))
			return -1;

		item_t itemList[8];

		RefreshItemList(itemList, simProxy);

		srand(time(NULL));
		while(true){		
			for(int i = 0; i < 5; i++){
				// read from the proxies
				playerc_client_read(robots[i].robot);
	
				if(robots[i].blobProxy->blobs_count == 0){
					//wander
					printf("%d wandering\n",i);
					Wander(&robots[i].forwardSpeed, &robots[i].turnSpeed);
				}
				else{
					//move towards the item
					printf("%d moving to item\n", i);
					MoveToItem(&robots[i].forwardSpeed, &robots[i].turnSpeed, robots[i].blobProxy);
				}
				if(robots[i].laserProxy->ranges_count >= 89 && robots[i].laserProxy->ranges[89] < 0.25){
					int destroyThis;
					destroyThis = FindItem(itemList, 8, simProxy,(char *)"bob1");
					
					if(destroyThis != -1){
						//move it out of the simulation
						printf("%d collecting item\n",i);
						playerc_simulation_set_pose2d(simProxy,itemList[destroyThis].name, -10, -10, 0);
						RefreshItemList(itemList, simProxy);
					}
				}
				//avoid obstacles
				AvoidObstacles(&robots[i].forwardSpeed, &robots[i].turnSpeed, robots[i].sonarProxy);

				//set motors
				playerc_position2d_set_cmd_vel(robots[i].p2dProxy, robots[i].forwardSpeed, 0.0, DTOR(robots[i].turnSpeed), 1);
			}
		
			if (playerc_client_peek(op, 100) > 0) 
				playerc_client_read(op);

			// If there's new data
			if (audio->data_count>0) {
				// print it
				printf("%d\n", ((test_t*)audio->data)->uint8);
				// clear read data
				audio->data_count=0;
			}
			// send a message approximately every two seconds
			if ( rand() % 4 == 0 ) {   
				// clear a new data packet
				player_opaque_data_t audio_msg;
				test_t *thingy;
				thingy->uint8 = 10;
				audio_msg.data_count=sizeof(test_t);
				playerc_opaque_cmd(audio, &audio_msg);
			}

			sleep(1);
			
		}


	/* Shutdown */
	for(int i=0; i<5; i++){
		playerc_position2d_unsubscribe(robots[i].p2dProxy);
		playerc_ranger_unsubscribe(robots[i].sonarProxy);
		playerc_blobfinder_unsubscribe(robots[i].blobProxy);
		playerc_ranger_unsubscribe(robots[i].laserProxy);

		playerc_position2d_destroy(robots[i].p2dProxy); 
		playerc_ranger_destroy(robots[i].sonarProxy);
		playerc_blobfinder_destroy(robots[i].blobProxy);
		playerc_ranger_destroy(robots[i].laserProxy);

		playerc_client_disconnect(robots[i].robot);
		playerc_client_destroy(robots[i].robot);
	}

	playerc_simulation_unsubscribe(simProxy);
	playerc_simulation_destroy(simProxy);
  playerc_opaque_unsubscribe(audio);
  playerc_opaque_destroy(audio);
  playerc_client_disconnect(op);
  playerc_client_destroy(op);
  return 0;
}

