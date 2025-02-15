#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include <libplayerc/playerc.h>
#include "audioshared.h"


struct Robot {
    playerc_client_t *robot;
    playerc_position2d_t *p2dProxy;
    playerc_ranger_t *sonarProxy;
    playerc_blobfinder_t *blobProxy;
    playerc_ranger_t *laserProxy;
    double forwardSpeed;
    double turnSpeed;
    int sink;
} typedef our_robot_t;

our_robot_t robots[5];

playerc_map_t *map;
int **og_map;
double map_width_m;
double map_height_m;

// simple Queue
// https://stackoverflow.com/a/43687183
typedef struct node {
    int i;
    int j;
    struct node *next;
} node_t;
void
enqueue (node_t ** head, int i, int j) {
    //printf("about to malloc\n");
    node_t *new_node = malloc (sizeof (node_t));
    if (!new_node)
	return;
    //printf("  no memory issue\n");
    new_node->i = i;
    new_node->j = j;
    new_node->next = *head;
    *head = new_node;
    //printf("  allocated new node\n");
}

node_t
dequeue (node_t ** head) {
    node_t *curr, *prev = NULL;
    node_t retval = { -1, -1, NULL };
    if (*head == NULL)
	return retval;
    curr = *head;
    while (curr->next != NULL) {
	prev = curr;
	curr = curr->next;
    }
    retval.i = curr->i;
    retval.j = curr->j;
    //printf("     %d %d\n", retval->i, retval->j);
    free (curr);
    if (prev) {
	prev->next = NULL;
    }
    else {
	*head = NULL;
    }
    return retval;
}

int
globalToCell (double g_max_m, double g_curr_m, int pixels) {
    //printf(" gtc: %f %f %d\n", g_curr_m, g_max_m, pixels);
    return (int) floor ((g_curr_m + g_max_m / 2) / g_max_m * pixels);
}

//return -1 if not found, 0 if north, 1 if east, 2 if south and 3 if west.
int
whereAudioFrom (int src_id, int rcv_id, double s_px, double s_py) {
    printf ("--- AUDIO PROP ALGO : %d -> %d\n", src_id, rcv_id);
    if (src_id == rcv_id) {
	return -1;
    }
    //printf(" src %d\n", src_id);
    //printf("  %d %f %f should equal %f %f\n", src_id, s_px, s_py, robots[src_id].p2dProxy->px, robots[src_id].p2dProxy->py);
    int r_i = globalToCell (map_height_m, robots[rcv_id].p2dProxy->py, map->height);
    int r_j = globalToCell (map_width_m, robots[rcv_id].p2dProxy->px, map->width);
    // generate a 2d array, perform BFS to find shortest path
    // from src_id -> rcv_id
    int t_map[map->height][map->width];
    memcpy (t_map, og_map, sizeof (int) * map->height * map->width);
    node_t *Q = NULL;
    int s_i = globalToCell (map_height_m, s_py, map->height);
    int s_j = globalToCell (map_width_m, s_px, map->width);
    printf ("    src  map cell: %d %d\n", s_i, s_j);
    printf ("    goal map cell: %d %d\n", r_i, r_j);
    enqueue (&Q, s_i, s_j);
    // do BFS
    int layer = 0;
    int brk = -1;
    while (Q != NULL && brk == -1) {	// todo, calculate layer
	//printf("BFS iter %d\n", layer);
	// dequeue Q
	node_t C = dequeue (&Q);
	// visit node
	if (t_map[C.i][C.j] > 0)
	    continue;		// already visited!
	t_map[C.i][C.j] = layer + 1;
	layer++;
	//printf(" %d %d\n", C.i, C.j);
	// calculate neighbors
	node_t top = { C.i - 1, C.j, NULL };
	node_t bot = { C.i + 1, C.j, NULL };
	node_t lef = { C.i, C.j - 1, NULL };
	node_t rig = { C.i, C.j + 1, NULL };
	// enqueue nodes
	//printf(" %d %d %d\n", top.i, top.j, t_map[top.i][top.j]);
	if (top.i > -1 && t_map[top.i][top.j] == 0) {
	    enqueue (&Q, top.i, top.j);
	}
	if (bot.i < map->height && t_map[bot.i][bot.j] == 0) {
	    enqueue (&Q, bot.i, bot.j);
	}
	if (lef.j > -1 && t_map[lef.i][lef.j] == 0) {
	    enqueue (&Q, lef.i, lef.j);
	}
	if (rig.j < map->width && t_map[rig.i][rig.j] == 0) {
	    enqueue (&Q, rig.i, rig.j);
	}
	//printf(" enqueue-d 4 nodes\n");
	// check for robot
	if (top.i == r_i && top.j == r_j) {
	    brk = 2;
	}
	if (bot.i == r_i && bot.j == r_j) {
	    brk = 0;
	}
	if (lef.i == r_i && lef.j == r_j) {
	    brk = 1;
	}
	if (rig.i == r_i && rig.j == r_j) {
	    brk = 3;
	}
	//printf(" %p %d %d\n", Q, brk, layer);
    }
    printf ("    done -- visited %d nodes -- brk = %d\n", layer, brk);
    while (Q != NULL) {
	dequeue (&Q);
    }
    // done
    return brk;
}


/**
Randomly assigns new speeds into the given addresses. 
This function will always write to the given addresses.
@param *forwardSpeed the address of the forward speed 
variable you want this function to change.
@param *turnSpeed the address of the turn speed variable 
you want this function to change.
*/
void
Wander (double *forwardSpeed, double *turnSpeed) {
    int maxSpeed = 1;
    int maxTurn = 90;
    double fspeed, tspeed;

    //fspeed is between 0 and 10
    fspeed = rand () % 11;
    //(fspeed/10) is between 0 and 1
    fspeed = (fspeed / 10) * maxSpeed;
    //fspeed = 0.7;

    tspeed = rand () % (2 * maxTurn);
    tspeed = tspeed - maxTurn;

    *forwardSpeed = fspeed;
    *turnSpeed = tspeed;

}
void
Wander_Mod (double *forwardSpeed, double *turnSpeed,playerc_ranger_t * sproxy) {
	int avoid = 0;
	double newspeed = 0.200;
	double really_close = 0.3;
	double just_close = 0.5;

	for(int i=0; i< sproxy->ranges_count-1;i++)
		if (sproxy->ranges[i] < really_close){
			newspeed= -0.100;
			avoid = 2;
		}else if (sproxy->ranges[i] < just_close){
			*forwardSpeed = 0;
			if(avoid != 2)
				avoid = 1;
		}
		
	if(avoid){
		if(sproxy->ranges[0] + sproxy->ranges[2] < sproxy->ranges[1] + sproxy->ranges[3])
			*turnSpeed = -30;
		else
			*turnSpeed = 30;
	}else{ 
		int r = rand()%4;
		int t = rand()%2;
		if(r == 0)
			if(t)
				*turnSpeed = 30;
			else
				*turnSpeed = -30;
		else
			*turnSpeed = 0;
	}
	*forwardSpeed = newspeed;	

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
void
AvoidObstacles (double *forwardSpeed, double *turnSpeed, playerc_ranger_t * sproxy) {
    double *sp = sproxy->ranges;	// pointer to range array

    //will avoid obstacles closer than 40cm
    double avoidDistance = 0.4;
    //will turn away at 60 degrees/sec
    int avoidTurnSpeed = 60;

    //left corner is sonar no. 2
    //right corner is sonar no. 3
    if (sp[2] < avoidDistance) {
	*forwardSpeed = 0;
	//turn right
	*turnSpeed = (-1) * avoidTurnSpeed;
	printf ("avoiding obstacle\n");
    } else if (sp[3] < avoidDistance) {
	*forwardSpeed = 0;
	//turn left
	*turnSpeed = avoidTurnSpeed;
	printf ("avoiding obstacle\n");
    } else if ((sp[0] < avoidDistance) && (sp[1] < avoidDistance)) {
	//back off a little bit
	*forwardSpeed = -0.2;
	// *turnSpeed = avoidTurnSpeed;
	printf ("avoiding obstacle\n");
    }
    return;			//do nothing
}

void
avoid_mod(double *forwardSpeed, double *turnSpeed,playerc_ranger_t * sproxy) {
	int avoid = 0;
	double really_close = 0.3;
	double just_close = 0.5;

	for(int i=0; i< sproxy->ranges_count-1;i++)
		if (sproxy->ranges[i] < really_close)
				avoid = 2;
		else if (sproxy->ranges[i] < just_close){
			if (avoid != 2)
				avoid = 1;
		}
		
	if(avoid)
		if(sproxy->ranges[0] + sproxy->ranges[2] < sproxy->ranges[1] + sproxy->ranges[3])
			*turnSpeed = -10;
		else
			*turnSpeed = 10;
	if(avoid == 2)
		*forwardSpeed = -0.2;

}

void
avoidsound_mod(double *forwardSpeed, double *turnSpeed,playerc_ranger_t * sproxy) {
	int avoid = 0;
	double really_close = 0.2;
	double just_close = 0.5;

	for(int i=0; i< sproxy->ranges_count-1;i++)
		if (sproxy->ranges[i] < really_close)
				avoid = 2;
		else if (sproxy->ranges[i] < just_close){
			if (avoid != 2)
				avoid = 1;
		}
		
	if(avoid){
		if(sproxy->ranges[0] + sproxy->ranges[2] < sproxy->ranges[1] + sproxy->ranges[3])
			*turnSpeed = -30;
		else
			*turnSpeed = 30;
		*forwardSpeed = .5;	
		if (avoid == 2)
			*forwardSpeed = -0.2;
	}
		*forwardSpeed = .2;	

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
void
MoveToItem (double *forwardSpeed, double *turnSpeed, playerc_blobfinder_t * bfp) {
    int i, centre;
    int noBlobs = bfp->blobs_count;
    playerc_blobfinder_blob_t blob;
    int turningSpeed = 5;	// in deg/s

    /*number of pixels away from the image centre a blob
       can be to be in front of the robot */
    int margin = 10;

    int biggestBlobArea = 0;
    int biggestBlob = 0;

    //find the largest blob
    for (i = 0; i < noBlobs; i++) {
	//get blob from proxy
	playerc_blobfinder_blob_t currBlob = bfp->blobs[i];
	// (.area is a negative cast into an unsigned int! oops.)
	if (abs ((int) currBlob.area) > biggestBlobArea) {
	    biggestBlob = i;
	    biggestBlobArea = abs ((int) currBlob.area);
	}
    }
    blob = bfp->blobs[biggestBlob];
    //printf("biggest blob is %i with area %d\n",biggestBlob,biggestBlobArea);
    // find centre of image
    centre = bfp->width / 2;
    //adjust turn to centre the blob in image
    /*if the blob's centre is within some margin of the image 
       centre then move forwards, otherwise turn so that it is 
       centred. */
    //printf("blob.x=%d, c=%d\n",blob.x,centre);

    //blob to the left of centre
    if (blob.x < centre - margin) {
	*forwardSpeed = 0;
	//turn left
	*turnSpeed = turningSpeed;
	//printf("turning left\n");
    }
    //blob to the right of centre
    else if (blob.x > centre + margin) {
	*forwardSpeed = 0;
	//turn right
	*turnSpeed = -turningSpeed;
	//printf("turning right\n");
    }
    //otherwise go straight ahead
    else {
	*forwardSpeed = 0.4;
	*turnSpeed = 0;
	//printf("straight on\n");
    }
    return;
}

void
our_set_move (int id, double fs, double ts) {
    robots[id].forwardSpeed = fs;
    robots[id].turnSpeed = ts;
    printf ("speed %f %f\n", fs, ts);
}

void
iGotSomeData (playerc_opaque_t * audio, int rec_id) {
    double x = ((our_audio_t *) audio->data)->px;
    double y = ((our_audio_t *) audio->data)->py;
    int source_id = ((our_audio_t *) audio->data)->id;
    int sink = ((our_audio_t *) audio->data)->sink;
    if (source_id == -1 || source_id == rec_id)
	return;
    printf ("receiving signal on %d:\n", rec_id);
    //printf ("%f %f %d %d\n", x, y, source_id, sink);
		robots[rec_id].sink = 1;
//      else if (sink != robots[rec_id].sink) return;
    audio->data_count = 0;
    int dir = whereAudioFrom (source_id, rec_id, x, y);
    double px = robots[rec_id].p2dProxy->px;
    double py = robots[rec_id].p2dProxy->py;
    double pa = robots[rec_id].p2dProxy->pa;
    double bound = 1.0 / 12.0;
    double ts = 10;
    double fs = 0.5;
    double tfs = 0.0;
    while (pa > 2 * M_PI)
	pa = pa - 2 * M_PI;
    while (pa < 0)
	pa = pa + 2 * M_PI;
    printf ("rec: %d, dir: %d yaw: %f\n", rec_id, dir, pa);
    if (dir == 0) {
	if (pa < M_PI / 2 - bound || pa > 3*M_PI / 2)
	    our_set_move (rec_id, tfs, ts);
	else if (pa > M_PI / 2 + bound && pa <= 3*M_PI / 2)
	    our_set_move (rec_id, tfs, -ts);
	else
	    our_set_move (rec_id, fs, 0);
    }
    else if (dir == 1) {
	if (pa < 2 * M_PI - bound && pa >= M_PI)
	    our_set_move (rec_id, tfs, ts);
	else if (pa > 0 + bound && pa < M_PI)
	    our_set_move (rec_id, tfs, -ts);
	else
	    our_set_move (rec_id, fs, 0);
    }
    else if (dir == 2) {
	if (pa < 3 * M_PI / 2 - bound && pa >= M_PI / 2)
	    our_set_move (rec_id, tfs, ts);
	else if (pa > 3 * M_PI / 2 + bound || pa < M_PI / 2)
	    our_set_move (rec_id, tfs, -ts);
	else
	    our_set_move (rec_id, fs, 0);
    }
    else if (dir == 3) {
	if (pa < M_PI - bound)
	    our_set_move (rec_id, tfs, ts);
	else if (pa > M_PI + bound)
	    our_set_move (rec_id, tfs, -ts);
	else
	    our_set_move (rec_id, fs, 0);
    }
}

int
main (int argc, char *argv[]) {
    playerc_simulation_t *simProxy;

    playerc_client_t *op;

    for (int i = 0; i < 5; i++) {
	robots[i].robot = playerc_client_create (NULL, "localhost", 6665 + i);
	if (0 != playerc_client_connect (robots[i].robot))
	    return -1;
	robots[i].p2dProxy = playerc_position2d_create (robots[i].robot, 0);
	if (playerc_position2d_subscribe (robots[i].p2dProxy, PLAYER_OPEN_MODE))
	    return -1;
	robots[i].sonarProxy = playerc_ranger_create (robots[i].robot, 0);
	if (playerc_ranger_subscribe (robots[i].sonarProxy, PLAYER_OPEN_MODE))
	    return -1;
	robots[i].blobProxy = playerc_blobfinder_create (robots[i].robot, 0);
	if (playerc_blobfinder_subscribe (robots[i].blobProxy, PLAYER_OPEN_MODE))
	    return -1;
	robots[i].laserProxy = playerc_ranger_create (robots[i].robot, 1);
	if (playerc_ranger_subscribe (robots[i].laserProxy, PLAYER_OPEN_MODE))
	    return -1;

	playerc_position2d_enable (robots[i].p2dProxy, 1);

	playerc_position2d_get_geom (robots[i].p2dProxy);
	playerc_ranger_get_geom (robots[i].sonarProxy);
	playerc_ranger_get_geom (robots[i].laserProxy);

	robots[i].sink = -1;
    }
    simProxy = playerc_simulation_create (robots[0].robot, 0);
    if (playerc_simulation_subscribe (simProxy, PLAYER_OPEN_MODE))
	return -1;
    map = playerc_map_create (robots[0].robot, 0);
    if (playerc_map_subscribe (map, PLAYER_OPEN_MODE))
	return -1;
    // calculate original map
    playerc_map_get_map (map);
    printf ("map info:\n  w: %d h: %d r: %f\n", map->width, map->height, map->resolution);	// map->resolution = m/cell
    map_width_m = map->width * map->resolution;
    map_height_m = map->height * map->resolution;
    printf ("  w: %fm h: %fm\n", map_width_m, map_height_m);
    og_map =
	malloc (sizeof (int *) * map->height +
		sizeof (int) * map->width * map->height);
    for (int i = 0; i < map->height; i++) {
	og_map[i] = ((int *) (og_map + map->height)) + map->width * i;
    }
    // memcpy(og_map, map->cells, sizeof(char) * map->width * map->height);
    // -1=empty, 0=unknown, 1=occupied
    for (int i = 0; i < map->height; i++) {
	for (int j = 0; j < map->width; j++) {
	    og_map[i][j] = (int) *(map->cells + i * map->height + j);
	    printf ("%d ", og_map[i][j]);
	    if (og_map[i][j] > 0) {
		og_map[i][j] = -1;
	    }
	    else {
		og_map[i][j] = 0;
	    }
	    printf ("%d ", og_map[i][j]);
	}
	printf ("\n");
    }

    op = playerc_client_create (NULL, "localhost", 6670);
    if (playerc_client_connect (op) != 0)
	return -1;
    playerc_client_datamode (op, PLAYER_DATAMODE_PUSH);

    playerc_opaque_t *audio = playerc_opaque_create (op, 0);
    if (playerc_opaque_subscribe (audio, PLAYER_OPEN_MODE))
	return -1;

    srand (time (NULL));
    while (true) {
	for (int i = 0; i < 5; i++) {
	    // read from the proxies
	    playerc_client_read (robots[i].robot);

	    //read audio from driver
	    if (playerc_client_peek (op, 100) > 0)
		playerc_client_read (op);
	    if (audio->data_count > 0) {
		//hear a signal
		iGotSomeData (audio, i);
	    }else robots[i].sink = -1;

	    if (robots[i].blobProxy->blobs_count == 0) {
		if (robots[i].sink != 1) {
		    //wander
		    printf ("%d wandering\n", i);
		    //Wander (&robots[i].forwardSpeed, &robots[i].turnSpeed);
		    Wander_Mod (&robots[i].forwardSpeed, &robots[i].turnSpeed, robots[i].sonarProxy);
		}
                // i see nothing -- don't send audio signal
	    } else {
		//move towards the item

		if (robots[i].sink != 1) {
		    printf ("%d found an item\n", i);
		    //print position and stuff to driver
		    player_opaque_data_t audio_msg;
		    our_audio_t thingy;
		    thingy.px = robots[i].p2dProxy->px;
		    thingy.py = robots[i].p2dProxy->py;
		    thingy.id = i;
		    thingy.sink = 1;
		    audio_msg.data = (uint8_t *) & thingy;
		    audio_msg.data_count = sizeof (our_audio_t);
		    //for (int j = 0; j < 5; j++)
			playerc_opaque_cmd (audio, &audio_msg);
		}
		MoveToItem (&robots[i].forwardSpeed, &robots[i].turnSpeed,
			    robots[i].blobProxy);
	    }
//	    if (robots[i].laserProxy->ranges_count >= 89
//		&& robots[i].laserProxy->ranges[89] < 0.25) {
//	    }
	    //avoid obstacles
	    if (robots[i].sink != 1)
		//AvoidObstacles (&robots[i].forwardSpeed, &robots[i].turnSpeed,robots[i].sonarProxy);
		avoid_mod(&robots[i].forwardSpeed, &robots[i].turnSpeed,robots[i].sonarProxy);
			else
		avoidsound_mod(&robots[i].forwardSpeed, &robots[i].turnSpeed,robots[i].sonarProxy);


	    //set motors
	    playerc_position2d_set_cmd_vel (robots[i].p2dProxy,
					    robots[i].forwardSpeed, 0.0,
					    DTOR (robots[i].turnSpeed), 1);

	}

	sleep (1);
    }


    /* Shutdown */
    for (int i = 0; i < 5; i++) {
	playerc_position2d_unsubscribe (robots[i].p2dProxy);
	playerc_ranger_unsubscribe (robots[i].sonarProxy);
	playerc_blobfinder_unsubscribe (robots[i].blobProxy);
	playerc_ranger_unsubscribe (robots[i].laserProxy);

	playerc_position2d_destroy (robots[i].p2dProxy);
	playerc_ranger_destroy (robots[i].sonarProxy);
	playerc_blobfinder_destroy (robots[i].blobProxy);
	playerc_ranger_destroy (robots[i].laserProxy);

	playerc_client_disconnect (robots[i].robot);
	playerc_client_destroy (robots[i].robot);
    }

    playerc_simulation_unsubscribe (simProxy);
    playerc_simulation_destroy (simProxy);
    playerc_opaque_unsubscribe (audio);
    playerc_opaque_destroy (audio);
    playerc_client_disconnect (op);
    playerc_client_destroy (op);

    for (int i = 0; i < map->height; i++) {
	free (og_map[i]);
    }
    free (og_map);

    return 0;
}
