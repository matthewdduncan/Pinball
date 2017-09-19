

//Demonstration of basic collision detection functionality between a ball and spherical / cube-like objects
//Uses basic graphics libraries, and handles simple graphics and physics.
//This is designed for use on windows.  Functionally not guaranteed on other operating systems
//Created by Matthew D. Duncan, 2017

#include <stdlib.h> 
#include <stdio.h>
#include <math.h>
#include <chrono>
#include <thread>

#include <GL/glut.h>	// OpenGL Graphics Utility Library

//Declare initial global variables

int ScreenWidth = 0;	// Width of the window in pixels, initialized to zero but set in main
int ScreenHeight = 0;	// Height of the window in pixels, initialized to zero but set in main

// Lighting values
float ambientLight[4] = {0.2f, 0.2f, 0.2f, 1.0f};
float Lt0amb[4] = {0.3f, 0.3f, 0.3f, 1.0f};
float Lt0diff[4] = {1.0, 1.0, 1.0, 1.0};
float Lt0spec[4] = {1.0, 1.0, 1.0, 1.0};
float matSpec[4] ={ 0.4f, 0.4f, 0.4f, 1.0f };
float matAmbDiff[4] = { 0.25f, 0.25f, 0.25f, 1.0 };
float Lt0pos[4] = { -20, 40.0, 100.0, 0.0 };
float Lt1pos[4] = { 40, 40.0, 100.0, 0.0 };

float pi = 3.14159265358979;

//defines viewing distance
float farout = 15;

//stage of flipper rotation
int lphase = 0;
int rphase = 0;
bool countlphase = false;
bool countrphase = false;

//Determines zoom level
bool frame = true;

float vector0[3] = { 0, 0, 0 }; // used when a zero vector is needed

//DEFINE PARAMETERS TO CREATE SHAPES

//ball
float origposition[3] = { .1,5,0 };
float position[3] = { .1,5,0 };
float velocity[3] = { 0, 0, 0 };
float gravity = 0.003;
float radius = 1;

//borders of playing field
float borderx[2] = { -20, 20 };
float bordery[2] = { -40, 40 };
float topborderpos[3] = { 0, 40, 0 };
float botborderpos[3] = { 0, -40, 0 };
float leftborderpos[3] = { -20.5, 0, 0 };
float rightborderpos[3] = { 20.5, 0, 0 };
float sidebordersize[3] = { 1, 81, 3 };
float topbordersize[3] = { 40, 1, 3 };
float borderbounce = 0.4;


//funnel pieces
float funnelbounce = 0.2;
float funnelsize[3] = { 15, 0.5, 3 };
float funnelpos[3] = { 14, -28, 0 };
float funnelrot[3] = { -pi/6, 0, 0 };
float funnelrot2[3] = { pi/6, 0, 0 };
float funnelpos2[3] = { funnelpos[0] * -1.0, funnelpos[1], funnelpos[2] };

//joint locations for flipper
float point[3] = { 7.5, -32, 0};
float point2[3] = {-7.5, -32, 0};

//center cube
float testpos[3] = { 0, 0, 0 };
float testrot[3] = { pi/4, 0, 0 };
float testsize[3] = { 1, 2, 3 };

//flippers
float flipperbounce = 0.5;
float speedflip = 0.1; // speed at which flippers rotate when activated
float flippersize[3] = { 5, 0.3, 3 };

//spherical obstacles
float obstructorpositions[3][3]{
	{0, -37, 0},
	{10, 0, 0},
	{-15, 0, 0}
};
float obstructorradius = 0.5;
float obstructorbounce = .5;
float numcircles = 3;



///////////////////Vector operations///////////////////

// Find the length of a vector
static float lenvec(float* vec) {
	float rv = sqrtf(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
	return rv;
}

//Normalize the length of a vector
static void normalizevec(float* vec) {
	float len = lenvec(vec);
	for (int i = 0; i <= 2; i++) {
		vec[i] = vec[i] / len;
	}
}

// Find the linear distance between two points
static float truedistance(float* vec1, float* vec2) {
	return sqrt(pow(vec1[0] - vec2[0], 2) + pow(vec1[1] - vec2[1], 2) + pow(vec1[2] - vec2[2], 2));
}

/////////////////////////////////////////////////////////

//abstract struct defining all objects used in the scene
struct shape {
	
	//position
	float xpos;
	float ypos;
	float zpos;

	//velocity
	float xvel;
	float yvel;
	float zvel;

	//bounciness
	float bounce;

	//constructor
	shape(float* position, float bounce, float* velocity) : 
		xpos(position[0]), ypos(position[1]), zpos(position[2]), bounce(bounce), xvel(velocity[0]), yvel(velocity[1]), zvel(velocity[2]) 
	{}

	virtual void draw() = 0;
};

// struct defining spherical shapes
struct sphere : shape {

	float radius;

	//constructor
	sphere(float* position, float radius, float bounce, float* velocity) :
		shape(position, bounce, velocity), radius(radius)
	{}

	// draws the sphere
	void draw() {

		glPushMatrix();

		//puts sphere in correct position
		glTranslatef(xpos, ypos, zpos);

		//draws sphere using glutSolidSphere
		glutSolidSphere(radius, 20, 20);

		glPopMatrix();
	}

	//determines the distance from this sphere to another
	float centerdistance(sphere mainball) {
		return sqrt(pow(xpos - mainball.xpos, 2) + pow(ypos - mainball.ypos, 2) + pow(zpos - mainball.zpos, 2));
	}

	//detects and resolves collisions between this sphere and another.  passed by reference for proper handling
	bool detectballcollision(sphere &mainball) {

		// if the spheres would intersect, handle the collision
		if (centerdistance(mainball) <= (radius + mainball.radius)) {

			//find and normalize vector between the centers of the spheres
			float directionvector[3] = { mainball.xpos - xpos, mainball.ypos - ypos, mainball.zpos - zpos };
			normalizevec(directionvector);
			
			// update position so spheres no longer intersect
			mainball.xpos = directionvector[0] * (radius + mainball.radius) + xpos;
			mainball.ypos = directionvector[1] * (radius + mainball.radius) + ypos;
			mainball.zpos = directionvector[2] * (radius + mainball.radius) + zpos;

			//used to find the projection of the velocity vector onto the direction vector
			float dotproduct = mainball.xvel*directionvector[0] + mainball.yvel*directionvector[1] + mainball.zvel*directionvector[2];

			//subtract the projection vector from the velocity vector (and multiply by a bounce factor) to resolve velocity change
			mainball.xvel -= ((dotproduct * directionvector[0]) * (1 + bounce));
			mainball.yvel -= ((dotproduct * directionvector[1]) * (1 + bounce));
			mainball.zvel -= ((dotproduct * directionvector[2]) * (1 + bounce));


			return true; // there is a collision
		}

		else return false; // there is not a collision
	}

	//adjust position of sphere by a vector
	void adjustposition(float* value) {
		xpos = value[0];
		ypos = value[1];
		zpos = value[2];
	}

	//apply gravity to a vector (must be called every frame)
	void gravity(float intensity) {
		yvel -= intensity;
	}

	//update position using the velocity (must be called every frame)
	void step() {
		xpos += xvel;
		ypos += yvel;
		zpos += zvel;
	}
};

//struct for defining cubes and rectangular prisms
struct cube : shape {

	//rotation (note that this is not in the same reference frame as the camera)
	float roll;
	float pitch;
	float yaw;

	//size
	float xscl;
	float yscl;
	float zscl;

	cube(float* position, float* rotation, float* size, float bounce, float* velocity) :
		shape(position, bounce, velocity), roll(rotation[0]), pitch(rotation[1]), yaw(rotation[2]), xscl(size[0]), yscl(size[1]), zscl(size[2])
	{  }

	// defines the matrix for a linear transformation into the local reference frame of the cube.  defines a rotation
	float rotationmatrix[4][4] =
	{
		{ cosf(roll)*cosf(pitch), (cosf(roll)*sinf(pitch)*sinf(yaw)) - (sinf(roll)*cosf(yaw)), cosf(roll)*sinf(pitch)*cosf(yaw) + sinf(roll)*sinf(yaw), 0 },
		{ sinf(roll)*cosf(pitch), sinf(roll)*sinf(pitch)*sinf(yaw) + cosf(roll)*cosf(yaw), sinf(roll)*sinf(pitch)*cosf(yaw) - cosf(roll)*sinf(yaw), 0 },
		{ -1 * sinf(pitch), cosf(pitch)*sinf(yaw), cosf(pitch)*cosf(yaw), 0 },
		{ 0, 0, 0, 1 }
	};

	// inverse of above.  since this is a rotation matrix it is simply the transpose of the above
	float irotationmatrix[4][4] =
	{
		{ rotationmatrix[0][0], rotationmatrix[1][0], rotationmatrix[2][0], 0 },
		{ rotationmatrix[0][1], rotationmatrix[1][1], rotationmatrix[2][1], 0 },
		{ rotationmatrix[0][2], rotationmatrix[1][2], rotationmatrix[2][2], 0 },
		{ 0, 0, 0, 1 }
	};

	// pointer to the rotation matrix
	float* rotmat = *rotationmatrix;

	void draw() { // draws the cube with the given parameters

		//draw cube
		glPushMatrix();

		glTranslatef(xpos, ypos, zpos); //position
		glMultMatrixf(rotmat); // rotation
		glScalef(xscl, yscl, zscl); // scale

		//draw cube using glutSolidCube
		glutSolidCube(1);

		glPopMatrix();
	}

	//handle collisions between the cube and a sphere.  passed by reference for proper handling
	bool detectBallCollision(sphere &mainball) {

		//define local varables
		float ballpos[3] = { mainball.xpos, mainball.ypos, mainball.zpos };
		float ballvelocity[3] = { mainball.xvel, mainball.yvel, mainball.zvel };
		float ballradius = mainball.radius;


		bool collision = false; // used for output.  this is updated if a collision is detected

		// apply linear transformation from camera's frame of reference into cube's frame of reference.  This makes the math far simpler.

		float newballpos[3] = { (ballpos[0] - xpos) * rotationmatrix[0][0] + (ballpos[1] - ypos) * rotationmatrix[0][1] + (ballpos[2] - zpos) * rotationmatrix[0][2],
			(ballpos[0] - xpos) * rotationmatrix[1][0] + (ballpos[1] - ypos) * rotationmatrix[1][1] + (ballpos[2] - zpos) * rotationmatrix[1][2],
			(ballpos[0] - xpos) * rotationmatrix[2][0] + (ballpos[1] - ypos) * rotationmatrix[2][1] + (ballpos[2] - zpos) * rotationmatrix[2][2],
		}; // relative location of ball to center of cube, in cube's local coordinates

		float newballvelocity[3] = { ballvelocity[0] * rotationmatrix[0][0] + ballvelocity[1] * rotationmatrix[0][1] + ballvelocity[2] * rotationmatrix[0][2],
			ballvelocity[0] * rotationmatrix[1][0] + ballvelocity[1] * rotationmatrix[1][1] + ballvelocity[2] * rotationmatrix[1][2],
			ballvelocity[0] * rotationmatrix[2][0] + ballvelocity[1] * rotationmatrix[2][1] + ballvelocity[2] * rotationmatrix[2][2],
		}; // relative velocity of ball, in cube's local reference frame


		// used for calculating the closest point along an edge to the sphere.  for handling edge collisions
		float xclosest = xscl / 2.0 * (newballpos[0] / abs(newballpos[0]));
		float yclosest = yscl / 2.0 * (newballpos[1] / abs(newballpos[1]));
		float zclosest = zscl / 2.0 * (newballpos[2] / abs(newballpos[2]));

		//determines if the surface of the sphere intersects the faces of the cube in any demension
		bool sinsideX = abs(newballpos[0]) - ballradius <= xscl / 2.0;
		bool sinsideY = abs(newballpos[1]) - ballradius <= yscl / 2.0;
		bool sinsideZ = abs(newballpos[2]) - ballradius <= zscl / 2.0;

		//determines if the center of the sphere intersects the faces of the cube in any demension
		bool cinsideX = (abs(newballpos[0]) <= xscl / 2.0);
		bool cinsideY = (abs(newballpos[1]) <= yscl / 2.0);
		bool cinsideZ = (abs(newballpos[2]) <= zscl / 2.0);


		//Detect and handle the collision
		
		// Collision with faces

		//Collisions are handled only once, and priority is given, if multiple collisions apply, to faces, then edges, then vertices
		//This order of handling prevents strange bounce directions near the edges.
		
		//X face
		if (sinsideX && !cinsideX && cinsideY && cinsideZ) { // determines if this is a collision with an X face

			collision = true;

			// if statement determines which X face (left or right) is hit, and corrects position accordingly
			if (newballpos[0] >= 0) {
				newballpos[0] = xscl / 2.0 + ballradius;
			}

			else {
				newballpos[0] = -xscl / 2.0 - ballradius;
			}

			// update velocity after bounce
			newballvelocity[0] = -1 * newballvelocity[0] * bounce;
		}

		//Y face
		else if (sinsideY && cinsideX && !cinsideY && cinsideZ) { //determines if this is a collision with a Y face

			collision = true;

			// if statement determines which Y face (top or bottom) is hit, and corrects position accordingly
			if (newballpos[1] >= 0) {
				newballpos[1] = yscl / 2.0 + ballradius;
			}

			else {
				newballpos[1] = -yscl / 2.0 - ballradius;
			}

			// update velocity after bounce
			newballvelocity[1] = -1 * newballvelocity[1] * bounce;
		}

		//Z face
		else if (sinsideZ && cinsideX && cinsideY && !cinsideZ) {  //determines if this is a collision with a Z face

			collision = true;

			// if statement determines which Z face (front or back) is hit, and corrects position accordingly
			if (newballpos[2] >= 0) {
				newballpos[2] = zscl / 2.0 + ballradius;
			}

			else {
				newballpos[2] = -zscl / 2.0 - ballradius;
			}

			// update velocity after bounce
			newballvelocity[2] = -1 * newballvelocity[2] * bounce;
		}

		//collision with edge

		// XY edge (formed between x faces and y faces)
		else if (sinsideX && sinsideY && !cinsideX && !cinsideY && cinsideZ) {//determines if this collision is possible (does not guarantee it will happen)

			// find closest point on edge to the colliding sphere
			float contactpoint[3] = { xclosest, yclosest, newballpos[2] };

			// create a 0-radius sphere at this point for the colliding sphere to collide with
			sphere contactsphere = sphere(contactpoint,0,bounce,vector0);

			// update values in mainball for passing into detectballcollision
			mainball.xpos = newballpos[0];
			mainball.ypos = newballpos[1];
			mainball.zpos = newballpos[2];

			mainball.xvel = newballvelocity[0];
			mainball.yvel = newballvelocity[1];
			mainball.zvel = newballvelocity[2];

			// handle collision
			collision = (contactsphere.detectballcollision(mainball) || collision);

			// update local variables with results
			newballpos[0] = mainball.xpos;
			newballpos[1] = mainball.ypos;
			newballpos[2] = mainball.zpos;

			newballvelocity[0] = mainball.xvel;
			newballvelocity[1] = mainball.yvel;
			newballvelocity[2] = mainball.zvel;

			
		}

		//XZ edge (formed between x faces and z faces)
		else if (sinsideX && sinsideZ && !cinsideX && cinsideY && !cinsideZ) {//determines if this collision is possible (does not guarantee it will happen)

			// find closest point on edge to the colliding sphere
			float contactpoint[3] = {xclosest, newballpos[1], zclosest};

			// create a 0-radius sphere at this point for the colliding sphere to collide with
			sphere contactsphere = sphere(contactpoint, 0, bounce, vector0);

			// update values in mainball for passing into detectballcollision
			mainball.xpos = newballpos[0];
			mainball.ypos = newballpos[1];
			mainball.zpos = newballpos[2];

			mainball.xvel = newballvelocity[0];
			mainball.yvel = newballvelocity[1];
			mainball.zvel = newballvelocity[2];

			// handle collision
			collision = (contactsphere.detectballcollision(mainball) || collision);

			// update local variables with results
			newballpos[0] = mainball.xpos;
			newballpos[1] = mainball.ypos;
			newballpos[2] = mainball.zpos;

			newballvelocity[0] = mainball.xvel;
			newballvelocity[1] = mainball.yvel;
			newballvelocity[2] = mainball.zvel;
		}

		// YZ edge (formed between y faces and z faces)
		else if (sinsideY && sinsideZ && cinsideX && !cinsideY && !cinsideZ) {//determines if this collision is possible (does not guarantee it will happen)

			// find closest point on edge to the colliding sphere
			float contactpoint[3] = { newballpos[0], yclosest, zclosest };

			// create a 0-radius sphere at this point for the colliding sphere to collide with
			sphere contactsphere = sphere(contactpoint, 0, bounce, vector0);

			// update values in mainball for passing into detectballcollision
			mainball.xpos = newballpos[0];
			mainball.ypos = newballpos[1];
			mainball.zpos = newballpos[2];

			mainball.xvel = newballvelocity[0];
			mainball.yvel = newballvelocity[1];
			mainball.zvel = newballvelocity[2];

			// handle collision
			collision = (contactsphere.detectballcollision(mainball) || collision);

			// update local variables with results
			newballpos[0] = mainball.xpos;
			newballpos[1] = mainball.ypos;
			newballpos[2] = mainball.zpos;

			newballvelocity[0] = mainball.xvel;
			newballvelocity[1] = mainball.yvel;
			newballvelocity[2] = mainball.zvel;
		}


		//collision with vertex
		else if (sinsideX && sinsideY && sinsideZ && !cinsideX && !cinsideY && !cinsideZ) {//determines if this collision is possible (does not guarantee it will happen)

			// define positions of all vertices
			float corners[8][3] =
			{ { xscl / 2.0, yscl / 2.0, zscl / 2.0 },
			{ xscl / 2.0, yscl / 2.0, -zscl / 2.0 },
			{ xscl / 2.0, -yscl / 2.0, zscl / 2.0 },
			{ xscl / 2.0, -yscl / 2.0, -zscl / 2.0 },
			{ -xscl / 2.0, yscl / 2.0, zscl / 2.0 },
			{ -xscl / 2.0, yscl / 2.0, -zscl / 2.0 },
			{ -xscl / 2.0, -yscl / 2.0, zscl / 2.0 },
			{ -xscl / 2.0, -yscl / 2.0, -zscl / 2.0 } };

			// detect collisions with each vertex
			for (int i = 0; i < 8; i++) {

				// create a 0-radius sphere at a vertex for the colliding sphere to collide with
				sphere corner = sphere(corners[i], 0, bounce, vector0);

				// update values in mainball for passing into detectballcollision
				mainball.xpos = newballpos[0];
				mainball.ypos = newballpos[1];
				mainball.zpos = newballpos[2];

				mainball.xvel = newballvelocity[0];
				mainball.yvel = newballvelocity[1];
				mainball.zvel = newballvelocity[2];

				// handle collision
				collision = (corner.detectballcollision(mainball) || collision);

				// update local variables with results
				newballpos[0] = mainball.xpos;
				newballpos[1] = mainball.ypos;
				newballpos[2] = mainball.zpos;

				newballvelocity[0] = mainball.xvel;
				newballvelocity[1] = mainball.yvel;
				newballvelocity[2] = mainball.zvel;

			}
		}

		// determine the sphere's new position in its own reference frame, using the inverse rotation matrix
		float finalballpos[3] = { newballpos[0] * irotationmatrix[0][0] + newballpos[1] * irotationmatrix[0][1] + newballpos[2] * irotationmatrix[0][2] + xpos,
			newballpos[0] * irotationmatrix[1][0] + newballpos[1] * irotationmatrix[1][1] + newballpos[2] * irotationmatrix[1][2] + ypos,
			newballpos[0] * irotationmatrix[2][0] + newballpos[1] * irotationmatrix[2][1] + newballpos[2] * irotationmatrix[2][2] + zpos,
		};

		// determine the sphere's new velocity in its own reference frame, using the inverse rotation matrix
		float finalballvelocity[3] = { newballvelocity[0] * irotationmatrix[0][0] + newballvelocity[1] * irotationmatrix[0][1] + newballvelocity[2] * irotationmatrix[0][2],
			newballvelocity[0] * irotationmatrix[1][0] + newballvelocity[1] * irotationmatrix[1][1] + newballvelocity[2] * irotationmatrix[1][2],
			newballvelocity[0] * irotationmatrix[2][0] + newballvelocity[1] * irotationmatrix[2][1] + newballvelocity[2] * irotationmatrix[2][2],
		};

		// updates the sphere's position
		mainball.xpos = finalballpos[0];
		mainball.ypos = finalballpos[1];
		mainball.zpos = finalballpos[2];

		mainball.xvel = finalballvelocity[0];
		mainball.yvel = finalballvelocity[1];
		mainball.zvel = finalballvelocity[2];

		return collision; //true if there was a collision, false otherwise
	}
};

// glutKeyboardFunc is called below to set this function to handle
//		all normal key presses.  
static void KeyPressFunc( unsigned char Key, int x, int y )
{
	/////////////////For complete description of controls, see print statements in main function////////////////////////////
	switch (Key) {
	case 27:	// Escape key
		exit(1);
	case 'x': // returns to start
		for (int i = 0; i < 2; i++) {
			position[i] = origposition[i];
			velocity[i] = 0;
		}
		position[2] = 0;
		velocity[2] = 0;
		glutPostRedisplay();
		break;

	case 'q':
		countlphase = true;
		break;

	case 'p':
		countrphase = true;
		break;

	case 't':
		testrot[2] += .03;
		break;

	case 'g':
		testrot[2] -= .03;
		break;

	case 'f':
		testrot[1] += .03;
		break;

	case 'h':
		testrot[1] -= .03;
		break;

	case 'r':
		testrot[0] -= .03;
		break;

	case 'y':
		testrot[0] += .03;
		break;

	case 'z':
		frame = !frame;
		break;

	case 'c':
		testrot[0] = pi / 4;
		testrot[1] = 0;
		testrot[2] = 0;
	}
}

/*
 * Animate() handles the animation and the redrawing of the
 *		graphics window contents.
 */


static void Animate(void)
{
	// Clear the redering window
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Clear the current matrix (Modelview)
	glLoadIdentity();

	// place everything in position where it can be easily viewed
	if (frame) glTranslatef(0.0, 0.0, -17.0);
	else glTranslatef(0.0, -1.0, 3.0);

	//Create lights
	glLightfv(GL_LIGHT0, GL_POSITION, Lt0pos);
	glLightfv(GL_LIGHT0, GL_POSITION, Lt1pos);
	glEnable(GL_LIGHTING);

	//Create the ball
	sphere mainball = sphere(position, 1, 1, velocity);

	//Calculate collision-independent physics
	mainball.gravity(gravity);
	mainball.step();

	//determine new reference frame (for playing field)
	glPushMatrix();
	glTranslatef(0, 0, farout*-1);
	glScalef(.2, .2, .2);


	//Create and draw spherical obsticles and detect collisions with them
	for (int pn = 0; pn < numcircles; pn++) {

		sphere working = sphere(obstructorpositions[pn], obstructorradius, obstructorbounce, vector0);

		working.draw();
		working.detectballcollision(mainball); // handle collisions

	}

	//Create borders of playing field
	cube topborder = cube(topborderpos, vector0, topbordersize, borderbounce, vector0);
	cube botborder = cube(botborderpos, vector0, topbordersize, borderbounce, vector0);
	cube leftborder = cube(leftborderpos, vector0, sidebordersize, borderbounce, vector0);
	cube rightborder = cube(rightborderpos, vector0, sidebordersize, borderbounce, vector0);

	//Draw borders of playing field
	topborder.draw();
	botborder.draw();
	leftborder.draw();
	rightborder.draw();

	//Handle collisions with borders
	topborder.detectBallCollision(mainball);
	botborder.detectBallCollision(mainball);
	leftborder.detectBallCollision(mainball);
	rightborder.detectBallCollision(mainball);


	//create pieces that funnel ball to flippers, draw them, and handle their collisions
	cube funnel1 = cube(funnelpos, funnelrot, funnelsize, funnelbounce, vector0);

	funnel1.draw();
	funnel1.detectBallCollision(mainball);

	cube funnel2 = cube(funnelpos2, funnelrot2, funnelsize, funnelbounce, vector0);
	funnel2.draw();
	funnel2.detectBallCollision(mainball);

	//create, draw, and handle collisions for center rectangular prism
	cube testcube = cube(testpos, testrot, testsize, .4, vector0);
	testcube.draw();
	testcube.detectBallCollision(mainball);

	///////Flippers

	//handle flipper motion within the context of the animation
	if (lphase >= 8) {
		countlphase = false;
	}
	if (countlphase) {
		lphase += 1;
	}
	else {
		if (lphase > 0) {
			lphase -= 1;
		}
	}

	if (rphase >= 8) {
		countrphase = false;
	}
	if (countrphase) {
		rphase += 1;
	}
	else {
		if (rphase > 0) {
			rphase -= 1;
		}
	}

	//define rotation of right flipper
	float rflipperrot[3] = { (pi / 6 - speedflip*rphase)*-1, 0, 0 };

	//define position of right flipper
	float rflipperpos[3] = { point[0] - flippersize[0] * cosf(-rflipperrot[0]) / 2.0, point[1] - flippersize[0] * sinf(-rflipperrot[0]) / 2.0, point[2] };

	//create and draw right flipper
	cube rightflipper = cube(rflipperpos, rflipperrot, flippersize, 0.5, vector0);
	rightflipper.draw();

	//handle collisions with right flipper, and use basic functionality to impart velocity
	if (rightflipper.detectBallCollision(mainball) && rphase != 0) { //collision can be handled inside if statement with no issue
		if (countrphase) { // if flipper has upward velocity

			//impart velocity (basic functionality, does not handle edges)
			float mainpos[3] = { mainball.xpos, mainball.ypos, mainball.zpos };
			mainball.xvel += sinf(rflipperrot[0]) * sqrt(pow(truedistance(point, mainpos), 2) - pow(mainball.radius, 2)) * speedflip;
			mainball.yvel += cosf(rflipperrot[0]) * sqrt(pow(truedistance(point, mainpos), 2) - pow(mainball.radius, 2)) * speedflip;
			mainball.step();
			mainball.ypos += .1;
		}

	}

	// define rotation of left flipper
	float lflipperrot[3] = { (pi / 6 - speedflip*lphase), 0, 0 };

	//define position of left flipper
	float lflipperpos[3] = { point2[0] + flippersize[0] * cosf(-lflipperrot[0]) / 2.0, point2[1] + flippersize[0] * sinf(-lflipperrot[0]) / 2.0, point2[2] };

	//create and draw left flipper
	cube leftflipper = cube(lflipperpos, lflipperrot, flippersize, 0.5, vector0);
	leftflipper.draw();

	//handle collisions with left flipper, and use basic functionality to impart velocity
	if (leftflipper.detectBallCollision(mainball) && lphase != 0) { //collision can be handled inside if statement with no issue
		if (countlphase) { //if flipper has upward velocity

			//impart velocity(basic functionality, does not handle edges)
			float mainpos[3] = { mainball.xpos, mainball.ypos, mainball.zpos };
			mainball.xvel += sinf(lflipperrot[0]) * sqrt(pow(truedistance(point2, mainpos), 2) - pow(mainball.radius, 2)) * speedflip;
			mainball.yvel += cosf(lflipperrot[0]) * sqrt(pow(truedistance(point2, mainpos), 2) - pow(mainball.radius, 2)) * speedflip;
			mainball.step();
			mainball.ypos += .1;
		}

	}


	// update global variables.  these are used to define the position and velocity of the ball next time animate is called
	position[0] = mainball.xpos;
	position[1] = mainball.ypos;
	position[2] = mainball.zpos;

	velocity[0] = mainball.xvel;
	velocity[1] = mainball.yvel;
	velocity[2] = mainball.zvel;

	//finally, after all physics is handled, draw the ball
	mainball.draw();

	glPopMatrix(); //back to default reference frame

	//limit frames per second to 62.5
	int fpslimit = clock() % 16;

	if (fpslimit != 0){
		std::this_thread::sleep_for(std::chrono::milliseconds(16 - fpslimit));
	}

	glutPostRedisplay(); //tell OpenGL to draw the scene

    glFlush();
    glutSwapBuffers();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////CODE BELOW THIS POINT IS MOSTLY NOT MY ORIGINAL WORK -- MOST WAS WRITTEN BY SAM BUSS//////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//except the description of the controls for the program


// ResizeWindow is called when the window is resized

// Initialize OpenGL's rendering modes
void OpenGLInit(void)
{
	//glShadeModel( GL_FLAT );
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// Set global ambient light
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight);

	// Light 0 light values.  
	glLightfv(GL_LIGHT0, GL_AMBIENT, Lt0amb);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, Lt0diff);
	glLightfv(GL_LIGHT0, GL_SPECULAR, Lt0spec);
	glLightfv(GL_LIGHT1, GL_AMBIENT, Lt0amb);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, Lt0diff);
	glLightfv(GL_LIGHT1, GL_SPECULAR, Lt0spec);

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, matAmbDiff);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, &matSpec[0]);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 80.0);

	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);

}

static void ResizeWindow(int w, int h)
{
    float aspectRatio;
	h = (h == 0) ? 1 : h;
	w = (w == 0) ? 1 : w;
	ScreenWidth = w;
	ScreenHeight = h;
	glViewport( 0, 0, w, h );	// View port uses whole window
	aspectRatio = (float)w/(float)h;

	// Set up the projection view matrix (not very well!)
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 30.0, aspectRatio, 10.0, 60.0 );

	// Select the Modelview matrix
    glMatrixMode( GL_MODELVIEW );
}


// Main routine
// Set up OpenGL, hook up callbacks, and start the main loop
int main( int argc, char** argv )
{
	fprintf(stdout,"  Pinball Collision Detection Demo\n  CONTROLS:\n\n" );
	fprintf(stdout, "  r, t, y, f, g, h - rotate center rectangular prism\n" );
	fprintf(stdout, "  q - left flipper\n" );
	fprintf(stdout, "  p - right flipper\n" );
	fprintf(stdout, "  x - reset ball position\n");
	fprintf(stdout, "  c - reset center rectangular prism rotation\n");
	fprintf(stdout, "  z - zoom in/out on center\n\n");
	fprintf(stdout, "  For in-depth look at sphere-cube collisions in 3 dimensions\n  at different angles, zooming in is recommended\n");

	// Need to double buffer for animation
	glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );

	// Create and position the graphics window
    glutInitWindowPosition( 0, 0 );
    glutInitWindowSize( 370, 720 );
    glutCreateWindow( "CollisionDemo" );

	// Initialize OpenGL.
    OpenGLInit();

	// Set up callback functions for key presses
	glutKeyboardFunc( KeyPressFunc );
	// glutSpecialFunc( SpecialKeyFunc );


	// Set up the callback function for resizing windows
    glutReshapeFunc( ResizeWindow );

	// Callback for graphics image redrawing
    glutDisplayFunc( Animate );
	
	// Start the main loop.  glutMainLoop never returns.
	glutMainLoop(  );

    return(0);			// Compiler requires this to be here. (Never reached)
}


