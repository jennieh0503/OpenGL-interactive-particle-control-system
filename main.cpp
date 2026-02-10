#include <iostream>
#include <GLAD/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shaderClass.h"

#define NUM_POINTS_IN_CIRCLE 32

#define RESTITUTION_WALLS 0.8f  //adding damping and energy loss in collisions (not percfectly elastic)
#define RESTITUTION_OBS 0.5f
#define TERMINAL_VELOCITY 10.0f //maximum velocity to prevent tunneling through obstacles at high speeds
#define FAST_VELOCITY 5.0f //velocity threshold for substeppig, allows better collision detection at high speeds and prevents tunneling through obstacles

#define DRAG 0.6f //drag coefficient for air resistance
#define MOVE_FORCE 3.0f //force applied when interacting using input
#define MAX_ENERGY 15.0f //maximum energy for input
#define ENERGY_DRAIN 10.0f //rate at which energy drains when applying input
#define ENERGY_RECOVERY 5.0f //rate at which energy recovers when not applying input

#define OBS_MIN_STEP 0.2f //minimum distance between obstacles
#define OBS_MAX_STEP 2.0f //maximum distance between obstacles
#define OBS_MIN_WIDTH 0.5f //minimum width of obstacles
#define OBS_MAX_WIDTH 1.0f //maximum width of obstacles
#define OBS_CAMERA_THRESHOLD 1.5f //distance from camera to start generating obstacles
#define DESPAWN_MARGIN 1.0f //distance from camera to despawn obstacles

#define ENERGY_MIN 0.2f //minimum energy threshold for failure condition
#define T_MAX 2.0f //maximum time allowed below energy threshold for failure condition

typedef struct mesh{
	unsigned int VBO;
	unsigned int VAO;
	unsigned int shaderProg;
} Mesh;

typedef struct particle {
	glm::vec3 position;
	glm::vec3 velocity;
	glm::vec3 force;

	float radius;
	float mass;
};

typedef struct obstacle {
	float mass;
	float width;
	float height;

	glm::vec3 position;
};

typedef struct camera {
	float Y;
	float followDistance;
};

GLFWwindow* initialize();

//ready gpu
Mesh initializeShape();
Mesh initializeObsShape();

void draw(GLFWwindow* window, Mesh m, Mesh mObs, particle& p, std::vector<obstacle> obstacles);
void update(GLFWwindow* window, particle& p, glm::vec3 gravity, float deltaTime, std::vector<obstacle> obstacles, float* inputEnergy);
void resolveCollision(particle& p, obstacle& o);
float randomFloat(float min, float max);
obstacle generateObstacle(float y);
void applyInput(GLFWwindow* window, particle& p, float* inputEnergy, float deltaTime, float* totalDrag);





//main
int main() {

	//initialize window including glfw
	GLFWwindow* window = initialize();
	if (window == NULL) {
		std::cout << "Failed to create window" << std::endl;
		glfwTerminate();
		return -1;
	}

	//set window as context
	glfwMakeContextCurrent(window);

	//start glad
	gladLoadGL();

	//set working or display area
	//the function requires coordinates for bottom left corner and top left corner of the interested area
	//In this experiment I used the entire window
	glViewport(0, 0, 1000, 1000);

	//initializing shape with buffers and shaders, sending data to gpu to avoid repeated work conuming time
	Mesh m = initializeShape();
	Mesh mObs = initializeObsShape();


	//initializing camera
	camera c;
	c.Y = 0.0f;
	c.followDistance = 0.3f;

	particle p;
	p.position = glm::vec3(0.0f, 1.0f, 0.0f);
	p.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	p.force = glm::vec3(0.0f);
	p.mass = 0.5f;
	p.radius = 0.05f;

	//initializing variables for obstacles
	std::vector<obstacle> obstacles;
	float nextObsY = c.Y - 1.0f;
	float timeAlive = 0.0f; //increase difficulty with time by increasing obstacle density
	float maxStep = OBS_MAX_STEP;

	//defining gravity and initializing time
	const glm::vec3 gravity(0.0f, -9.81f, 0.0f);
	float time = glfwGetTime();

	//defining variables for failure condition
	float energy;  //fail if kinetic energy is too low for too long
	float lowEnergyTime = 0.0f;
	float inputEnergy = MAX_ENERGY;
	
	//let window run until closure, added logic for a "bouncy" shape
	while (!glfwWindowShouldClose(window)) {
		//update delta time (no longer frame dependent)
		float newTime = glfwGetTime();
		float deltaTime = newTime - time;
		time = newTime;
		timeAlive += deltaTime;

		energy = 0.5 * p.mass * glm::length(p.velocity) * glm::length(p.velocity);
		if (energy < ENERGY_MIN) {
			lowEnergyTime += deltaTime;
		} else {
			lowEnergyTime = 0.0f;
		}

		if (lowEnergyTime > T_MAX) {
			std::cout << "Failure condition met: low energy for too long" << std::endl;
			break;
		}

		

		//generate new obstacles as the camera moves down
		maxStep = OBS_MAX_STEP - (OBS_MAX_STEP - OBS_MIN_STEP) * (timeAlive / 60.0f);  //gradually increase difficulty by reducing max step over time, down to a minimum of OBS_MIN_STEP
		while (nextObsY > c.Y - OBS_CAMERA_THRESHOLD) {
			obstacles.push_back(generateObstacle(nextObsY));
			nextObsY -= randomFloat(OBS_MIN_STEP, maxStep);
		}
		//delete obstacles that are out of view to save resources
		obstacles.erase(std::remove_if(obstacles.begin(), obstacles.end(), [&](obstacle& o) {
			return o.position.y > c.Y + DESPAWN_MARGIN;
			}), obstacles.end());

		//apply substepping
		int steps = 1;
		if (glm::length(p.velocity) > FAST_VELOCITY) {
			steps = static_cast<int>(glm::length(p.velocity) / FAST_VELOCITY) + 1;  //more steps for higher velocities
		}
		else {
			steps = 1;  //no substepping needed for low velocities

		}

		float subDeltaTime = deltaTime / steps;

		for (int i = 0; i < steps; i++) {

			//update particle by applying gravity (also checking for boundaries)
			update(window, p, gravity, subDeltaTime, obstacles, &inputEnergy);

			//apply terminal velocity to prevent tunneling through obstacles at high speeds
			if (glm::length(p.velocity) > TERMINAL_VELOCITY) {
				p.velocity = glm::normalize(p.velocity) * TERMINAL_VELOCITY;
			}

			//move camera with the particle, only downwards to avoid jittering
			if (p.position.y - c.followDistance < c.Y) {
				c.Y = p.position.y - c.followDistance;
			}

		}


		//update view matrix in shaders to move the camera
		unsigned int viewLoc = glGetUniformLocation(m.shaderProg, "view");
		unsigned int viewLocObs = glGetUniformLocation(mObs.shaderProg, "view");
		glm::mat4 view = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -c.Y, 0.0f));
		glUseProgram(m.shaderProg);
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, &view[0][0]);
		glUseProgram(mObs.shaderProg);
		glUniformMatrix4fv(viewLocObs, 1, GL_FALSE, &view[0][0]);

		draw(window, m, mObs, p, obstacles);

		glfwPollEvents();
	}


	//clean used resources
	glDeleteVertexArrays(1, &m.VAO);
	glDeleteBuffers(1, &m.VBO);
	glDeleteProgram(m.shaderProg);
	glDeleteVertexArrays(1, &mObs.VAO);
	glDeleteBuffers(1, &mObs.VBO);
	glDeleteProgram(mObs.shaderProg);
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}





//initializing glfw, window
GLFWwindow* initialize() {
	//initialize glfw
	glfwInit();

	//initialize window, specify version (3.3 or higher in this case)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	//create window and check errors
	//glfw creation functions requires width, length and name as parameters
	//there are two extra parameters that are monitor and window, that I am going to ignore for the scope of this project
	GLFWwindow* window = glfwCreateWindow(1000, 1000, "minimalRender", NULL, NULL);

	return window;
}


//shape initialization function, creates the shape by sending data to the gpu and creating shader
//updated to create circles (particles simplification for collision simulation)
//updated by using glm for vector operations
Mesh initializeShape() {
	Mesh m;

	float angle = 360.0f / NUM_POINTS_IN_CIRCLE;

	//create vertices vector
	//z axis kept to a constant to represent a flat 2d object
	std::vector<glm::vec3> vertices;
	vertices.push_back(glm::vec3(0.0f, 0.0f, 0.0f)); //center point
	for (int i = 0; i <= NUM_POINTS_IN_CIRCLE;  i++) {
		float currentAngle = angle * i;
		float x = 1.0f * cos(glm::radians(currentAngle));
		float y = 1.0f * sin(glm::radians(currentAngle));
		vertices.push_back(glm::vec3(x, y, 0.0f));  //2d, constant z
	}

	//creating shaders using the shader class
	Shader sProg("vert.txt", "frag.txt");
	m.shaderProg = sProg.ID;

	//vertices need to be passed to the gpu, buffers are used to send large amounts without having to consume more time
	//creating a vertix buffer (not using an array because of only one object present to send)
	//a vertex array object is also needed to switch between objects faster
	glGenVertexArrays(1, &m.VAO);
	glGenBuffers(1, &m.VBO);
	glBindVertexArray(m.VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m.VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

	//configuration of VAO to interpret the data sent to the buffer.
	//the vertices were saved as sets of coordinates, all in one single array, therefore each 3 belond to a vertex
	//(starting from element 0 of the array vertices)
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	return m;
}



//shape initialization function for obstacles, rectangular shape
Mesh initializeObsShape() {
	Mesh m;

	std::vector<glm::vec3> vertices = {
		glm::vec3(-0.5f, -0.5f, 0.0f),
		glm::vec3(0.5f, -0.5f, 0.0f),
		glm::vec3(-0.5f, 0.5f, 0.0f),
		glm::vec3(-0.5f, 0.5f, 0.0f),
		glm::vec3(0.5f, -0.5f, 0.0f),
		glm::vec3(0.5f, 0.5f, 0.0f)
	};


	Shader sProg("vertObs.txt", "fragObs.txt");
	m.shaderProg = sProg.ID;

	glGenVertexArrays(1, &m.VAO);
	glGenBuffers(1, &m.VBO);
	glBindVertexArray(m.VAO);
	glBindBuffer(GL_ARRAY_BUFFER, m.VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), vertices.data(), GL_STATIC_DRAW);

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	return m;
}


//drawing function
void draw(GLFWwindow* window, Mesh m, Mesh mObs, particle& p, std::vector<obstacle> obstacles) {

	glClearColor(0.0891, 0.0873, 0.0900, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	unsigned int transformationLoc = glGetUniformLocation(m.shaderProg, "transformation"); //get location of transformation matrix in shader for particle

	//update transformation matrix
	//because the standard shape has a radius of 1 scaling is done to get the particle's actual size
	//translation allows to showcase motion (without having to modify the actual vertex data)
	glm::mat4 transform = glm::mat4(1.0f);
	transform = glm::translate(transform, p.position);
	transform = glm::scale(transform, glm::vec3(p.radius, p.radius, 1.0f));

	//pass transformation matrix to vertex shader
	glUseProgram(m.shaderProg);
	glUniformMatrix4fv(transformationLoc, 1, GL_FALSE, &transform[0][0]);

	//draw the particle
	glUseProgram(m.shaderProg);
	glBindVertexArray(m.VAO);
	glDrawArrays(GL_TRIANGLE_FAN, 0, NUM_POINTS_IN_CIRCLE+2);


	//draw obstacles
	unsigned int obsTransformationLoc = glGetUniformLocation(mObs.shaderProg, "transformation");  //get location of transformation matrix in shader for obstacles
	for (obstacle& o : obstacles) {
		glm::mat4 obsTransform = glm::mat4(1.0f);
		obsTransform = glm::translate(obsTransform, o.position);
		obsTransform = glm::scale(obsTransform, glm::vec3(o.width, o.height, 1.0f));
		glUseProgram(mObs.shaderProg);
		glUniformMatrix4fv(obsTransformationLoc, 1, GL_FALSE, &obsTransform[0][0]);
		glUseProgram(mObs.shaderProg);
		glBindVertexArray(mObs.VAO);
		glDrawArrays(GL_TRIANGLES, 0, 6);
	}

	
	glfwSwapBuffers(window);
}





//update particle by applying gravity and side boundaries, vertical ones aren't needed anymore with the addition of the camera
void update(GLFWwindow* window, particle& p, glm::vec3 gravity, float deltaTime, std::vector<obstacle> obstacles, float* inputEnergy) {
	float totalDrag;
	p.force = gravity * p.mass;
	applyInput(window, p, inputEnergy, deltaTime, &totalDrag);
	p.force += -DRAG * p.velocity;  //applying drag for air resistance
	p.velocity += (p.force/p.mass) * deltaTime;
	p.position += p.velocity * deltaTime;

	//boundaries
	if (p.position.x < -1.0f + p.radius) {
		p.position.x = -1.0f + p.radius;
		p.velocity.x *= -RESTITUTION_WALLS;
	}
	else if (p.position.x > 1.0f - p.radius) {
		p.position.x = 1.0f - p.radius;
		p.velocity.x *= -RESTITUTION_WALLS;
	}

	//collisions
	//circle-AABB collision detection
	for (int i = 0; i < 2; i++) {  //multiple iterations for better stability and to prevent tunneling through obstacles at high speeds
		for (obstacle o : obstacles) {
			resolveCollision(p, o);
		}
	}
}


//collision resolution function, using circle-AABB collision detection and response, with particular case when the particle is inside the obstacle
void resolveCollision(particle& p, obstacle& o) {
	glm::vec3 closestPoint;
	closestPoint.x = glm::clamp(p.position.x, o.position.x - o.width / 2, o.position.x + o.width / 2);
	closestPoint.y = glm::clamp(p.position.y, o.position.y - o.height / 2, o.position.y + o.height / 2);
	closestPoint.z = 0.0f;

	float distance = glm::length(p.position - closestPoint);

	if (distance >= p.radius) {
		return;  //no collision
	}

	//define variables
	glm::vec3 n;
	float percentage = 0.8f;  //percentage to move each particle, gradual corrections allow better stability and prevent jittering
	float slop = 0.01f;  //small value to prevent sinking due to numerical errors
	float penetration;

	//special case for when the particle is inside the obstacle
	if (distance < FLT_EPSILON) {
		float dxLeft = p.position.x - (o.position.x - o.width / 2);
		float dxRight = (o.position.x + o.width / 2) - p.position.x;
		float dyBottom = p.position.y - (o.position.y - o.height / 2);
		float dyTop = (o.position.y + o.height / 2) - p.position.y;

		if (std::min(dxLeft, dxRight) < std::min(dyBottom, dyTop)) {
			if (dxLeft < dxRight) {
				n = glm::vec3(-1.0f, 0.0f, 0.0f);
				penetration = p.radius + dxLeft;
			}
			else {
				n = glm::vec3(1.0f, 0.0f, 0.0f);
				penetration = p.radius + dxRight;
			}
		}
		else {
			if (dyBottom < dyTop) {
				n = glm::vec3(0.0f, -1.0f, 0.0f);
				penetration = p.radius + dyBottom;
			}
			else {
				n = glm::vec3(0.0f, 1.0f, 0.0f);
				penetration = p.radius + dyTop;
			}
		}
	}
	else {
		//if not inside
		n = glm::normalize(p.position - closestPoint);
		penetration = p.radius - distance;
	}


	//correct position
	if (penetration > slop) {
		glm::vec3 correction = (penetration - slop) * percentage * n;
		p.position += correction;
	}

	//calculate relative velocity
	float relativeVelocity = glm::dot(p.velocity, n);

	if (relativeVelocity < 0.0f) {  //only resolve if particle is moving towards the obstacle
		p.velocity -= (1 + RESTITUTION_OBS) * relativeVelocity * n;
	}

}



float randomFloat(float min, float max) {
	return min + static_cast <float> (rand()) / (static_cast <float> (RAND_MAX / (max - min)));
}


//obstacle random generatio function
obstacle generateObstacle(float y) {
	obstacle o;

	o.mass = INFINITY;  //infinite mass, static obstacle
	o.width = randomFloat(OBS_MIN_WIDTH, OBS_MAX_WIDTH);
	o.height = 0.05f;

	o.position.y = y;
	o.position.x = randomFloat(-1.0f + o.width / 2, 1.0f - o.width / 2);  //ensuring the obstacle stays within horizontal boundaries
	o.position.z = 0.0f;

	return o;
}


//input handling
void applyInput(GLFWwindow* window, particle& p, float* inputEnergy, float deltaTime, float* totalDrag) {
	float scale = *inputEnergy / MAX_ENERGY;
	if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
		*inputEnergy -= ENERGY_DRAIN * deltaTime;
		p.force.x -= MOVE_FORCE * scale * scale;
	}
	else {
		*inputEnergy += ENERGY_RECOVERY * deltaTime;  //recover energy when not applying input
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
		*inputEnergy -= ENERGY_DRAIN * deltaTime;
		p.force.x += MOVE_FORCE * scale * scale;
	}
	else {
		*inputEnergy += ENERGY_RECOVERY * deltaTime;  //recover energy when not applying input
	}

	*inputEnergy = glm::clamp(*inputEnergy, 0.0f, MAX_ENERGY);  //clamping energy to valid range
}