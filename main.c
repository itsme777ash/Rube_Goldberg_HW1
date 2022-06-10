

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"


char filename[] = "C:/Year_3_Intern_UIC/MuJoCo_Software/mujoco200_win64/mujoco200_win64/myproject/HW1/Rube_GoldBerg.xml";

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");


    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(filename, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);


    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

     double arr_view[] = {90, -45, 6, 0.000000, -0.500000, 0.000000};
     cam.azimuth = arr_view[0];
     cam.elevation = arr_view[1];
     cam.distance = arr_view[2];
     cam.lookat[0] = arr_view[3];
     cam.lookat[1] = arr_view[4];
     cam.lookat[2] = arr_view[5];

     //m->opt.gravity[2] = -0.01; //Changing gravity value given in mjmodel.h by accessing the mjOption using m object(Check API reference)

  //  qpos is dim nqx1 = 7x1 = 3 translations + 4 quarternions
    int omeg = 3; //angular velocity of ball
    int r = 2; // radius of circle made by ball
    d->qpos[2] = 0.1; // Changing initial z coord of the ball which is available in mjData given by qpos(Check API reference)
    d->qpos[1] = 10;
    d->qpos[0] = 2;
    d->qvel[0] = 0;  //Specifying the initial x velocity of ball
    d->qvel[1] = -10;  //Speciying the initial y velocity of ball
    d->qvel[2] = 0;  //Speciying the initial z velocity of ball

    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        double px,py,pz;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);

            //force should be applied in while loop to change the position of the ball for every scene based on velocity and accn
            //drag force = -c*v^2*unit_vector[v]; v = sqrt(vx^2 +vy^2 + vz^2)
            //vector (v) = vx i + vy j + vz k;
            //unit_vector(v) = vector(v)/void;
            //fx = -c*v*vx;
            //fy = -c*v*vy;
            //fz = -c*v*vz;
            //  px = d->qpos[0]; py = d->qpos[1]; pz = d->qpos[2];
            // // double v;
            // // v = sqrt(vx*vx + vy*vy + vz*vz);
            //  double fx,fy,fz;
            // // double c = 1;
            //  fx = -omeg*omeg*px;
            //  fy = -omeg*omeg*py;
            // // fz = -c*v*vz;
            //  d->qfrc_applied[0] = fx;  //force applied in x direction
            //  d->qfrc_applied[1] = fy; //force applied in y direction
            // d->qfrc_applied[2] = fz; //force applied in z direction
        }
        printf("{%f, %f, %f}\n",px,py,pz );
       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        printf("{%d, %d, %d};\n",px,py,pz );

          // update scene and render
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        // cam.lookat[0] = d->qpos[0]; //changing camera x position based on ball x position
        // cam.lookat[1] = d->qpos[1];
        mjr_render(viewport, &scn, &con);
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}
