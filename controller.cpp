// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using 
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>
#include <fstream>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";

#define JOINT_CONTROLLER      0
#define POSORI_CONTROLLER     1

enum STATE {
    AIMING,
    READY_POSITION,
    SWING,
    FOLLOW_THRU,
    SLOW_DOWN,
    HOLD
};

STATE state = AIMING;

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

std::string GOAL_POSITION_KEY = "opencv2:shoot_decision";
std::string GOAL_LEFT = "LEFT";
std::string GOAL_CENTER = "CENTER";
std::string GOAL_RIGHT = "RIGHT";

std::string goal_position = "NO_GOAL";

unsigned long long controller_counter = 0;

const bool flag_simulation = false;
// const bool flag_simulation = true;

const bool inertia_regularization = true;

VectorXd saturate_torques(VectorXd torques) {
    VectorXd max_torques(7);
    max_torques << 85.0, 85.0, 85.0, 85.0, 10.0, 10.0, 10.0;
    for (int i=0;i<7;i++) {
        if (abs(torques[i]) > max_torques[i]) {
            if (torques[i] > 0.0) torques[i] = max_torques[i];
            else torques[i] = -max_torques[i];
        }
    }
    return torques;
}

int main() {
	// Choose where to get sensor values
	if(flag_simulation)
	{
		JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
		JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
	}
	else
	{
		JOINT_TORQUES_COMMANDED_KEY = "sai2::FrankaPanda::actuators::fgc";
		JOINT_ANGLES_KEY  = "sai2::FrankaPanda::sensors::q";
		JOINT_VELOCITIES_KEY = "sai2::FrankaPanda::sensors::dq";
		JOINT_TORQUES_SENSED_KEY = "sai2::FrankaPanda::sensors::torques";
		MASSMATRIX_KEY = "sai2::FrankaPanda::sensors::model::massmatrix";
		CORIOLIS_KEY = "sai2::FrankaPanda::sensors::model::coriolis";
		ROBOT_GRAVITY_KEY = "sai2::FrankaPanda::sensors::model::robot_gravity";		
	}

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.07);
    Vector3d current_velocity;
    Vector3d current_pos;
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

#ifdef USING_OTG
	posori_task->_use_interpolation_flag = false;
#else
	posori_task->_use_velocity_saturation_flag = false;
#endif
	
	VectorXd posori_task_torques = VectorXd::Zero(dof);
    posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = false;
	joint_task->_use_velocity_saturation_flag = false;
#else
	joint_task->_use_velocity_saturation_flag = false;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0; //50;
	joint_task->_kv = 15.0;


	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	double taskStart_time = 0.0;
	double tTask = 0.0;

    // For printing trajectory to file 
    ofstream trajectory;
    trajectory.open("trajectory.txt");
    
    // For printing desired trajectory to file 
    ofstream des_trajectory;
    des_trajectory.open("des_trajectory.txt");
    
    // For printing the joint configuration to a file
    ofstream joints;
    joints.open("joints.txt");
    
    // For printing the velocity to a file
    ofstream velocity;
    velocity.open("velocity.txt");
    
    // For printing the command torques5
    ofstream torques;
    torques.open("torques.txt");
    
    // For printing the joint velocities
    ofstream joint_velocity;
    joint_velocity.open("joint_velocity.txt");

    // DON'T NEED (Old cartesian controller)
    // Intermediate point of the trajectory
    Vector3d xDesInter = Vector3d(0.5328,0.09829, 0.20);
    // End point of the trajectory
    Vector3d xDesF = Vector3d(0.5328,-0.1, 0.25);

    //Initialize vectors
	VectorXd q_ready_pos = initial_q;
	VectorXd q_inter_pos1 = initial_q;
	VectorXd q_inter_pos2 = initial_q;
	VectorXd q_final_pos1 = initial_q;
	VectorXd q_final_pos2 = initial_q;
	VectorXd q_interpolated = initial_q;


 
	double tContact;
	double tFollowThru;
	double tSlowDown;
	double tHold;
    	char direction = 'x'; // Defined to x as an invalid input



	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop(); 
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
       	 	robot->linearVelocity(current_velocity, control_link, control_point);
       		robot->position(current_pos, control_link, control_point);

        	goal_position = redis_client.get(GOAL_POSITION_KEY);

		// update model
		if(flag_simulation)
		{
			robot->updateModel();
		}
		else
		{
			robot->updateKinematics();
			robot->_M = redis_client.getEigenMatrixJSON(MASSMATRIX_KEY);
			if(inertia_regularization)
			{
				robot->_M(4,4) += 0.07;
				robot->_M(5,5) += 0.07;
				robot->_M(6,6) += 0.07;
			}
			robot->_M_inv = robot->_M.inverse();
		}
        
        if (state == AIMING)
        {
   //      	while (true){
   //      		joint_task->_desired_velocity.setZero();
   //      		state = AIMING;
			// // Update task model and set hierarchy
   //      		N_prec.setIdentity();
   //      		joint_task->updateTaskModel(N_prec);
   //      		joint_task->_use_velocity_saturation_flag = true;
   //      		joint_task->computeTorques(joint_task_torques);
   //      		command_torques = saturate_torques(joint_task_torques);
   //          // Ask for the direction the ball should be kicked
   //      		cout << "Please enter the direction David should kick the ball (l, r, m): ";
   //      		cin >> direction;
   //      		if (direction == 'r' or direction == 'l' or direction == 'm') break;
   //      	}
        		if (goal_position == GOAL_LEFT) {
					std::cout << "LEFT GOAL POSITION" << endl;
					// state = LEFT_SWING;
					direction = 'l';
				} else if (goal_position == GOAL_CENTER) {
					std::cout << "CENTER GOAL POSITION" << endl;
					// state = CENTER_SWING;
					direction = 'c';
				} else if (goal_position == GOAL_RIGHT) {
					std::cout << "RIGHT GOAL POSITION" << endl;
					// state = RIGHT_SWING;
					direction = 'r';
				} else {
					std::cout << "IMPOSSIBLE GOAL POSITION: " << goal_position << endl;
				}

            // Change the desired configuration based on user input
            if (direction == 'r')
            {
                q_ready_pos << -0.29748,1.38033,-1.67338,-1.76919,-0.0499905,1.95202,-1.23665;
                q_inter_pos1 << 0.261701,1.35196,-1.76101,-1.6411,1.84065,1.36388,-0.27407;
                q_inter_pos2 << 0.710947,1.38855,-1.86157,-1.19313,1.8442,1.41126,0.204385;
                q_final_pos1 << 1.12918,1.36053,-2.02022,-1.19424,1.84424,1.40299,-0.277308;
                q_final_pos2 <<0.383048,1.52992,-1.74511,-2.32493,0.170002,1.95088,-1.23669;
                tContact = 0.84;
                tFollowThru = 0.35;
                tSlowDown = 0.8;
                tHold = 5;
            }
            else if (direction  == 'l')
            {
                q_ready_pos << -0.29748,1.38033,-1.67338,-1.76919,-0.0499905,1.95202,-1.23665;
                q_inter_pos1 << 0.2297,1.31169,-1.66643,-0.985295,1.38547,1.55638,-1.011448;
                q_inter_pos2 << 0.768317,1.15231,-1.77563,-0.958731,1.38585,1.56129,-0.787051;
                q_final_pos1 <<1.10297,1.11148,-1.96972,-0.98946,1.38545,1.38392,-0.786491;
                q_final_pos2 <<0.383048,1.52992,-1.74511,-2.32493,0.170002,1.95088,-1.23669;
                tContact = 1;
                tFollowThru = 0.38;
                tSlowDown = 0.5;
                tHold = 5;
            } else {
                q_ready_pos << -0.29748,1.38033,-1.67338,-1.76919,-0.0499905,1.95202,-1.23665;
                q_inter_pos1 << 0.461504,1.41206,-1.89663,-1.19394,1.50597,1.43634,-0.378772;
                q_inter_pos2 << 0.860086,1.42542,-1.92116,-1.16575,1.71669,1.51686,-0.59824;
                q_final_pos1 << 1.30728,1.43812,-2.01119,-1.1699,1.71639,1.49235,-0.598261;
                q_final_pos2 <<0.383048,1.52992,-1.74511,-2.32493,0.170002,1.95088,-1.23669;
                tContact = 0.7;
                tFollowThru = 0.33;
                tSlowDown = 0.4;
                tHold = 5;
            }
            state = READY_POSITION;
        }
        else if(state == READY_POSITION)
		{

            /*
             * Moves the robot to the desired initial configuration that marks the 
             * start of its swing.
             */
            
			// joint_task->_kp = 250.0; //50;
			// joint_task->_kv = 20.0;
			joint_task->_desired_position = q_ready_pos;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            		joint_task->_use_velocity_saturation_flag = true;

			// Compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

			 // Print the torques
			 trajectory << current_pos.transpose() << ' ' << time << endl;
			 joints << robot->_q.transpose() << ' ' << time << endl;
			 joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
			 velocity << current_velocity.transpose() << ' ' << time << endl;
			 torques << command_torques.transpose() << ' ' << time << endl;


            // Once the robot has reached close to its desired initial configuration,
            // change states to start the swing controller and start the task timer
			if( (robot->_q - q_ready_pos).norm() < 0.1 )
			{
				cout << "SWING STATE\n" <<endl;
				joint_task->_kp = 250.0;
				joint_task->_kv = 15.0;
                		joint_task->_use_velocity_saturation_flag = false;
				state = SWING;
				taskStart_time = timer.elapsedTime();
			}
		}

		else if(state == SWING)
		{
            /*
             * Controller for the main swing of the robot to the ball.
             */
            
			// Initialize the task timer
           		 tTask = timer.elapsedTime() - taskStart_time;

            		// Set up the linear interpolation beteween the intermediate position and the initial position
            		q_interpolated = (q_ready_pos + (q_inter_pos1-q_ready_pos)*tTask/tContact);
			joint_task->_desired_position = q_interpolated;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            		joint_task->_use_velocity_saturation_flag = false;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

		        // Print the torques
		        trajectory << current_pos.transpose() << ' ' << time << endl;
		        des_trajectory << xDesF.transpose() << ' ' << time << endl;
		        joints << robot->_q.transpose() << ' ' << time << endl;
		        joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
		        velocity << current_velocity.transpose() << ' ' << time << endl;
		        torques << command_torques.transpose() << ' ' << time << endl;


            // Once the robot has reached close enough to the desired intermediate point, 
            // change to the follow through controller 
			if ( (robot->_q - q_inter_pos1).norm() < 0.3 ) {
				cout << "FOLLOW THROUGH STATE\n" <<endl;
				state = FOLLOW_THRU;
				taskStart_time = timer.elapsedTime();
			}

		}
	else if(state == FOLLOW_THRU)
		{
            /*
             * Controller for the follow through of the robot after initial contact is made
             */

			// Initialize the task timer
            		tTask = timer.elapsedTime() - taskStart_time;
            
            
		    	q_interpolated = (q_inter_pos1 + (q_inter_pos2-q_inter_pos1)*tTask/tFollowThru);
			joint_task->_desired_position = q_interpolated;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            		joint_task->_use_velocity_saturation_flag = false;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

			    // Print the torques
			    trajectory << current_pos.transpose() << ' ' << time << endl;
			    des_trajectory << xDesF.transpose() << ' ' << time << endl;
			    joints << robot->_q.transpose() << ' ' << time << endl;
			    joint_velocity << robot->_dq.transpose() << ' ' << time << endl;
			    velocity << current_velocity.transpose() << ' ' << time << endl;
			    torques << command_torques.transpose() << ' ' << time << endl;

            // Once the arm is close enough to the final position, change the controller
            // to hold the arm at that position
			if ( (robot->_q - q_inter_pos2).norm() < 0.3 ) {
				cout << "SLOW_DOWN\n" <<endl;
				joint_task->_kp = 150.0;
				joint_task->_kv = 15.0;
				state = SLOW_DOWN;
				// state = HOLD;
				taskStart_time = timer.elapsedTime();
            } 
		}
        else if(state == SLOW_DOWN)
		{
            /*
             * Controller for the follow through of the robot after initial contact is made
             */

			// Initialize the task timer
            		tTask = timer.elapsedTime() - taskStart_time;
            
            
		    	q_interpolated = (q_inter_pos2 + (q_final_pos1-q_inter_pos2)*tTask/tSlowDown);
			joint_task->_desired_position = q_interpolated;

			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            		joint_task->_use_velocity_saturation_flag = false;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);


            // Once the arm is close enough to the final position, change the controller
            // to hold the arm at that position
			if ( (robot->_q - q_final_pos1).norm() < 0.3 ) {
				cout << "HOLD STATE\n" <<endl;
				joint_task->_kp = 150.0;
				joint_task->_kv = 15.0;
                state = HOLD;
            } 
		}
        else if (state == HOLD) 
        {
            /*
             * Hold the arm at the final position once the full swing motion has been completed.
             */

            		tTask = timer.elapsedTime() - taskStart_time;
            
        		q_interpolated = (q_final_pos1 + (q_final_pos2-q_final_pos1)*tTask/tHold);
			joint_task->_desired_position = q_interpolated;

			if ( (robot->_q - q_final_pos2).norm() < 0.2 ) {
				joint_task->_desired_velocity.setZero();
				joint_task->_desired_position = q_final_pos2;
			}
			// Update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);
            		joint_task->_use_velocity_saturation_flag = true;
			joint_task->computeTorques(joint_task_torques);
			command_torques = saturate_torques(joint_task_torques);

		}

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		controller_counter++;

	}

    trajectory.close();
    torques.close();
    des_trajectory.close();
    joints.close();
    joint_velocity.close();
    velocity.close();

	double end_time = timer.elapsedTime();
   // std::cout << "\n";
   // std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
   // std::cout << "Controller Loop updates   : " << timer.elapsedCycles() <<30 "\n";
   // std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
	return 0;
}
