#include "ros/ros.h"
#include "boost/thread.hpp"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose.h"
#include <std_msgs/Float64.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/io.h>
#include <sys/time.h>
#include <netdb.h>


#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chaindynparam.hpp>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>


using namespace std;
using namespace KDL;


int row = 3001;
int n = row;
int col = 7;
float vett[3001][7];
float q1[3001];
float q2[3001];
float q3[3001];
float q4[3001];
float q5[3001];
float q6[3001];
float q7[3001]; 

typedef struct ROBOT_STATE_STR {
  double jstate[14];
}ROBOT_STATE_STR;

typedef struct ROBOT_STATE {
  double jstate[7];
}ROBOT_STATE;

typedef struct ROBOT_CMD {
  double jcmd[7];
}ROBOT_CMD;

typedef struct ROBOT_CMD_STR {
  double jcmd[14];
}ROBOT_CMD_STR;

//Creazione socket in LETTURA
inline bool listener_socket(int port_number, int *sock) {
      sockaddr_in si_me;

  if ( (*sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    std::cout << "Listener::Open: error during socket creation!" << std::endl;
    return false;
  }

  memset((char *) &si_me, 0, sizeof(si_me));

  /* allow connections to any address port */
  si_me.sin_family = AF_INET;
  si_me.sin_port = htons(port_number);
  si_me.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_ok = bind(*sock, (struct sockaddr*)&si_me, sizeof(si_me));

  if ( bind_ok == -1 )
    return false;
  else
    return true;

}

//Creazione socket in SCRITTURA
inline int create_socket(char* dest, int port, int *sock) {
  struct sockaddr_in temp;
  struct hostent *h;
  int error;

  temp.sin_family=AF_INET;
  temp.sin_port=htons(port);
  h=gethostbyname(dest);

  if (h==0) {
    printf("Gethostbyname fallito\n");
    exit(1);
  }

  bcopy(h->h_addr,&temp.sin_addr,h->h_length);
  *sock=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  error=connect(*sock, (struct sockaddr*) &temp, sizeof(temp));
  return error;
}




class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
		bool init_robot_model();
		void ctrl_loop();
                void function_sub1(std_msgs::Float64 x);
                void function_sub2(std_msgs::Float64 x);


	private:
      	        int _jstate_socket;
                int _jcommand_socket;
                float flag_allarm1 = 0;
                float flag_allarm2 = 0;
	
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
	
		KDL::ChainFkSolverPos_recursive *_fksolver; //Forward position solver	
		KDL::ChainIkSolverVel_pinv *_ik_solver_vel;   	//Inverse velocity solver
		KDL::ChainIkSolverPos_NR *_ik_solver_pos;

		KDL::Chain _k_chain;
	
             
		KDL::JntArray *_q_in;
		KDL::	Frame _p_out;
               
               ros::Subscriber _my_sub1;
               ros::Subscriber _my_sub2;

               ros::Publisher _my_pub1;
               ros::Publisher _my_pub2;
               ros::Publisher _my_pub3;
               ros::Publisher _my_pub4;
               ros::Publisher _my_pub5;
               ros::Publisher _my_pub6;
               ros::Publisher _my_pub7;
};


bool KUKA_INVKIN::init_robot_model() {
	std::string robot_desc_string;
	_nh.param("robot_description", robot_desc_string, std::string());
	if (!kdl_parser::treeFromString(robot_desc_string, iiwa_tree)){
		ROS_ERROR("Failed to construct kdl tree");
		return false;
	}

	std::string base_link = "lbr_iiwa_link_0";
	std::string tip_link  = "lbr_iiwa_link_7";
	if ( !iiwa_tree.getChain(base_link, tip_link, _k_chain) ) return false;
	_fksolver = new KDL::ChainFkSolverPos_recursive( _k_chain );

	_ik_solver_vel = new KDL::ChainIkSolverVel_pinv( _k_chain );
	_ik_solver_pos = new KDL::ChainIkSolverPos_NR( _k_chain, *_fksolver, *_ik_solver_vel, 100, 1e-6 );

	_q_in = new KDL::JntArray( _k_chain.getNrOfJoints() );
	return true;
}



void KUKA_INVKIN::function_sub1( std_msgs::Float64 x ) {

	flag_allarm1 = x.data;
        //cout<<" 1: "<<x<<endl;
        
}

void KUKA_INVKIN::function_sub2( std_msgs::Float64 x ) {

	flag_allarm2 = x.data;
        //cout<<" 1: "<<x<<endl;
        
}

KUKA_INVKIN::KUKA_INVKIN() {

	if (!init_robot_model()) exit(1); 
	ROS_INFO("Robot tree correctly loaded from parameter server!");

	cout << "Joints and Link: " << iiwa_tree.getNrOfJoints() << " - " << iiwa_tree.getNrOfSegments() << endl;
 
	listener_socket(9030, &_jstate_socket);
        create_socket("192.170.10.146",9031,&_jcommand_socket);

        _my_pub1 = _nh.advertise< std_msgs::Float64 > ("/topic_pub1",0);
        _my_pub2 = _nh.advertise< std_msgs::Float64 > ("/topic_pub2",0);
        _my_pub3 = _nh.advertise< std_msgs::Float64 > ("/topic_pub3",0);
        _my_pub4 = _nh.advertise< std_msgs::Float64 > ("/topic_pub4",0);
        _my_pub5 = _nh.advertise< std_msgs::Float64 > ("/topic_pub5",0);
        _my_pub6 = _nh.advertise< std_msgs::Float64 > ("/topic_pub6",0);
        _my_pub7 = _nh.advertise< std_msgs::Float64 > ("/topic_pub7",0);
        


        _my_sub1 = _nh.subscribe("/topic_sub1", 0, &KUKA_INVKIN::function_sub1, this);
        _my_sub2 = _nh.subscribe("/topic_sub2", 0, &KUKA_INVKIN::function_sub2, this);

}






void KUKA_INVKIN::ctrl_loop() {

	std_msgs::Float64 d;

        std_msgs::Float64 cmd;
        int slen2, rlen2;	
        ROBOT_STATE_STR rs2;
	sockaddr_in si_other2;

       	KDL::Frame F_dest;
	KDL::JntArray q_out(_k_chain.getNrOfJoints());

        ofstream myfile_trq;
        ofstream myfile_q;
        myfile_trq.open("/home/utente/ros_ws/src/iiwa_kdl/src/trq_read.m");
        myfile_q.open("/home/utente/ros_ws/src/iiwa_kdl/src/q_read.m");

        myfile_trq<<"trq = [";
        myfile_q<<"q = [";





        int click;
        float count = 0;
        float z0 = 0;
        float x0 = 0;
        float y0 = 0;


        int init = 0;

        double q0[7];
        float trq_read[7];

         int start = 0;

        
        
	
	ros::Rate r(200);
        
        
        rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
        if(rlen2>0) {
                
               
                for(int i=0; i<7; i++ ) {
               _q_in->data[i] = rs2.jstate[i];
               
               }
               for(int i=0; i<7; i++ ) {
              trq_read[i] = rs2.jstate[i+7];
               
               }
                }
               
       
        _fksolver->JntToCart(*_q_in, _p_out);

	F_dest.p.data[0] = _p_out.p.x(); 
	F_dest.p.data[1] = _p_out.p.y();
	F_dest.p.data[2] = _p_out.p.z();
        z0 = _p_out.p.z();
        x0 = _p_out.p.x();
        y0 = _p_out.p.y();

	for(int i=0; i<9; i++ )
		F_dest.M.data[i] = _p_out.M.data[i];


                ROBOT_CMD_STR rc2;
                for (int i=0; i<7; i++)
                rc2.jcmd[i] = _q_in->data[i];
                for (int i=7; i<14; i++)
                rc2.jcmd[i] = 0;
		write( _jcommand_socket, &rc2, sizeof(rc2) ); //write commands over socket
         
         cout<<"PRESS TO START"<<endl;
	 cin>>click;

        int i = 0;
        int init_snd = 0;
	while( ros::ok() ) {	
                
               
        

                
                
               
               rlen2 = recvfrom( _jstate_socket, &rs2, sizeof(rs2),0,(struct sockaddr*)&si_other2, (socklen_t*)&slen2);
               if(rlen2>0) { 
                for(int i=0; i<7; i++ ) {
               _q_in->data[i] = rs2.jstate[i];
               
               } 
                for(int i=0; i<7; i++ ) {
              trq_read[i] = rs2.jstate[i+7];
               
               }
               
              }

              myfile_trq<<trq_read[0]<<" "<<trq_read[1]<<" "<<trq_read[2]<<" "<<trq_read[3]<<" "<<trq_read[4]<<" "<<trq_read[5]<<" "<<trq_read[6]<<";";
              myfile_trq<<"\n";
             
              myfile_q<<_q_in->data[0]<<" "<<_q_in->data[1]<<" "<<_q_in->data[2]<<" "<<_q_in->data[3]<<" "<<_q_in->data[4]<<" "<<_q_in->data[5]<<" "<<_q_in->data[6]<<";";
              myfile_q<<"\n";
               
             if(i<3001){
              cmd.data = trq_read[0]+q1[i]*0;
              _my_pub1.publish (cmd);
              cmd.data = trq_read[1]+q2[i]*0;
              _my_pub2.publish (cmd);
              cmd.data = trq_read[2]+q3[i]*0;
              _my_pub3.publish (cmd);
              cmd.data = trq_read[3]+q4[i]*0;
              _my_pub4.publish (cmd);
              cmd.data = trq_read[4]+q5[i]*0;
              _my_pub5.publish (cmd);
              cmd.data = trq_read[5]+q6[i]*0;
              _my_pub6.publish (cmd);
              cmd.data = trq_read[6]+q7[i]*0;
              _my_pub7.publish (cmd);

              i = i + 1;
              
               
              if(i == 3001) {
              i = 3000;
              } 

             cout<<"I "<<i<<endl;
              

               
             cout<<"flag1: "<<flag_allarm1<<"flag2:"<<flag_allarm2<<endl;
                if (flag_allarm1 == 1){
                cout<<"contact"<<endl;
                //cin>>click;
                //count = 2;

              //cin>>click;
              }

              if (flag_allarm2 == 1){
                cout<<"contact"<<endl;


              //cin>>click;
              }

              if (flag_allarm1 == 1 && flag_allarm2 == 1 ){
                cout<<"STOP"<<endl;
                myfile_trq<<trq_read[0]<<" "<<trq_read[1]<<" "<<trq_read[2]<<" "<<trq_read[3]<<" "<<trq_read[4]<<" "<<trq_read[5]<<" "<<trq_read[6]<<"];";
                myfile_q<<_q_in->data[0]<<" "<<_q_in->data[1]<<" "<<_q_in->data[2]<<" "<<_q_in->data[3]<<" "<<_q_in->data[4]<<" "<<_q_in->data[5]<<" "<<_q_in->data[6]<<"];";
                myfile_trq.close();
                myfile_q.close();
                cin>>click;
                count = 2;

              //cin>>click;
              }
               

              
                
                //count = count + 0.5;

               
                }
               
               /*
               if (trq_read[1] > 1){
                cout<<"contact"<<endl;
                cin>>click;
               }*/

               if (count <=0.4) {        
               F_dest.p.data[2] = z0 -  count;
                //F_dest.p.data[0] = x0 -  count;
                //F_dest.p.data[1] = y0 -  count;
                count = count + 0.0001; 
               }
               

		if( _ik_solver_pos->CartToJnt(*_q_in, F_dest, q_out) != KDL::SolverI::E_NOERROR ) 
			cout << "failing in ik!" << endl;
               

		//Set the command
                ROBOT_CMD_STR rc2;
                for (int i=0; i<7; i++)
                rc2.jcmd[i] = q_out.data[i];
                for (int i=7; i<14; i++)
                rc2.jcmd[i] = 0;
		write( _jcommand_socket, &rc2, sizeof(rc2) ); //write commands over socket
                
                r.sleep();
                
                }

		
	}





void KUKA_INVKIN::run() {

	boost::thread ctrl_loop_t ( &KUKA_INVKIN::ctrl_loop, this);
	ros::spin();	

}




int main(int argc, char** argv) {


       FILE *fd;

        
        
        ifstream my_file;
        my_file.open("/home/utente/ros_ws/src/iiwa_kdl/src/trq.txt");
        
        

        //istringstream ss(line);
        for(int i=0;i<row;++i){
	   for(int j=0;j<col;++j){
        my_file >> vett[i][j];
        }
         }


        for(int i=0;i<row;i++){
        q1[i] = vett[i][0];
        q2[i] = vett[i][1];
        q3[i] = vett[i][2];
       q4[i] = vett[i][3];
       q5[i] = vett[i][4];
       q6[i] = vett[i][5];
       q7[i] = vett[i][6];


   }   
        
        

        cout<<"FINISH"<<endl;
        my_file.close();
         
       
	ros::init(argc, argv, "iiwa_kdl");
	KUKA_INVKIN ik;
	ik.run();

	return 0;
}
