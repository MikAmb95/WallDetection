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
#include <Eigen/Dense>


using namespace Eigen;
using namespace std;
using namespace KDL;




class KUKA_INVKIN {
	public:
		KUKA_INVKIN();
		void run();
                void pd_g();
                

                void function_sub1(std_msgs::Float64 x);
                void function_sub2(std_msgs::Float64 x);
                void function_sub3(std_msgs::Float64 x);
                void function_sub4(std_msgs::Float64 x);
                void function_sub5(std_msgs::Float64 x);
                void function_sub6(std_msgs::Float64 x);
                void function_sub7(std_msgs::Float64 x);
                


		
                
               

	private:
      	        int _jstate_socket;
                int _jcommand_socket;
                float trq_vec[7];
                int W_L;
                int W_d;
                int N;
	        
                ros::Subscriber _my_sub2_1;
                ros::Subscriber _my_sub2_2;
                ros::Subscriber _my_sub2_3;
                ros::Subscriber _my_sub2_4;
                ros::Subscriber _my_sub2_5;
                ros::Subscriber _my_sub2_6;
                ros::Subscriber _my_sub2_7;


                ros::Publisher _my_pub2;
                
 
		ros::NodeHandle _nh;
		KDL::Tree iiwa_tree;
                Eigen::MatrixXd init_data;
                Eigen::MatrixXd V;
	
		

                bool first_flag;
                int flag_init[7] = {1,1,1,1,1,1,1};

	
};





//SUBSCRIBER

void KUKA_INVKIN::function_sub1( std_msgs::Float64 x ) { 
       flag_init[0] = 0;
       trq_vec[0]=x.data;
       //cout<<"TRQ: "<<trq_vec[0];
       
}

void KUKA_INVKIN::function_sub2( std_msgs::Float64 x ) {
        flag_init[1] = 0;
        trq_vec[1]=x.data; 
         //cout<<" "<<trq_vec[1];   
        
       
}

void KUKA_INVKIN::function_sub3( std_msgs::Float64 x ) {
        flag_init[2] = 0;
        trq_vec[2]=x.data; 
        //cout<<" "<<trq_vec[2];   
       
}

void KUKA_INVKIN::function_sub4( std_msgs::Float64 x ) {
     flag_init[3] = 0;
     trq_vec[3]=x.data;  
     //cout<<" "<<trq_vec[3];      
}

void KUKA_INVKIN::function_sub5( std_msgs::Float64 x ) {
        flag_init[4] = 0;
        trq_vec[4]=x.data;    
       // cout<<" "<<trq_vec[4];    
}

void KUKA_INVKIN::function_sub6( std_msgs::Float64 x ) {
        flag_init[5] = 0;
        trq_vec[5]=x.data; 
        //cout<<" "<<trq_vec[5]; 
            
}

void KUKA_INVKIN::function_sub7( std_msgs::Float64 x ) {
        flag_init[6] = 0;
        trq_vec[6]=x.data;   
        //cout<<" "<<trq_vec[6]<<endl;     
}



KUKA_INVKIN::KUKA_INVKIN() {

        _my_pub2 = _nh.advertise< std_msgs::Float64 > ("/topic_sub1",0);
        
        _my_sub2_1 = _nh.subscribe("/topic_pub1", 0, &KUKA_INVKIN::function_sub1, this);
        _my_sub2_2 = _nh.subscribe("/topic_pub2", 0, &KUKA_INVKIN::function_sub2, this);
        _my_sub2_3 = _nh.subscribe("/topic_pub3", 0, &KUKA_INVKIN::function_sub3, this);
        _my_sub2_4 = _nh.subscribe("/topic_pub4", 0, &KUKA_INVKIN::function_sub4, this);
        _my_sub2_5 = _nh.subscribe("/topic_pub5", 0, &KUKA_INVKIN::function_sub5, this);
        _my_sub2_6 = _nh.subscribe("/topic_pub6", 0, &KUKA_INVKIN::function_sub6, this);
        _my_sub2_7 = _nh.subscribe("/topic_pub7", 0, &KUKA_INVKIN::function_sub7, this);
        
        W_L = 70;
        W_d = 50;
        N = 1000;

        init_data = MatrixXd::Zero(W_L,7);
        V = MatrixXd::Zero(W_L,1);

        


}





void KUKA_INVKIN::pd_g() {

int flag = 0;

std_msgs::Float64 cmd;



//float mu0_hat[7];

VectorXd mu0_hat(7);
VectorXd sigma0_all(7);
VectorXd sigma0_hat(7);
MatrixXd mu1_hat(W_L,7);
VectorXd mu2_hat(7);
VectorXd delta_torque(7);
VectorXd sup_S(7);


MatrixXd S = MatrixXd::Zero(W_L,7);
MatrixXd d = MatrixXd::Zero(N,7);
MatrixXd j_starL = MatrixXd::Zero(N,7);
MatrixXd g_GLR = MatrixXd::Zero(N,7);
MatrixXd flag_residual = MatrixXd::Zero(1,7);
MatrixXd t_residual = MatrixXd::Zero(1,7);
MatrixXd flag_torque = MatrixXd::Zero(1,7);
MatrixXd t_torque = MatrixXd::Zero(1,7);


MatrixXd data2 = MatrixXd::Zero(W_d+1,7);
MatrixXd data3 = MatrixXd::Zero(W_d+1,7);
MatrixXd data4 = MatrixXd::Zero(W_d,7);

MatrixXd init_data2 = MatrixXd::Zero(W_d+W_L+1,7);
MatrixXd init_data = MatrixXd::Zero(W_d+W_L+1,7);

Eigen::Index maxRow,maxCol;

//ros::Rate r(1000);


float j_starW = 0;
float j_star = 0;
float sum = 0 ;
float thres_g = 2000+2000000*0;
float thres_torque = 1.5;

int flag_alarm = 0;
int target_torque1 = 1 ;
int target_torque2 = 3 ;

while(flag_init[0] == 1) {
cout<<"Nothing received yet"<<endl;
}
cout<<"Start Init"<<endl;
 for(int i = 0;i<W_d+W_L+1;i++){
     for(int j = 0;j<7;j++){
     while(flag_init[j] == 1);
     init_data2(i,j) = trq_vec[j];
     flag_init[j] = 1;
     }
     
}

for(int i = W_L;i<W_L+W_d+1;i++){
     for(int j = 0;j<7;j++){
     data2(i-W_L,j) = init_data2(i,j);
     }
     
}

//cout<<"Data2"<<data2<<endl;

cout<<"End Init"<<endl;
//cin>>flag;
init_data = init_data2;

VectorXd Y(W_d+1);
MatrixXd K = MatrixXd::Constant(W_d+1,2,1);
MatrixXd Kk = MatrixXd::Zero(2,2);
MatrixXd temp_vec = MatrixXd::Zero(W_L,7);
VectorXd time(30000);
MatrixXd slope_est = MatrixXd::Zero(30000,7);
VectorXd theta_hat(2);

float Ts = 0.005;

for(int i = 0;i<30000;i++){
time(i) = i*Ts;
}

int fk = 0;

while(fk < 70){
 for(int oo = 0; oo < 7; oo ++) {
        for(int i = 0;i<W_d+1;i++){
         K(i,0) = time(i+fk);
         }   
        for(int i = 0;i<W_d+1;i++){
        Y(i) = init_data(i+fk,oo);
        }
        Kk = K.transpose()*K;
        theta_hat = Kk.inverse()*K.transpose()*Y;      
        slope_est(W_d+fk,oo) = theta_hat(0);
        temp_vec(fk,oo) = theta_hat(0);
}
//cout<<"slope: "<<slope_est.row(W_d+fk)<<endl;
fk = fk + 1;
}

//cout<<"slope: "<<slope_est<<endl;*/





for(int j = 0;j<7;j++){
V = temp_vec.col(j);
mu0_hat(j) = V.mean();
}
//cout<<"mu0_hat: " <<mu0_hat.transpose()<<endl;
//cin>>flag;



for(int j=0;j<7;j++){

double variance = 0;
double variance_f = 0;
double t = temp_vec(0,j);

for (int i = 1; i<W_L; i++){

        t += temp_vec(i,j);
        double diff = ((i+1)*temp_vec(i,j))-t;
        variance += (diff*diff)/((i+1.0)*i);
}

variance_f = variance / (W_L - 1);
sigma0_all(j) = sqrt(variance_f);
}

//cout<<"sigma0_all: " <<sigma0_all.transpose()<<endl;
//cin>>flag;

for( int j = 0; j < 7; j++){
sigma0_hat(j) = sigma0_all.maxCoeff();
}

//cout<<"sigma0_hat: " <<sigma0_hat.transpose()<<endl;

for( int j = 0; j < 7; j++){
for( int i = 0; i < W_L; i++){
mu1_hat(i,j) = mu0_hat(j);
}
}


//cout<<"mu1_hat: " <<mu1_hat<<endl;

int j = 0;
int cnt2 = 0;
//cin>>j;

int k = 0;
       while( ros::ok() ) {

          for(int j = 0; j < 7; j++){
          for(int i = 1; i<W_d+1; i++) {
           data3(i-1,j) = data2(i,j);
          }
          }
         
          for(int j = 0;j<7;j++){
          while(flag_init[j] == 1);
          data3(W_d,j) = trq_vec[j];
          flag_init[j] = 1;
          }
          data2 = data3;

         for(int oo = 0; oo < 7; oo ++) {
        for(int i = 0;i<W_d+1;i++){
         K(i,0) = time(i+fk);
         }   
        for(int i = 0;i<W_d+1;i++){
        Y(i) = data2(i,oo);
        }
        Kk = K.transpose()*K;
        theta_hat = Kk.inverse()*K.transpose()*Y;      
        slope_est(W_d+fk,oo) = theta_hat(0);
        //temp_vec(fk,oo) = theta_hat(0);
        //cout<<"Th: "<<theta_hat(0)<<endl;
        //cout<<"fk"<<fk<<endl;
         
         //cout<<"Inside main loop"<<endl;

          //cout<<"first for"<<endl;
          for(int j = 0;j<W_L;j++){
          sum = 0;
          for(int i = j;i<W_L; i++) {
          sum = sum + (slope_est(i+W_d+k,oo)-mu0_hat(oo));
          //cout<<sum<<endl;
          }
          
          S(j,oo) = (1/(sigma0_hat(oo)*sigma0_hat(oo)))*sum*sum;
         }

          V = S.col(oo);
          //cout<<"V: "<<V.transpose()<<endl;
          sup_S(oo) = V.maxCoeff(&maxRow,&maxCol);
          j_star = maxRow;
          //cout<<"Sup "<<sup_S(oo)<<" "<<oo<<" "<<k<<endl;
          
          
          sum = 0;
          cnt2 = 0;
          for(int j = j_star+W_d+k; j < 121+k; j++){
          sum = sum + slope_est(j,oo);
          //cout<<"s_est: "<<j<<" "<<slope_est(j,oo)<<endl;
          cnt2 = cnt2 + 1;
          }
          //cout<<"sum"<<sum/(71-j_star)<<" "<<oo<<" "<<k<<endl;
          mu2_hat(oo) = sum/(71- j_star);
          
          //cout<<"mu2_hat: "<<oo<<" "<<mu2_hat(oo)<<endl;
          //cout<<"mu0_hat: "<<oo<<" "<<mu0_hat(oo)<<" "<<k<<endl;
          //cin>>sum;
          delta_torque(oo) = abs(mu2_hat(oo) - mu0_hat(oo));
          if(delta_torque(oo) > thres_torque && flag_torque(0,oo) == 0) {
                 flag_torque(0,oo) = 1;
                 cout<<"FLAG SLOPE "<<oo<<" "<<k+120<<endl;
          }

          if(sup_S(oo) > thres_g && flag_residual(0,oo) == 0) {
                 flag_residual(0,oo) = 1;
                 cout<<"FLAG RESIDUAL SLOPE "<<oo<<" "<<k+120<<endl;
          }

          if(flag_residual(0,target_torque1) == 1 && flag_torque(0,target_torque1) == 1 && flag_residual(0,target_torque2) == 1 && flag_torque(0,target_torque2) == 1 && flag_alarm == 0) {
          flag_alarm = 1;
          cout<<"FLAG ON"<<endl;
          
          cmd.data = flag_alarm;
          _my_pub2.publish (cmd);
          //cin>>flag;
          }
         } 


       cout<<"sup_S: "<<sup_S.transpose()<<endl;
       cout<<"delta_slope: "<<delta_torque.transpose()<<endl;
       fk = fk + 1;
               
  
       

        k = k + 1;
        //r.sleep();
        

       
        }




}


void KUKA_INVKIN::run() {
	boost::thread pd_g ( &KUKA_INVKIN::pd_g, this);
	ros::spin();	
}




int main(int argc, char** argv) {
	ros::init(argc, argv, "iiwa_kdl_ctrl");
	KUKA_INVKIN ik;
	ik.run();
	return 0;
}
