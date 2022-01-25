#include <string>
#include <iostream>
#include <math.h>
#include <cmath>

float ld = 5;
float state_x=0;
float state_y=0;
float state_theta=0;
float x_front=0;
float y_front=0;
float l = 2;
float lr=1;
float nearist_calc=0;
float nearist_calc1=0;
float ld_calc=0;
float arr_x[10000];
float arr_y[10000];
float ld_calc_real=0;
int end_iteration=100;
int start_iteration=0;
float ld_x=0;
float ld_y=0;
float error=0;
float steering_angle=0.1;
float alpha=0;
float velocity=2;
float sample_time=0.01;
using namespace std;
void bicycle_model();

int main(){
	bicycle_model();
}

void bicycle_model(){
float beta=atan(   (  lr*(tan(steering_angle)/l)  )  );
float x_cg_dot=velocity*cos(state_theta+beta);
float y_cg_dot=velocity*sin(state_theta+beta);
float theta_dot=velocity*cos(beta)*tan(steering_angle)/l;      
state_x+=x_cg_dot*sample_time;      
state_y+=y_cg_dot*sample_time;      
state_theta+=theta_dot*sample_time;
}
void pure_pursuit(void){
int save_i=0;
//x_front=l*cos(state_theta);
//y_front=l*sin(state_theta);
float m = atan(state_theta);
float b= -1/m;
float c = -state_x-(b*state_y);
float min_distance=100000;
float nearist_x=0;
float nearist_y=0;
float point_position_check=0;

for(int i=start_iteration;i<end_iteration;i++){
nearist_calc= sqrt(pow(arr_x[i]-state_x,2)+pow(arr_y[i]-state_y,2));
nearist_calc1= sqrt(pow(arr_x[i+1]-state_x,2)+pow(arr_y[i+1]-state_y,2));
if(nearist_calc1<nearist_calc && nearist_calc1<=min_distance){
min_distance=nearist_calc1;
save_i=i;
nearist_x=arr_x[i+1];
nearist_y=arr_y[i+1];
}
if(nearist_calc<nearist_calc1 && nearist_calc<=min_distance){
min_distance=nearist_calc;
save_i=i;
nearist_x=arr_x[i];
nearist_y=arr_y[i];
}
start_iteration=save_i;
end_iteration=start_iteration+100;

}

ld =(   0.5*min_distance    +   0.5*v   );
    if (min_distance<3)
      {ld =2*v;}
	  
for(int j=0;j<100;j++){
ld_calc= sqrt(pow(nearist_x-arr_x[save_i+j+1],2)+pow(nearist_y-arr_y[save_i+j+1],2));
if(ld_calc>ld && ld_calc<ld+2){
ld_x=arr_x[save_i+j+1];
ld_y=arr_y[save_i+j+1];
j=100;
}
}
ld_calc_real=sqrt(pow(ld_x-state_x,2)+pow(ld_y-state_y,2));
error=abs(ld_x+b*ld_y+c)/(sqrt(1+pow(b,2)));
alpha=asin(error/ld_calc_real);
steering_angle=atan((2*l*sin(alpha))/ld_calc_real);
point_position_check=(ld_x-nearist_x)(state_y-nearist_y)-(ld_y-nearist_y)(state_x-nearist_x);
if(point_position_check<0){
steering_angle=-steering_angle;

}
}