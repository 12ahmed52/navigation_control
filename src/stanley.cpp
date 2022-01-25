#include <string>
#include <iostream>
#include <math.h>
#include <cmath>
# define M_PI           3.14159265358979323846

float k_const=0.5;
float softening=1;
float state_x=0;
float state_y=0;
float state_theta=0;
float steering_angle=0.1;
float l = 2;
float lr=1;
int arr_x[10000];
int arr_y[10000];
int end_iteration=100;
int start_iteration=0;
float velocity=2;
float sample_time=0.01;
int save_i=0;
using namespace std;

void bicycle_model();
void stanley_controller();

int main(){
	for(int i=0;i<9000;i++){
		arr_x[i]=5;
		arr_y[i]=i;
	}
	state_x=0;
	state_y=0;
	state_theta=0;
	for(int j=0;j<10000;j++){
		stanley_controller();
		bicycle_model();
		cout<<state_x<<"               "<<state_y<<"                        "<<steering_angle<<endl;
	
	}

}


void bicycle_model(){
float x_front_dot=velocity*cos(state_theta+steering_angle);
float y_front_dot=velocity*sin(state_theta+steering_angle);
float theta_dot=velocity*sin(steering_angle)/l;      
state_x+=x_front_dot*sample_time;      
state_y+=y_front_dot*sample_time;      
state_theta+=theta_dot*sample_time;


}

void stanley_controller(){
float a=-(arr_y[save_i+3]-arr_y[save_i]);
float b=(arr_x[save_i+3]-arr_x[save_i]);
float c=(-(a)arr_x[save_i]-b(arr_y[save_i]));
float yaw_path=-atan(a/b);
if(yaw_path==1.5708){
 a=(arr_y[save_i+3]-arr_y[save_i]);
 b=(arr_x[save_i+3]-arr_x[save_i]);
 c=(-(a)arr_x[save_i]-b(arr_y[save_i]));
 yaw_path=atan(a/b);

}

float heading_error=yaw_path-state_theta;

/*if (heading_error > M_PI){
	heading_error -= 2*M_PI;
}
if (heading_error < -M_PI){
	heading_error += 2 * M_PI;
	}
	*/
float cross_track_error=(a*state_x+b*state_y+c)/sqrt(pow(a,2)+pow(b,2));
float yaw_cross_track=atan((state_y-arr_y[save_i])/(state_x-arr_x[save_i]));
float yaw_path2ct=yaw_path-yaw_cross_track;
/*if (yaw_path2ct > M_PI){
	yaw_path2ct -= 2 * M_PI;
}
if (yaw_path2ct < - M_PI){
	yaw_path2ct += 2 * M_PI;
	}
	*/
float check_point=((state_x-arr_x[save_i])(arr_y[save_i+3]-arr_y[save_i]))-((state_y-arr_y[save_i])(arr_x[save_i+3]-arr_x[save_i]));
if ( check_point>0){
        cross_track_error = abs(cross_track_error);
	}
if( check_point < 0){
        cross_track_error = - abs(cross_track_error);
	}


	float cross_track_steering=atan((cross_track_error)/(velocity));
	steering_angle = heading_error + cross_track_steering;
	if (steering_angle > 0.532){
        steering_angle = 0.532;
	}
    if (steering_angle < -0.523){
        steering_angle = -0.532;
	}
	save_i++;
}
