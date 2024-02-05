#include <iostream>
#include "imu_lib.hpp"
ICM20948 imu1;
int8_t index1;

int main(int argc, char* argv[])
{	
    int sampling_rate=1125;
    imu1.halfT=1.0/(sampling_rate*2);
    int sampling_time=(1000000/225)+1;
    imu1.I2Cinit(7,REG_VAL_BIT_GYRO_FS_1000DPS,REG_VAL_BIT_ACCEL_FS_4g);
    int sample_count=0;
    int max_sample=225*4;
    Eigen::MatrixXd A(6, 4);
    Eigen::MatrixXd B(6, 3);
    B << 1,0,0,
        -1,0,0,
        0,1,0,
        0,-1,0,
        0,0,1,
        0,0,-1;
    Eigen::Vector3d sample_sum={0,0,0};
    // x axis up: (1,0,0) 
    // x axis down: (-1,0,0) 
    // y axis up: (0,1,0) 
    // y axis down: (0,-1,0) 
    // z axis up:(0,0,1) 
    // z axis down(gravity):(0,0,-1), 
    printf("x axis up sampling------\n");
    printf("prepare------\n");
    sleep(5);
    while(sample_count<max_sample){
        // x axis up
        
        imu1.imuDataGet(false);
        usleep(sampling_time);
        double accel_x =imu1.stAccelRawData.s16X/imu1.ACCEL_SCALE_FACTOR;
        double accel_y =imu1.stAccelRawData.s16Y/imu1.ACCEL_SCALE_FACTOR;
        double accel_z =imu1.stAccelRawData.s16Z/imu1.ACCEL_SCALE_FACTOR;
        Eigen::Vector3d sample={accel_x,accel_y,accel_z};
        double accel_scale = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z; 
        if ((accel_x>0.98)&&(accel_x<1.06)&&(accel_scale<1.124))
        {
            printf("accel_x:%f,accel_y:%f,accel_z:%f\n",accel_x,accel_y,accel_z);
            sample_sum=sample_sum+sample;
            ++sample_count;
        }
        else{
            printf("please place imu in x_up_axis direction\n");
        }


    }
    A.row(0) << sample_sum[0]/max_sample, sample_sum[1]/max_sample, sample_sum[2]/max_sample, 1.0;
    sample_sum={0,0,0};
    sample_count=0;
    printf("x axis down sampling------\n");
    printf("prepare------\n");
    sleep(5);
    while(sample_count<max_sample){
        // x axis up
        
        imu1.imuDataGet(false);
        usleep(sampling_time);
        double accel_x =imu1.stAccelRawData.s16X/imu1.ACCEL_SCALE_FACTOR;
        double accel_y =imu1.stAccelRawData.s16Y/imu1.ACCEL_SCALE_FACTOR;
        double accel_z =imu1.stAccelRawData.s16Z/imu1.ACCEL_SCALE_FACTOR;
        Eigen::Vector3d sample={accel_x,accel_y,accel_z};
        double accel_scale = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z; 
        if ((accel_x<-0.98)&&(accel_x>-1.06)&&(accel_scale<1.124))
        {
            printf("accel_x:%f,accel_y:%f,accel_z:%f\n",accel_x,accel_y,accel_z);
            sample_sum=sample_sum+sample;
            ++sample_count;
        }
        else{
            printf("please place imu in x_down_axis direction\n");
        }
   
    }
    A.row(1) << sample_sum[0]/max_sample, sample_sum[1]/max_sample, sample_sum[2]/max_sample, 1.0;
    sample_sum={0,0,0};
    sample_count=0;
    printf("y axis up sampling------\n");
    printf("prepare------\n");
    sleep(5);
    while(sample_count<max_sample){
        // x axis up
        
        imu1.imuDataGet(false);
        usleep(sampling_time);
        double accel_x =imu1.stAccelRawData.s16X/imu1.ACCEL_SCALE_FACTOR;
        double accel_y =imu1.stAccelRawData.s16Y/imu1.ACCEL_SCALE_FACTOR;
        double accel_z =imu1.stAccelRawData.s16Z/imu1.ACCEL_SCALE_FACTOR;
        Eigen::Vector3d sample={accel_x,accel_y,accel_z};
        double accel_scale = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z; 
        if ((accel_y>0.98)&&(accel_y<1.06)&&(accel_scale<1.124))
        {
            printf("accel_x:%f,accel_y:%f,accel_z:%f\n",accel_x,accel_y,accel_z);
            sample_sum=sample_sum+sample;
            ++sample_count;
        }
        else{
            printf("please place imu in y_up_axis direction\n");
        }

    }
    A.row(2) << sample_sum[0]/max_sample, sample_sum[1]/max_sample, sample_sum[2]/max_sample, 1.0;
    sample_sum={0,0,0};
    sample_count=0;
    printf("y axis down sampling------\n");
    printf("prepare------\n");
    sleep(5);
    while(sample_count<max_sample){
        // x axis up
        
        imu1.imuDataGet(false);
        usleep(sampling_time);
        double accel_x =imu1.stAccelRawData.s16X/imu1.ACCEL_SCALE_FACTOR;
        double accel_y =imu1.stAccelRawData.s16Y/imu1.ACCEL_SCALE_FACTOR;
        double accel_z =imu1.stAccelRawData.s16Z/imu1.ACCEL_SCALE_FACTOR;
        Eigen::Vector3d sample={accel_x,accel_y,accel_z}; 
        double accel_scale = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z; 
        if ((accel_y<-0.98)&&(accel_y>-1.06)&&(accel_scale<1.124))
        {
            printf("accel_x:%f,accel_y:%f,accel_z:%f\n",accel_x,accel_y,accel_z);
            sample_sum=sample_sum+sample;
            ++sample_count;
        }
        else{
            printf("please place imu in y_down_axis direction\n");
        }

    }
    A.row(3) << sample_sum[0]/max_sample, sample_sum[1]/max_sample, sample_sum[2]/max_sample, 1.0;
    sample_sum={0,0,0};
    sample_count=0;
    printf("z axis up sampling------\n");
    printf("prepare------\n");
    sleep(5);
    while(sample_count<max_sample){
        // x axis up
        
        imu1.imuDataGet(false);
        usleep(sampling_time);
        double accel_x =imu1.stAccelRawData.s16X/imu1.ACCEL_SCALE_FACTOR;
        double accel_y =imu1.stAccelRawData.s16Y/imu1.ACCEL_SCALE_FACTOR;
        double accel_z =imu1.stAccelRawData.s16Z/imu1.ACCEL_SCALE_FACTOR;
        double accel_scale = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z; 
        Eigen::Vector3d sample={accel_x,accel_y,accel_z}; 
        if ((accel_z>0.98)&&(accel_z<1.06)&&(accel_scale<1.124))
        {
            printf("accel_x:%f,accel_y:%f,accel_z:%f\n",accel_x,accel_y,accel_z);
            sample_sum=sample_sum+sample;
            ++sample_count;
            
        }
        else{
            printf("please place imu in z_up_axis direction\n");
        }

    }
    A.row(4) << sample_sum[0]/max_sample, sample_sum[1]/max_sample, sample_sum[2]/max_sample, 1.0;
    sample_sum={0,0,0};
    sample_count=0;
    printf("z axis down sampling------\n");
    printf("prepare------\n");
    sleep(5);
    while(sample_count<max_sample){
        // x axis up
        
        imu1.imuDataGet(false);
        usleep(sampling_time);
        double accel_x =imu1.stAccelRawData.s16X/imu1.ACCEL_SCALE_FACTOR;
        double accel_y =imu1.stAccelRawData.s16Y/imu1.ACCEL_SCALE_FACTOR;
        double accel_z =imu1.stAccelRawData.s16Z/imu1.ACCEL_SCALE_FACTOR;
        Eigen::Vector3d sample={accel_x,accel_y,accel_z};
        double accel_scale = accel_x*accel_x + accel_y*accel_y + accel_z*accel_z; 
        if ((accel_z<-0.98)&&(accel_z>-1.06)&&(accel_scale<1.124))
        {
            printf("accel_x:%f,accel_y:%f,accel_z:%f\n",accel_x,accel_y,accel_z);
            sample_sum=sample_sum+sample;
            ++sample_count;
        }
        else{
            printf("please place imu in z_down_axis direction\n");
        }

    }
    A.row(5) << sample_sum[0]/max_sample, sample_sum[1]/max_sample, sample_sum[2]/max_sample, 1.0;
    sample_sum={0,0,0};
    sample_count=0;
    Eigen::MatrixXd X = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    // 결과 출력
    std::cout << "Solution:\n" << X << std::endl;
    // imu1.gyro_stAvgBuf[0].s16AvgBuffer[index1]=imu1.stGyroRawData.s16X;
    // imu1.gyro_stAvgBuf[1].s16AvgBuffer[index1]=imu1.stGyroRawData.s16Y;
    // imu1.gyro_stAvgBuf[2].s16AvgBuffer[index1]=imu1.stGyroRawData.s16Z;
    // imu1.accel_stAvgBuf[0].s16AvgBuffer[index1]=imu1.stAccelRawData.s16X;
    // imu1.accel_stAvgBuf[1].s16AvgBuffer[index1]=imu1.stAccelRawData.s16Y;
    // imu1.accel_stAvgBuf[2].s16AvgBuffer[index1]=imu1.stAccelRawData.s16Z;
    


    // float angular_velocity_x = imu1.stGyroRawData.s16X/imu1.GYRO_SCALE_FACTOR;
    // float angular_velocity_y = imu1.stGyroRawData.s16Y/imu1.GYRO_SCALE_FACTOR;
    // float angular_velocity_z = imu1.stGyroRawData.s16Y/imu1.GYRO_SCALE_FACTOR;
    // float angular_velocity_x=angular_velocity_x*RAD_CONSTANT;
    // float angular_velocity_y=angular_velocity_y*RAD_CONSTANT;
    // float angular_velocity_z=angular_velocity_z*RAD_CONSTANT;

    
    

    imu1.Close();
}