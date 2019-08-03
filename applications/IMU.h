#ifndef _IMU_H_
#define _IMU_H_



typedef struct{
    float yaw,pitch,roll;
}Attitude_t;

typedef struct{
    int x,y,z;
}Int_xyz_t;

typedef struct{
    float x,y,z;
}Float_xyz_t;




void IMU_Updata(float gx, float gy, float gz, float ax, float ay, float az,Attitude_t* Q);

#endif
