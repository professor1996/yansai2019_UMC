#pragma once
#include <ros/ros.h>
#include <math.h>
#include <stdio.h>      // standard input / output functions
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitionss

#define MSGTOMCU_SIZE 7
#define MSGFROMMCU_SIZE 16
struct RobotMsgToMCU
{
  short int mode=0;
  short int velocity_x=0;
  short int velocity_y=0;
  short int velocity_yaw=0;
  short int gimbal_yaw=0;
  short int gimbal_pitch=0;
  short int enemy_distance =0;
  void setMsg(short int _mode,short int _velocity_x,short int _velocity_y,short int _velocity_yaw,short int _gimbal_yaw,short int _gimbal_pitch,short int _enemy_distance){
    mode=_mode;
    velocity_x=_velocity_x;
    velocity_y=_velocity_y;
    velocity_yaw=_velocity_yaw;
    gimbal_yaw=_gimbal_yaw;
    gimbal_pitch=_gimbal_pitch;
    enemy_distance=_enemy_distance;
  };
  void clear(){
    mode=0;velocity_x=0;velocity_y=0;velocity_yaw=0;gimbal_yaw=0;gimbal_pitch=0;enemy_distance=0;
  };
};

struct RobotMsgFromMCU
{
	short int ax=0;
	short int ay=0;
	short int az=0;
	short int gx=0;
	short int gy=0;
	short int gz=0;

	short int pit=0;
	short int rol=0;

};

class Serial
{
public:
  int fd;
  struct termios port_settings;     
  Serial(const char* dev_name);
  ~Serial();
  
  void configurePort();
  bool SendData(struct RobotMsgToMCU msg);
  bool ReadData(struct RobotMsgFromMCU& msg);
  
};

Serial::Serial(const char* dev_name)
{
  fd = open(dev_name, O_RDWR |O_NONBLOCK);//| O_NONBLOCK  O_NDELAY
  if(fd == -1){
    printf("open_port: Unable to open /dev/ttyTHS2. \n");
  }
  else  {
    fcntl(fd, F_SETFL, 0);
    printf("%d  port is open.\n",fd);
  }
}
Serial::~Serial()
{
  close(fd);
}

void Serial::configurePort(){                      // configure the port
  // structure to store the port settings in
  cfsetispeed(&port_settings, B115200);       // set baud rates
  cfsetospeed(&port_settings, B115200);
  
  port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits
  port_settings.c_cflag &= ~CSTOPB;
  port_settings.c_cflag &= ~CSIZE;
  port_settings.c_cflag |= CS8;
  port_settings.c_cflag |=CLOCAL|CREAD;
  tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port
}

bool Serial::SendData(struct RobotMsgToMCU msg){
  unsigned char send_bytes[MSGTOMCU_SIZE*2+2] = {0x00};
  send_bytes[0]=0x7F;
  send_bytes[MSGTOMCU_SIZE*2+1]=0x7E;
  unsigned char* ptr_send_bytes = send_bytes + 1;
  memcpy(ptr_send_bytes,&msg,sizeof(msg));
  
  //printf ("%X %X%X %X%X %X%X %X%X %X%X %X%X %X%X %X\n",send_bytes[0],send_bytes[1],send_bytes[2],send_bytes[3],send_bytes[4],send_bytes[5 ],send_bytes[6],send_bytes[7],send_bytes[8],send_bytes[9],send_bytes[10],send_bytes[11],send_bytes[12],send_bytes[13],send_bytes[14],send_bytes[15]);
  
  if (16 == write(fd, send_bytes,16))      //Send data
    return true;
  return false;
}

bool Serial::ReadData(struct RobotMsgFromMCU& msg)
{
  uint8_t tmp[1]={0};
  uint8_t buf[255]={0};
  uint8_t *ptr=buf;
  int data_ready=0;
  int data_start=0;
  while(!data_ready)
  {
    
    int ret = read(fd, tmp, 1);
    if (ret==0)
      continue;

    if (tmp[0]==0x7F)
    {
      data_start=1;
    }

    if (data_start)
    {
        *ptr=tmp[0];
	ptr++;

        if (tmp[0]==0x7E)
        {
            if (buf[MSGFROMMCU_SIZE+1]==0x7E)
            {
 
	        data_ready=1;
 
	        memcpy(&msg,buf+1,sizeof(msg));

	        tcflush(fd, TCIFLUSH);
	

	        return true;
             }
             else
	       return false;
        }
     }
  }
  return false;
}













