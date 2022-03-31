#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include "std_msgs/String.h"
#include <sstream>  
#include <iostream>  
void blue_send(char* );
std::string blue_receive_string(int);
int s, status=0;
struct sockaddr_rc addr = { 0 };
char dest[18] = "A8:E2:C1:99:28:71";
std::stringstream input_stream;
std::set<std::string> sounds={"1234","exterminate","kick","seek and destroy","activate","fire","laser","shake","affirmative","goal","laugh","shooting","bowling","goodbye","like","shut down","brick eating","grab","mission accomplished","slam dunk","celebrate","hammer","no","strike","chuckle","hello","ouch","success chime","countdown","hi","ping","tadaa","countdown tick","hi 5","play","target acquired","damage","hit","punch","target destroyed","deactivate","horn","reverse","whirl","delivery","humming","revving","wow","dizzy","hydraulics down","sad","yes","error","hydraulics up","scanning","yipee","explosion","initialize","scared","yuck"};
std::set<std::string> images={"angry","arrow_e","arrow_n","arrow_ne","arrow_nw","arrow_s","arrow_se","arrow_sw","arrow_w","asleep","butterfly","chessboard","clock1","clock2","clock3","clock4","clock5","clock6","clock7","clock8","clock","clock1","clock11","clock12","confused","cow","diamond","diamond_small","duck","fabulous","ghost","giraffe","go_down","go_left","go_right","go_up","happy","heart","heart_small","house","meh","music_crotchet","music_quaver","music_quavers","no","pacman","pitchfork","rabbit","rollerskate","sad","silly","skull","smile","snake","square","square_small","stickfigure","surprised","sword","target","tortoise","triangle","triangle_left","tshirt","umbrella","xmas","yes"};
std::string str_tolower(std::string );
std::string blue_get_line();
bool isCommand(std::string);
std::string blue_receive_command();
double* blue_get_pose();
int step=0;

std::string str_tolower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), 
                // static_cast<int(*)(int)>(std::tolower)         // wrong
                // [](int c){ return std::tolower(c); }           // wrong
                // [](char c){ return std::tolower(c); }          // wrong
                   [](unsigned char c){ return std::tolower(c); } // correct
                  );
    return s;
}

void legoSoundCallback(const std_msgs::String::ConstPtr& msg)
{
  if(sounds.find(str_tolower(msg->data)) != sounds.end())
  {
    char command[100];
    std::string command_string="sound,"+msg->data;
    strcpy(command,command_string.c_str());
    ROS_INFO("LegoSound. I heard: [%s]", msg->data.c_str()); 
    blue_send(command);  
  }
  else
  {
    std::cout<<"Sound does not exists"<<std::endl;
  }
}
void legoImageCallback(const std_msgs::String::ConstPtr& msg)
{
  if(images.find(str_tolower(msg->data)) != images.end())
  {
    //char command;
    char command[100];
    std::string command_string="image,"+msg->data;
    strcpy(command,command_string.c_str());
    ROS_INFO("LegoImage. I heard: [%s]", msg->data.c_str()); 
    blue_send(command);
  }
  else
  {
    std::cout<<"Image does not exists"<<std::endl;
  }

}

class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot(): cmd{0,0,0,0},pos{0,0,0,0},last_pos{0,0,0,0},vel{0,0,0,0},eff{0,0,0,0},last_cmd{0,0,0,0},conv_cmd{0,0,0,0}
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("joint_1", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("joint_2", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   hardware_interface::JointStateHandle state_handle_c("joint_3", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_c);

   hardware_interface::JointStateHandle state_handle_d("joint_4", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_d);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("joint_1"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_a);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("joint_2"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_b);

   hardware_interface::JointHandle pos_handle_c(jnt_state_interface.getHandle("joint_3"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_c);

   hardware_interface::JointHandle pos_handle_d(jnt_state_interface.getHandle("joint_4"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_d);

   registerInterface(&jnt_pos_interface);
   size_cmd=sizeof(double)*4;
   
   //registerInterface(&jnt_effort_interface);
  }

  void hw_read() { 
    memcpy(pos,blue_get_pose(),size_cmd);
    // Leer sensores del robot.
     };
  void hw_write() { 
    if (first_command)
    {// Clean first command.
      first_command=false;
      memcpy(last_cmd,cmd,size_cmd);
    }
    if (memcmp(last_cmd,cmd,size_cmd)==0)
    {
      //std::cout<<"cmd: "<<cmd[0]<<std::endl;
    }
    else
    {
      
      //last_cmd=cmd[0];
      memcpy(last_cmd,cmd,size_cmd);
      memcpy(conv_cmd,cmd,size_cmd);
      //std::cout<<"cmd: "<< cmd[0] <<std::endl;
      std::string command_string="["+std::to_string(conv_cmd[0])+","+std::to_string(conv_cmd[1])+","+std::to_string(conv_cmd[2])+","+std::to_string(conv_cmd[3])+"]";
      
      std::cout<<"step: "<<step++<<" command: "<<command_string<<std::endl;
      
      char command[100];
      strcpy(command,command_string.c_str());
      blue_send(command);
      
    }
    //pos[0]=cmd[0];
    //pos[1]=cmd[1];
    //pos[2]=cmd[2];
    // Enviar al robot

  }; 

private:

  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  //hardware_interface::EffortJointInterface jnt_effort_interface;
  double cmd[4];
  double pos[4];
  double last_pos[4];
  double vel[4];
  double eff[4];
  double last_cmd[4];
  double conv_cmd[4];
  size_t size_cmd;
  bool first_command=true;

};
  void connect()
  {
    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );
    // connect to server
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
    std::cout<<"status conexion: "<<status<<std::endl;
    blue_send("0");
  }
  void disconnect()
  {
    close(s);
  }
  void blue_send(char* msg)
  {
    // send a message
    if( status == 0 ) {
      status = write(s, msg, strlen(msg));
      //std::cout<<"status: "<<status<<std::endl;
      sleep(0.001);
      //blue_receive_string(100);
      status=0;
    }
    if( status < 0 ) perror("uh oh");
  }
  void blue_receive()
  {
    void *buf;
    read(s, buf, 1000);
    std::cout<<"buf: "<<(char *)buf <<std::endl;
  }
  std::string blue_receive_string(int bytes) {
    std::string output(bytes, 0);
    if (read(s, &output[0], bytes-1)<0) {
        std::cerr << "Failed to read data from socket.\n";
    }
    //std::cout<<"buf: "<<output<<std::endl;
    input_stream<< output;
    return output;
}
std::string blue_get_line()
{
  blue_receive_string(1000);
  std::string line;
  std::getline(input_stream, line);
  input_stream.clear();
  //std::cout <<"line: "<< line << '\n';
  //std::cout<<"getline: "<<std::getline(input_stream, line)<<std::endl;
/*
  for (line; std::getline(input_stream, line,'\r');) {
        
  }*/
  return line;
} 
bool isCommand(std::string command)
{
    char ch = '[';
    int count = 0;
    for (int i = 0; (i = command.find(ch, i)) != std::string::npos; i++) {
        count++;
    }
    if (count>1)
      return false;
    else
      return true;
}
std::string blue_receive_command()
{
  std::string command=blue_get_line();
  //std::cout<<"Raw Command: "<<command<<std::endl;
  if (isCommand(command))
  {
    return command;
  }
  else
    return "";
}
double* blue_get_pose()
{
  static double pos[4]={0.0d,0.0d,0.0d,0.0d};
  std::string raw_pose=blue_receive_command();
  //std::cout<<"Raw pose: "<<raw_pose<<std::endl;
  std::size_t start,end;
  start=raw_pose.find("[");
  if (start!=std::string::npos)
  {
    //std::cout <<"start: "<< start<<"std::endl";
    end=raw_pose.find("]",start);
    //std::cout <<"end: "<< end<<"std::endl";
    std::string pose="";
    if (end!=std::string::npos )
    {
      //std::cout<<"Processing pose "<<std::endl;
      pose=raw_pose.substr(start+1,end-start-1);
      //std::cout<<"Filtered Command: "<<command<<std::endl;
      std::stringstream pose_stream(pose);
      std::string joint_pose;
      int i=0;
      while (getline(pose_stream,joint_pose, ',')) {  
        try {
          pos[i++]=std::stod(joint_pose);     
        } catch (const std::invalid_argument&) {
          std::cerr << "Argument is invalid: "<<pose<<"\n";
          
          //throw;
        } catch (const std::out_of_range&) {
          std::cerr << "Argument is out of range for a double\n";
          //throw;
        }     
      }
    }
    
  }
  //std::cout<<"Pose processed"<<pos[0]<<" - "<<pos[1]<<" - "<<pos[2]<<" - "<<pos[3]<<std::endl;
  return pos;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rrbot");
  ros::NodeHandle nh;  
  ros::Subscriber sub_sound = nh.subscribe("/lego/sound", 1000, legoSoundCallback);
  ros::Subscriber sub_image = nh.subscribe("/lego/image", 1000, legoImageCallback);
  //std::string hub_address="A8:E2:C1:99:28:71";
  MyRobot robot;
  controller_manager::ControllerManager cm(&robot);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  connect();

   // return 0;

  /*
      inquiry_info *ii = NULL;
    int max_rsp, num_rsp;
    int dev_id, sock, len, flags;
    int i;
    char addr[19] = { 0 };
    char name[248] = { 0 };

    dev_id = hci_get_route(NULL);
    sock = hci_open_dev( dev_id );
    if (dev_id < 0 || sock < 0) {
        perror("opening socket");
        exit(1);
    }

    len  = 8;
    max_rsp = 255;
    flags = IREQ_CACHE_FLUSH;
    ii = (inquiry_info*)malloc(max_rsp * sizeof(inquiry_info));
    
    num_rsp = hci_inquiry(dev_id, len, max_rsp, NULL, &ii, flags);
    if( num_rsp < 0 ) perror("hci_inquiry");

    for (i = 0; i < num_rsp; i++) {
        ba2str(&(ii+i)->bdaddr, addr);
        memset(name, 0, sizeof(name));
        if (hci_read_remote_name(sock, &(ii+i)->bdaddr, sizeof(name), 
            name, 0) < 0)
        strcpy(name, "[unknown]");
        printf("%s  %s\n", addr, name);
    }

    free( ii );
    close( sock );
    return 0;
*/
  ros::Time timestamp;
  ros::Duration period;
  auto stopwatch_last = std::chrono::steady_clock::now();
  auto stopwatch_now = stopwatch_last;
  timestamp = ros::Time::now();
  stopwatch_now = std::chrono::steady_clock::now();
  period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
  stopwatch_last = stopwatch_now;

  //ros::Duration d(0,10000000);
  while (ros::ok())
  {
     robot.hw_read();
     //cm.update(ros::Time::now(),d);
    // Get current time and elapsed time since last read
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    cm.update(timestamp, period);
     robot.hw_write();
     //sleep(0.01);
     //blue_receive_string(100);
     //blue_send("1");
  }
  disconnect();
}
