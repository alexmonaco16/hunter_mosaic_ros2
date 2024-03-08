#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <functional>
#include <memory>

// ROS includes
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "hunter_msgs/msg/hunter_status.hpp"
#include "my_control_msgs/msg/pid_data.hpp"
#include "my_control_msgs/msg/leader_data.hpp"

// Sending rate in milliseconds (SIMULATION mode)
#define SIM_RATE 20
// simulation_speed = SCALE_FACTOR * real_speed
#define SCALE_FACTOR 20

// UDP ports and address
#define IN_PORT 1234
#define OUT_ADDRESS "127.0.0.1"
#define OUT_PORT 1235

// Architecture
    // .__________________________________.                 .__________________________________.
    // |                                  |  UDP port 1234  |                                  |
    // |     AgilexNodeApp       outSocket|---------------->|inSocket      alex_hil_agilex     |
    // |  (Mosaic simulation)             |                 |             (this applicative)   |
    // |                          inSocket|<----------------|outSocket                         |
    // L__________________________________|  UDP port 1235  L__________________________________|
    //

// struct identiche a quelle introdotte nel codice di HilNodeApp
// per poter correttamente interpretare, da ambo le parti, i pacchetti trasmessi sulle socket

// alex_hil_agilex (this applicative) --> AgilexNodeApp (Mosaic) message
typedef struct {
    int32_t number;
    int32_t placeholder = 0;
    double speed;
} ros2mosaic;

// AgilexNodeApp (Mosaic) --> alex_hil_agilex (this applicative) message
typedef struct {
    double leader_distance;
    double leader_velocity;
} mosaic2ros;

void initSockets();
void realSpeed(int, char**);
void simulateSpeed();
void sendData(int, double);

// might be needed in future, with bidirectional communication to and from Mosaic app
/*int32_t getNextInt32(char*, int&);
double getNextDouble(char* buffer, int& offset);
std::string getNextString(char*, int&);
*/
