#include "alex_hil_agilex.h"

// RX socket and address (from Mosaic app)
int inSocket;
struct sockaddr_in inAddr;

// TX socket and address (to Mosaic app)
int outSocket;
struct sockaddr_in outAddr;

// counter of received messages
int txCount = 0;

using std::placeholders::_1;
using namespace std::chrono_literals;

// ROS Subscriber class (for subscribing to publisher)
class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber()
  : Node("minimal_subscriber") {
    // nav_msgs::msg::Odometry is a ROS type containing many infos
    // among which, postion and speed on all 3 axis (x, y, z)
    subscriptionHunter_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&MinimalSubscriber::topic_callbackHunter, this,std::placeholders::_1));
    publisherHunter_ = this->create_publisher<control_msgs::msg::LeaderData>("leader_data", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  void topic_callbackHunter(const nav_msgs::msg::Odometry::SharedPtr msgReceived) {
    // pose.position -> position on the x,y,z axis
    double actX = msgReceived->pose.pose.position.x;
    double actY = msgReceived->pose.pose.position.y;
    double actZ = msgReceived->pose.pose.position.z;
    // twist -> linear speed on the x,y,z axis
    double actVx = msgReceived->twist.twist.linear.x;
    double actVy = msgReceived->twist.twist.linear.y;
    double actVz = msgReceived->twist.twist.linear.z;

    // debug prints
    RCLCPP_INFO(this->get_logger(), "actX'%f'",actX);
    RCLCPP_INFO(this->get_logger(), "actY'%f'",actY);
    RCLCPP_INFO(this->get_logger(), "actZ'%f'",actZ);
    RCLCPP_INFO(this->get_logger(), "actVx'%f'",actVx);
    RCLCPP_INFO(this->get_logger(), "actVy'%f'",actVy);
    RCLCPP_INFO(this->get_logger(), "actVz'%f'",actVz);

    // send x speed to Mosaic app
    sendData(txCount, actVx*SCALE_FACTOR);
    txCount++;
  }

  void timer_callback()
    {
      auto leadermsg = control_msgs::msg::LeaderData();
      leadermsg.leader_distance = 100.0;
      leadermsg.leader_velocity = 500.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: distance='%f'", leadermsg.leader_distance);
      RCLCPP_INFO(this->get_logger(), "Publishing: speed='%f'", leadermsg.leader_velocity);
      publisherHunter_->publish(leadermsg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriptionHunter_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<control_msgs::msg::LeaderData>::SharedPtr publisherHunter_;
    size_t count_;
};

int main(int argc, char * argv[]) {
  // initialise the socket(s)
  initSockets();
  
  int choice = 0;
  while (true) {
    std::cout << "[MODE] Select mode" << std::endl;
    std::cout << "[MODE][1] Subscribe to AgileX Hunter (on the same network connection)" << std::endl;
    std::cout << "[MODE][2] Simulate custom speed (without AgileX Hunter)" << std::endl;
    std::cin >> choice;

    if (choice == 1) {
      realSpeed(argc, argv);
    } else if (choice == 2) {
      simulateSpeed();
    }
  }
  
  return 0;
}

void initSockets() {

    std::cout << "[INIT] Creating inSocket" << std::endl;
    inSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (inSocket < 0) {
        std::cout << "[INIT] ERROR in creating inSocket" << std::endl;
    } else {
        std::cout << "[INIT] Created inSocket" << std::endl;
    }

    // listening port and interface (inSocket)
    inAddr.sin_family = AF_INET;
    inAddr.sin_port = htons(IN_PORT);
    inAddr.sin_addr.s_addr = INADDR_ANY;
    std::cout << "[INIT] Binding inSocket" << std::endl;
    if (bind(inSocket, (struct sockaddr*)&inAddr, sizeof(inAddr)) < 0) {
        std::cout << "[INIT] ERROR in binding inSocket" << std::endl;
    } else {
        std::cout << "[INIT] Binded inSocket" << std::endl;
    }

    std::cout << "[INIT] Creating outSocket" << std::endl;
    outSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (outSocket < 0) {
        std::cout << "[INIT] ERROR in creating outSocket" << std::endl;
    } else {
        std::cout << "[INIT] Created outSocket" << std::endl;
    }

    memset(&outAddr, 0, sizeof(outAddr));
    outAddr.sin_family = AF_INET;
    outAddr.sin_port = htons(OUT_PORT);
    inet_pton(AF_INET, OUT_ADDRESS, &(outAddr.sin_addr));

    std::cout << "------------------------------------------------------------------" << std::endl;
}

// to use real speed data, published by the Agilex Hunter
void realSpeed(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
}

// to use simulated speed data, without an Agilex Hunter to work with
void simulateSpeed() {
  int nSpeeds = 0;
  double actVx = 0;
  std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
  std::cout << "[SIMULATE] How many different speeds (integer):" << std::endl;
  std::cin >> nSpeeds;

  double* speeds = new double[nSpeeds];
  int* iterations = new int[nSpeeds];

  for (int i = 0; i < nSpeeds; i++) {
    std::cout << "[SIMULATE][SPEED " << i << "] Insert speed (double):" << std::endl;
    std::cin >> speeds[i];
    std::cout << "[SIMULATE][SPEED " << i << "] Insert number of iterations (integer):" << std::endl;
    std::cin >> iterations[i];
  }

  while (true) {
    for (int i = 0; i < nSpeeds; i++) {
      actVx = speeds[i];
      for (int j = 0; j < iterations[i]; j++) {
        do {
          currentTime = std::chrono::steady_clock::now();
          elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();
        } while (elapsedTime < SIM_RATE);
        
        lastTime = std::chrono::steady_clock::now();

        std::cout << "[SIMULATE][SPEED " << i << "] Sending ros2mosaic" << std::endl;
        std::cout << "[SIMULATE][SPEED " << i << "] txCount=" << txCount << " | actVx=" << actVx << std::endl;

        sendData(txCount, actVx);
        txCount++;
      }
    }
  }
}

// send a packet to Mosaic app
void sendData(int number, double speed) {
    ros2mosaic s;
    s.number = number;
    s.speed = abs(speed);
    sendto(outSocket, &s, sizeof(s), 0, (struct sockaddr*)&outAddr, sizeof(outAddr));
    std::cout << "[HIL][" << txCount << "] SPEED sent (number=" << number << ", speed=" << speed << ") to " << OUT_ADDRESS << ":" << OUT_PORT << std::endl;
    std::cout << "------------------------------------------------------------------" << std::endl;
}

// might be needed in future, with bidirectional communication to and from Mosaic app
/* 
int32_t getNextInt32(char* buffer, int& offset) {
    uint32_t temp = 0;
    memcpy(&temp, buffer+offset, 4);

    offset = offset + 4;
    return temp;
}

double getNextDouble(char* buffer, int& offset) {
    double temp = 0;
    memcpy(&temp, buffer+offset, 8);

    offset = offset + 8;
    return temp;
}

std::string getNextString(char* buffer, int& offset) {
    char nextStringBuffer[1000];
    // setto l'intero buffer a 0
    memset(nextStringBuffer, 0, sizeof(nextStringBuffer));

    // calcolo la lunghezza della stringa corrente vedendo dove si trova il terminatore
    size_t nextStringLength = std::strchr(buffer+offset, '\0') - (buffer+offset);
    // uso string_view per evitare la copia fisica dei caratteri
    std::string_view nextStringView(buffer+offset, nextStringLength);

    // sposto l'offset della lunghezza della stringa corrente + 2 bytes per il carattere '/0'
    offset = offset + nextStringLength + 2;
    return nextStringView.data();
}
*/