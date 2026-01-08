#include <rclcpp/rclcpp.hpp>
#include <bumperbot_msgs/srv/add_two_ints.hpp>
#include <chrono>
#include <functional> // Required for std::bind

using namespace std::chrono_literals;
using std::placeholders::_1;


class SimpleServiceClient : public rclcpp::Node
{
    public:
        SimpleServiceClient(int a, int b) : Node("simple_service_client")
        {
            client_ = create_client<bumperbot_msgs::srv::AddTwoInts>("add_two_ints");

            auto request = std::make_shared<bumperbot_msgs::srv::AddTwoInts::Request>();
            request->a = a;
            request->b = b;

            // FIX: The while loop condition was reversed. It should wait WHILE the service is NOT available.
            while (!client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service");
                    return;
                }
                RCLCPP_INFO_STREAM(get_logger(), "Service not available, waiting again.... ");
            }

            // The async_send_request call is correct, it sends the request and returns a future.
            // The bind call is also correct, but the responseCallback signature needs to match the future.
            client_->async_send_request(request, std::bind(&SimpleServiceClient::responseCallback, this, _1));
        }

    private:
        rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedPtr client_;

        // FIX: The signature of the callback must be corrected to take a std::shared_future of the response.
        void responseCallback(rclcpp::Client<bumperbot_msgs::srv::AddTwoInts>::SharedFuture future)
        {
            // The future is valid only if the service call was successful.
            // We can check if the future holds a result to see if the call was successful.
            if (future.valid()) {
                RCLCPP_INFO_STREAM(get_logger(), "Service response: " << future.get()->sum);
            } else {
                RCLCPP_ERROR(get_logger(), "Service call failed!");
            }
            rclcpp::shutdown();
        }
};


int main(int argc, char* argv[]){

    if (argc != 3){
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Wrong no. of arguments! Usage: simple_service_client A B");
        return 1;
    }

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceClient>(atoi(argv[1]), atoi(argv[2]));
    rclcpp::spin(node);

    return 0;
}
