#include "rclcpp/rclcpp.hpp"
#include "sft_interfaces/msg/sft_data.hpp"
//#include "std_msgs/msg/string.hpp"

class SubSample2 : public rclcpp::Node
{
	public:
		SubSample2() : Node("SubSample2")
		{
			subscription_ = this->create_subscription<sft_interfaces::msg::SftData>(
				"topic", 10, std::bind(&SubSample2::topic_callback, this, std::placeholders::_1));
		}

	private:
		void topic_callback(const sft_interfaces::msg::SftData::SharedPtr msg) const
		{
			/////Print recieved data/////
			RCLCPP_INFO(this->get_logger(), "Index,Fx,Fy,Fz,Mx,My,Mz: %d %f %f %f %f %f %f",
											msg->index, msg->data[0], msg->data[1], msg->data[2],
											msg->data[3], msg->data[4], msg->data[5]);
  		}

		rclcpp::Subscription<sft_interfaces::msg::SftData>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubSample2>());
  rclcpp::shutdown();
  return 0;
}
