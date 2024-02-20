#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
using std::placeholders::_1;


class MergeArraysNode : public rclcpp::Node
{
  public:
    MergeArraysNode() : Node("merge_arrays_node")
    { 
      subscription_array1 = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/input/array1", 10, std::bind(&MergeArraysNode::array1_callback, this, _1));
      subscription_array2 = this->create_subscription<std_msgs::msg::Int32MultiArray>(
      "/input/array2", 10, std::bind(&MergeArraysNode::array2_callback, this, _1));
      merged_publisher = this->create_publisher<std_msgs::msg::Int32MultiArray>("/output/array", 10);
    }

  private:
    std::vector<int32_t> array1 {};
    std::vector<int32_t> array2 {};
    
    // This function loops through each of the vectors from the subscribed topics and compares
    // their current elements. Whichever element is smaller gets added to the output vector
    // and the current element in that vector shifts to the next index.
    void mergeAndSort() const
    {
      int currentIndex1 = 0;
      int currentIndex2 = 0;
      int currentValue = 0;
      std_msgs::msg::Int32MultiArray output {std_msgs::msg::Int32MultiArray()};
      for (int i{ 0 }; i < (size(array1) + size(array2)); i++)
      {
        if (currentIndex2 >= size(array2))
        { // reached end of array 2
            currentValue = array1[currentIndex1];
            currentIndex1++;
        }
        else if (currentIndex1 >= size(array1))
        { // reached end of array 1
            currentValue = array2[currentIndex2];
            currentIndex2++;
        }
        else if (array1[currentIndex1] <= array2[currentIndex2])
        { // current array 1 element is smaller or equal
            currentValue = array1[currentIndex1];
            currentIndex1++;
        }
        else
        { // current array 2 element is smaller
            currentValue = array2[currentIndex2];
            currentIndex2++;
        }
      	output.data.push_back(currentValue);
      }
      merged_publisher->publish(output);
    }
 
    void array1_callback(const std_msgs::msg::Int32MultiArray & msg)
    {
    	array1 = msg.data;
    	mergeAndSort();
    }
    
    void array2_callback(const std_msgs::msg::Int32MultiArray & msg)
    {
    	array2 = msg.data;
    	mergeAndSort();
    }
    
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_array1;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_array2;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr merged_publisher;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeArraysNode>());
  rclcpp::shutdown();
  return 0;
}
