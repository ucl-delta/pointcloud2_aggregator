#include "aggregator.hpp"

#include <iostream>
#include <vector>
#include <map>
#include <regex>
#include "rclcpp_components/component_manager.hpp"
#include "composition_interfaces/srv/load_node.hpp"
#include "composition_interfaces/srv/unload_node.hpp"
#include "rcl_interfaces/msg/parameter.hpp"

using namespace std::chrono_literals;

namespace pointcloud2 {
    class AggregateMonitorLoader: public rclcpp::Node
    {
        public:
            AggregateMonitorLoader();
            
        private:
            void timer_callback();
            void request_callback(rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedFuture future);
            void load_component(std::string topic);
            void unload_component(uint64_t id);

            double topic_read_freq;
            const std::string filter_regex;
            std::map<std::string, uint64_t> currently_running;
            std::vector<std::string> requested_ids;

            rclcpp::TimerBase::SharedPtr receive_timer;
            rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedPtr component_load_client;
            rclcpp::Client<composition_interfaces::srv::UnloadNode>::SharedPtr component_unload_client;
    };

    AggregateMonitorLoader::AggregateMonitorLoader():
        Node("pointcloud2_aggregator_monitor")
    {
        // Declare Parameters
        this->declare_parameter("node_config", "");
        this->declare_parameter("topic_read_interval", 5.0); // seconds 
        this->declare_parameter("lidar_topic_regex", "^\\/livox\\/lidar_(?!.*aggregate).*$"); // By default only take expressions matching /livox/lidar_* but exclude strings that include the wod aggregate anywhere

        // Start the clients
        this->component_load_client = this->create_client<composition_interfaces::srv::LoadNode>("ComponentManager/_container/load_node");
        this->component_unload_client = this->create_client<composition_interfaces::srv::UnloadNode>("ComponentManager/_container/unload_node");
        while(!this->component_load_client->wait_for_service(1s) ||
              !this->component_unload_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting on the service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }

        // Start the timer callback
        double topic_read_interval = this->get_parameter("topic_read_interval").as_double();
        this->receive_timer = this->create_wall_timer(
            std::chrono::duration<float>(topic_read_interval), std::bind(&AggregateMonitorLoader::timer_callback, this));
    }

    void AggregateMonitorLoader::timer_callback() 
    {
        // RCLCPP_INFO(this->get_logger(), "----------------Timer Callback----------------");

        if(this->requested_ids.size() > 0) {
            RCLCPP_INFO(this->get_logger(), "There are existing service requested, skipping this loop");
            for(const auto& v: this->requested_ids) {
                RCLCPP_INFO(this->get_logger(), "%s", v.c_str());
            }
            return;
        }

        std::regex pattern(this->get_parameter("lidar_topic_regex").as_string());
        std::vector<std::string> filtered_topics;

        auto service_names = this->get_topic_names_and_types();
        for(const auto& entry: service_names) {
            // RCLCPP_INFO(this->get_logger(), "Topics: %s, Types:", entry.first.c_str());
            // for (const auto& type: entry.second) {
            //     RCLCPP_INFO(this->get_logger(), "\ttype: %s", type.c_str());
            // }
            if (std::regex_match(entry.first, pattern)) {
                filtered_topics.push_back(entry.first);
            }
        }

        // Stop old topic components where they are no longer in the filtered topics, but were still in currently running
        std::vector<std::string> to_remove;
        for (const auto& running_topic: this->currently_running) {
            if (std::find(filtered_topics.begin(), filtered_topics.end(), running_topic.first) == filtered_topics.end()) {
                to_remove.push_back(running_topic.first);

                // Call service to unload component with id running_topic.second
                this->unload_component(running_topic.second);
                RCLCPP_INFO(this->get_logger(), "Removed Topic for Aggregation: %s", running_topic.first.c_str());
            }
        }
        for (const auto& topic: to_remove) {
            // Remove topic from list
            this->currently_running.erase(topic);
        }
        
        
        // Start new topics
        for (const auto& topic: filtered_topics) {
            // RCLCPP_INFO(this->get_logger(), "Filtered Topics: %s", topic.c_str());
            if(this->currently_running.find(topic) == this->currently_running.end()) {
                // If topic is not currently running, load component and add to map
                this->load_component(topic);
                RCLCPP_INFO(this->get_logger(), "Initialise Topic for Aggregation: %s", topic.c_str());
            } 
        }

    }

    void AggregateMonitorLoader::request_callback(rclcpp::Client<composition_interfaces::srv::LoadNode>::SharedFuture future) {
        if(future.get()) {
            auto response = future.get(); // Type composition_interfaces/srv/LoadNode
            if(response->success) {
                std::string full_name = response->full_node_name;
                size_t lastSlashPos = full_name.find_last_of('/');
                auto topic_name = full_name.substr(lastSlashPos+1);

                std::string full_topic_name = "";
                size_t index = 0;
                for(index; index < this->requested_ids.size(); index++) {
                // for(const auto& str: this->requested_ids) {
                    const auto& str = this->requested_ids[index];
                    if(str.find(topic_name) != std::string::npos) {
                        full_topic_name = str;
                        break;
                    }
                }

                if(full_topic_name == "") {
                    RCLCPP_ERROR(this->get_logger(), "Something weird happened, received response for a non-requested topic %s", full_name.c_str());
                    return;
                }

                if(index < this->requested_ids.size()) {
                    this->requested_ids.erase(this->requested_ids.begin() + index);
                }

                this->currently_running.insert({full_topic_name, response->unique_id});
                RCLCPP_INFO(this->get_logger(), "Service response succeeded for %s", topic_name.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "Service response failed: %s", response->error_message.c_str());
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "Service call failed");
        }
    }

    void AggregateMonitorLoader::load_component(std::string topic) {

        size_t lastSlashPos = topic.find_last_of('/');
        
        auto request = std::make_shared<composition_interfaces::srv::LoadNode::Request>();
        request->package_name = "pointcloud2_aggregator";
        request->plugin_name = "pointcloud2::Pointcloud2_Aggregator";
        request->node_name = topic.substr(lastSlashPos + 1); // String is anything after the last slash
        
        std::string node_config = this->get_parameter("node_config").as_string();
        if(node_config != ""){
            std::vector<rcl_interfaces::msg::Parameter> param;
            // auto config_file_param = rcl_interfaces::msg::Parameter();
            // config_file_param.name = node_config;
            auto config_file_param = rcl_interfaces::msg::Parameter();
            config_file_param.name = "pointcloud_topic";
            config_file_param.value.type = 4;
            config_file_param.value.string_value = topic;
            param.push_back(config_file_param);
            request->parameters = param;
        }
        this->requested_ids.push_back(topic);

        auto future = this->component_load_client->async_send_request(
            request, 
            std::bind(&AggregateMonitorLoader::request_callback, this, std::placeholders::_1));
    }

    void AggregateMonitorLoader::unload_component(uint64_t id) {
        
        auto request = std::make_shared<composition_interfaces::srv::UnloadNode::Request>();
        request->unique_id = id;

        auto future = this->component_unload_client->async_send_request(request);
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Must be started with a component manager
  //   auto manager = std::make_shared<rclcpp_components::ComponentManager>(exec);  
  //   exec->add_node(manager);

  auto monitor = std::make_shared<pointcloud2::AggregateMonitorLoader>();
  exec->add_node(monitor);

  exec->spin();

  rclcpp::shutdown();

  return 0;
}