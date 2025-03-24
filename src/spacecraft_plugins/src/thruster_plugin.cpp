#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <thread>
#include <ignition/math/Vector3.hh>

namespace gazebo_plugins
{

    class ThrusterPlugin : public gazebo::ModelPlugin
    {
        public:

        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            auto plugin_name = "[ThrusterPlugin] ";
            std::cout << plugin_name << "Starting Thruster Plugin: " << model->GetName() << std::endl;

            auto links = model->GetLinks();
            if (links.empty()) {
                std::cerr << "[ApplyForcePlugin] ERROR: No links in model.\n";
            } else {
                std::cout << "[ApplyForcePlugin] Found links:\n";
                for (const auto& link : links) {
                    std::cout << " - " << link->GetName() << std::endl;
                }
            }

            std::cout << plugin_name << "Finished Loading.";

        }

    };

}

GZ_REGISTER_MODEL_PLUGIN(gazebo_plugins::ThrusterPlugin)