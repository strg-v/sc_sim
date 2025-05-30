#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <spacecraft_msgs/msg/thrust_command.hpp>
#include <ignition/math/Vector3.hh>
#include <thread>

static std::mutex print_mutex;


namespace gazebo_plugins
{
    class ApplyForcePlugin : public gazebo::ModelPlugin
    {
        private:

        gazebo::physics::ModelPtr model_;
        gazebo::physics::LinkPtr link_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<spacecraft_msgs::msg::ThrustCommand>::SharedPtr sub_;
        std::thread rclcpp_thread_;

        gazebo::event::ConnectionPtr update_connection_;
        gazebo::common::Time end_time_;
        gazebo::physics::WorldPtr world_;

        std::string plugin_name = "[ApplyForcePlugin] ";
        std::string thruster_name;

        ignition::math::Vector3d mounting_point;
        ignition::math::Vector3d possible_direction;
        double force;
        float min_duration;
        bool fire = false;
        unsigned int counter = 0;

        double remaining_impulse = 0.0;
        gazebo::common::Time last_sim_time;
        

        ignition::math::Vector3d current_force;

        bool currently_firing;
        bool previously_firing;

        // Effect publisher
        gazebo::transport::PublisherPtr visPub;

        public:
        
        void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
        {
            std::cout << this->plugin_name << "Hello from ApplyForcePlugin! Model: " << model->GetName() << std::endl;
            
            if(!this->check_elements_of_model(sdf))
            {
                return;
            }
            
            this->current_force = ignition::math::Vector3d(0.0, 0.0, 0.0);
            this->world_ = model->GetWorld();
            currently_firing = false;
            previously_firing = false;

            auto links = model->GetLinks();
            if (links.empty()) {
                std::cerr << "[ApplyForcePlugin] ERROR: No links in model.\n";
                return;
            } else {
                std::cout << "[ApplyForcePlugin] Found links:\n";
                for (const auto& link : links) {
                    std::cout << " - " << link->GetName() << std::endl;
                }
            }

            // Try fallback: use first link if base_link is not found
            this->link_ = model->GetLink("base_link");
            if (!this->link_) {
                std::cerr << "[ApplyForcePlugin] WARNING: 'base_link' not found. Using first available link instead.\n";
                this->link_ = links.front();
                if (this->link_) {
                    std::cout << "[ApplyForcePlugin] Fallback link: " << this->link_->GetName() << "\n";
                } else {
                    std::cerr << "[ApplyForcePlugin] ERROR: Still no link found. Plugin won't work.\n";
                    return;
                }
            } else {
                std::cout << "[ApplyForcePlugin] Successfully attached to base_link\n";
            }

            int argc = 0;
            char** argv = nullptr;
            if (!rclcpp::ok()) {
                rclcpp::init(argc, argv);
            }

            this->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
                std::bind(&ApplyForcePlugin::OnUpdate, this)
            );

            gazebo::transport::NodePtr gz_tramsport_node(new gazebo::transport::Node());
            gz_tramsport_node->Init(); 

            visPub = gz_tramsport_node->Advertise<gazebo::msgs::Visual>("~/visual");
            
            node_ = rclcpp::Node::make_shared("apply_force_plugin_" + model->GetName() + "_" + thruster_name);
            sub_ = node_->create_subscription<spacecraft_msgs::msg::ThrustCommand>(
                "/spacecraft/thruster/" + thruster_name, 10,
                std::bind(&ApplyForcePlugin::OnForceMsg, this, std::placeholders::_1));

            rclcpp_thread_ = std::thread([this]() {
                std::cout << "[ApplyForcePlugin] ROS 2 spinning thread started.\n";
                rclcpp::spin(node_);
            });

            std::cout << "[ApplyForcePlugin] Loaded and listening on /spacecraft/thruster/" << thruster_name << std::endl;
        }

        int clamp_direction(double v)
        {
            if (v > 0.0) return 1;
            if (v < 0.0) return -1;
            return 0;
        }

        void OnForceMsg(const spacecraft_msgs::msg::ThrustCommand::SharedPtr msg)
        {

            //std::cout << "[ApplyForcePlugin] Received Message: x: " << msg->direction_selection.x << " y: " << msg->direction_selection.y << " z: " << msg->direction_selection.z << std::endl;
            

            rclcpp::Duration ros_duration = msg->duration;
            double duration_sec = ros_duration.seconds();
            
            if(duration_sec <this->min_duration){
                return;
            }

            gazebo::common::Time gz_duration(duration_sec);            

            this->current_force = this->possible_direction * this->force;

            RCLCPP_INFO(this->node_->get_logger(), 
            "Force: [%.6f, %.6f, %.6f], Duration: %.3e", 
            this->current_force.X(),
            this->current_force.Y(),
            this->current_force.Z(),
            duration_sec
            
            );

            auto sim_time = this->world_->SimTime();
            this->end_time_ = sim_time + gz_duration;

            this->counter += (int)round(duration_sec/0.001);

            RCLCPP_INFO(this->node_->get_logger(), "Force applied to %s %d times", this->thruster_name.c_str(), this->counter);

        }

        void OnUpdate()
        {

            auto sim_time = world_->SimTime();
            //if(sim_time < this->end_time_)
            if(this->counter > 0)
            {
                this->counter--;
                this->fire = false;

                ignition::math::Quaterniond rot = link_->WorldPose().Rot();

                // Rotate body force into world force
                ignition::math::Vector3d world_force = rot.RotateVector(this->current_force);

                this->link_->AddForceAtRelativePosition(world_force, this->mounting_point);

                //this->link_->AddLinkForce(this->current_force, this->mounting_point);

                // Firing effect
                if(!this->currently_firing)
                {
                    ignition::math::Vector3d world_position = link_->WorldPose().CoordPositionAdd(this->mounting_point);
                    ignition::math::Vector3d force_dir = this->current_force.Normalized();
                    ignition::math::Vector3d offset_mounting_point = this->mounting_point - force_dir * 0.15;


                    ignition::math::Quaterniond force_quat;
                    force_quat.From2Axes(ignition::math::Vector3d::UnitZ, force_dir);

                    gazebo::msgs::Visual visMsg;
                    visMsg.set_name("spacecraft::base_link::cone_visual_" + this->thruster_name); // Full scoped name
                    visMsg.set_parent_name("spacecraft::base_link");
                    gazebo::msgs::Set(visMsg.mutable_pose(), ignition::math::Pose3d(offset_mounting_point, force_quat));
                    visMsg.set_visible(true);
                    visMsg.set_delete_me(false);
                    visPub->Publish(visMsg);

                }

                this->currently_firing = true;

            } else 
            {
                this->currently_firing = false;
                this->end_time_ = this->world_->SimTime();


                // Deactivate visual effect
                if(this->previously_firing == true)
                {
                    gazebo::msgs::Visual visMsg;
                    visMsg.set_name("spacecraft::base_link::cone_visual_" + this->thruster_name); // Full scoped name
                    visMsg.set_parent_name("spacecraft::base_link");
                    visMsg.set_visible(false);
                    visMsg.set_delete_me(false);
                    visPub->Publish(visMsg);
                }

            }

            this->previously_firing = this->currently_firing;


        }

        ~ApplyForcePlugin() override
        {
            rclcpp::shutdown();
            if(rclcpp_thread_.joinable())
            {
                rclcpp_thread_.join();
            }
        }

        private:

        bool check_elements_of_model(sdf::ElementPtr sdf)
        {
            if(!sdf->HasElement("thruster_name"))
            {
                std::cout << this->plugin_name << "Missing 'thruster_name' field in plugin. Abort." << std::endl;
                return false;
            } else {
                this->thruster_name = sdf->Get<std::string>("thruster_name");
                std::cout << this->plugin_name << "Found thruster name: " << this->thruster_name << std::endl;
            }

            if(!sdf->HasElement("mounting_point"))
            {
                std::cout << this->plugin_name << "Missing 'mounting_point' field in plugin. Abort." << std::endl;
                return false;
            } else {
                this->mounting_point = sdf->Get<ignition::math::Vector3d>("mounting_point");
                std::cout << this->plugin_name << "Found mounting point: [" << this->mounting_point << "]" << std::endl;
            }

            if(!sdf->HasElement("possible_directions"))
            {
                std::cout << this->plugin_name << "Missing 'possible_directions' field in plugin. Abort." << std::endl;
                return false;
            } else {
                this->possible_direction = sdf->Get<ignition::math::Vector3d>("possible_directions");
                std::cout << this->plugin_name << "Found possible directions: [" << this->possible_direction << "]" << std::endl;
            }

            if(!sdf->HasElement("force"))
            {
                std::cout << this->plugin_name << "Missing 'force' field in plugin. Abort." << std::endl;
                return false;
            } else {
                this->force = sdf->Get<float>("force");
                std::cout << this->plugin_name << "Found force: " << this->force << std::endl;
            }

            if(!sdf->HasElement("min_duration"))
            {
                std::cout << this->plugin_name << "Missing 'min_duration' field in plugin. Abort." << std::endl;
                return false;
            } else {
                this->min_duration = sdf->Get<float>("min_duration");
                std::cout << this->plugin_name << "Found min_duration: " << this->min_duration << std::endl;
            }

            return true;
        }

    };

}

GZ_REGISTER_MODEL_PLUGIN(gazebo_plugins::ApplyForcePlugin)
