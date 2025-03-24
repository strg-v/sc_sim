#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <iostream>

namespace gazebo_plugins
{
    
  class PrintPlugin : public gazebo::ModelPlugin
  {
  public:
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      std::cout << "[PrintPlugin] Hello from PrintPlugin! Model: " << model->GetName() << std::endl;
    }
  };
}

GZ_REGISTER_MODEL_PLUGIN(gazebo_plugins::PrintPlugin)