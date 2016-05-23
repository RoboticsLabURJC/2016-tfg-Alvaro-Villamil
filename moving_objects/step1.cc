#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include <ignition/math/Pose3.hh>

namespace gazebo
{
class Step1 : public WorldPlugin
{
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    sdf::SDF cylinderSDF;
    cylinderSDF.SetFromString(
       "<sdf version ='1.6'>\
          <model name ='cylinder1'>\
            <pose>0 0 0 0 0 0</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <cylinder><radius>.5</radius><length>1</length></cylinder>\
                </geometry>\
              </collision>\
              <visual name ='visual'>\
                <geometry>\
                  <cylinder><radius>.5</radius><length>1</length></cylinder>\
                </geometry>\
              </visual>\
            </link>\
          </model>\
        </sdf>");
    sdf::ElementPtr model = cylinderSDF.Root()->GetElement("model");
    model->GetAttribute("name")->SetFromString("unique_cylinder");
    _parent->InsertModelSDF(cylinderSDF);
  }

};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Step1)
}
