#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/World.hh"
#include "boost/bind.hpp"
#include "stdio.h"


namespace gazebo
{
class Step2 : public ModelPlugin
{
  // Pointer to the model
  private: physics::ModelPtr model;

  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;

  int count = 0;
  struct position {
      float x;
      float y;
      float z;
      float h;
      float q0;
      float q1;
      float q2;
      float q3;
  };

  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&Step2::OnUpdate, this, _1));
  }


  // Called by the world update start event
  public: void OnUpdate(const common::UpdateInfo & /*_info*/)
  {
    count++;

/*
    printf("%d\n",count);

    printf(GetSimTime());
    printf(GetRealTime());

    math::Pose wpos = model->GetWorldPose();
    position.x = wpos.pos.x;
    position.y = wpos.pos.y;
    position.z = wpos.pos.z;
    position.h = wpos.pos.h;
    position.q0 = wpos.rot.w;
    position.q1 = wpos.rot.x;
    position.q2 = wpos.rot.y;
    position.q3 = wpos.rot.z;

    printf("Posicion = ("+position.x+", "+position.y+", "+position.z+", "+position.h+", "+
                        position.q0+", "+position.q1+", "+position.q2+", "+position.q3+").\n");
*/

    if(count < 1000){
        this->model->SetLinearVel(math::Vector3(0, 0, 0));
    }else if(count < 2000){
        this->model->SetLinearVel(math::Vector3(0.5, 0, 0));
    }else if(count < 3000){
        this->model->SetLinearVel(math::Vector3(0, 0.5, 0));
    }else if(count < 4000){
        this->model->SetLinearVel(math::Vector3(-0.5, 0, 0));
    }else if(count < 5000){
        this->model->SetLinearVel(math::Vector3(0, -0.5, 0));
    }else{
        count=0;
    };

  }

};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Step2)
}
