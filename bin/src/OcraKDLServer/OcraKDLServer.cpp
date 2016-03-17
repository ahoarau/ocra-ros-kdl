#include <ocra-recipes/ControllerServer.h>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/ConnPolicy.hpp>
#include <rtt/Component.hpp>

#include <OcraKDLModel/OcraKDLModel.h>

#include <chrono>
#include <memory>
#include <ros/ros.h>

class OcraRTTServer : public ocra_recipes::ControllerServer , public RTT::TaskContext
{
public:
    OcraRTTServer(const std::string& name):
    ocra_recipes::ControllerServer(ocra_recipes::WOCRA_CONTROLLER,false),
    RTT::TaskContext(name)
    {
        this->addAttribute("solver_elapsed",solver_elapsed);
        // Inputs
        this->ports()->addPort("JointPosition", port_joint_position_in).doc("");
        this->ports()->addPort("JointVelocity", port_joint_velocity_in).doc("");
        
        // Outputs
        this->ports()->addPort("JointTorqueCommand", port_joint_torque_cmd_out).doc("");
    }
    
    void getRobotState(VectorXd& q, VectorXd& qd, Eigen::Displacementd& H_root, Eigen::Twistd& T_root)
    {
        jnt_pos_in_fs = port_joint_position_in.readNewest(q);
        jnt_vel_in_fs = port_joint_velocity_in.readNewest(qd);
    }
    
    std::shared_ptr< ocra::Model > loadRobotModel()
    {
        /*if(bool(getRobotModel()))
            return getRobotModel();
        RTT::log(RTT::Info) << "Creating KDL/ROS Model" << RTT::endlog();
        auto ocra_kdl_model = std::make_shared<OcraKDLModel>(); 
        RTT::log(RTT::Info) << "Configuring KDL/ROS Model" << RTT::endlog();
        if(!ocra_kdl_model->initialize())
        {
            RTT::log(RTT::Fatal) << "Error while configuring KDL/ROS Model" << RTT::endlog();
            return nullptr;
        }
        return ocra_kdl_model;*/
        return std::make_shared<OcraKDLModel>("arm",true);
    }
    
    bool configureHook()
    {
        if(!this->initialize())
        {
            RTT::log(RTT::Fatal) << "Error while initializing Controller Server" << RTT::endlog();
            return false;
        }
        Eigen::VectorXd w_full      = Eigen::VectorXd::Ones(7);
        
        std::string name("fullPostureTask");
    
        featState = new ocra::FullModelState(name + ".FullModelState", *(getRobotModel()), ocra::FullState::INTERNAL);
        featDesState = new  ocra::FullTargetState(name + ".FullTargetState", *(getRobotModel()), ocra::FullState::INTERNAL);

        feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
        featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);
        
        
        task = (getController()->createTask(name, *feat, *featDes));
        
        if(!task) return false;
        
        task->setTaskType(ocra::Task::ACCELERATIONTASK);
        getController()->addTask(task);


        task->setStiffness(150.0);
        task->setDamping(2.*sqrt(150.0));
        task->setWeight(w_full);

        task->activateAsObjective();
        
        jnt_pos_in.resize(7);
        jnt_vel_in.resize(7);
        return true;
    }
    bool startHook()
    {
        return true;
    }
    void updateHook()
    {
        RTT::os::TimeService::ticks  ticks_start = RTT::os::TimeService::Instance()->getTicks();
        const Eigen::VectorXd& tau = this->computeTorques();
        solver_elapsed = RTT::os::TimeService::Instance()->getSeconds(ticks_start);
        const Eigen::VectorXd& G = this->getRobotModel()->getGravityTerms();
        port_joint_torque_cmd_out.write(tau - G);
    }
protected:
    double solver_elapsed;
    RTT::FlowStatus jnt_pos_in_fs,
                    jnt_vel_in_fs;

    RTT::InputPort<Eigen::VectorXd>  port_joint_position_in,
                                     port_joint_velocity_in;

    Eigen::VectorXd jnt_pos_in,
                    jnt_vel_in;

    RTT::OutputPort<Eigen::VectorXd> port_joint_torque_cmd_out;
    std::shared_ptr<ocra::Task>  task;
    ocra::FullModelState*  featState;
    ocra::FullTargetState* featDesState;
        
    ocra::FullStateFeature* feat;
    ocra::FullStateFeature* featDes;
    
};

ORO_CREATE_COMPONENT(OcraRTTServer)
