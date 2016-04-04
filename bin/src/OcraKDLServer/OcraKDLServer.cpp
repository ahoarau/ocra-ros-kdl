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
#include <yarp/os/Network.h>

class OcraRTTServer : public ocra_recipes::ControllerServer , public RTT::TaskContext
{
public:
    OcraRTTServer(const std::string& name):
    ocra_recipes::ControllerServer(ocra_recipes::WOCRA_CONTROLLER,ocra_recipes::QPOASES,true),
    RTT::TaskContext(name)
    {
        this->addAttribute("solver_elapsed",solver_elapsed);
        this->addAttribute("task_xml_path",task_xml_path);
        this->addAttribute("solver_type",solver_type);
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
        return std::make_shared<OcraKDLModel>("arm",true);
    }
    
    bool configureHook()
    {
        if(!this->initialize())
        {
            RTT::log(RTT::Fatal) << "Error while initializing Controller Server" << RTT::endlog();
            return false;
        }
        std::cout  << "Loading tasks from ["<<task_xml_path<<"]" << std::endl;
        return this->addTaskManagersFromXmlFile(task_xml_path);
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
    std::string task_xml_path;
    ocra_recipes::SOLVER_TYPE solver_type;
    yarp::os::Network yarp;
    
    
};

ORO_CREATE_COMPONENT(OcraRTTServer)
