#include <ocra/control/Model.h>
#include <OcraKDLModel/OcraKDLModel.h>
#include <ocra/optim/OneLevelSolver.h>
#include <ocra/control/Controller.h>
#include <wocra/WocraController.h>
#include <ocra/control/TaskManagers/TaskManager.h>
#include <ocra/control/TaskManagers/FullPostureTaskManager.h>
#include <Eigen/Dense>
#include <memory>
#include <ros/ros.h>
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std;

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "ocra_kdl_server");
    
    cout << "Creating Ocra Model" << endl;
    OcraKDLModel model;

    
    cout << "Creating Internal Solver" << endl;
    ocra::OneLevelSolverWithQuadProg internalSolver;
    const bool useReducedProblem = false;
    
    cout << "Creating Controller" << endl;
    wocra::WocraController ctrl("icubControl", 
                                model, 
                                internalSolver, 
                                useReducedProblem);
 
    
    Eigen::VectorXd q_full      = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd q           = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd qd          = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd w_full      = Eigen::VectorXd::Ones(7);

    const bool usesYARP = false;
    cout << "Creating TaskManagers" << endl;
    //std::map<std::string,std::shared_ptr<ocra::TaskManager>> taskManagers;
    //taskManagers["fullPostureTask"] = std::make_shared<ocra::FullPostureTaskManager>(*ctrl, *model, "fullPostureTask", ocra::FullState::INTERNAL, 5.0, 2*sqrt(5.0), w_full, q_full, usesYARP);
    
    std::string name("fullPostureTask");

   /* featState = new ocra::FullModelState(name + ".FullModelState", model, fullStateType);
    featDesState = new ocra::FullTargetState(name + ".FullTargetState", model, fullStateType);
    feat = new ocra::FullStateFeature(name + ".FullStateFeature", *featState);
    featDes = new ocra::FullStateFeature(name + ".FullStateFeature_Des", *featDesState);*/
    
    
    
    ocra::FullModelState            featState(name + ".FullModelState", model, ocra::FullState::INTERNAL);
    ocra::FullTargetState           featDesState(name + ".FullTargetState", model, ocra::FullState::INTERNAL);
    
    ocra::FullStateFeature          feat(name + ".FullStateFeature", featState);
    ocra::FullStateFeature          featDes(name + ".FullStateFeature_Des", featDesState);
    
    

    
    // The feature initializes as Zero for posture
    std::shared_ptr<ocra::Task>  task;
    task = (ctrl.createTask(name, feat, featDes));
    
    if(task)
        cout << "Youpi task created "<<endl;
    
    task->setTaskType(ocra::Task::ACCELERATIONTASK);
    ctrl.addTask(task);


    task->setStiffness(5.0);
    task->setDamping(2.*sqrt(5.0));
    task->setWeight(w_full);

    task->activateAsObjective();
    
    
    
    Eigen::VectorXd torque_out = Eigen::VectorXd::Zero(7);

    std::chrono::time_point<std::chrono::system_clock> start, end;
    std::chrono::duration<double> elapsed_seconds;
    
    while(true)
    {
        
        start = std::chrono::system_clock::now();
        
        
        
        // q = lwr.readJoints()
        // qdot = lwr.readQDOT()
        // model.updateinternalKDLState(q,qdot);
        // model.computeMassmatrix();
        cout << "Update State !" << endl;
        model.setState(q,qd);
        
        start = std::chrono::system_clock::now();
        
        ctrl.computeOutput(torque_out);
        torque_out -= model.getGravityTerms();
        std::cout << "Torque out : "<<torque_out.transpose()<<std::endl;
        
        //lwr.sendtorque(torque_out);
        
        
        end = std::chrono::system_clock::now();
 
        elapsed_seconds = end-start;
        std::time_t end_time = std::chrono::system_clock::to_time_t(end);
    
        std::cout << "finished computation at " << std::ctime(&end_time)
                << "elapsed time: " << elapsed_seconds.count() << "s\n";
    }
    return 0;
}
