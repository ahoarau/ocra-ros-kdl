#include <OcraKDLModel/OcraKDLModel.h>
#include <map>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Lgsm>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <boost/scoped_ptr.hpp>
#include <rtt_ros_kdl_tools/chain_utils.hpp>

#include <ros/ros.h>
using namespace rtt_ros_kdl_tools;
//=================================  Class methods  =================================//
OcraKDLModel::OcraKDLModel(const std::string& robot_name,bool initialize):
ocra::Model(robot_name, getNumberOfJointsFromROSParamURDF(), false) //TODO : make a new ()
{
    if(initialize)
        this->initialize();
}
bool OcraKDLModel::initialize()
{
    chain = std::make_shared<rtt_ros_kdl_tools::ChainUtils>(false);

    if(!chain->init())
    {
        std::cerr << "Error while creating the KDL chain" << std::endl;
        return false;
    }

    this->nb_segments = chain->getNrOfSegments();
    this->nb_joints = chain->getNrOfJoints();
    M_inv.resize(chain->getNrOfJoints(),chain->getNrOfJoints());
    M_inv.setZero();
    jl_low.resize(chain->getNrOfJoints());
    jl_up.resize(chain->getNrOfJoints());
    jl_up.setZero();
    jl_low.setZero();
    eigen_vector_3_zero.setZero();
    eigen_matrix_zero.setZero(chain->getNrOfJoints(),chain->getNrOfJoints());
    eigen_3_dyn_zero.setZero(3,chain->getNrOfJoints());
    joint_torques_.setZero(chain->getNrOfJoints());
    jac_tmp.setZero(6,chain->getNrOfJoints());
    this->actuated_dofs = Eigen::VectorXd::Ones(nb_joints);
    eigen_vector_zero = Eigen::VectorXd::Zero(nb_joints);
    return true;
}

const Eigen::VectorXd& OcraKDLModel::getJointTorques() const
{
    return joint_torques_;
}

void OcraKDLModel::doSetState(const Eigen::VectorXd& q, const Eigen::VectorXd& q_dot)
{
    chain->updateModel();
}

int OcraKDLModel::nbSegments() const
{
    return chain->getNrOfSegments();
}

const Eigen::VectorXd& OcraKDLModel::getActuatedDofs() const
{
    return this->actuated_dofs;
}

const Eigen::VectorXd& OcraKDLModel::getJointLowerLimits() const
{
    jl_low = Eigen::Map<Eigen::VectorXd>(chain->getJointLowerLimits().data(),chain->getNrOfJoints());
    return jl_low;
}

const Eigen::VectorXd& OcraKDLModel::getJointUpperLimits() const
{
    jl_up = Eigen::Map<Eigen::VectorXd>(chain->getJointUpperLimits().data(),chain->getNrOfJoints());
    return jl_up;
}

const Eigen::VectorXd& OcraKDLModel::getJointPositions() const
{
    return chain->getJointPositions().data;
}

const Eigen::VectorXd& OcraKDLModel::getJointVelocities() const
{
    return chain->getJointVelocities().data;
}

const std::string& OcraKDLModel::getJointName(int index) const
{
    return chain->getSegment(index).getJoint().getName();
}

const int OcraKDLModel::getSegmentIndex(const std::string& segmentName) const
{
    return chain->getSegmentIndex(segmentName);
}

const Eigen::Displacementd& OcraKDLModel::getFreeFlyerPosition() const
{
    return this->eigen_displacement_zero;
}

const Eigen::Twistd& OcraKDLModel::getFreeFlyerVelocity() const
{
    return this->eigen_twist_zero;
}

const Eigen::MatrixXd& OcraKDLModel::getInertiaMatrix() const
{

    return chain->getInertiaMatrix().data;
}

const Eigen::MatrixXd& OcraKDLModel::getInertiaMatrixInverse() const
{
    M_inv = chain->getInertiaMatrix().data.inverse();
    return M_inv;
}

const Eigen::MatrixXd& OcraKDLModel::getDampingMatrix() const
{
    return eigen_matrix_zero;
}

const Eigen::VectorXd& OcraKDLModel::getNonLinearTerms() const
{
    return chain->getCoriolisTorque().data;
}

const Eigen::VectorXd& OcraKDLModel::getLinearTerms() const
{
    return eigen_vector_zero;
}

const Eigen::VectorXd& OcraKDLModel::getGravityTerms() const
{
    return chain->getGravityTorque().data;
}

double OcraKDLModel::getMass() const
{
    double mass = 0.0;
    for(int i=0;i<chain->getNrOfSegments();i++)
        mass += chain->getSegment(i).getInertia().getMass();
    return mass;
}

const Eigen::Vector3d& OcraKDLModel::getCoMPosition() const
{
    return eigen_vector_3_zero;
}

const Eigen::Vector3d& OcraKDLModel::getCoMVelocity() const
{
    return eigen_vector_3_zero;
}

const Eigen::Vector3d& OcraKDLModel::getCoMAngularVelocity() const
{
    return eigen_vector_3_zero;
}

const Eigen::Vector3d& OcraKDLModel::getCoMJdotQdot() const
{
    return eigen_vector_3_zero;
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& OcraKDLModel::getCoMJacobian() const
{
    return eigen_3_dyn_zero;
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& OcraKDLModel::getCoMAngularJacobian() const
{
    return eigen_3_dyn_zero;
}

const Eigen::Matrix<double,3,Eigen::Dynamic>& OcraKDLModel::getCoMJacobianDot() const
{
    return eigen_3_dyn_zero;
}

const Eigen::Displacementd& OcraKDLModel::getSegmentPosition(int index) const
{
    kdl_tools::KDLFrameToEigenDispd(this->chain->getSegmentPosition(index),seg_pose_eigen);
    return seg_pose_eigen;
}

const Eigen::Twistd& OcraKDLModel::getSegmentVelocity(int index) const
{
    kdl_tools::KDLTwistToOcraTwistVector(this->chain->getSegmentVelocity(index),twist_ocra);
    return twist_ocra;
}

double OcraKDLModel::getSegmentMass(int index) const
{
    return chain->getSegment(index).getInertia().getMass();
}

const Eigen::Vector3d& OcraKDLModel::getSegmentCoM(int index) const
{
    KDL::Vector cog = chain->getSegment(index).getInertia().getCOG();
    eigen_cog << cog.x(),cog.y(),cog.z();
    return eigen_cog;
}

const Eigen::Matrix<double,6,6>& OcraKDLModel::getSegmentMassMatrix(int index) const
{
    return eigen_matrix_6_6_zero;
}

const Eigen::Vector3d& OcraKDLModel::getSegmentMomentsOfInertia(int index) const
{
    return eigen_vector_3_zero;
}

const Eigen::Rotation3d& OcraKDLModel::getSegmentInertiaAxes(int index) const
{
    return eigen_rotation_zero;
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OcraKDLModel::getSegmentJacobian(int index) const
{
    return kdl_tools::KDLJacobianToOcraJacobian(chain->getSegmentJacobian(index),jac_tmp);
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OcraKDLModel::getSegmentJdot(int index) const
{
    return kdl_tools::KDLJacobianToOcraJacobian(chain->getSegmentJdot(index),jac_tmp);;
}

const Eigen::Matrix<double,6,Eigen::Dynamic>& OcraKDLModel::getJointJacobian(int index) const
{
    return kdl_tools::KDLJacobianToOcraJacobian(chain->getSegmentJacobian(index),jac_tmp);;
}

const Eigen::Twistd& OcraKDLModel::getSegmentJdotQdot(int index) const
{
    const KDL::Twist& jdotqdot = this->chain->getSegmentJdotQdot(index);
    kdl_tools::KDLTwistToOcraTwistVector(jdotqdot,twist_ocra);
    return twist_ocra;
}

void OcraKDLModel::doSetJointPositions(const Eigen::VectorXd& q)
{
    chain->setJointPositions(q);
}

void OcraKDLModel::doSetJointVelocities(const Eigen::VectorXd& qdot)
{
    chain->setJointVelocities(qdot);
}

void OcraKDLModel::doSetFreeFlyerPosition(const Eigen::Displacementd& Hroot)
{
}

void OcraKDLModel::doSetFreeFlyerVelocity(const Eigen::Twistd& Troot)
{
}

int OcraKDLModel::doGetSegmentIndex(const std::string& name) const
{
    return this->chain->getSegmentIndex(name);
}

int OcraKDLModel::doGetDofIndex(const std::string &name) const
{
    return this->chain->getSegmentIndex(name);
}

const std::string& OcraKDLModel::doGetDofName(int index) const
{
    return chain->getSegmentName(index);
}


const std::string& OcraKDLModel::doGetSegmentName(int index) const
{
    return this->chain->getSegment(index).getName();
}

const std::string OcraKDLModel::doSegmentName(const std::string& name) const
{
    return name;
}

const std::string OcraKDLModel::doDofName(const std::string& name) const
{
    return name;
}


void OcraKDLModel::printAllData()
{
    std::cout<<"nbSegments:\n";
    std::cout<<nbSegments()<<"\n";

    std::cout<<"nbDofs:\n";
    std::cout<<nbDofs()<<std::endl;

    std::cout<<"nbInternalDofs:\n";
    std::cout<<nbInternalDofs()<<std::endl;

    std::cout<<"actuatedDofs:\n";
    std::cout<<getActuatedDofs()<<"\n";

    std::cout<<"lowerLimits:\n";
    std::cout<<getJointLowerLimits()<<"\n";

    std::cout<<"upperLimits:\n";
    std::cout<<getJointUpperLimits()<<"\n";

    std::cout<<"q:\n";
    std::cout<<getJointPositions().transpose()<<"\n";

    std::cout<<"dq:\n";
    std::cout<<getJointVelocities().transpose()<<"\n";

    std::cout<<"Hroot:\n";
    std::cout<<getFreeFlyerPosition()<<"\n";

    std::cout<<"Troot:\n";
//    std::cout<<getFreeFlyerVelocity().transpose()<<"\n";

    std::cout<<"total_mass:\n";
    std::cout<<getMass()<<"\n";

    std::cout<<"M:\n";
    std::cout<<getInertiaMatrix()<<"\n";

    std::cout<<"Minv:\n";
    std::cout<<getInertiaMatrixInverse()<<"\n";

    std::cout<<"B:\n";
    std::cout<<getDampingMatrix()<<"\n";

    std::cout<<"n:\n";
    std::cout<<getNonLinearTerms()<<"\n";

    std::cout<<"g:\n";
    std::cout<<getGravityTerms()<<"\n";

    std::cout<<"l:\n";
    std::cout<<getLinearTerms()<<"\n";

    std::cout<<"comPosition:\n";
    std::cout<<getCoMPosition().transpose()<<"\n";

    std::cout<<"comVelocity:\n";
    std::cout<<getCoMVelocity().transpose()<<"\n";

    std::cout<<"comJdotQdot:\n";
    std::cout<<getCoMJdotQdot().transpose()<<"\n";

    std::cout<<"comJacobian:\n";
    std::cout<<getCoMJacobian()<<"\n";

    std::cout<<"comJacobianDot:\n";
    std::cout<<getCoMJacobianDot()<<"\n";



    for (int idx=0; idx<nbSegments(); idx++)
    {
        std::cout<<"segPosition "<<idx<<":\n";
        std::cout<<getSegmentPosition(idx)<<"\n";

        std::cout<<"segVelocity "<<idx<<":\n";
        std::cout<<getSegmentVelocity(idx)<<"\n";

        std::cout<<"segMass "<<idx<<":\n";
        std::cout<<getSegmentMass(idx)<<"\n";

        std::cout<<"segCoM "<<idx<<":\n";
        std::cout<<getSegmentCoM(idx)<<"\n";

        std::cout<<"segMassMatrix "<<idx<<":\n";
        std::cout<<getSegmentMassMatrix(idx)<<"\n";

        std::cout<<"segMomentsOfInertia "<<idx<<":\n";
        std::cout<<getSegmentMomentsOfInertia(idx)<<"\n";

        std::cout<<"segInertiaAxes "<<idx<<":\n";
        std::cout<<getSegmentInertiaAxes(idx)<<"\n";

        std::cout<<"segJacobian "<<idx<<":\n";
        std::cout<<getSegmentJacobian(idx)<<"\n";

        std::cout<<"segJdot "<<idx<<":\n";
        std::cout<<getSegmentJdot(idx)<<"\n";

        std::cout<<"segJointJacobian "<<idx<<":\n";
        std::cout<<getJointJacobian(idx)<<"\n";

        std::cout<<"segJdotQdot "<<idx<<":\n";
        std::cout<<getSegmentJdotQdot(idx).transpose()<<"\n";

    }

}
