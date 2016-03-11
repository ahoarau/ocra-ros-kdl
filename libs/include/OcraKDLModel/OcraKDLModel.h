#ifndef __OCRA_KDL_MODEL__
#define __OCRA_KDL_MODEL__


#include <ocra/control/Model.h>
#include <rtt_ros_kdl_tools/chain_utils.hpp>
#include <boost/smart_ptr.hpp>

class OcraKDLModel: public ocra::Model
{
public:



//===========================Constructor/Destructor===========================//
    OcraKDLModel(const std::string& robotName = "arm",bool initialize = false);
    virtual ~OcraKDLModel();

//=============================General functions==============================//
     int                          nbSegments               () const;
     const Eigen::VectorXd&       getActuatedDofs          () const;
     const Eigen::VectorXd&       getJointLowerLimits      () const;
     const Eigen::VectorXd&       getJointUpperLimits      () const;
     const Eigen::VectorXd&       getJointPositions        () const;
     const Eigen::VectorXd&       getJointVelocities       () const;
     const Eigen::Displacementd&  getFreeFlyerPosition     () const;
     const Eigen::Twistd&         getFreeFlyerVelocity     () const;

     const std::string&           getJointName             (int index) const;
     const int                    getSegmentIndex          (const std::string& segmentName) const;

//=============================Dynamic functions==============================//
     const Eigen::MatrixXd&       getInertiaMatrix         () const;
     const Eigen::MatrixXd&       getInertiaMatrixInverse  () const;
     const Eigen::MatrixXd&       getDampingMatrix         () const;
     const Eigen::VectorXd&       getNonLinearTerms        () const;
     const Eigen::VectorXd&       getLinearTerms           () const;
     const Eigen::VectorXd&       getGravityTerms          () const;
     const Eigen::VectorXd&       getJointTorques()            const;
//===============================CoM functions================================//
     double                                         getMass            () const;
     const Eigen::Vector3d&                         getCoMPosition     () const;
     const Eigen::Vector3d&                         getCoMVelocity     () const;
     const Eigen::Vector3d&                         getCoMJdotQdot     () const;
     const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobian     () const;
     const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMJacobianDot  () const;
     const Eigen::Vector3d&                         getCoMAngularVelocity     () const;
     const Eigen::Matrix<double,3,Eigen::Dynamic>&  getCoMAngularJacobian     () const;

//=============================Segment functions==============================//
     const Eigen::Displacementd&                    getSegmentPosition          (int index) const;
     const Eigen::Twistd&                           getSegmentVelocity          (int index) const;
     double                                         getSegmentMass              (int index) const;
     const Eigen::Vector3d&                         getSegmentCoM               (int index) const;
     const Eigen::Matrix<double,6,6>&               getSegmentMassMatrix        (int index) const;
     const Eigen::Vector3d&                         getSegmentMomentsOfInertia  (int index) const;
     const Eigen::Rotation3d&                       getSegmentInertiaAxes       (int index) const;
     const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJacobian          (int index) const;
     const Eigen::Matrix<double,6,Eigen::Dynamic>&  getSegmentJdot              (int index) const;
     const Eigen::Matrix<double,6,Eigen::Dynamic>&  getJointJacobian            (int index) const;
     const Eigen::Twistd&                           getSegmentJdotQdot          (int index) const;
     void setState(const Eigen::VectorXd& q,const Eigen::VectorXd& qdot);
     void printAllData();
     bool initialize();


protected:

//===========================Update state functions===========================//
     void                doSetState();
     void                doSetJointPositions     (const Eigen::VectorXd& q);
     void                doSetJointVelocities    (const Eigen::VectorXd& dq);
     void                doSetFreeFlyerPosition  (const Eigen::Displacementd& Hroot);
     void                doSetFreeFlyerVelocity  (const Eigen::Twistd& Troot);
    
//============================Index name functions============================//
     int                 doGetSegmentIndex       (const std::string& name) const;
     const std::string&  doGetSegmentName        (int index) const;
     int                 doGetDofIndex           (const std::string& name) const;
     const std::string&  doGetDofName            (int index) const;
     const std::string   doSegmentName           (const std::string& name) const;
     const std::string   doDofName               (const std::string& name) const;

private:
    std::shared_ptr<rtt_ros_kdl_tools::ChainUtils> chain;
    Eigen::VectorXd actuated_dofs;
    unsigned int nb_segments,nb_joints;
    Eigen::Displacementd eigen_displacement_zero;
    Eigen::Twistd eigen_twist_zero;
    Eigen::VectorXd eigen_vector_zero;
    mutable Eigen::VectorXd jl_up,jl_low;
    mutable Eigen::MatrixXd M_inv;
    mutable Eigen::MatrixXd joint_jacobian;

};








#endif
