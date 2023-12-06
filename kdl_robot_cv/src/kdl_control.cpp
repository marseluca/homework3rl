#include "kdl_ros_control/kdl_control.h"

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

//IMPLEMENTAZIONE PROF
Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current joint state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity() /*friction compensation?*/;
}


//IMPLEMENTAZIONE NOSTRA
Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
   // // Retrieve initial ee pose
    KDL::Frame Fi =  robot_->getEEFrame();
   // calculate gain matrices
   Eigen::Matrix<double,6,6> Kp, Kd;
   Kp=Eigen::MatrixXd::Zero(6,6);
   Kd=Eigen::MatrixXd::Zero(6,6);
   Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity(); //costruisco la matrice 3x3 dei guadagni sull'errore di posizione
   Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity(); //costruisco la matrice 3x3 dei guadagni sull'errore di orientamento
   Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();//costruisco la matrice 3x3 dei guadagni sulla derivata dell'errore di posizione
   Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();//costruisco la matrice 3x3 dei guadagni sulla derivata dell'errore di orientamento




    //CONVERSIONE DA JACOBIAN A MATRIX SI PUO' FARE SEMPLICEMENTE COSI
    Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;

   Eigen::Matrix<double,7,7> I = Eigen::Matrix<double,7,7>::Identity();
   Eigen::Matrix<double,7,7> M = robot_->getJsim();
   //Eigen::Matrix<double,7,6> Jpinv = weightedPseudoInverse(M,J);
   Eigen::Matrix<double,7,6> Jpinv = pseudoinverse(J);

   // position
   Eigen::Vector3d p_d(_desPos.p.data);
   Eigen::Vector3d p_e(robot_->getEEFrame().p.data);
   Eigen::Matrix<double,3,3,Eigen::RowMajor> R_d(_desPos.M.data);
   Eigen::Matrix<double,3,3,Eigen::RowMajor> R_e(robot_->getEEFrame().M.data);
   R_d = matrixOrthonormalization(R_d);
   R_e = matrixOrthonormalization(R_e);

   // velocity
   Eigen::Vector3d dot_p_d(_desVel.vel.data);
   Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);
   Eigen::Vector3d omega_d(_desVel.rot.data);
   Eigen::Vector3d omega_e(robot_->getEEVelocity().rot.data);

   // acceleration
   Eigen::Matrix<double,6,1> dot_dot_x_d;
   Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);
   Eigen::Matrix<double,3,1> dot_dot_r_d(_desAcc.rot.data);

   // compute linear errors
   Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
   Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);

   // shared control
   // Eigen::Vector3d lin_acc;
   // lin_acc << _desAcc.vel.x(), _desAcc.vel.y(), _desAcc.vel.z(); //use desired acceleration
   // lin_acc << dot_dot_p_d + _Kdp*(dot_e_p) + _Kpp*(e_p); // assuming no friction no loads
   // Eigen::Matrix<double,3,3> R_sh = shCntr(lin_acc);

   // compute orientation errors
//      Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
//    Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
//                                                                        omega_e,
//                                                                        R_d,
 //                                                                       R_e);   
   Eigen::Matrix<double,3,1> e_o_w = computeOrientationError(toEigen(Fi.M), toEigen(robot_->getEEFrame().M)); 
   Eigen::Matrix<double,3,1> e_o = computeOrientationError(R_d,R_e);
   Eigen::Matrix<double,3,1> dot_e_o = computeOrientationVelocityError(omega_d,
                                                                       omega_e,
                                                                       R_d,
                                                                       R_e);

   //ERRORE                                                                    
   Eigen::Matrix<double,6,1> x_tilde;
   Eigen::Matrix<double,6,1> dot_x_tilde;
   x_tilde << e_p, e_o_w[0],e_o[1],e_o[2];
   dot_x_tilde << dot_e_p, -omega_e;//dot_e_o;
   dot_dot_x_d << dot_dot_p_d, dot_dot_r_d;

   // null space control
   double cost;
   Eigen::VectorXd grad = gradientJointLimits(robot_->getJntValues(),robot_->getJntLimits(),cost);

//    std::cout << "---------------------" << std::endl;
//    std::cout << "p_d: " << std::endl << p_d << std::endl;
//    std::cout << "p_e: " << std::endl << p_e << std::endl;
//    std::cout << "dot_p_d: " << std::endl << dot_p_d << std::endl;
//    std::cout << "dot_p_e: " << std::endl << dot_p_e << std::endl;
//    std::cout << "R_sh*R_d: " << std::endl << R_d << std::endl;
//    std::cout << "R_e: " << std::endl << R_e << std::endl;
//    std::cout << "omega_d: " << std::endl << omega_d << std::endl;
//    std::cout << "omega_e: " << std::endl << omega_e << std::endl;
//    std::cout << "x_tilde: " << std::endl << x_tilde << std::endl;
//    std::cout << "dot_x_tilde: " << std::endl << dot_x_tilde << std::endl;
//    std::cout << "jacobian: " << std::endl << robot_->getJacobian() << std::endl;
//    std::cout << "jpinv: " << std::endl << Jpinv << std::endl;
//    std::cout << "jsim: " << std::endl << robot_->getJsim() << std::endl;
//    std::cout << "c: " << std::endl << robot_->getCoriolis().transpose() << std::endl;
//    std::cout << "g: " << std::endl << robot_->getGravity().transpose() << std::endl;
//    std::cout << "q: " << std::endl << robot_->getJntValues().transpose() << std::endl;
//    std::cout << "Jac Dot qDot: " << std::endl << robot_->getJacDotqDot().transpose() << std::endl;
//    std::cout << "Jnt lmt cost: " << std::endl << cost << std::endl;
//    std::cout << "Jnt lmt gradient: " << std::endl << grad.transpose() << std::endl;
//    std::cout << "---------------------" << std::endl;

   // inverse dynamics

   //stampiamo le velocita q_dot
   //std::cout<<robot_->getJntVelocities()<<std::endl;

    //ci salviamo i valori di J_dot in una matrice per facilitare il prodotto
    
    // std::cout<<Jdot<<std::endl;
    // std::cout<<"debug5"<<std::endl;

    //creiamo una matrice contenente y = xd_dot_dot - J_dot*q_dot + Kd*x_tilde_dot + Kp*x_tilde
    Eigen::Matrix<double,6,1> y;
    y << dot_dot_x_d - robot_->getEEJacDotqDot() + Kd*dot_x_tilde + Kp*x_tilde;

    //restituiamo l'ingresso di controllo u = By + n
       return M * (Jpinv*y)+ robot_->getGravity() + robot_->getCoriolis();
           //(I-Jpinv*J)*(/*- 10*grad */- 1*robot_->getJntVelocities())

    
}


Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp,
                                      double _Kdp)
{
   
   // calculate gain matrices
   Eigen::Matrix<double,3,3> Kp, Kd;
   Kp = _Kpp*Eigen::Matrix3d::Identity(); //costruisco la matrice 3x3 dei guadagni sull'errore di posizione
   Kd = _Kdp*Eigen::Matrix3d::Identity();//costruisco la matrice 3x3 dei guadagni sulla derivata dell'errore di posizione
   

   Eigen::Matrix<double,6,7> J = robot_->getEEJacobian().data;
   Eigen::Matrix<double,3,7> J_red = J.topRows(3);
   Eigen::Matrix<double,7,7> M = robot_->getJsim();
   Eigen::Matrix<double,7,3> Jpinv = pseudoinverse(J_red);

   // position
   Eigen::Vector3d p_d(_desPos.p.data);
   Eigen::Vector3d p_e(robot_->getEEFrame().p.data);

   // velocity
   Eigen::Vector3d dot_p_d(_desVel.vel.data);
   Eigen::Vector3d dot_p_e(robot_->getEEVelocity().vel.data);


   // acceleration
   Eigen::Matrix<double,3,1> dot_dot_x_d;
   Eigen::Matrix<double,3,1> dot_dot_p_d(_desAcc.vel.data);

   // compute linear errors
   Eigen::Matrix<double,3,1> e_p = computeLinearError(p_d,p_e);
   Eigen::Matrix<double,3,1> dot_e_p = computeLinearError(dot_p_d,dot_p_e);
   //ERRORE                                                                    
   Eigen::Matrix<double,3,1> x_tilde;
   Eigen::Matrix<double,3,1> dot_x_tilde;
   x_tilde << e_p;
   dot_x_tilde << dot_e_p;
   dot_dot_x_d << dot_dot_p_d;

Eigen::Matrix<double,3,1> y;

    y << dot_dot_x_d - robot_->getEEJacDotqDot_red() + Kd*dot_x_tilde + Kp*x_tilde;
return M * (Jpinv*y)+ robot_->getGravity() + robot_->getCoriolis();

           
}


