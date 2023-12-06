#include "kdl_ros_control/kdl_planner.h"
#include <cmath>

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

// CIRCULAR TRAJECTORY CONSTRUCTOR DEFINITION

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius)
{
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajRadius_ = _trajRadius;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd,double _trajRadius)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
    trajRadius_ = _trajRadius;
}



void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

void KDLPlanner::trapezoidal_vel(double time, double &s, double &dots,double &ddots)
{
  
  double si=0;
  double sf=1;

  double ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(sf-si);

  if(time <= accDuration_)
  {
    s = si + 0.5*ddot_traj_c*std::pow(time,2);
    dots = ddot_traj_c*time;
    ddots = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    s = si + ddot_traj_c*accDuration_*(time-accDuration_/2);
    dots = ddot_traj_c*accDuration_;
    ddots = 0;
  }
  else
  {
    s = sf - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    dots = ddot_traj_c*(trajDuration_-time);
    ddots = -ddot_traj_c;
  }

}

void KDLPlanner::cubic_polinomial(double time, double &s, double &dots,double &ddots)
{
  
  double si=0;
  double sf=1;
  double dsi=0;
  double dsf=0;
  //finding polinomial coefficients
  double a0=si;
  double a1=dsi;
  double a2=3/std::pow(trajDuration_,2);
  double a3=-2/(std::pow(trajDuration_,3));

  s=a3*std::pow(time,3)+a2*std::pow(time,2)+a1*time+a0;
  dots=3*a3*std::pow(time,2)+2*a2*time+a1;
  ddots=6*a3*time+2*a2;
}


trajectory_point KDLPlanner::path_primitive_linear( double &s, double &dots,double &ddots){ //sono input soltanto, ma evito la copia
  trajectory_point traj;
  Eigen::Vector3d pi=trajInit_;
  Eigen::Vector3d pf=trajEnd_;
  Eigen::Vector3d dif=pf-pi;
  /*double pnorm=dif.norm();
  traj.pos=pi+s*dif/pnorm;
  traj.vel=dots*dif/pnorm;
  traj.acc=ddots*dif/pnorm;*/
  traj.pos=(1-s)*pi+s*pf;
  traj.vel=(-pi+pf)*dots;
  traj.acc=(-pi+pf)*ddots;
  //std::cout<<"pos: "<<traj.pos[1]<<" "<<traj.pos[2]<<" "<<traj.pos[3] <<std::endl;
  return traj;  
}

trajectory_point KDLPlanner::path_primitive_circular( double &s, double &dots,double &ddots){ //sono input soltanto, ma evito la copia
  trajectory_point traj;
  Eigen::Vector3d pi = trajInit_;
  Eigen::Vector3d pf=trajEnd_;
  Eigen::Vector3d dif=pf-pi;

  
  // DEFINE  THE CENTER
  
  Eigen::Vector3d p0;
  p0[0] = pi[0]; // centro x
  p0[1] = pi[1]+trajRadius_; // centro y
  p0[2] = pi[2]; // centro z
  

  // POSIZIONE
  traj.pos[0] = p0[0]; // x
  traj.pos[1] = p0[1] - trajRadius_*cos(2*M_PI*s); // y
  traj.pos[2] = p0[2] - trajRadius_*sin(2*M_PI*s); // z

  // VELOCITA
  traj.vel[0] = 0;
  traj.vel[1] = trajRadius_*(2*M_PI)*dots*sin(2*M_PI*s);
  traj.vel[2] = -trajRadius_*(2*M_PI)*dots*cos(2*M_PI*s);

  // ACCELERAZIONE
  traj.acc[0] = 0;
  traj.acc[1] = trajRadius_*(2*M_PI)*(dots*dots*2*M_PI*cos(2*M_PI*s)+ddots*sin(2*M_PI*s));
  traj.acc[2] = -trajRadius_*(2*M_PI)*(-dots*dots*2*M_PI*sin(2*M_PI*s)+ddots*cos(2*M_PI*s));



  //std::cout<<"pos: "<<traj.pos[1]<<" "<<traj.pos[2]<<" "<<traj.pos[3] <<std::endl;
  return traj;  
}

trajectory_point KDLPlanner::compute_trapezoidal_linear( double t){
  double st;
  double dst;
  double ddst;
  trapezoidal_vel(t,st,dst,ddst);

  // std::cout<<"time: "<<t<<" s: "<<st<<" s dot: " <<dst<<" s dot dot: "<<ddst<<std::endl;
  //std::cout<<"time: "<<t<<" ";
  return path_primitive_linear(st,dst,ddst);
}

trajectory_point KDLPlanner::compute_cubic_linear( double t){
  double st;
  double dst;
  double ddst;
  cubic_polinomial(t,st,dst,ddst);

    std::cout<<"time: "<<t<<" s: "<<st<<" s dot: " <<dst<<" s dot dot: "<<ddst<<std::endl;
  //std::cout<<"time: "<<t<<" ";
  return path_primitive_linear(st,dst,ddst);
}

trajectory_point KDLPlanner::compute_cubic_circular( double t){
  double st;
  double dst;
  double ddst;
  cubic_polinomial(t,st,dst,ddst);

    std::cout<<"time: "<<t<<" s: "<<st<<" s dot: " <<dst<<" s dot dot: "<<ddst<<std::endl;
  //std::cout<<"time: "<<t<<" ";
  return path_primitive_circular(st,dst,ddst);
}

trajectory_point KDLPlanner::compute_trapezoidal_circular(double time)
{
  double st;
  double dst;
  double ddst;
  trapezoidal_vel(time,st,dst,ddst);

    //std::cout<<"time: "<<time<<" s: "<<st<<" s dot: " <<dst<<" s dot dot: "<<ddst<<std::endl;
  //std::cout<<"time: "<<t<<" ";
  return path_primitive_circular(st,dst,ddst);
}

trajectory_point KDLPlanner::compute_trajectory(double time,std::string profile, std::string path)
{
  if(profile=="cubic"){
    if(path=="linear") return compute_cubic_linear(time);
    else return compute_cubic_circular(time); 
  }else{
    if(path=="linear") return compute_trapezoidal_linear(time);
    else return compute_trapezoidal_circular(time); 
  }
 
}


