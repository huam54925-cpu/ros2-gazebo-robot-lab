// Paused actuator commit + post-physics execution receipts for one isolated model.
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <mutex>
#include <sstream>
#include <iomanip>
#include <cmath>

namespace local_drive {
using namespace gz;
using namespace gz::sim;
class SyncDrive : public System, public ISystemConfigure,
                  public ISystemPreUpdate, public ISystemPostUpdate {
  transport::Node node;
  transport::Node::Publisher odomPub;
  Model model{kNullEntity};
  Entity left{kNullEntity}, right{kNullEntity};
  math::DiffDriveOdometry odom{1};
  std::mutex mutex;
  long long episode{0}, action{-1}, pendingId{-1}, appliedNs{-1}, firstNs{-1}, lastNs{-1};
  unsigned long long steps{0};
  double v{0}, w{0}, pendingV{0}, pendingW{0}, leftSpeed{0}, rightSpeed{0};
  double measuredLeft{0}, measuredRight{0};
  std::string error;
  bool pending{false}, committed{false}, healthy{true}, paused{true};
  static constexpr double radius=0.3, separation=1.25;
  static long long Ns(const std::chrono::steady_clock::duration &d) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
  }
  bool Command(const msgs::StringMsg &req, msgs::Boolean &rep) {
    std::lock_guard<std::mutex> guard(mutex);
    long long ep, id; double cv,cw; std::string extra;
    std::istringstream in(req.data());
    bool valid=bool(in>>ep>>id>>cv>>cw) && !(in>>extra);
    valid=valid && ep==episode && id==action+1 && !pending && paused && healthy &&
      std::isfinite(cv) && std::isfinite(cw) && cv>=-0.2-1e-9 && cv<=0.6+1e-9 && std::abs(cw)<=0.8+1e-9;
    if (valid) {pendingId=id;pendingV=cv;pendingW=cw;pending=true;}
    rep.set_data(valid); return true;
  }
  bool Status(const msgs::Empty &, msgs::StringMsg &rep) {
    std::lock_guard<std::mutex> guard(mutex);
    std::ostringstream s; s<<std::setprecision(17);
    s<<"{\"episode_id\":"<<episode<<",\"action_id\":"<<action
     <<",\"committed\":"<<(committed?"true":"false")<<",\"healthy\":"<<(healthy?"true":"false")
     <<",\"error\":\""<<error<<"\""<<",\"paused\":"<<(paused?"true":"false")<<",\"applied_ns\":"<<appliedNs
     <<",\"first_ns\":"<<firstNs<<",\"last_ns\":"<<lastNs<<",\"steps\":"<<steps
     <<",\"v\":"<<v<<",\"w\":"<<w<<",\"left_cmd\":"<<leftSpeed<<",\"right_cmd\":"<<rightSpeed
     <<",\"left_measured\":"<<measuredLeft<<",\"right_measured\":"<<measuredRight<<"}";
    rep.set_data(s.str()); return true;
  }
 public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 EntityComponentManager &, EventManager &) override {
    model=Model(entity); episode=std::stoll(sdf->Get<std::string>("episode_id"));
    odom.SetWheelParams(separation,radius,radius);
    odomPub=node.Advertise<msgs::Odometry>("/model/vehicle/odometry");
    node.Advertise("/local_drive/command",&SyncDrive::Command,this);
    node.Advertise("/local_drive/status",&SyncDrive::Status,this);
  }
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override {
    std::lock_guard<std::mutex> guard(mutex); paused=info.paused;
    if (left==kNullEntity) left=model.JointByName(ecm,"left_wheel_joint");
    if (right==kNullEntity) right=model.JointByName(ecm,"right_wheel_joint");
    if (left==kNullEntity || right==kNullEntity) return;
    for (auto joint : {left,right}) {
      if (ecm.Component<components::JointForceCmd>(joint) || ecm.Component<components::JointVelocityReset>(joint)) {healthy=false;error="conflicting actuator writer";return;}
      if (!ecm.Component<components::JointPosition>(joint)) ecm.CreateComponent(joint,components::JointPosition());
      if (!ecm.Component<components::JointVelocity>(joint)) ecm.CreateComponent(joint,components::JointVelocity());
    }
    if (pending) {
      if (!info.paused) {healthy=false;error="missing feedback or not paused";return;}
      action=pendingId;v=pendingV;w=pendingW;pending=false;committed=false;
      leftSpeed=(v-w*separation/2)/radius;rightSpeed=(v+w*separation/2)/radius;
      steps=0;firstNs=-1;lastNs=-1;appliedNs=Ns(info.simTime);
    }
    // This plugin is the sole wheel actuator writer; no ordinary DiffDrive plugin.
    ecm.SetComponentData<components::JointVelocityCmd>(left,{leftSpeed});
    ecm.SetComponentData<components::JointVelocityCmd>(right,{rightSpeed});
    auto l=ecm.Component<components::JointVelocityCmd>(left);
    auto r=ecm.Component<components::JointVelocityCmd>(right);
    committed=action>=0 && l && r && l->Data().size()==1 && r->Data().size()==1 &&
              l->Data()[0]==leftSpeed && r->Data()[0]==rightSpeed;
    if (!odom.Initialized()) odom.Init(std::chrono::steady_clock::time_point(info.simTime));
    if (!info.paused && (!committed || Ns(info.dt)!=1000000)) {healthy=false;error="preupdate dt or uncommitted";}
  }
  void PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm) override {
    std::lock_guard<std::mutex> guard(mutex);
    if (info.paused || left==kNullEntity || right==kNullEntity) return;
    auto lp=ecm.Component<components::JointPosition>(left), rp=ecm.Component<components::JointPosition>(right);
    auto lv=ecm.Component<components::JointVelocity>(left), rv=ecm.Component<components::JointVelocity>(right);
    auto lc=ecm.Component<components::JointVelocityCmd>(left), rc=ecm.Component<components::JointVelocityCmd>(right);
    if (!lp || !rp || lp->Data().empty() || rp->Data().empty() || !lv || !rv || lv->Data().empty() || rv->Data().empty()) {healthy=false;error="missing feedback or not paused";return;}
    // Gazebo 10 Physics consumes and REMOVES JointVelocityCmd during Update.
    // PreUpdate verified the values; PostUpdate requires their consumption.
    if (lc || rc) {healthy=false;error="physics did not consume joint command";}
    measuredLeft=lv->Data()[0];measuredRight=rv->Data()[0];
    if (steps==0) firstNs=Ns(info.simTime);
    lastNs=Ns(info.simTime);++steps;
    odom.Update(lp->Data()[0],rp->Data()[0],std::chrono::steady_clock::time_point(info.simTime));
    msgs::Odometry msg;
    msg.mutable_header()->mutable_stamp()->set_sec(lastNs/1000000000);
    msg.mutable_header()->mutable_stamp()->set_nsec(lastNs%1000000000);
    msg.mutable_pose()->mutable_position()->set_x(odom.X());msg.mutable_pose()->mutable_position()->set_y(odom.Y());
    math::Quaterniond q(0,0,odom.Heading().Radian());
    msg.mutable_pose()->mutable_orientation()->set_w(q.W());msg.mutable_pose()->mutable_orientation()->set_z(q.Z());
    msg.mutable_twist()->mutable_linear()->set_x(odom.LinearVelocity());
    msg.mutable_twist()->mutable_angular()->set_z(odom.AngularVelocity().Radian());
    auto ep=msg.mutable_header()->add_data();ep->set_key("episode_id");ep->add_value(std::to_string(episode));
    odomPub.Publish(msg);
  }
};
}
GZ_ADD_PLUGIN(local_drive::SyncDrive, gz::sim::System, local_drive::SyncDrive::ISystemConfigure,
 local_drive::SyncDrive::ISystemPreUpdate,local_drive::SyncDrive::ISystemPostUpdate)
