#include "meco/sim/PracticeMatchController.h"
#ifndef __FRC_ROBORIO__
#include <frc/RobotBase.h>
#include <frc/Timer.h>
#include <frc/RobotController.h>
#include <frc/DriverStation.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/DriverStation.h>
#include <hal/simulation/DriverStationData.h>
#include <networktables/NetworkTable.h>
#include <networktables/StringTopic.h>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <algorithm>
namespace meco::sim {
using DS = frc::sim::DriverStationSim;
namespace {
thread_local const void* writingController = nullptr;
struct WriteScope {
  const void* previous;
  explicit WriteScope(const void* owner):previous(writingController) { writingController=owner; }
  ~WriteScope() { writingController=previous; }
};
}
struct PracticeMatchController::Impl {
  PracticeMatchProfile profile;
  std::shared_ptr<nt::NetworkTable> table;
  nt::StringSubscriber commands;
  std::vector<std::unique_ptr<frc::sim::CallbackStore>> callbacks;
  std::atomic<bool> running{false}, changed{false};
  std::string state{"idle"}, result{"R"};
  uint64_t start{};
  double remaining{};
  bool oldDs{}, oldFms{};
  HAL_AllianceStationID oldAlliance{};
  HAL_MatchInfo oldInfo{};
  static void Changed(const char*, void* param, const HAL_Value*) {
    auto& self=*static_cast<Impl*>(param);
    if(self.running && writingController!=&self) self.changed=true;
  }
  explicit Impl(PracticeMatchProfile p, nt::NetworkTableInstance nt):profile(p),
      table(nt.GetTable("/Sim/Practice")),
      commands(table->GetStringTopic("Command").Subscribe("", {.pollStorage=32, .keepDuplicates=true})) {
    if(!frc::RobotBase::IsSimulation()) throw std::logic_error("Practice matches require simulation");
    for(double duration:{p.autonomousSeconds,p.assessmentSeconds,p.teleopSeconds})
      if(!std::isfinite(duration)||duration<1e-6||duration>86400) throw std::invalid_argument("Invalid practice duration");
    table->GetEntry("Config/AllianceStation").SetDefaultString("Blue1");
    table->GetEntry("Config/AutoResult").SetDefaultString("R");
    table->GetEntry("Profile/AutoSeconds").SetDouble(p.autonomousSeconds);
    table->GetEntry("Profile/AssessmentSeconds").SetDouble(p.assessmentSeconds);
    table->GetEntry("Profile/TeleopSeconds").SetDouble(p.teleopSeconds);
    commands.ReadQueue(); // A retained command is never an instruction to enable.
    callbacks.push_back(DS::RegisterEnabledCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    callbacks.push_back(DS::RegisterAutonomousCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    callbacks.push_back(DS::RegisterTestCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    callbacks.push_back(DS::RegisterEStopCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    callbacks.push_back(DS::RegisterDsAttachedCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    callbacks.push_back(DS::RegisterFmsAttachedCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    callbacks.push_back(DS::RegisterAllianceStationIdCallback([this](std::string_view,const HAL_Value* value) {Changed(nullptr,this,value);},false));
    Publish();
  }
  void Packet() { HALSIM_NotifyDriverStationNewData(); }
  void Publish() {
    table->GetEntry("State").SetString(state);
    table->GetEntry("Running").SetBoolean(running);
    table->GetEntry("RemainingSeconds").SetDouble(remaining);
    table->GetEntry("MatchTime").SetDouble(DS::GetMatchTime());
    table->GetEntry("GameData").SetString(frc::DriverStation::GetGameSpecificMessage());
  }
  void Finish(const std::string& next, const std::string& reason) {
    WriteScope scope(this);
    DS::SetEnabled(false);
    running=false;
    DS::SetAutonomous(false); DS::SetTest(false); DS::SetMatchTime(-1);
    DS::SetDsAttached(oldDs); DS::SetFmsAttached(oldFms);
    DS::SetAllianceStationId(oldAlliance);
    HALSIM_SetMatchInfo(&oldInfo);
    state=next; remaining=0;
    table->GetEntry("Reason").SetString(reason);
    Packet();
    // WPILib's new-data notification itself sets DS attached. Restore the
    // saved raw flag after notifying; the next external packet may attach again.
    DS::SetDsAttached(oldDs);
  }
  bool Command(const std::string& command) {
    if(command=="abort" || command=="reset") {
      if(running) Finish(command=="reset"?"idle":"aborted",command);
      else if(command=="reset") {state="idle";remaining=0;}
      return true;
    }
    if(command!="start") { table->GetEntry("Reason").SetString("unknown-command");return false; }
    if(running || DS::GetEnabled() || DS::GetEStop()) {
      table->GetEntry("Reason").SetString("start-requires-disabled-and-no-estop");return false;
    }
    const auto station=table->GetEntry("Config/AllianceStation").GetString("Blue1");
    const std::vector<std::string> stations{"Red1","Red2","Red3","Blue1","Blue2","Blue3"};
    const auto found=std::find(stations.begin(),stations.end(),station);
    const auto chosen=table->GetEntry("Config/AutoResult").GetString("R");
    if(found==stations.end() || (chosen!="R"&&chosen!="B")) {
      table->GetEntry("Reason").SetString("invalid-configuration");return false;
    }
    oldDs=DS::GetDsAttached();oldFms=DS::GetFmsAttached();oldAlliance=DS::GetAllianceStationId();
    HALSIM_GetMatchInfo(&oldInfo);
    result=chosen;start=frc::RobotController::GetFPGATime();changed=false;
    WriteScope scope(this);
    DS::SetDsAttached(true);DS::SetFmsAttached(true);
    DS::SetAllianceStationId(static_cast<HAL_AllianceStationID>(HAL_AllianceStationID_kRed1 + (found-stations.begin())));
    DS::SetEventName("MECO Simulation Practice");DS::SetMatchType(frc::DriverStation::MatchType::kPractice);
    DS::SetMatchNumber(1);DS::SetReplayNumber(0);DS::SetGameSpecificMessage("");
    DS::SetTest(false);DS::SetAutonomous(true);DS::SetMatchTime(profile.autonomousSeconds);
    DS::SetEnabled(true);running=true;state="autonomous";remaining=profile.autonomousSeconds;
    table->GetEntry("Reason").SetString("");Packet();return true;
  }
  void Update() {
    if(running && (changed || DS::GetEStop())) {
      Finish("aborted","external-ds-change");
      commands.ReadQueue(); // Commands queued before interruption cannot restart the run.
      Publish();return;
    }
    for(const auto& update:commands.ReadQueue()) {
      bool accepted=Command(update.value);
      table->GetEntry("LastCommand").SetString(update.value);
      table->GetEntry("LastCommandAccepted").SetBoolean(accepted);
    }
    if(running) {
      WriteScope scope(this);
      const auto elapsed=frc::RobotController::GetFPGATime()-start;
      const uint64_t autoEnd=std::llround(profile.autonomousSeconds*1e6);
      const uint64_t gapEnd=autoEnd+std::llround(profile.assessmentSeconds*1e6);
      const uint64_t end=gapEnd+std::llround(profile.teleopSeconds*1e6);
      if(elapsed<autoEnd) {
        remaining=(autoEnd-elapsed)/1e6;DS::SetMatchTime(remaining);
      } else if(elapsed<gapEnd) {
        state="assessment";remaining=(gapEnd-elapsed)/1e6;
        DS::SetEnabled(false);DS::SetMatchTime(0);
      } else if(elapsed<end) {
        state="teleop";remaining=(end-elapsed)/1e6;
        DS::SetAutonomous(false);DS::SetGameSpecificMessage(result);DS::SetMatchTime(remaining);DS::SetEnabled(true);
      } else Finish("complete","");
      if(running) Packet();
    }
    Publish();
  }
  ~Impl() {if(running) Finish("aborted","controller-destroyed");callbacks.clear();}
};
PracticeMatchController::PracticeMatchController(PracticeMatchProfile p,nt::NetworkTableInstance nt):impl_(std::make_unique<Impl>(p,nt)) {}
PracticeMatchController::~PracticeMatchController()=default;
void PracticeMatchController::Update() {impl_->Update();}
}
#endif
