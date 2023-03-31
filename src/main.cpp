#include <signal.h>
#include "ros/ros.h"
#include "cti_spdlog.h"
#include "MissionScheduleNode.h"
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>
#include "common/ContainerInjector.h"
#include "deviceMiscellaneous/PathPlanner.h"
#include "deviceMiscellaneous/MissionPlanner.h"
#include "deviceMiscellaneous/ElevatorPlanner.h"
#include "deviceMiscellaneous/DeviceRuntimeUtility.h"
#include "deviceMiscellaneous/PlatformMissionCenter.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "cti_save_crash_log.hpp"

// 59
// pn%3WEpj
// 71
// Lsw3!bsk
// 71 new
// 212cti
// 93
// FW5Nzr7B
// LG55
// Ln2td8n$
// 13160686695
// 86  zw#sXZYQ
// 248 NBBv$5M!
// 221 gbDj&wC4
// 223 $WSwQ8YK
// 225 2ZUj9x@j
// 229 L%B4sEde
// 236 QxktaeV9
// 246 bPTx#yvq
// 250 vMeKMC3b
// 261 w%qJG9c5
// 254 9Zu$HSfn
// 255 jNBfG&tC
// 267 urUws@nm
// 269 fzG$8zT&

// catkin_make install --pkg mission_schedule local_schedule cloud_scheduling_node infrary_communication cti_record_log robot_base robot_docking
// ./make-deb ../../workspace/install/ br-robot-scheduling 8.12.15b TT

// sudo vim /etc/NetworkManager/NetworkManager.conf
// nameserver 211.136.20.203
// nameserver 114.114.114.114

// set fileencodings=utf-8,ucs-bom,gb18030,gbk,gb2312,cp936
// set termencoding=utf-8
// set encoding=utf-8

bool isTerminated = false;

static const char* main_tree_xml_text = R"(
<root main_tree_to_execute = "mainTree">
  <BehaviorTree ID="platformMissionTree">
    <Sequence name="platformMissionSequence">
      <schedulePlatformMission/>
      <Fallback name="allocateElevator">
          <Inverter>
            <Condition ID="isSwitchFloorNeed"/>
          </Inverter>
          <allocateElevatorPlan/>
      </Fallback>
      <executePlatformMission/>
    </Sequence>
  </BehaviorTree>

  <BehaviorTree ID="parkingManagementTree">
    <Fallback name="parkingLocationSelector">
      <IfThenElse name="checkLocation">
        <isCurrentParkingLocationGood/>
        <IfThenElse name="isAppropriateToReboot">
          <isAppropriateToReboot/>
          <robotAutonomicReboot/>
          <forceFailure/>
        </IfThenElse>
        <Sequence name="parkingLocationAllocateSequence">
          <findParkingLocation/>
          <Fallback name="allocateElevator">
            <Inverter>
              <Condition ID="isSwitchFloorNeed"/>
            </Inverter>
            <allocateElevatorPlan/>
          </Fallback>
          <executeParkingMission/>
        </Sequence>
      </IfThenElse>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="powerManagementTree">
    <Fallback name="powerManagement">
      <IfThenElse name="robotLowPowerManagement">
        <isRobotFreeOfSystemPowerManagement/>
        <forceFailure/>
        <Sequence name="chargingLocationAllocateSequence">
          <findChargingLocation/>
          <Fallback name="allocateElevator">
            <Inverter>
              <Condition ID="isSwitchFloorNeed"/>
            </Inverter>
            <allocateElevatorPlan/>
          </Fallback>
          <executeChargingMission/>
        </Sequence>
      </IfThenElse>
      <SubTree ID="parkingManagementTree"/>
    </Fallback>
  </BehaviorTree>

  <BehaviorTree ID="isPowerSuppetToAllocateOrder">
    <IfThenElse name="checkCharging">
      <isDeviceCharging/>
      <isDeviceChargingToBasicPower/>
      <Inverter>
        <isDevicePowerBad/>
      </Inverter>
    </IfThenElse>
  </BehaviorTree>

  <BehaviorTree ID="scheduleTree">
    <IfThenElse name="checkManualMode">
      <isManualMode/>
      <Fallback name="manualModeSequence">
        <Sequence name="disabledExpireCheck">
          <isRobotShouldCancelOrdersWhenDisabled/>
          <isPlatformMissionRunning/>
          <cancelAllPlatformMissions reason="local schedule disabled"/>
        </Sequence>
        <executeCloudDirectCommand/>
      </Fallback>
      <IfThenElse name="checkRobotStatus">
        <isDeviceStatusOk/>
        <Sequence name="rootSequence">
          <resetRecoveryFlag/>
          <updateElevatorGraph/>
          <broadcastDesiredDestination/>
          <updateStationOccupyRelationship/>
          <IfThenElse name="checkDirectCommand">
            <isCloudDirectCommandEmpty/>
            <IfThenElse name="checkManualOrder">
              <Sequence>
                <isManualOrderEmpty/>
                <Inverter>
                  <allocateManualOrderToRobot/>
                </Inverter>
              </Sequence>
              <IfThenElse name="checkChargeTobeContinue">
                <isChargingTobeContinue/>
                <executeChargingMission/>
                <IfThenElse name="checkPlatformMission">
                  <Fallback name="checkOrder">
                    <loadOrderLocalStorage/>
                    <isPlatformMissionRunning/>
                    <Sequence>
                      <SubTree ID="isPowerSuppetToAllocateOrder"/>
                      <allocateHiveOrderToRobot/>
                    </Sequence>
                    <isHiveLoaded/>
                  </Fallback>
                  <IfThenElse name="platformMissionDecision">
                    <isDevicePowerBad/>
                    <Sequence>
                      <cancelAllPlatformMissions reason="robot power bad"/>
                      <Fallback>
                        <isDeviceCharging/>
                        <abortAllPlatformMissions/>
                      </Fallback>
                    </Sequence>
                    <executePlatformMission/>
                  </IfThenElse>
                  <Fallback name="localScheduleManagement">
                    <Condition ID="isPlatformMissionRunning"/>
                    <SubTree ID="powerManagementTree"/>
                  </Fallback>
                </IfThenElse>
              </IfThenElse>
              <executeManualOrder/>
            </IfThenElse>
            <Sequence>
              <executeCloudDirectCommand/>
              <IfThenElse name="checkSterilize">
                <isSterilizeOrderReachStation/>
                <cancelHiveSterilize/>
                <forceSuccess/>
              </IfThenElse>
              <Fallback>
                <isManualOrderEmpty/>
                <cancelManualOrder/>
              </Fallback>
            </Sequence>
          </IfThenElse>
        </Sequence>
        <IfThenElse name="checkDeviceFault">
          <isDeviceStatusFault/>
          <Fallback name="robotRecovery">
            <Sequence name="faultExpireCheck">
              <isRobotShouldCancelOrdersWhenFault/>
              <isPlatformMissionRunning/>
              <cancelAllPlatformMissions reason="robot state is fault"/>
            </Sequence>
            <executeRobotRecoverBehavior/>
            <executeCloudDirectCommand/>
          </Fallback>
          <executeCloudDirectCommand/>
        </IfThenElse>
      </IfThenElse>
    </IfThenElse>
  </BehaviorTree>

  <BehaviorTree ID="mainTree">
    <IfThenElse name="checkStillMode">
      <Fallback name="robotStatusCheck">
        <isStillMode/>
        <isZigbeeModuleFault/>
      </Fallback>
      <Fallback name="robotStillBehavior">
        <executeRobotStillBehavior/>
        <IfThenElse name="checkDirectCommand">
          <isDirectedCommandAllowedInStillMode/>
          <executeCloudDirectCommand/>
          <executeRobotStillBehavior/>
        </IfThenElse>
      </Fallback>
      <SubTree ID="scheduleTree"/>
    </IfThenElse>
  </BehaviorTree>
</root>
 )";

void signalHandle(std::string file_name)
{
  if (auto nodeHandle = cti::missionSchedule::common::getContainer()->resolveOrNull<ros::NodeHandle>())
  {
    nodeHandle->shutdown();
  }
  ros::shutdown();
  isTerminated = true;

  if (auto ioservice = cti::missionSchedule::common::getContainer()->resolveOrNull<boost::asio::io_service>())
  {
    ioservice->stop();
  }
}

void runtimeInjectorInitialize(std::shared_ptr<ros::NodeHandle> nodeHandle)
{
  auto container = cti::missionSchedule::common::getContainer();
  container->assemble(nodeHandle);
  auto ioService = std::make_shared<boost::asio::io_service>();
  container->assemble(ioService);
  container->assemble(std::make_shared<boost::asio::io_service::work>(*ioService));
  auto deviceUtility = cti::missionSchedule::createRuntimeStatusMonitor();
  container->assemble(deviceUtility);
  auto pathPlanner = cti::missionSchedule::createPathPlanner();
  container->assemble(pathPlanner);
  auto elevatorPlanner = cti::missionSchedule::createElevatorPlanner();
  container->assemble(elevatorPlanner);
  auto platformMissionCenter = cti::missionSchedule::createPlatformMissionCenter();
  container->assemble(platformMissionCenter);
  auto missionPlanner = cti::missionSchedule::createMissionPlanner();
  container->assemble(missionPlanner);
  SPDLOG_INFO("MissionSchedule initialize injector container Ok!");
}

int main(int argc, char** argv)
{
  BT::NodeStatus scheduleStatus;
  ros::init(argc, argv, PROJECT_NAME);
  auto nodeHandle = std::make_shared<ros::NodeHandle>("~");
  // struct sigaction sigIntHandler;
  // sigIntHandler.sa_handler = signalHandle;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;
  // sigaction(SIGINT,  &sigIntHandler, NULL);
  // sigaction(SIGABRT, &sigIntHandler, NULL);
  // sigaction(SIGTRAP, &sigIntHandler, NULL);
  // sigaction(SIGSEGV, &sigIntHandler, NULL);
  // sigaction(SIGUSR1, &sigIntHandler, NULL);

  cti::buildingrobot::log::SpdLog log(PROJECT_NAME, 100 * 1024 * 1024, 20);

  std::string robot_num;

  nodeHandle->getParam("/robot_attribute/number", robot_num);
  cti::robot::registerSignalHandler(robot_num, "mission_schedule", signalHandle);

  runtimeInjectorInitialize(nodeHandle);
  SPDLOG_INFO(" ====> MISSION SCHEDULE NODE STARTUP @ v{}{}", MAIN_VERSION_INFO, PATCH_VERSION_INFO);
  SPDLOG_INFO("compiled time: {}, branch: {}, user: {}, email: {}", COMPILED_TIME, GIT_BRANCH, GIT_USER_NAME, GIT_USER_EMAIL);


  BT::BehaviorTreeFactory factory;
  cti::missionSchedule::registerNodes(factory);
  auto scheduleTree = factory.createTreeFromText(main_tree_xml_text);
  SPDLOG_INFO("MissionSchedule create schedule tree Ok!");

  ros::AsyncSpinner spinner(2); // Use 4 threads
  spinner.start();

  auto missionPlanner = cti::missionSchedule::common::getContainer()->resolveOrNull<cti::missionSchedule::IMissionPlanner>();
  auto relocateJson = missionPlanner->readRelocateFromLocalStorage();
  if (!relocateJson.empty())
  {
    SPDLOG_INFO("MissionSchedule get relocate json :{}", relocateJson.dump());
  }

  auto deviceUtility = cti::missionSchedule::common::getContainer()->resolveOrNull<cti::missionSchedule::IDeviceRuntimeUtility>();
  int waitingRelocateTime = 0;
  while (!isTerminated && ((!deviceUtility->getRobotInfo()) || (!deviceUtility->getRobotInfo()->localizedSet()) || !deviceUtility->getRobotInfo()->localized()))
  {
    SPDLOG_INFO("MissionSchedule wait for starting!");
    std::this_thread::sleep_for(5s);
    waitingRelocateTime += 5;
    if (!relocateJson.empty() && waitingRelocateTime >= 20)
    {
      missionPlanner->publishRelocateCommand(relocateJson);
      waitingRelocateTime = 0;
    }
    if (relocateJson.contains("hiveId") && relocateJson.at("hiveId").is_string() && !relocateJson.at("hiveId").get<std::string>().empty())
    {
      if (auto robotInfo = deviceUtility->getRobotInfo())
      {
        if (!robotInfo->hiveAttachedSet() || !robotInfo->hiveAttached())
        {
          missionPlanner->bindLocalHive(true, relocateJson.at("hiveId").get<std::string>());
        }
      }
    }
  }

  auto ioService = cti::missionSchedule::common::getContainer()->resolveOrNull<boost::asio::io_service>();
  boost::asio::io_service::work tmpWork(*ioService);
  boost::thread newThread(boost::bind(&boost::asio::io_service::run, ioService));

  SPDLOG_INFO("MissionSchedule start scheduling!");

  ros::Rate loopRate(1);
  while (!isTerminated)
  {
    scheduleStatus = scheduleTree.tickRoot();
    loopRate.sleep();
  }

  newThread.join();
  return 0;
}
