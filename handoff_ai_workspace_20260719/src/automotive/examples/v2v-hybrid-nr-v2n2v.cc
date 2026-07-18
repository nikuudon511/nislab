/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * NR migration workspace for thesis evaluation.
 *
 * This file is split from v2v-hybrid-cbr-80211p-nrv2x.cc so the existing
 * 802.11p/LTE baseline remains reproducible while NR sidelink V2V and NR Uu
 * based V2N2V are migrated incrementally.
 */

#include "ns3/antenna-module.h"
#include "ns3/application.h"
#include "ns3/BSContainer.h"
#include "ns3/command-line.h"
#include "ns3/config-store-module.h"
#include "ns3/core-module.h"
#include "ns3/DCC.h"
#include "ns3/geonet.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/packet-socket-address.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/traci-module.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-helper.h"

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VHybridNrV2n2v");

enum class ActiveRoute
{
  DsrcV2v,
  NrSidelinkV2v,
  MecV2n2v
};

enum class MecRecoveryPolicy
{
  OffloadOnly,
  Staged,
  DuplicateAlways
};

enum class MecObjectPolicy
{
  AllObjects,
  HighPriorityOnly,
  AdaptiveProbability
};

struct VehicleRuntime
{
  Ptr<Node> node;
  Ptr<BSContainer> dsrcContainer;
  Ptr<BSContainer> nrContainer;
  Ptr<BSContainer> mecContainer;
  Ptr<DCC> dcc;
  ActiveRoute activeRoute = ActiveRoute::DsrcV2v;
  bool dsrcTxActive = false;
  bool nrTxActive = false;
  bool mecTxActive = false;
  bool sendCpm = true;
  Time mecHoldUntil = Seconds (0.0);
};

struct ThesisEvaluationStats
{
  double recognitionRatioSum = 0.0;
  uint64_t recognitionVehicleSamples = 0;
  double highPriorityRecognitionRatioSum = 0.0;
  uint64_t highPriorityRecognitionVehicleSamples = 0;
  double lowPriorityRecognitionRatioSum = 0.0;
  uint64_t lowPriorityRecognitionVehicleSamples = 0;
  double sensorRecognitionRatioSum = 0.0;
  uint64_t sensorRecognitionVehicleSamples = 0;
  double sensorHighPriorityRecognitionRatioSum = 0.0;
  uint64_t sensorHighPriorityRecognitionVehicleSamples = 0;
  double sensorLowPriorityRecognitionRatioSum = 0.0;
  uint64_t sensorLowPriorityRecognitionVehicleSamples = 0;
  double cooperativeRecognitionRatioSum = 0.0;
  uint64_t cooperativeRecognitionVehicleSamples = 0;
  double ttlViolationRatioSum = 0.0;
  uint64_t ttlViolationVehicleSamples = 0;
  double neverReceivedRatioSum = 0.0;
  uint64_t neverReceivedVehicleSamples = 0;
  double highPriorityTtlViolationRatioSum = 0.0;
  uint64_t highPriorityTtlViolationVehicleSamples = 0;
  double highPriorityNeverReceivedRatioSum = 0.0;
  uint64_t highPriorityNeverReceivedVehicleSamples = 0;
  double lowPriorityTtlViolationRatioSum = 0.0;
  uint64_t lowPriorityTtlViolationVehicleSamples = 0;
  double lowPriorityNeverReceivedRatioSum = 0.0;
  uint64_t lowPriorityNeverReceivedVehicleSamples = 0;
  uint64_t dsrcLastCpmRx = 0;
  uint64_t nrLastCpmRx = 0;
  uint64_t mecLastCpmRx = 0;
  uint64_t dsrcIdealCpmRx = 0;
  uint64_t dsrcTrueCpmRx = 0;
  uint64_t nrIdealCpmRx = 0;
  uint64_t nrTrueCpmRx = 0;
  uint64_t mecIdealCpmRx = 0;
  std::array<double, 4> v2vAoiRecognitionRatioSum = {{0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 4> v2vAoiRecognitionVehicleSamples = {{0, 0, 0, 0}};
  std::array<double, 4> v2vOnlyAoiRecognitionRatioSum = {{0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 4> v2vOnlyAoiRecognitionVehicleSamples = {{0, 0, 0, 0}};
  std::array<double, 4> mecAoiRecognitionRatioSum = {{0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 4> mecAoiRecognitionVehicleSamples = {{0, 0, 0, 0}};
  std::array<double, 4> mecOnlyAoiRecognitionRatioSum = {{0.0, 0.0, 0.0, 0.0}};
  std::array<uint64_t, 4> mecOnlyAoiRecognitionVehicleSamples = {{0, 0, 0, 0}};
  uint64_t mecTrueCpmRx = 0;
  uint64_t idealMecLastHighAttempts = 0;
  uint64_t idealMecLastHighSuccesses = 0;
  uint64_t idealMecLastLowAttempts = 0;
  uint64_t idealMecLastLowSuccesses = 0;
  uint64_t nrUpdateExpected = 0;
  uint64_t nrUpdateSuccess = 0;
  uint64_t mecUpdateExpected = 0;
  uint64_t mecUpdateSuccess = 0;
  uint64_t rmrDeletedEvalExpectedTotal = 0;
  uint64_t rmrDeletedEvalExpectedHigh = 0;
  uint64_t rmrDeletedEvalExpectedLow = 0;
  uint64_t rmrDeletedEvalUnrecognizedTotal = 0;
  uint64_t rmrDeletedEvalUnrecognizedHigh = 0;
  uint64_t rmrDeletedEvalUnrecognizedLow = 0;
  double receiverFreshRedundancy200Sum = 0.0;
  double receiverFreshRedundancy500Sum = 0.0;
  double receiverFreshRedundancyHigh200Sum = 0.0;
  double receiverFreshRedundancyLow200Sum = 0.0;
  double receiverRv200Sum = 0.0;
  double receiverRlSum = 0.0;
  double receiverRlMedianSum = 0.0;
  double receiverRvDeloozSum = 0.0;
  double receiverRvDeloozMedianSum = 0.0;
  double receiverHighRlSum = 0.0;
  double receiverLowRlSum = 0.0;
  uint64_t receiverRlSamples = 0;
  uint64_t receiverRlVehicleSamples = 0;
  uint64_t receiverHighRlSamples = 0;
  uint64_t receiverLowRlSamples = 0;
  uint64_t receiverRlLt1 = 0;
  uint64_t receiverRlGe1 = 0;
  uint64_t receiverRlGe2 = 0;
  uint64_t receiverFreshRedundancySamples = 0;
  uint64_t receiverFreshRedundancyHighSamples = 0;
  uint64_t receiverFreshRedundancyLowSamples = 0;
  uint64_t receiverFreshRedundancyGe2_200 = 0;
  uint64_t receiverFreshRedundancyGe2_500 = 0;
  double rmrDeletedEvalDistanceSum = 0.0;
  uint64_t rmrDeletedEvalDistanceSamples = 0;
  uint64_t rmrDeletedEvalFreshRedundancyGe2_200 = 0;
  uint64_t rmrDeletedEvalFreshRedundancyGe2_500 = 0;
  uint64_t rmrDeletedEvalTtcLeThreshold = 0;
  uint64_t rmrDeletedEvalClosingSpeedGeThreshold = 0;
  uint64_t rmrDeletedSenderEvalTotal = 0;
  uint64_t rmrDeletedSenderEvalHigh = 0;
  uint64_t rmrDeletedSenderEvalLow = 0;
  double rmrDeletedSenderEvalDistanceSum = 0.0;
  uint64_t rmrDeletedSenderEvalDistanceSamples = 0;
  uint64_t rmrDeletedSenderEvalTtcLeThreshold = 0;
  uint64_t rmrDeletedSenderEvalClosingSpeedGeThreshold = 0;
  uint64_t highIdealCpmRx = 0;
  uint64_t highTrueCpmRx = 0;
  uint64_t lowIdealCpmRx = 0;
  uint64_t lowTrueCpmRx = 0;
  uint64_t nrHighIdealCpmRx = 0;
  uint64_t nrHighTrueCpmRx = 0;
  uint64_t nrLowIdealCpmRx = 0;
  uint64_t nrLowTrueCpmRx = 0;
  std::unordered_map<std::string, uint64_t> dsrcLastCpmTxByVehicle;
  std::unordered_map<std::string, uint64_t> nrLastCpmTxByVehicle;
  std::unordered_map<std::string, uint64_t> mecLastCpmTxByVehicle;
};

struct ReceiverPriorityCounts
{
  uint32_t high = 0;
  uint32_t low = 0;

  uint32_t Total () const
  {
    return high + low;
  }
};

struct IdealMecVehicleSnapshot
{
  std::string vehicleId;
  libsumo::TraCIPosition position;
  double loopPositionMeters = 0.0;
};

struct IdealMecLoopIndexEntry
{
  double loopPositionMeters = 0.0;
  std::size_t vehicleIndex = 0;
};

struct SensorExternalEventState
{
  bool triggered = false;
  bool recognized = false;
  bool completed = false;
  bool hasPreviousSample = false;
  double previousDistanceMeters = 0.0;
  Time previousSampleTime = Seconds (0.0);
  Time triggerTime = Seconds (0.0);
};

struct PredictiveCbrState
{
  bool initialized = false;
  Time lastUpdate = Seconds (0.0);
  double lastCbr = -1.0;
  double cbrRatePerSecond = 0.0;
  double predictedCbr = -1.0;
  uint64_t lastDsrcCpmTx = 0;
  double dsrcCpmTxRate = 0.0;
  uint32_t lastDsrcCpmSizeBytes = 0;
};

struct TrafficFlowPredictionState
{
  bool initialized = false;
  Time lastUpdate = Seconds (0.0);
  uint32_t lastActiveVehicles = 0;
  double qInVehiclesPerSecond = 0.0;
  double qOutVehiclesPerSecond = 0.0;
  double trafficFlowCbr = 0.0;
  double deltaCbr = 0.0;
  double meanSpeedMetersPerSecond = 0.0;
  uint32_t rsuCount = 0;
  double localVehicleCountAvg = 0.0;
  double upstreamVehicleCountAvg = 0.0;
  double futureVehicleCountAvg = 0.0;
  double rsuPredictedCbrAvg = 0.0;
  double rsuPredictedCbrMax = 0.0;
  uint32_t i2vVehicleCount = 0;
  double i2vPredictedCbrAvg = 0.0;
  double i2vPredictionLeadTimeAvg = 0.0;
};

struct TrafficFlowRsuInfo
{
  std::string id;
  double x = 0.0;
  double y = 0.0;
};

struct TrafficFlowRsuPrediction
{
  double measuredCbr = 0.0;
  double predictedCbr = 0.0;
  double peakTimeSeconds = 0.0;
  uint32_t localVehicles = 0;
  double futureVehicles = 0.0;
};

struct TrafficFlowVehicleSnapshot
{
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double loopPositionMeters = 0.0;
  double speedMps = 0.0;
  int direction = 0;
};

struct TrafficFlowSegmentStats
{
  uint32_t vehicleCount = 0;
  double averageSpeedMps = 0.0;
  double cbr = 0.0;
  double observedCpmTxRateHz = 0.0;
  double observedCpmSizeBytes = 0.0;
};

struct MovingBackgroundConfig
{
  uint32_t packetSize = 0;
  Time packetInterval = Seconds (0.0);
  Time stopTime = Seconds (0.0);
  bool enabled = false;
  bool fixedZones = false;
  double periodSeconds = 20.0;
  double amplitude = 0.8;
  double speedMps = 25.0;
  double minFactor = 0.2;
  double widthMeters = 500.0;
  double roadLengthMeters = 3000.0;
  uint64_t generation = 0;
};

struct PriorityMotionState
{
  Vector position;
  Vector velocity;
  Time timestamp = Seconds (0.0);
  bool hasVelocity = false;
};

struct PredictiveRmrConfig
{
  double predictionHorizonSeconds = 20.0;
  double cpmSizeNormBytes = 1200.0;
  double cpmTxRateNorm = 10.0;
  double activeVehicleNorm = 100.0;
  double trafficFlowRoadLengthMeters = 2000.0;
  double trafficFlowMessageRateHz = 10.0;
  double trafficFlowAvgPacketSizeBytes = 500.0;
  double trafficFlowChannelRateMbps = 6.0;
  double rsuPassivePdrLowCbr = 0.95;
  double rsuPassivePdrMidCbr = 0.90;
  double rsuPassiveSaturationCbr = 0.90;
  double rsuI2vRangeMeters = 300.0;
  double cpmSizeWeight = 0.0;
  double cpmTxRateWeight = 0.0;
  double activeVehicleWeight = 0.0;
};

struct HybridRouteConfig
{
  double switchCbr = 0.60;
  double releaseCbr = 0.50;
  double maxCbr = 0.80;
  double lowV2vMinProbability = 0.20;
  double lowV2vAlpha = 1.0;
  bool highPriorityDualTx = true;
  MecRecoveryPolicy mecRecoveryPolicy = MecRecoveryPolicy::Staged;
  double mecMinHoldTimeSeconds = 0.0;
};

static std::unordered_map<std::string, VehicleRuntime> g_vehicleRuntime;
static std::unordered_map<std::string, uint64_t> g_backgroundGenerationByVehicle;
static std::unordered_map<std::string, Ptr<Node>> g_allVehicleNodes;
static std::unordered_map<uint64_t, std::string> g_vehicleIdByStationId;
static std::unordered_map<std::string, PredictiveCbrState> g_predictiveCbrState;
static TrafficFlowPredictionState g_trafficFlowPrediction;
static PredictiveRmrConfig g_predictiveRmrConfig;
static HybridRouteConfig g_hybridRouteConfig;
static std::vector<TrafficFlowRsuInfo> g_trafficFlowRsus;
static bool g_enablePredictiveRmrControl = false;
static std::string g_fixedRsuCbrMode = "disabled";
static const std::array<double, 6> g_fixedRsuCbrValues = {{0.4, 0.8, 0.4, 0.9, 0.6, 0.4}};
static std::unordered_map<std::string, TrafficFlowRsuPrediction>
    g_trafficFlowRsuPredictions;
static std::unordered_map<std::string, uint32_t> g_trafficFlowRsuI2vCounts;
static std::ofstream g_routeLog;
static std::ofstream g_observationLog;
static std::ofstream g_rsuPredictionLog;
static std::ofstream g_cpmInputDiagLog;
static ThesisEvaluationStats g_thesisStats;
static uint32_t g_thesisEvalStartMinVehicles = 0;
static bool g_thesisEvalStartUseAllVehicles = true;
static bool g_thesisEvaluationStarted = false;
static bool g_holdTrafficUntilEvaluationStart = false;
static bool g_trafficReleased = false;
static Time g_thesisEvaluationStartTime = Seconds (0.0);
static double g_thesisEvalWarmupSeconds = 5.0;
static uint64_t g_camRx = 0;
static uint64_t g_cpmRx = 0;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>> g_latestCpmRxByReceiver;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>>
    g_latestNrCpmRxByReceiver;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>>
    g_latestNrCpmGenerationTimeByReceiver;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>>
    g_latestMecCpmRxByReceiver;
static std::array<std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>>, 4>
    g_latestMecCpmRxByReceiverAoiThreshold;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>>
    g_latestMecCpmGenerationTimeByReceiver;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>>
    g_latestActualCpmObjectRxByReceiver;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, std::deque<Time>>>
    g_cpmObjectUpdateHistoryByReceiver;
static uint64_t g_interferenceTx = 0;
static uint64_t g_interferenceBytes = 0;
static uint64_t g_interferenceDrops = 0;
static bool g_nrBgMovingWaveForLog = false;
static double g_nrBgWaveSpeedMpsForLog = 25.0;
static double g_nrBgWaveWidthMetersForLog = 500.0;
static double g_trafficFlowRoadLengthMetersForLog = 3000.0;
static uint64_t g_mecUplinkPackets = 0;
static uint64_t g_mecUplinkBytes = 0;
static uint64_t g_mecForwardedPackets = 0;
static uint64_t g_mecForwardedBytes = 0;
static uint64_t g_lastMecUplinkBytesForBusyRatio = 0;
static uint64_t g_lastMecForwardedBytesForBusyRatio = 0;
static uint64_t g_mecBackgroundUplinkBytes = 0;
static uint64_t g_mecBackgroundDownlinkBytes = 0;
static std::vector<double> g_mecUlBusyRatioSamples;
static std::vector<double> g_mecDlBusyRatioSamples;
static uint64_t g_mecForwardDrops = 0;
static uint64_t g_mecForwardNoReceiver = 0;
static uint64_t g_idealMecTx = 0;
static uint64_t g_idealMecRx = 0;
static uint64_t g_idealMecHighAttempts = 0;
static uint64_t g_idealMecHighSuccesses = 0;
static uint64_t g_idealMecLowAttempts = 0;
static uint64_t g_idealMecLowSuccesses = 0;
static uint32_t g_idealMecPacketSizeBytes = 500;
static Time g_idealMecLatency = MilliSeconds (39.4);
static std::string g_idealMecDelayModel = "empirical";
static double g_idealMecLatencyMeanMs = 39.4;
static double g_idealMecLatencyStddevMs = 20.0;
static double g_idealMecLatencyMinMs = 6.1;
static double g_idealMecLatencyP90Ms = 59.86;
static double g_idealMecLatencyP99Ms = 120.33;
static double g_idealMecLatencyMaxMs = 201.0;
static Time g_idealMecInterval = MilliSeconds (100);
static double g_mecForwardRangeMeters = 150.0;
static bool g_useIdealMecLink = true;
static bool g_idealMecCapacityModel = false;
static double g_idealMecBandwidthMHz = 30.0;
static double g_idealMecSpectralEfficiencyBpsHz = 4.5234;
static double g_idealMecUplinkCapacityMbps = 135.702;
static double g_idealMecDownlinkCapacityMbps = 135.702;
static double g_idealMecMaxQueueDelayMs = 0.0;
static Time g_idealMecUplinkAvailableAt = Seconds (0.0);
static Time g_idealMecDownlinkAvailableAt = Seconds (0.0);
static std::vector<double> g_idealMecCapacityDelaySamplesMs;
static uint64_t g_idealMecCapacityQueueDrops = 0;
static double g_idealMecUlFirstLossRate = 0.0;
static double g_idealMecDlFirstLossRate = 0.0;
static double g_idealMecRetxSuccessProbability = 1.0;
static uint32_t g_idealMecMaxRetransmissions = 0;
static double g_idealMecRetxDelayMs = 8.0;
static uint64_t g_idealMecUlFirstLosses = 0;
static uint64_t g_idealMecDlFirstLosses = 0;
static uint64_t g_idealMecUlRetxRecovered = 0;
static uint64_t g_idealMecDlRetxRecovered = 0;
static uint64_t g_idealMecUlFinalLosses = 0;
static uint64_t g_idealMecDlFinalLosses = 0;
static std::vector<double> g_idealMecRetxDelaySamplesMs;
static MecObjectPolicy g_mecObjectPolicy = MecObjectPolicy::AllObjects;
static double g_mecAdaptiveLowMaxProbability = 0.5;
static Ptr<NormalRandomVariable> g_idealMecLatencyRv;
static Ptr<UniformRandomVariable> g_idealMecLatencyUniformRv;
static std::vector<double> g_idealMecLatencySamplesMs;
static std::vector<double> g_idealMecAoiSamplesMs;
static std::vector<double> g_v2vAoiSamplesMs;
static uint64_t g_v2vAoiSamples = 0;
static std::array<uint64_t, 4> g_v2vAoiWithinThreshold = {{0, 0, 0, 0}};
static uint64_t g_idealMecAoiSamples = 0;
static uint64_t g_idealMecHighAoiSamples = 0;
static uint64_t g_idealMecLowAoiSamples = 0;
static std::array<uint64_t, 4> g_idealMecAoiWithinThreshold = {{0, 0, 0, 0}};
static std::array<uint64_t, 4> g_idealMecHighAoiWithinThreshold = {{0, 0, 0, 0}};
static std::array<uint64_t, 4> g_idealMecLowAoiWithinThreshold = {{0, 0, 0, 0}};
static const std::array<double, 4> g_idealMecAoiThresholdsMs = {{200.0, 300.0, 400.0, 500.0}};
static bool g_idealMecAoiFilter = false;
static double g_idealMecAoiFilterThresholdMs = 200.0;
static bool g_mecBackgroundLoad = false;
static bool g_mecBackgroundPerVehicle = true;
static uint32_t g_mecBackgroundPacketSizeBytes = 500;
static Time g_mecBackgroundInterval = MilliSeconds (100);
static std::unordered_map<uint32_t, std::string> g_mecVehicleIdByIp;
static std::unordered_map<std::string, Ipv4Address> g_mecIpByVehicleId;
static Ptr<TraciClient> g_sumoClient;
static double g_orrRangeMeters = 100.0;
static double g_priorityDistanceThresholdMeters = 100.0;
static double g_priorityClosingSpeedThresholdMps = 3.0;
static double g_priorityTtcThresholdSeconds = 5.0;
static double g_sensorRangeMeters = 100.0;
static double g_highPriorityCpmRecognitionTtlSeconds = 0.5;
static double g_lowPriorityCpmRecognitionTtlSeconds = 1.0;
static std::unordered_map<uint64_t, PriorityMotionState> g_priorityMotionHistory;
static bool g_dsrcInterferencePerVehicle = false;
static double g_dsrcInterferenceNodeOfferedCbr = 0.0;
static double g_dsrcInterferenceOfferedCbr = 0.0;
static double g_predictiveChannelRateBps = 6.0e6;
static bool g_trafficFlowIsLoop = true;
static Ptr<UniformRandomVariable> g_hybridRouteRandom;
static uint32_t g_maxCommunicationVehicles = 0;
static uint32_t g_communicationVehicleCount = 0;
static double g_cpmStartDelaySeconds = 0.0;
static uint64_t g_dsrcCpmStartCalls = 0;
static uint64_t g_nrCpmStartCalls = 0;
static uint64_t g_mecCpmStartCalls = 0;
static double g_idealMecDownlinkPdr = 1.0;
static Ptr<UniformRandomVariable> g_idealMecDownlinkRandom;
static Ptr<UniformRandomVariable> g_idealMecRadioRandom;
static bool g_sensorExternalEventsEnabled = false;
static double g_sensorExternalEventProbability = 0.20;
static double g_sensorExternalEventActivationRangeMeters = 400.0;
static std::unordered_set<uint64_t> g_nonCavStationIds;
static std::unordered_map<uint64_t, SensorExternalEventState> g_sensorExternalEvents;
static uint64_t g_sensorExternalEventCount = 0;
static uint64_t g_sensorExternalEventRecognized = 0;
static uint64_t g_sensorExternalEventMissed = 0;
static uint64_t g_sensorExternalEventCandidateVehicles = 0;
static uint64_t g_sensorExternalEventSelectedVehicles = 0;
static std::unordered_set<uint64_t> g_sensorExternalEventCandidateVehicleIds;
static std::unordered_set<uint64_t> g_sensorExternalEventSelectedVehicleIds;
static std::unordered_set<uint64_t> g_sensorExternalEventActivatedVehicleIds;
static std::unordered_map<uint64_t, Time> g_sensorExternalEventActivationTimes;
static std::unordered_map<uint64_t, Time> g_sensorExternalEventEndTimes;
static std::vector<double> g_sensorExternalEventDelaysMs;

static double ComputePredictiveRmrActionProbability (double predictedCbr);
static void SetRmrActionProbability (VehicleRuntime& runtime, double probability);
static bool IsClosingHighPriorityObject (uint64_t egoStationId,
                                         uint64_t objectStationId,
                                         const Vector& egoPosition,
                                         const Vector& objectPosition,
                                         double distanceMeters,
                                         Time now);

static std::string
RouteName (ActiveRoute route)
{
  if (route == ActiveRoute::DsrcV2v)
    {
      return "LEGACY_V2V";
    }
  if (route == ActiveRoute::NrSidelinkV2v)
    {
      return "NR_SIDELINK_V2V";
    }
  return "MEC_V2N2V";
}

static double AverageLatestPrimaryCpmSizeBytes ();

static std::string
RouteStateName (bool dsrcActive, bool nrActive, bool mecActive)
{
  std::string name;
  if (dsrcActive)
    {
      name += "LEGACY_V2V";
    }
  if (nrActive)
    {
      if (!name.empty ())
        {
          name += "+";
        }
      name += "NR_SIDELINK_V2V";
    }
  if (mecActive)
    {
      if (!name.empty ())
        {
          name += "+";
        }
      name += "MEC_V2N2V";
    }
  return name.empty () ? "NONE" : name;
}

static std::string
MecRecoveryPolicyName (MecRecoveryPolicy policy)
{
  if (policy == MecRecoveryPolicy::OffloadOnly)
    {
      return "offload-only";
    }
  if (policy == MecRecoveryPolicy::DuplicateAlways)
    {
      return "duplicate-always";
    }
  return "staged";
}

static MecRecoveryPolicy
ParseMecRecoveryPolicy (const std::string& policy)
{
  if (policy == "offload-only")
    {
      return MecRecoveryPolicy::OffloadOnly;
    }
  if (policy == "staged")
    {
      return MecRecoveryPolicy::Staged;
    }
  if (policy == "duplicate-always")
    {
      return MecRecoveryPolicy::DuplicateAlways;
    }
  NS_FATAL_ERROR ("Unknown --mec-recovery-policy: " << policy
                                                    << " (use offload-only, staged, or duplicate-always)");
}

static MecObjectPolicy
ParseMecObjectPolicy (const std::string& policy)
{
  if (policy == "all-objects")
    {
      return MecObjectPolicy::AllObjects;
    }
  if (policy == "high-priority-only")
    {
      return MecObjectPolicy::HighPriorityOnly;
    }
  if (policy == "adaptive-probability")
    {
      return MecObjectPolicy::AdaptiveProbability;
    }
  NS_FATAL_ERROR ("Unknown --mec-object-policy: " << policy
                                                  << " (use all-objects, high-priority-only, or adaptive-probability)");
}

static double
ComputeMecAdaptiveLowProbability (const std::string& senderVehicleId)
{
  double predictedCbr = g_hybridRouteConfig.switchCbr;
  const auto predictiveIt = g_predictiveCbrState.find (senderVehicleId);
  if (predictiveIt != g_predictiveCbrState.end () && predictiveIt->second.initialized)
    {
      predictedCbr = predictiveIt->second.predictedCbr;
    }

  const double span = std::max (g_hybridRouteConfig.maxCbr - g_hybridRouteConfig.switchCbr, 1e-9);
  const double normalized =
      (g_hybridRouteConfig.maxCbr - predictedCbr) / span;
  return std::max (0.0,
                   std::min (g_mecAdaptiveLowMaxProbability,
                             g_mecAdaptiveLowMaxProbability * normalized));
}

static void
UpdateDsrcInterferenceOfferedCbr ()
{
  if (g_dsrcInterferencePerVehicle)
    {
      g_dsrcInterferenceOfferedCbr =
          g_dsrcInterferenceNodeOfferedCbr * static_cast<double> (g_vehicleRuntime.size ());
    }
}

static void
ReceiveCAM (asn1cpp::Seq<CAM> cam,
            Address from,
            StationID_t myStationId,
            StationType_t myStationType,
            SignalInfo phyInfo)
{
  (void) cam;
  (void) from;
  (void) myStationId;
  (void) myStationType;
  (void) phyInfo;
  ++g_camRx;
}

static void
MarkCpmObjectsRecognizedByReceiver (uint64_t receiverStationId,
                                    uint64_t senderStationId,
                                    Time now);

static void
RecordCpmObjectUpdate (uint64_t receiverStationId, uint64_t objectStationId, Time now);

static void
ReceiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
            Address from,
            StationID_t myStationId,
            StationType_t myStationType,
            SignalInfo phyInfo,
            ActiveRoute route)
{
  (void) from;
  (void) myStationType;
  (void) phyInfo;
  auto* routeUpdates =
      route == ActiveRoute::NrSidelinkV2v
          ? &g_latestNrCpmRxByReceiver
          : (route == ActiveRoute::MecV2n2v ? &g_latestMecCpmRxByReceiver : nullptr);
  const uint64_t receiverStationId = static_cast<uint64_t> (myStationId);
  const Time now = Simulator::Now ();
  Time generatedAt = now;
  if (route == ActiveRoute::NrSidelinkV2v && phyInfo.timestamp > 0.0)
    {
      generatedAt = NanoSeconds (static_cast<int64_t> (phyInfo.timestamp));
      if (generatedAt > now)
        {
          generatedAt = now;
        }
    }
  const double v2vAoiMs =
      route == ActiveRoute::NrSidelinkV2v
          ? std::max (0.0, static_cast<double> ((now - generatedAt).GetMilliSeconds ()))
          : 0.0;
  if (cpm->header.stationId > 0 &&
      static_cast<uint64_t> (cpm->header.stationId) != receiverStationId)
    {
      MarkCpmObjectsRecognizedByReceiver (receiverStationId,
                                          static_cast<uint64_t> (cpm->header.stationId),
                                          now);
      if (routeUpdates != nullptr)
        {
          (*routeUpdates)[receiverStationId][static_cast<uint64_t> (cpm->header.stationId)] = now;
          if (route == ActiveRoute::NrSidelinkV2v)
            {
              g_latestNrCpmGenerationTimeByReceiver[receiverStationId]
                                                    [static_cast<uint64_t> (cpm->header.stationId)] =
                                                        generatedAt;
              g_v2vAoiSamplesMs.push_back (v2vAoiMs);
              ++g_v2vAoiSamples;
              for (std::size_t i = 0; i < g_idealMecAoiThresholdsMs.size (); ++i)
                {
                  if (v2vAoiMs <= g_idealMecAoiThresholdsMs[i])
                    {
                      ++g_v2vAoiWithinThreshold[i];
                    }
                }
            }
          const int containerCount =
              asn1cpp::sequenceof::getSize (cpm->payload.cpmContainers);
          for (int i = 0; i < containerCount; ++i)
            {
              auto container = asn1cpp::sequenceof::getSeq (
                  cpm->payload.cpmContainers, WrappedCpmContainer, i);
              if (asn1cpp::getField (
                      container->containerData.present,
                      WrappedCpmContainer__containerData_PR) !=
                  WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
                {
                  continue;
                }
              auto perceivedObjects = asn1cpp::getSeq (
                  container->containerData.choice.PerceivedObjectContainer,
                  PerceivedObjectContainer);
              const int objectCount =
                  asn1cpp::sequenceof::getSize (perceivedObjects->perceivedObjects);
              for (int j = 0; j < objectCount; ++j)
                {
                  auto object = asn1cpp::sequenceof::getSeq (
                      perceivedObjects->perceivedObjects, PerceivedObject, j);
                  const long objectId = asn1cpp::getField (object->objectId, long);
                  if (objectId > 0 && static_cast<uint64_t> (objectId) != receiverStationId)
                    {
                      g_latestCpmRxByReceiver[receiverStationId]
                                                   [static_cast<uint64_t> (objectId)] = now;
                      g_latestActualCpmObjectRxByReceiver[receiverStationId]
                                                          [static_cast<uint64_t> (objectId)] = now;
                      RecordCpmObjectUpdate (receiverStationId,
                                             static_cast<uint64_t> (objectId),
                                             now);
                      (*routeUpdates)[receiverStationId][static_cast<uint64_t> (objectId)] = now;
                      if (route == ActiveRoute::NrSidelinkV2v)
                        {
                          g_latestNrCpmGenerationTimeByReceiver[receiverStationId]
                                                                [static_cast<uint64_t> (objectId)] =
                                                                    generatedAt;
                        }
                    }
                }
            }
        }
    }
  ++g_cpmRx;
}

static void
OpenRouteLog (const std::string& path)
{
  if (path.empty ())
    {
      return;
    }

  g_routeLog.open (path, std::ios::out);
  if (!g_routeLog.is_open ())
    {
      NS_FATAL_ERROR ("Unable to open route log file: " << path);
    }

  g_routeLog << "time_s,vehicle_id,cbr,route,reason" << std::endl;
}

static void
WriteRouteLog (const std::string& vehicleId,
               double cbr,
               ActiveRoute route,
               const std::string& reason)
{
  if (!g_routeLog.is_open ())
    {
      return;
    }

  g_routeLog << Simulator::Now ().GetSeconds () << "," << vehicleId << "," << cbr << ","
             << RouteName (route) << "," << reason << std::endl;
}

static void
WriteRouteStateLog (const std::string& vehicleId,
                    double cbr,
                    bool dsrcActive,
                    bool nrActive,
                    bool mecActive,
                    const std::string& reason)
{
  if (!g_routeLog.is_open ())
    {
      return;
    }

  g_routeLog << Simulator::Now ().GetSeconds () << "," << vehicleId << "," << cbr << ","
             << RouteStateName (dsrcActive, nrActive, mecActive) << "," << reason
             << std::endl;
}

static double
Clamp01 (double value)
{
  return std::max (0.0, std::min (1.0, value));
}

static double
PacketLossFromPrr (double prr, uint64_t txPackets)
{
  if (txPackets == 0)
    {
      return -1.0;
    }
  return Clamp01 (1.0 - prr);
}

static uint64_t
VehicleIdToStationId (const std::string& vehicleId)
{
  if (vehicleId.rfind ("veh.", 0) == 0)
    {
      return static_cast<uint64_t> (std::stoul (vehicleId.substr (4)) + 1);
    }
  return static_cast<uint64_t> (std::stoul (vehicleId.substr (3)));
}

static void
RecordCpmObjectUpdate (uint64_t receiverStationId, uint64_t objectStationId, Time now)
{
  auto& history = g_cpmObjectUpdateHistoryByReceiver[receiverStationId][objectStationId];
  history.push_back (now);
  const Time window = Seconds (1.0);
  while (!history.empty () && now - history.front () > window)
    {
      history.pop_front ();
    }
}

static uint32_t
CountRecentCpmObjectUpdates (uint64_t receiverStationId, uint64_t objectStationId, Time now)
{
  auto receiverIt = g_cpmObjectUpdateHistoryByReceiver.find (receiverStationId);
  if (receiverIt == g_cpmObjectUpdateHistoryByReceiver.end ())
    {
      return 0;
    }
  auto objectIt = receiverIt->second.find (objectStationId);
  if (objectIt == receiverIt->second.end ())
    {
      return 0;
    }
  auto& history = objectIt->second;
  const Time window = Seconds (1.0);
  while (!history.empty () && now - history.front () > window)
    {
      history.pop_front ();
    }
  return static_cast<uint32_t> (history.size ());
}

static double
ComputeDeloozRv (double rl)
{
  return std::exp (-7.0 * std::exp (-2.31337 * std::max (0.0, rl)));
}

static void
MarkCpmObjectsRecognizedByReceiver (
    uint64_t receiverStationId,
    uint64_t senderStationId,
    Time now,
    std::unordered_map<uint64_t, Time>& recognized);

static void
MarkCpmObjectsRecognizedByReceiver (uint64_t receiverStationId,
                                    uint64_t senderStationId,
                                    Time now)
{
  auto& recognized = g_latestCpmRxByReceiver[receiverStationId];
  MarkCpmObjectsRecognizedByReceiver (receiverStationId,
                                      senderStationId,
                                      now,
                                      recognized);
}

static void
MarkCpmObjectsRecognizedByReceiver (
    uint64_t receiverStationId,
    uint64_t senderStationId,
    Time now,
    std::unordered_map<uint64_t, Time>& recognized)
{
  recognized[senderStationId] = now;

  const auto senderVehicleIt = g_vehicleIdByStationId.find (senderStationId);
  if (senderVehicleIt == g_vehicleIdByStationId.end ())
    {
      return;
    }
  const auto senderNodeIt = g_allVehicleNodes.find (senderVehicleIt->second);
  if (senderNodeIt == g_allVehicleNodes.end () || senderNodeIt->second == nullptr)
    {
      return;
    }

  Ptr<MobilityModel> senderMobility = senderNodeIt->second->GetObject<MobilityModel> ();
  if (senderMobility == nullptr)
    {
      return;
    }

  for (const auto& entry : g_allVehicleNodes)
    {
      const uint64_t objectStationId = VehicleIdToStationId (entry.first);
      if (objectStationId == receiverStationId)
        {
          continue;
        }
      Ptr<MobilityModel> objectMobility =
          entry.second != nullptr ? entry.second->GetObject<MobilityModel> () : nullptr;
      if (objectMobility != nullptr &&
          senderMobility->GetDistanceFrom (objectMobility) <= g_sensorRangeMeters)
        {
          recognized[objectStationId] = now;
        }
    }
}

static double
DistanceMeters2d (const libsumo::TraCIPosition& a, const libsumo::TraCIPosition& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt ((dx * dx) + (dy * dy));
}

static double
ProjectLoopPositionMeters (const libsumo::TraCIPosition& position)
{
  if (!g_trafficFlowIsLoop)
    {
      return std::max (0.0,
                       std::min (g_predictiveRmrConfig.trafficFlowRoadLengthMeters,
                                 position.x));
    }

  const double sideMeters =
      std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters / 4.0, 1.0);
  auto clampSide = [sideMeters] (double value) {
    return std::max (0.0, std::min (sideMeters, value));
  };

  struct Candidate
  {
    double distanceToEdge;
    double loopPosition;
  };

  const std::array<Candidate, 4> candidates = {{
      {std::abs (position.y), clampSide (position.x)},
      {std::abs (position.x - sideMeters), sideMeters + clampSide (position.y)},
      {std::abs (position.y - sideMeters), (2.0 * sideMeters) + (sideMeters - clampSide (position.x))},
      {std::abs (position.x), (3.0 * sideMeters) + (sideMeters - clampSide (position.y))},
  }};

  double loopPosition =
      std::min_element (candidates.begin (),
                        candidates.end (),
                        [] (const Candidate& lhs, const Candidate& rhs) {
                          return lhs.distanceToEdge < rhs.distanceToEdge;
                        })
          ->loopPosition;
  const double loopLengthMeters = std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters, 1.0);
  loopPosition = std::fmod (loopPosition, loopLengthMeters);
  if (loopPosition < 0.0)
    {
      loopPosition += loopLengthMeters;
    }
  return loopPosition;
}

static bool
IsHighPriorityPair (const std::string& senderVehicleId,
                    const libsumo::TraCIPosition& senderPos,
                    const std::string& receiverVehicleId,
                    const libsumo::TraCIPosition& receiverPos,
                    double distanceMeters)
{
  if (distanceMeters <= g_priorityDistanceThresholdMeters)
    {
      return true;
    }

  const uint64_t senderStationId = VehicleIdToStationId (senderVehicleId);
  const uint64_t receiverStationId = VehicleIdToStationId (receiverVehicleId);
  const Vector senderVector (senderPos.x, senderPos.y, 0.0);
  const Vector receiverVector (receiverPos.x, receiverPos.y, 0.0);
  return IsClosingHighPriorityObject (senderStationId,
                                      receiverStationId,
                                      senderVector,
                                      receiverVector,
                                      distanceMeters,
                                      Simulator::Now ());
}

static ReceiverPriorityCounts
CountReceiversByPriorityWithinBaseline (Ptr<TraciClient> sumoClient,
                                        const std::string& senderVehicleId,
                                        double baselineMeters)
{
  if (sumoClient == nullptr || baselineMeters <= 0.0)
    {
      return {};
    }

  const libsumo::TraCIPosition senderPos =
      sumoClient->TraCIAPI::vehicle.getPosition (senderVehicleId);
  ReceiverPriorityCounts receivers;
  for (const auto& entry : g_vehicleRuntime)
    {
      if (entry.first == senderVehicleId)
        {
          continue;
        }

      const libsumo::TraCIPosition receiverPos =
          sumoClient->TraCIAPI::vehicle.getPosition (entry.first);
      const double distanceMeters = DistanceMeters2d (senderPos, receiverPos);
      if (distanceMeters <= baselineMeters)
        {
          if (IsHighPriorityPair (senderVehicleId,
                                  senderPos,
                                  entry.first,
                                  receiverPos,
                                  distanceMeters))
            {
              ++receivers.high;
            }
          else
            {
              ++receivers.low;
            }
        }
    }

  return receivers;
}

class MecV2n2vForwarder : public Application
{
public:
  static TypeId GetTypeId ()
  {
    static TypeId tid = TypeId ("ns3::MecV2n2vForwarder")
                            .SetParent<Application> ()
                            .SetGroupName ("Applications")
                            .AddConstructor<MecV2n2vForwarder> ();
    return tid;
  }

  void Configure (Ptr<TraciClient> sumoClient,
                  uint16_t listenPort,
                  uint16_t vehiclePort,
                  double forwardRangeMeters,
                  Time processingDelay)
  {
    m_sumoClient = sumoClient;
    m_listenPort = listenPort;
    m_vehiclePort = vehiclePort;
    m_forwardRangeMeters = forwardRangeMeters;
    m_processingDelay = processingDelay;
  }

private:
  void StartApplication () override
  {
    if (m_socket == nullptr)
      {
        m_socket = Socket::CreateSocket (GetNode (), TypeId::LookupByName ("ns3::UdpSocketFactory"));
        if (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_listenPort)) == -1)
          {
            NS_FATAL_ERROR ("Failed to bind MEC V2N2V UDP socket");
          }
      }
    m_socket->SetRecvCallback (MakeCallback (&MecV2n2vForwarder::HandleRead, this));
  }

  void StopApplication () override
  {
    if (m_socket != nullptr)
      {
        m_socket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket>> ());
        m_socket->Close ();
        m_socket = nullptr;
      }
  }

  void HandleRead (Ptr<Socket> socket)
  {
    Address from;
    Ptr<Packet> packet;
    while ((packet = socket->RecvFrom (from)) != nullptr)
      {
        if (!InetSocketAddress::IsMatchingType (from))
          {
            ++g_mecForwardDrops;
            continue;
          }

        const Ipv4Address senderIp = InetSocketAddress::ConvertFrom (from).GetIpv4 ();
        const auto senderIt = g_mecVehicleIdByIp.find (senderIp.Get ());
        if (senderIt == g_mecVehicleIdByIp.end ())
          {
            ++g_mecForwardDrops;
            continue;
          }

        ++g_mecUplinkPackets;
        g_mecUplinkBytes += packet->GetSize ();
        Simulator::Schedule (m_processingDelay,
                             &MecV2n2vForwarder::ForwardPacket,
                             this,
                             packet->Copy (),
                             senderIt->second);
      }
  }

  void ForwardPacket (Ptr<Packet> packet, std::string senderVehicleId)
  {
    if (m_socket == nullptr || m_sumoClient == nullptr ||
        g_vehicleRuntime.find (senderVehicleId) == g_vehicleRuntime.end ())
      {
        ++g_mecForwardDrops;
        return;
      }

    const libsumo::TraCIPosition senderPos =
        m_sumoClient->TraCIAPI::vehicle.getPosition (senderVehicleId);
    uint32_t targets = 0;
    for (const auto& entry : g_mecIpByVehicleId)
      {
        const std::string& receiverVehicleId = entry.first;
        if (receiverVehicleId == senderVehicleId ||
            g_vehicleRuntime.find (receiverVehicleId) == g_vehicleRuntime.end ())
          {
            continue;
          }

        const libsumo::TraCIPosition receiverPos =
            m_sumoClient->TraCIAPI::vehicle.getPosition (receiverVehicleId);
        if (DistanceMeters2d (senderPos, receiverPos) > m_forwardRangeMeters)
          {
            continue;
          }

        const int sent = m_socket->SendTo (packet->Copy (),
                                           0,
                                           InetSocketAddress (entry.second, m_vehiclePort));
        if (sent >= 0)
          {
            ++targets;
            ++g_mecForwardedPackets;
            g_mecForwardedBytes += packet->GetSize ();
          }
        else
          {
            ++g_mecForwardDrops;
          }
      }

    if (targets == 0)
      {
        ++g_mecForwardNoReceiver;
      }
  }

  Ptr<Socket> m_socket;
  Ptr<TraciClient> m_sumoClient;
  uint16_t m_listenPort = 49000;
  uint16_t m_vehiclePort = 49001;
  double m_forwardRangeMeters = 150.0;
  Time m_processingDelay = MilliSeconds (0);
};

NS_OBJECT_ENSURE_REGISTERED (MecV2n2vForwarder);

struct IdealMecRadioResult
{
  bool delivered = true;
  bool firstLoss = false;
  bool recovered = false;
  Time delay = Seconds (0.0);
};

static IdealMecRadioResult
ResolveIdealMecRadioLoss (double firstLossRate)
{
  IdealMecRadioResult result;
  if (g_idealMecRadioRandom == nullptr ||
      g_idealMecRadioRandom->GetValue (0.0, 1.0) >= firstLossRate)
    {
      return result;
    }

  result.firstLoss = true;
  result.delivered = false;
  for (uint32_t attempt = 0; attempt < g_idealMecMaxRetransmissions; ++attempt)
    {
      result.delay += MilliSeconds (g_idealMecRetxDelayMs);
      if (g_idealMecRadioRandom->GetValue (0.0, 1.0) <=
          g_idealMecRetxSuccessProbability)
        {
          result.delivered = true;
          result.recovered = true;
          break;
        }
    }
  return result;
}

static void
DeliverIdealMecCpmBatch (uint64_t senderStationId,
                         std::vector<std::pair<uint64_t, bool>> receivers,
                         Time generatedAt,
                         double latencyMs)
{
  const Time now = Simulator::Now ();
  const double aoiMs =
      std::max (0.0, static_cast<double> ((now - generatedAt).GetMilliSeconds ()));
  for (const auto& receiver : receivers)
    {
      const uint64_t receiverStationId = receiver.first;
      if (g_idealMecDownlinkRandom != nullptr &&
          g_idealMecDownlinkRandom->GetValue (0.0, 1.0) > g_idealMecDownlinkPdr)
        {
          ++g_mecForwardDrops;
          continue;
        }
      const bool aoiAccepted =
          !g_idealMecAoiFilter || aoiMs <= g_idealMecAoiFilterThresholdMs;
      MarkCpmObjectsRecognizedByReceiver (
          receiverStationId,
          senderStationId,
          generatedAt,
          g_latestMecCpmGenerationTimeByReceiver[receiverStationId]);
      std::array<bool, 4> aoiThresholdAccepted = {{false, false, false, false}};
      for (std::size_t i = 0; i < g_idealMecAoiThresholdsMs.size (); ++i)
        {
          aoiThresholdAccepted[i] = aoiMs <= g_idealMecAoiThresholdsMs[i];
          if (aoiThresholdAccepted[i])
            {
              MarkCpmObjectsRecognizedByReceiver (
                  receiverStationId,
                  senderStationId,
                  now,
                  g_latestMecCpmRxByReceiverAoiThreshold[i][receiverStationId]);
            }
        }
      if (aoiAccepted)
        {
          MarkCpmObjectsRecognizedByReceiver (receiverStationId, senderStationId, now);
          MarkCpmObjectsRecognizedByReceiver (
              receiverStationId,
              senderStationId,
              now,
              g_latestActualCpmObjectRxByReceiver[receiverStationId]);
          g_latestMecCpmRxByReceiver[receiverStationId][senderStationId] = now;
          if (receiver.second)
            {
              ++g_idealMecHighSuccesses;
            }
          else
            {
              ++g_idealMecLowSuccesses;
            }
        }
      ++g_idealMecRx;
      g_idealMecLatencySamplesMs.push_back (latencyMs);
      g_idealMecAoiSamplesMs.push_back (aoiMs);
      ++g_idealMecAoiSamples;
      if (receiver.second)
        {
          ++g_idealMecHighAoiSamples;
        }
      else
        {
          ++g_idealMecLowAoiSamples;
        }
      for (std::size_t i = 0; i < g_idealMecAoiThresholdsMs.size (); ++i)
        {
          if (aoiMs <= g_idealMecAoiThresholdsMs[i])
            {
              ++g_idealMecAoiWithinThreshold[i];
              if (receiver.second)
                {
                  ++g_idealMecHighAoiWithinThreshold[i];
                }
              else
                {
                  ++g_idealMecLowAoiWithinThreshold[i];
                }
            }
        }
    }
}

static Time
SampleIdealMecLatency ()
{
  if (g_idealMecDelayModel == "empirical")
    {
      const double u =
          g_idealMecLatencyUniformRv != nullptr
              ? g_idealMecLatencyUniformRv->GetValue (0.0, 1.0)
              : 0.5;
      double latencyMs = g_idealMecLatencyMeanMs;
      if (u < 0.90)
        {
          latencyMs = g_idealMecLatencyMinMs +
                      (u / 0.90) * (g_idealMecLatencyP90Ms - g_idealMecLatencyMinMs);
        }
      else if (u < 0.99)
        {
          latencyMs = g_idealMecLatencyP90Ms +
                      ((u - 0.90) / 0.09) *
                          (g_idealMecLatencyP99Ms - g_idealMecLatencyP90Ms);
        }
      else
        {
          latencyMs = g_idealMecLatencyP99Ms +
                      ((u - 0.99) / 0.01) *
                          (g_idealMecLatencyMaxMs - g_idealMecLatencyP99Ms);
        }
      latencyMs = std::max (g_idealMecLatencyMinMs,
                            std::min (g_idealMecLatencyMaxMs, latencyMs));
      return MilliSeconds (latencyMs);
    }

  double latencyMs = g_idealMecLatencyMeanMs;
  if (g_idealMecLatencyRv != nullptr && g_idealMecLatencyStddevMs > 0.0)
    {
      latencyMs = g_idealMecLatencyRv->GetValue ();
    }
  latencyMs = std::max (g_idealMecLatencyMinMs, std::min (g_idealMecLatencyMaxMs, latencyMs));
  return MilliSeconds (latencyMs);
}

static Time
ReserveIdealMecCapacity (Time arrivalTime, uint32_t bytes, double capacityMbps, Time& availableAt)
{
  if (!g_idealMecCapacityModel || capacityMbps <= 0.0 || bytes == 0)
    {
      return Seconds (0.0);
    }
  const Time start = availableAt > arrivalTime ? availableAt : arrivalTime;
  const double transmissionSeconds =
      (static_cast<double> (bytes) * 8.0) / (capacityMbps * 1.0e6);
  const Time finish = start + Seconds (transmissionSeconds);
  availableAt = finish;
  return finish - arrivalTime;
}

static void
GenerateIdealMecBackgroundLoad ()
{
  if (!g_mecBackgroundLoad)
    {
      return;
    }

  const uint64_t packets =
      g_mecBackgroundPerVehicle
          ? static_cast<uint64_t> (std::max<std::size_t> (g_vehicleRuntime.size (), 0))
          : 1;
  const uint64_t bytes = packets * static_cast<uint64_t> (g_mecBackgroundPacketSizeBytes);
  const Time now = Simulator::Now ();
  if (bytes > 0)
    {
      g_mecBackgroundUplinkBytes += bytes;
      g_mecBackgroundDownlinkBytes += bytes;
      ReserveIdealMecCapacity (now,
                               static_cast<uint32_t> (std::min<uint64_t> (
                                   bytes, std::numeric_limits<uint32_t>::max ())),
                               g_idealMecUplinkCapacityMbps,
                               g_idealMecUplinkAvailableAt);
      ReserveIdealMecCapacity (now,
                               static_cast<uint32_t> (std::min<uint64_t> (
                                   bytes, std::numeric_limits<uint32_t>::max ())),
                               g_idealMecDownlinkCapacityMbps,
                               g_idealMecDownlinkAvailableAt);
    }

  Simulator::Schedule (g_mecBackgroundInterval, &GenerateIdealMecBackgroundLoad);
}

static void
GenerateIdealMecCpm ()
{
  if (g_sumoClient == nullptr)
    {
      Simulator::Schedule (g_idealMecInterval, &GenerateIdealMecCpm);
      return;
    }

  std::vector<IdealMecVehicleSnapshot> vehiclePositions;
  std::vector<std::size_t> activeSenderIndexes;
  vehiclePositions.reserve (g_vehicleRuntime.size ());
  activeSenderIndexes.reserve (g_vehicleRuntime.size ());

  for (const auto& entry : g_vehicleRuntime)
    {
      libsumo::TraCIPosition pos;
      try
        {
          pos = g_sumoClient->TraCIAPI::vehicle.getPosition (entry.first);
        }
      catch (...)
        {
          continue;
        }

      vehiclePositions.push_back ({entry.first, pos, ProjectLoopPositionMeters (pos)});
      if (entry.second.mecTxActive)
        {
          activeSenderIndexes.push_back (vehiclePositions.size () - 1);
        }
    }

  const double loopLengthMeters = std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters, 1.0);
  std::vector<IdealMecLoopIndexEntry> loopIndex;
  loopIndex.reserve (vehiclePositions.size () * (g_trafficFlowIsLoop ? 3 : 1));
  for (std::size_t i = 0; i < vehiclePositions.size (); ++i)
    {
      if (g_trafficFlowIsLoop)
        {
          loopIndex.push_back ({vehiclePositions[i].loopPositionMeters - loopLengthMeters, i});
        }
      loopIndex.push_back ({vehiclePositions[i].loopPositionMeters, i});
      if (g_trafficFlowIsLoop)
        {
          loopIndex.push_back ({vehiclePositions[i].loopPositionMeters + loopLengthMeters, i});
        }
    }
  std::sort (loopIndex.begin (),
             loopIndex.end (),
             [] (const IdealMecLoopIndexEntry& lhs, const IdealMecLoopIndexEntry& rhs) {
               return lhs.loopPositionMeters < rhs.loopPositionMeters;
             });

  for (const std::size_t senderIndex : activeSenderIndexes)
    {
      const auto& senderEntry = vehiclePositions[senderIndex];
      const uint32_t mecPacketSizeBytes =
          std::max (1u,
                    static_cast<uint32_t> (
                        std::lround (AverageLatestPrimaryCpmSizeBytes ())));
      ++g_idealMecTx;
      ++g_mecUplinkPackets;
      g_mecUplinkBytes += mecPacketSizeBytes;
      const Time generatedAt = Simulator::Now ();
      const Time uplinkCapacityDelay =
          ReserveIdealMecCapacity (generatedAt,
                                   mecPacketSizeBytes,
                                   g_idealMecUplinkCapacityMbps,
                                   g_idealMecUplinkAvailableAt);

      uint32_t targets = 0;
      std::vector<std::pair<uint64_t, bool>> receivers;
      const double minLoopPosition = senderEntry.loopPositionMeters - g_mecForwardRangeMeters;
      const double maxLoopPosition = senderEntry.loopPositionMeters + g_mecForwardRangeMeters;
      auto firstCandidate =
          std::lower_bound (loopIndex.begin (),
                            loopIndex.end (),
                            minLoopPosition,
                            [] (const IdealMecLoopIndexEntry& entry, double value) {
                              return entry.loopPositionMeters < value;
                            });
      for (auto candidateIt = firstCandidate;
           candidateIt != loopIndex.end () && candidateIt->loopPositionMeters <= maxLoopPosition;
           ++candidateIt)
        {
          const auto& receiverEntry = vehiclePositions[candidateIt->vehicleIndex];
          if (candidateIt->vehicleIndex == senderIndex)
            {
              continue;
            }

          const double distanceMeters = DistanceMeters2d (senderEntry.position, receiverEntry.position);
          if (distanceMeters > g_mecForwardRangeMeters)
            {
              continue;
            }

          const bool highPriority =
              IsHighPriorityPair (senderEntry.vehicleId,
                                  senderEntry.position,
                                  receiverEntry.vehicleId,
                                  receiverEntry.position,
                                  distanceMeters);
          if (g_mecObjectPolicy == MecObjectPolicy::HighPriorityOnly && !highPriority)
            {
              continue;
            }
          if (g_mecObjectPolicy == MecObjectPolicy::AdaptiveProbability && !highPriority)
            {
              const double lowProbability =
                  ComputeMecAdaptiveLowProbability (senderEntry.vehicleId);
              if (g_hybridRouteRandom == nullptr ||
                  g_hybridRouteRandom->GetValue (0.0, 1.0) > lowProbability)
                {
                  continue;
                }
            }

          if (highPriority)
            {
              ++g_idealMecHighAttempts;
            }
          else
            {
              ++g_idealMecLowAttempts;
            }
          ++targets;
          ++g_mecForwardedPackets;
          g_mecForwardedBytes += mecPacketSizeBytes;
          receivers.emplace_back (VehicleIdToStationId (receiverEntry.vehicleId), highPriority);
        }

      if (targets == 0)
        {
          ++g_mecForwardNoReceiver;
        }
      else
        {
          const IdealMecRadioResult ulRadio =
              ResolveIdealMecRadioLoss (g_idealMecUlFirstLossRate);
          if (ulRadio.firstLoss)
            {
              ++g_idealMecUlFirstLosses;
            }
          if (ulRadio.recovered)
            {
              ++g_idealMecUlRetxRecovered;
            }
          if (!ulRadio.delivered)
            {
              ++g_idealMecUlFinalLosses;
              ++g_mecForwardDrops;
              continue;
            }
          if (ulRadio.delay.GetMilliSeconds () > 0)
            {
              g_idealMecRetxDelaySamplesMs.push_back (ulRadio.delay.GetMilliSeconds ());
            }
          const Time latency = SampleIdealMecLatency ();
          if (!g_idealMecCapacityModel)
            {
              for (const auto& receiver : receivers)
                {
                  const IdealMecRadioResult dlRadio =
                      ResolveIdealMecRadioLoss (g_idealMecDlFirstLossRate);
                  if (dlRadio.firstLoss)
                    {
                      ++g_idealMecDlFirstLosses;
                    }
                  if (dlRadio.recovered)
                    {
                      ++g_idealMecDlRetxRecovered;
                    }
                  if (!dlRadio.delivered)
                    {
                      ++g_idealMecDlFinalLosses;
                      ++g_mecForwardDrops;
                      continue;
                    }
                  if (dlRadio.delay.GetMilliSeconds () > 0)
                    {
                      g_idealMecRetxDelaySamplesMs.push_back (dlRadio.delay.GetMilliSeconds ());
                    }
                  const Time totalLatency = latency + ulRadio.delay + dlRadio.delay;
                  std::vector<std::pair<uint64_t, bool>> oneReceiver;
                  oneReceiver.push_back (receiver);
                  Simulator::Schedule (totalLatency,
                                       &DeliverIdealMecCpmBatch,
                                       VehicleIdToStationId (senderEntry.vehicleId),
                                       std::move (oneReceiver),
                                       generatedAt,
                                       totalLatency.GetMilliSeconds ());
                }
            }
          else
            {
              for (const auto& receiver : receivers)
                {
                  const Time downlinkArrival =
                      generatedAt + latency + uplinkCapacityDelay + ulRadio.delay;
                  const Time downlinkCapacityDelay =
                      ReserveIdealMecCapacity (downlinkArrival,
                                               mecPacketSizeBytes,
                                               g_idealMecDownlinkCapacityMbps,
                                               g_idealMecDownlinkAvailableAt);
                  const IdealMecRadioResult dlRadio =
                      ResolveIdealMecRadioLoss (g_idealMecDlFirstLossRate);
                  if (dlRadio.firstLoss)
                    {
                      ++g_idealMecDlFirstLosses;
                    }
                  if (dlRadio.recovered)
                    {
                      ++g_idealMecDlRetxRecovered;
                    }
                  if (!dlRadio.delivered)
                    {
                      ++g_idealMecDlFinalLosses;
                      ++g_mecForwardDrops;
                      continue;
                    }
                  if (dlRadio.delay.GetMilliSeconds () > 0)
                    {
                      g_idealMecRetxDelaySamplesMs.push_back (dlRadio.delay.GetMilliSeconds ());
                    }
                  const Time totalLatency =
                      latency + uplinkCapacityDelay + downlinkCapacityDelay + ulRadio.delay +
                      dlRadio.delay;
                  const double capacityDelayMs =
                      (uplinkCapacityDelay + downlinkCapacityDelay).GetMilliSeconds ();
                  if (g_idealMecMaxQueueDelayMs > 0.0 &&
                      capacityDelayMs > g_idealMecMaxQueueDelayMs)
                    {
                      ++g_mecForwardDrops;
                      ++g_idealMecCapacityQueueDrops;
                      continue;
                    }
                  g_idealMecCapacityDelaySamplesMs.push_back (capacityDelayMs);
                  std::vector<std::pair<uint64_t, bool>> oneReceiver;
                  oneReceiver.push_back (receiver);
                  Simulator::Schedule (totalLatency,
                                       &DeliverIdealMecCpmBatch,
                                       VehicleIdToStationId (senderEntry.vehicleId),
                                       std::move (oneReceiver),
                                       generatedAt,
                                       totalLatency.GetMilliSeconds ());
                }
            }
        }
    }

  Simulator::Schedule (g_idealMecInterval, &GenerateIdealMecCpm);
}

static double
Percentile (std::vector<double> values, double percentile)
{
  if (values.empty ())
    {
      return 0.0;
    }
  percentile = std::max (0.0, std::min (100.0, percentile));
  std::sort (values.begin (), values.end ());
  const double rank = (percentile / 100.0) * static_cast<double> (values.size () - 1);
  const auto lower = static_cast<std::size_t> (std::floor (rank));
  const auto upper = static_cast<std::size_t> (std::ceil (rank));
  if (lower == upper)
    {
      return values[lower];
    }
  const double weight = rank - static_cast<double> (lower);
  return values[lower] + ((values[upper] - values[lower]) * weight);
}

static double
Mean (const std::vector<double>& values)
{
  if (values.empty ())
    {
      return 0.0;
    }
  return std::accumulate (values.begin (), values.end (), 0.0) /
         static_cast<double> (values.size ());
}

static double
PercentRate (uint64_t numerator, uint64_t denominator)
{
  return denominator > 0 ? 100.0 * static_cast<double> (numerator) /
                               static_cast<double> (denominator)
                         : 0.0;
}

static double
PercentMissRate (uint64_t withinThreshold, uint64_t total)
{
  return total > 0 && total >= withinThreshold ? PercentRate (total - withinThreshold, total) : 0.0;
}

struct SumoScreenshotConfig
{
  std::string dir;
  std::string viewId = "View #0";
  double centerX = 1500.0;
  double centerY = 38.0;
  double spanX = 900.0;
  double spanY = 180.0;
  int width = 1600;
  int height = 900;
  double intervalSeconds = 0.0;
  double stopSeconds = 0.0;
};

static void
CaptureSumoGuiScreenshot (Ptr<TraciClient> sumoClient, SumoScreenshotConfig config)
{
  const double now = Simulator::Now ().GetSeconds ();
  try
    {
      if (config.spanX > 0.0 && config.spanY > 0.0)
        {
          sumoClient->TraCIAPI::gui.setBoundary (config.viewId,
                                                 config.centerX - config.spanX / 2.0,
                                                 config.centerY - config.spanY / 2.0,
                                                 config.centerX + config.spanX / 2.0,
                                                 config.centerY + config.spanY / 2.0);
        }
      std::ostringstream filename;
      filename << config.dir << "/sumo_t" << std::setw (5) << std::setfill ('0')
               << static_cast<int> (std::lround (now * 10.0)) << ".png";
      sumoClient->TraCIAPI::gui.screenshot (config.viewId,
                                            filename.str (),
                                            config.width,
                                            config.height);
      std::cout << "SUMO GUI screenshot: " << filename.str () << std::endl;
    }
  catch (const std::exception& e)
    {
      std::cerr << "SUMO GUI screenshot failed at t=" << now << "s: " << e.what ()
                << std::endl;
    }
  if (config.intervalSeconds > 0.0 && now + config.intervalSeconds <= config.stopSeconds + 1e-9)
    {
      Simulator::Schedule (Seconds (config.intervalSeconds),
                           &CaptureSumoGuiScreenshot,
                           sumoClient,
                           config);
    }
}

static Ptr<BSContainer>
GetRouteContainer (const VehicleRuntime& runtime, ActiveRoute route)
{
  if (route == ActiveRoute::DsrcV2v)
    {
      return runtime.dsrcContainer;
    }
  if (route == ActiveRoute::NrSidelinkV2v)
    {
      return runtime.nrContainer;
    }
  return runtime.mecContainer;
}

static bool
IsClosingHighPriorityObject (uint64_t egoStationId,
                             uint64_t objectStationId,
                             const Vector& egoPosition,
                             const Vector& objectPosition,
                             double distanceMeters,
                             Time now)
{
  if (g_priorityClosingSpeedThresholdMps <= 0.0 || g_priorityTtcThresholdSeconds <= 0.0 ||
      distanceMeters <= 1e-6)
    {
      return false;
    }

  const auto egoPrevIt = g_priorityMotionHistory.find (egoStationId);
  const auto objectPrevIt = g_priorityMotionHistory.find (objectStationId);
  if (egoPrevIt == g_priorityMotionHistory.end () ||
      objectPrevIt == g_priorityMotionHistory.end ())
    {
      return false;
    }

  const double egoDt = (now - egoPrevIt->second.timestamp).GetSeconds ();
  const double objectDt = (now - objectPrevIt->second.timestamp).GetSeconds ();
  if (egoDt <= 0.0 || objectDt <= 0.0)
    {
      return false;
    }

  const Vector egoDelta = egoPosition - egoPrevIt->second.position;
  const Vector objectDelta = objectPosition - objectPrevIt->second.position;
  const Vector egoVelocity (egoDelta.x / egoDt, egoDelta.y / egoDt, egoDelta.z / egoDt);
  const Vector objectVelocity (objectDelta.x / objectDt, objectDelta.y / objectDt, objectDelta.z / objectDt);
  const Vector relativePosition = objectPosition - egoPosition;
  const Vector relativeVelocity = objectVelocity - egoVelocity;
  const double rangeRate =
      ((relativePosition.x * relativeVelocity.x) + (relativePosition.y * relativeVelocity.y) +
       (relativePosition.z * relativeVelocity.z)) /
      distanceMeters;
  const double closingSpeed = -rangeRate;
  if (closingSpeed < g_priorityClosingSpeedThresholdMps)
    {
      return false;
    }

  const double ttcSeconds = distanceMeters / closingSpeed;
  return ttcSeconds <= g_priorityTtcThresholdSeconds;
}

static double
StableEventProbability (uint64_t egoStationId, uint64_t objectStationId)
{
  uint64_t value = egoStationId * 0x9e3779b97f4a7c15ULL;
  value ^= objectStationId + 0xbf58476d1ce4e5b9ULL + (value << 6) + (value >> 2);
  value ^= value >> 30;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27;
  value *= 0x94d049bb133111ebULL;
  value ^= value >> 31;
  return static_cast<double> (value >> 11) / static_cast<double> (1ULL << 53);
}

static void
RestoreSensorExternalEventVehicle (std::string vehicleId)
{
  if (g_sumoClient == nullptr || g_allVehicleNodes.find (vehicleId) == g_allVehicleNodes.end ())
    {
      return;
    }
  try
    {
      g_sumoClient->TraCIAPI::vehicle.setMaxSpeed (vehicleId, 27.78);
      g_sumoClient->TraCIAPI::vehicle.setSpeed (vehicleId, -1.0);
    }
  catch (...)
    {
    }
}

static void
MonitorSensorExternalEvents ()
{
  if (!g_sensorExternalEventsEnabled || g_sumoClient == nullptr)
    {
      return;
    }

  const Time now = Simulator::Now ();
  const bool evaluationActive =
      (!g_holdTrafficUntilEvaluationStart || g_trafficReleased) &&
      now >= g_thesisEvaluationStartTime;
  if (evaluationActive)
    {
      for (const uint64_t objectId : g_nonCavStationIds)
        {
          if (g_sensorExternalEventCandidateVehicleIds.insert (objectId).second)
            {
              ++g_sensorExternalEventCandidateVehicles;
              const double draw = StableEventProbability (0, objectId);
              if (draw <= g_sensorExternalEventProbability)
                {
                  g_sensorExternalEventSelectedVehicleIds.insert (objectId);
                  ++g_sensorExternalEventSelectedVehicles;
                  const double normalized =
                      g_sensorExternalEventProbability > 0.0
                          ? draw / g_sensorExternalEventProbability
                          : 0.0;
                  g_sensorExternalEventActivationTimes[objectId] =
                      g_thesisEvaluationStartTime + Seconds (1.0 + 10.0 * normalized);
                }
            }

          const auto activationIt = g_sensorExternalEventActivationTimes.find (objectId);
          if (activationIt == g_sensorExternalEventActivationTimes.end () ||
              now < activationIt->second ||
              g_sensorExternalEventActivatedVehicleIds.count (objectId) > 0)
            {
              continue;
            }
          const auto vehicleIt = g_vehicleIdByStationId.find (objectId);
          if (vehicleIt == g_vehicleIdByStationId.end ())
            {
              continue;
            }
          const auto objectNodeIt = g_allVehicleNodes.find (vehicleIt->second);
          if (objectNodeIt == g_allVehicleNodes.end () || objectNodeIt->second == nullptr)
            {
              continue;
            }
          Ptr<MobilityModel> objectMobility =
              objectNodeIt->second->GetObject<MobilityModel> ();
          bool cavInActivationRange = false;
          if (objectMobility != nullptr)
            {
              for (const auto& egoEntry : g_vehicleRuntime)
                {
                  const auto egoNodeIt = g_allVehicleNodes.find (egoEntry.first);
                  if (egoNodeIt == g_allVehicleNodes.end () || egoNodeIt->second == nullptr)
                    {
                      continue;
                    }
                  Ptr<MobilityModel> egoMobility =
                      egoNodeIt->second->GetObject<MobilityModel> ();
                  if (egoMobility != nullptr &&
                      egoMobility->GetDistanceFrom (objectMobility) <=
                          g_sensorExternalEventActivationRangeMeters)
                    {
                      cavInActivationRange = true;
                      break;
                    }
                }
            }
          if (!cavInActivationRange)
            {
              continue;
            }
          g_sensorExternalEventActivatedVehicleIds.insert (objectId);
          g_sensorExternalEventEndTimes[objectId] = now + Seconds (5.0);
          try
            {
              g_sumoClient->TraCIAPI::vehicle.setMaxSpeed (vehicleIt->second, 33.33);
              g_sumoClient->TraCIAPI::vehicle.setSpeed (vehicleIt->second, 33.33);
              Simulator::Schedule (Seconds (5.0),
                                   &RestoreSensorExternalEventVehicle,
                                   vehicleIt->second);
            }
          catch (...)
            {
            }
        }

      for (const auto& egoEntry : g_vehicleRuntime)
        {
          const uint64_t egoId = VehicleIdToStationId (egoEntry.first);
          const auto egoNodeIt = g_allVehicleNodes.find (egoEntry.first);
          if (egoNodeIt == g_allVehicleNodes.end () || egoNodeIt->second == nullptr)
            {
              continue;
            }
          Ptr<MobilityModel> egoMobility = egoNodeIt->second->GetObject<MobilityModel> ();
          if (egoMobility == nullptr)
            {
              continue;
            }

          for (const uint64_t objectId : g_nonCavStationIds)
            {
              const auto objectVehicleIt = g_vehicleIdByStationId.find (objectId);
              if (objectVehicleIt == g_vehicleIdByStationId.end ())
                {
                  continue;
                }
              const auto objectNodeIt = g_allVehicleNodes.find (objectVehicleIt->second);
              if (objectNodeIt == g_allVehicleNodes.end () || objectNodeIt->second == nullptr)
                {
                  continue;
                }
              Ptr<MobilityModel> objectMobility =
                  objectNodeIt->second->GetObject<MobilityModel> ();
              if (objectMobility == nullptr)
                {
                  continue;
                }

              const double distance = egoMobility->GetDistanceFrom (objectMobility);
              const uint64_t key = (egoId << 32) ^ objectId;
              const auto eventEndIt = g_sensorExternalEventEndTimes.find (objectId);
              if (eventEndIt == g_sensorExternalEventEndTimes.end () ||
                  now > eventEndIt->second)
                {
                  continue;
                }
              auto stateIt = g_sensorExternalEvents.find (key);
              if (stateIt == g_sensorExternalEvents.end ())
                {
                  stateIt = g_sensorExternalEvents.emplace (key, SensorExternalEventState {}).first;
                }

              SensorExternalEventState& state = stateIt->second;
              if (state.completed)
                {
                  continue;
                }

              double closingSpeed = 0.0;
              if (state.hasPreviousSample)
                {
                  const double dt = (now - state.previousSampleTime).GetSeconds ();
                  if (dt > 0.0)
                    {
                      closingSpeed = (state.previousDistanceMeters - distance) / dt;
                    }
                }
              state.hasPreviousSample = true;
              state.previousDistanceMeters = distance;
              state.previousSampleTime = now;
              const double ttc =
                  closingSpeed > 0.0 ? distance / closingSpeed
                                     : std::numeric_limits<double>::infinity ();

              if (!state.triggered && distance > g_sensorRangeMeters &&
                  distance <= g_orrRangeMeters &&
                  closingSpeed >= g_priorityClosingSpeedThresholdMps &&
                  ttc <= g_priorityTtcThresholdSeconds)
                {
                  state.triggered = true;
                  state.triggerTime = now;
                  ++g_sensorExternalEventCount;
                }
              if (!state.triggered)
                {
                  continue;
                }

              const auto receiverIt = g_latestActualCpmObjectRxByReceiver.find (egoId);
              const auto updateIt =
                  receiverIt != g_latestActualCpmObjectRxByReceiver.end ()
                      ? receiverIt->second.find (objectId)
                      : std::unordered_map<uint64_t, Time>::const_iterator ();
              if (receiverIt != g_latestActualCpmObjectRxByReceiver.end () &&
                  updateIt != receiverIt->second.end () &&
                  updateIt->second >= state.triggerTime &&
                  (now - updateIt->second).GetSeconds () <=
                      g_highPriorityCpmRecognitionTtlSeconds)
                {
                  state.recognized = true;
                  state.completed = true;
                  ++g_sensorExternalEventRecognized;
                  g_sensorExternalEventDelaysMs.push_back (
                      std::max (
                          0.0,
                          static_cast<double> (
                              (now - state.triggerTime).GetMilliSeconds ())));
                }
              else if ((now - state.triggerTime).GetSeconds () >
                       g_highPriorityCpmRecognitionTtlSeconds)
                {
                  state.completed = true;
                  ++g_sensorExternalEventMissed;
                }
            }
        }
    }

  Simulator::Schedule (MilliSeconds (100), &MonitorSensorExternalEvents);
}

static bool
ShouldAccumulateThesisMetrics ()
{
  if (g_holdTrafficUntilEvaluationStart)
    {
      if (!g_trafficReleased || Simulator::Now () < g_thesisEvaluationStartTime)
        {
          return false;
        }
      g_thesisEvaluationStarted = true;
      return true;
    }
  if (g_thesisEvaluationStarted)
    {
      return true;
    }
  if (g_thesisEvalStartMinVehicles == 0)
    {
      g_thesisEvaluationStarted = true;
      return true;
    }

  const uint32_t vehicleCount =
      g_thesisEvalStartUseAllVehicles ? static_cast<uint32_t> (g_allVehicleNodes.size ())
                                      : static_cast<uint32_t> (g_vehicleRuntime.size ());
  if (vehicleCount >= g_thesisEvalStartMinVehicles)
    {
      g_thesisEvaluationStarted = true;
      return true;
    }
  return false;
}

static void
HoldTrafficUntilEvaluationStart ()
{
  if (!g_holdTrafficUntilEvaluationStart || g_trafficReleased || g_sumoClient == nullptr)
    {
      return;
    }

  const uint32_t vehicleCount =
      g_thesisEvalStartUseAllVehicles ? static_cast<uint32_t> (g_allVehicleNodes.size ())
                                      : static_cast<uint32_t> (g_vehicleRuntime.size ());
  if (vehicleCount >= g_thesisEvalStartMinVehicles)
    {
      for (const auto& vehicle : g_allVehicleNodes)
        {
          try
            {
              g_sumoClient->TraCIAPI::vehicle.setSpeed (vehicle.first, -1.0);
            }
          catch (...)
            {
            }
        }
      g_trafficReleased = true;
      g_thesisEvaluationStartTime =
          Simulator::Now () + Seconds (g_thesisEvalWarmupSeconds);
      std::cout << "Traffic released at " << Simulator::Now ().GetSeconds ()
                << " s with " << g_allVehicleNodes.size ()
                << " active vehicles; thesis evaluation starts at "
                << g_thesisEvaluationStartTime.GetSeconds () << " s" << std::endl;
      return;
    }

  for (const auto& vehicle : g_allVehicleNodes)
    {
      try
        {
          g_sumoClient->TraCIAPI::vehicle.setSpeed (vehicle.first, 0.0);
        }
      catch (...)
        {
        }
    }
  Simulator::Schedule (MilliSeconds (100), &HoldTrafficUntilEvaluationStart);
}

static void
AccumulateThesisRecognitionSample ()
{
  const uint32_t activeVehicles = static_cast<uint32_t> (g_vehicleRuntime.size ());
  if (activeVehicles == 0 || g_allVehicleNodes.size () <= 1)
    {
      return;
    }

  std::unordered_map<uint64_t, Ptr<MobilityModel>> activeStationMobility;
  std::unordered_map<uint64_t, Vector> activeStationPositions;
  std::unordered_map<uint64_t, Vector> activeStationVelocities;
  for (const auto& entry : g_allVehicleNodes)
    {
      Ptr<MobilityModel> mobility =
          entry.second != nullptr ? entry.second->GetObject<MobilityModel> () : nullptr;
      if (mobility != nullptr)
        {
          const uint64_t stationId = VehicleIdToStationId (entry.first);
          const Vector position = mobility->GetPosition ();
          activeStationMobility[stationId] = mobility;
          activeStationPositions[stationId] = position;
          const auto previousIt = g_priorityMotionHistory.find (stationId);
          if (previousIt != g_priorityMotionHistory.end ())
            {
              const double dt = (Simulator::Now () - previousIt->second.timestamp).GetSeconds ();
              if (dt > 0.0)
                {
                  const Vector delta = position - previousIt->second.position;
                  activeStationVelocities[stationId] =
                      Vector (delta.x / dt, delta.y / dt, delta.z / dt);
                }
            }
        }
    }
  const Time now = Simulator::Now ();

  for (const auto& entry : g_vehicleRuntime)
    {
      const uint64_t selfStationId = VehicleIdToStationId (entry.first);
      auto selfMobilityIt = activeStationMobility.find (selfStationId);
      if (selfMobilityIt == activeStationMobility.end ())
        {
          continue;
        }

      std::set<uint64_t> expectedStationIds;
      std::set<uint64_t> highPriorityExpectedStationIds;
      std::set<uint64_t> sensorRecognizedStationIds;
      std::set<uint64_t> cooperativeExpectedStationIds;
      for (const auto& candidate : activeStationMobility)
        {
          if (candidate.first == selfStationId)
            {
              continue;
            }

          const Vector selfPosition = activeStationPositions[selfStationId];
          const Vector candidatePosition = activeStationPositions[candidate.first];
          const double distanceMeters = selfMobilityIt->second->GetDistanceFrom (candidate.second);
          if (distanceMeters <= g_orrRangeMeters)
            {
              expectedStationIds.insert (candidate.first);
              if (distanceMeters <= g_sensorRangeMeters)
                {
                  sensorRecognizedStationIds.insert (candidate.first);
                }
              else
                {
                  cooperativeExpectedStationIds.insert (candidate.first);
                }
              if (distanceMeters <= g_priorityDistanceThresholdMeters ||
                  IsClosingHighPriorityObject (selfStationId,
                                               candidate.first,
                                               selfPosition,
                                               candidatePosition,
                                               distanceMeters,
                                               now))
                {
                  highPriorityExpectedStationIds.insert (candidate.first);
                }
            }
        }
      if (expectedStationIds.empty ())
        {
          continue;
        }

      for (const uint64_t senderStationId : expectedStationIds)
        {
          const auto senderVehicleIt = g_vehicleIdByStationId.find (senderStationId);
          if (senderVehicleIt == g_vehicleIdByStationId.end ())
            {
              continue;
            }
          const auto senderRuntimeIt = g_vehicleRuntime.find (senderVehicleIt->second);
          if (senderRuntimeIt == g_vehicleRuntime.end ())
            {
              continue;
            }
          const bool highPriority =
              highPriorityExpectedStationIds.count (senderStationId) > 0;
          const double updateTtlSeconds =
              highPriority ? g_highPriorityCpmRecognitionTtlSeconds
                           : g_lowPriorityCpmRecognitionTtlSeconds;
          auto hasFreshRouteUpdate =
              [selfStationId, senderStationId, now, updateTtlSeconds] (
                  const std::unordered_map<uint64_t,
                                           std::unordered_map<uint64_t, Time>>& updates) {
                const auto receiverIt = updates.find (selfStationId);
                if (receiverIt == updates.end ())
                  {
                    return false;
                  }
                const auto senderIt = receiverIt->second.find (senderStationId);
                return senderIt != receiverIt->second.end () &&
                       (now - senderIt->second).GetSeconds () <= updateTtlSeconds;
              };
          if (senderRuntimeIt->second.nrTxActive)
            {
              ++g_thesisStats.nrUpdateExpected;
              if (hasFreshRouteUpdate (g_latestNrCpmRxByReceiver))
                {
                  ++g_thesisStats.nrUpdateSuccess;
                }
            }
          if (senderRuntimeIt->second.mecTxActive)
            {
              ++g_thesisStats.mecUpdateExpected;
              if (hasFreshRouteUpdate (g_latestMecCpmRxByReceiver))
                {
                  ++g_thesisStats.mecUpdateSuccess;
                }
            }
        }

      auto hasFreshGenerationUpdate =
          [selfStationId, now] (
              const std::unordered_map<uint64_t,
                                       std::unordered_map<uint64_t, Time>>& updates,
              uint64_t stationId,
              double thresholdMs) {
            const auto receiverIt = updates.find (selfStationId);
            if (receiverIt == updates.end ())
              {
                return false;
              }
            const auto stationIt = receiverIt->second.find (stationId);
            return stationIt != receiverIt->second.end () &&
                   (now - stationIt->second).GetMilliSeconds () <= thresholdMs;
          };
      auto freshRedundancyCount = [&] (uint64_t stationId, double thresholdMs) {
        uint32_t count = sensorRecognizedStationIds.count (stationId) > 0 ? 1 : 0;
        if (hasFreshGenerationUpdate (g_latestNrCpmGenerationTimeByReceiver,
                                      stationId,
                                      thresholdMs))
          {
            ++count;
          }
        if (hasFreshGenerationUpdate (g_latestMecCpmGenerationTimeByReceiver,
                                      stationId,
                                      thresholdMs))
          {
            ++count;
          }
        return count;
      };
      std::unordered_map<uint64_t, uint32_t> freshRedundancy200ByStation;
      std::unordered_map<uint64_t, uint32_t> freshRedundancy500ByStation;
      std::vector<double> receiverRlValues;
      std::vector<double> receiverRvValues;
      for (const uint64_t stationId : expectedStationIds)
        {
          const uint32_t fresh200 =
              freshRedundancyCount (stationId, g_idealMecAoiThresholdsMs[0]);
          const uint32_t fresh500 =
              freshRedundancyCount (stationId, g_idealMecAoiThresholdsMs[3]);
          freshRedundancy200ByStation[stationId] = fresh200;
          freshRedundancy500ByStation[stationId] = fresh500;

          g_thesisStats.receiverFreshRedundancy200Sum += fresh200;
          g_thesisStats.receiverFreshRedundancy500Sum += fresh500;
          g_thesisStats.receiverRv200Sum += std::min (1.0, static_cast<double> (fresh200) / 2.0);
          ++g_thesisStats.receiverFreshRedundancySamples;

          uint32_t receivedUpdates =
              CountRecentCpmObjectUpdates (selfStationId, stationId, now);
          if (sensorRecognizedStationIds.count (stationId) > 0)
            {
              ++receivedUpdates;
            }
          double distanceChangeMeters = 0.0;
          double speedChangeMps = 0.0;
          const auto currentPositionIt = activeStationPositions.find (stationId);
          const auto previousStateIt = g_priorityMotionHistory.find (stationId);
          if (currentPositionIt != activeStationPositions.end () &&
              previousStateIt != g_priorityMotionHistory.end ())
            {
              const Vector delta = currentPositionIt->second - previousStateIt->second.position;
              distanceChangeMeters =
                  std::sqrt ((delta.x * delta.x) + (delta.y * delta.y) + (delta.z * delta.z));
              const auto currentVelocityIt = activeStationVelocities.find (stationId);
              if (currentVelocityIt != activeStationVelocities.end () &&
                  previousStateIt->second.hasVelocity)
                {
                  const double currentSpeed =
                      std::sqrt ((currentVelocityIt->second.x * currentVelocityIt->second.x) +
                                 (currentVelocityIt->second.y * currentVelocityIt->second.y) +
                                 (currentVelocityIt->second.z * currentVelocityIt->second.z));
                  const double previousSpeed =
                      std::sqrt ((previousStateIt->second.velocity.x *
                                  previousStateIt->second.velocity.x) +
                                 (previousStateIt->second.velocity.y *
                                  previousStateIt->second.velocity.y) +
                                 (previousStateIt->second.velocity.z *
                                  previousStateIt->second.velocity.z));
                  speedChangeMps = std::abs (currentSpeed - previousSpeed);
                }
            }
          const uint32_t requiredUpdates = static_cast<uint32_t> (
              std::ceil (std::max ({distanceChangeMeters / 4.0, speedChangeMps / 0.5, 1.0})));
          const double rl =
              static_cast<double> (receivedUpdates) / static_cast<double> (requiredUpdates);
          const double rv = ComputeDeloozRv (rl);
          receiverRlValues.push_back (rl);
          receiverRvValues.push_back (rv);
          g_thesisStats.receiverRlSum += rl;
          g_thesisStats.receiverRvDeloozSum += rv;
          ++g_thesisStats.receiverRlSamples;
          if (rl < 1.0)
            {
              ++g_thesisStats.receiverRlLt1;
            }
          if (rl >= 1.0)
            {
              ++g_thesisStats.receiverRlGe1;
            }
          if (rl >= 2.0)
            {
              ++g_thesisStats.receiverRlGe2;
            }
          if (fresh200 >= 2)
            {
              ++g_thesisStats.receiverFreshRedundancyGe2_200;
            }
          if (fresh500 >= 2)
            {
              ++g_thesisStats.receiverFreshRedundancyGe2_500;
            }

          if (highPriorityExpectedStationIds.count (stationId) > 0)
            {
              g_thesisStats.receiverFreshRedundancyHigh200Sum += fresh200;
              ++g_thesisStats.receiverFreshRedundancyHighSamples;
              g_thesisStats.receiverHighRlSum += rl;
              ++g_thesisStats.receiverHighRlSamples;
            }
          else
            {
              g_thesisStats.receiverFreshRedundancyLow200Sum += fresh200;
              ++g_thesisStats.receiverFreshRedundancyLowSamples;
              g_thesisStats.receiverLowRlSum += rl;
              ++g_thesisStats.receiverLowRlSamples;
            }
        }
      if (!receiverRlValues.empty ())
        {
          g_thesisStats.receiverRlMedianSum += Percentile (receiverRlValues, 50.0);
          g_thesisStats.receiverRvDeloozMedianSum += Percentile (receiverRvValues, 50.0);
          ++g_thesisStats.receiverRlVehicleSamples;
        }

      std::set<uint64_t> cpmRecognizedStationIds;
      uint32_t ttlViolationObjects = 0;
      uint32_t neverReceivedObjects = 0;
      uint32_t highPriorityTtlViolationObjects = 0;
      uint32_t highPriorityNeverReceivedObjects = 0;
      uint32_t lowPriorityTtlViolationObjects = 0;
      uint32_t lowPriorityNeverReceivedObjects = 0;
      const auto cpmRxIt = g_latestCpmRxByReceiver.find (selfStationId);
      if (cpmRxIt != g_latestCpmRxByReceiver.end ())
        {
          for (const auto& rxEntry : cpmRxIt->second)
            {
              if (expectedStationIds.count (rxEntry.first) > 0)
                {
                  const double ttlSeconds =
                      highPriorityExpectedStationIds.count (rxEntry.first) > 0
                          ? g_highPriorityCpmRecognitionTtlSeconds
                          : g_lowPriorityCpmRecognitionTtlSeconds;
                  if ((now - rxEntry.second).GetSeconds () <= ttlSeconds)
                    {
                      cpmRecognizedStationIds.insert (rxEntry.first);
                    }
                }
            }
        }
      for (const uint64_t stationId : expectedStationIds)
        {
          if (sensorRecognizedStationIds.count (stationId) > 0)
            {
              continue;
            }

          const bool highPriority = highPriorityExpectedStationIds.count (stationId) > 0;
          auto receivedIt = cpmRxIt != g_latestCpmRxByReceiver.end ()
                                ? cpmRxIt->second.find (stationId)
                                : std::unordered_map<uint64_t, Time>::const_iterator ();
          if (cpmRxIt == g_latestCpmRxByReceiver.end () || receivedIt == cpmRxIt->second.end ())
            {
              ++neverReceivedObjects;
              if (highPriority)
                {
                  ++highPriorityNeverReceivedObjects;
                }
              else
                {
                  ++lowPriorityNeverReceivedObjects;
                }
              continue;
            }

          const double ttlSeconds =
              highPriority ? g_highPriorityCpmRecognitionTtlSeconds
                           : g_lowPriorityCpmRecognitionTtlSeconds;
          if ((now - receivedIt->second).GetSeconds () > ttlSeconds)
            {
              ++ttlViolationObjects;
              if (highPriority)
                {
                  ++highPriorityTtlViolationObjects;
                }
              else
                {
                  ++lowPriorityTtlViolationObjects;
                }
            }
        }

      std::set<uint64_t> recognizedStationIds = sensorRecognizedStationIds;
      recognizedStationIds.insert (cpmRecognizedStationIds.begin (), cpmRecognizedStationIds.end ());
      std::array<std::set<uint64_t>, 4> v2vAoiRecognizedStationIds;
      for (std::size_t i = 0; i < g_idealMecAoiThresholdsMs.size (); ++i)
        {
          v2vAoiRecognizedStationIds[i] = sensorRecognizedStationIds;
          std::set<uint64_t> v2vOnlyAoiRecognizedStationIds;
          const auto nrGenerationIt =
              g_latestNrCpmGenerationTimeByReceiver.find (selfStationId);
          if (nrGenerationIt != g_latestNrCpmGenerationTimeByReceiver.end ())
            {
              for (const auto& rxEntry : nrGenerationIt->second)
                {
                  if (expectedStationIds.count (rxEntry.first) > 0 &&
                      (now - rxEntry.second).GetMilliSeconds () <=
                          g_idealMecAoiThresholdsMs[i])
                    {
                      v2vAoiRecognizedStationIds[i].insert (rxEntry.first);
                      v2vOnlyAoiRecognizedStationIds.insert (rxEntry.first);
                    }
                }
            }
          uint32_t v2vAoiRecognizedObjects = 0;
          for (const uint64_t stationId : v2vAoiRecognizedStationIds[i])
            {
              if (expectedStationIds.count (stationId) > 0)
                {
                  ++v2vAoiRecognizedObjects;
                }
            }
          g_thesisStats.v2vAoiRecognitionRatioSum[i] +=
              Clamp01 (static_cast<double> (v2vAoiRecognizedObjects) /
                       static_cast<double> (expectedStationIds.size ()));
          ++g_thesisStats.v2vAoiRecognitionVehicleSamples[i];
          g_thesisStats.v2vOnlyAoiRecognitionRatioSum[i] +=
              Clamp01 (static_cast<double> (v2vOnlyAoiRecognizedStationIds.size ()) /
                       static_cast<double> (expectedStationIds.size ()));
          ++g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[i];
        }
      std::array<std::set<uint64_t>, 4> mecAoiRecognizedStationIds;
      for (std::size_t i = 0; i < g_idealMecAoiThresholdsMs.size (); ++i)
        {
          mecAoiRecognizedStationIds[i] = recognizedStationIds;
          std::set<uint64_t> mecOnlyAoiRecognizedStationIds;
          const auto mecGenerationIt =
              g_latestMecCpmGenerationTimeByReceiver.find (selfStationId);
          if (mecGenerationIt != g_latestMecCpmGenerationTimeByReceiver.end ())
            {
              for (const auto& rxEntry : mecGenerationIt->second)
                {
                  if (expectedStationIds.count (rxEntry.first) == 0)
                    {
                      continue;
                    }
                  if ((now - rxEntry.second).GetMilliSeconds () <=
                      g_idealMecAoiThresholdsMs[i])
                    {
                      mecAoiRecognizedStationIds[i].insert (rxEntry.first);
                      mecOnlyAoiRecognizedStationIds.insert (rxEntry.first);
                    }
                }
            }
          uint32_t mecAoiRecognizedObjects = 0;
          for (const uint64_t stationId : mecAoiRecognizedStationIds[i])
            {
              if (expectedStationIds.count (stationId) > 0)
                {
                  ++mecAoiRecognizedObjects;
                }
            }
          g_thesisStats.mecAoiRecognitionRatioSum[i] +=
              Clamp01 (static_cast<double> (mecAoiRecognizedObjects) /
                       static_cast<double> (expectedStationIds.size ()));
          ++g_thesisStats.mecAoiRecognitionVehicleSamples[i];
          g_thesisStats.mecOnlyAoiRecognitionRatioSum[i] +=
              Clamp01 (static_cast<double> (mecOnlyAoiRecognizedStationIds.size ()) /
                       static_cast<double> (expectedStationIds.size ()));
          ++g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[i];
        }
      uint32_t recognizedObjects = 0;
      uint32_t highPriorityRecognizedObjects = 0;
      uint32_t lowPriorityRecognizedObjects = 0;
      for (uint64_t stationId : recognizedStationIds)
        {
          if (expectedStationIds.count (stationId) > 0)
            {
              ++recognizedObjects;
            }
          if (highPriorityExpectedStationIds.count (stationId) > 0)
            {
              ++highPriorityRecognizedObjects;
            }
          else if (expectedStationIds.count (stationId) > 0)
            {
              ++lowPriorityRecognizedObjects;
            }
        }

      for (const auto& senderEntry : g_vehicleRuntime)
        {
          if (senderEntry.second.nrContainer == nullptr)
            {
              continue;
            }
          const auto& deletedIds =
              senderEntry.second.nrContainer->getCPBasicService ()->getLastRmrDeletedIds ();
          for (const uint64_t deletedId : deletedIds)
            {
              uint64_t resolvedDeletedId = deletedId;
              if (expectedStationIds.count (resolvedDeletedId) == 0 &&
                  expectedStationIds.count (deletedId + 1) > 0)
                {
                  resolvedDeletedId = deletedId + 1;
                }
              else if (deletedId > 0 && expectedStationIds.count (resolvedDeletedId) == 0 &&
                       expectedStationIds.count (deletedId - 1) > 0)
                {
                  resolvedDeletedId = deletedId - 1;
                }
              if (expectedStationIds.count (resolvedDeletedId) == 0 ||
                  sensorRecognizedStationIds.count (resolvedDeletedId) > 0)
                {
                  continue;
                }
              const bool highPriority =
                  highPriorityExpectedStationIds.count (resolvedDeletedId) > 0;
              ++g_thesisStats.rmrDeletedEvalExpectedTotal;
              if (highPriority)
                {
                  ++g_thesisStats.rmrDeletedEvalExpectedHigh;
                }
              else
                {
                  ++g_thesisStats.rmrDeletedEvalExpectedLow;
                }
              const auto deletedPositionIt = activeStationPositions.find (resolvedDeletedId);
              if (deletedPositionIt != activeStationPositions.end ())
                {
                  const Vector selfPosition = activeStationPositions[selfStationId];
                  const Vector deletedPosition = deletedPositionIt->second;
                  const double dx = deletedPosition.x - selfPosition.x;
                  const double dy = deletedPosition.y - selfPosition.y;
                  const double dz = deletedPosition.z - selfPosition.z;
                  const double distanceMeters = std::sqrt ((dx * dx) + (dy * dy) + (dz * dz));
                  g_thesisStats.rmrDeletedEvalDistanceSum += distanceMeters;
                  ++g_thesisStats.rmrDeletedEvalDistanceSamples;

                  const auto selfPrevIt = g_priorityMotionHistory.find (selfStationId);
                  const auto deletedPrevIt = g_priorityMotionHistory.find (resolvedDeletedId);
                  if (selfPrevIt != g_priorityMotionHistory.end () &&
                      deletedPrevIt != g_priorityMotionHistory.end () &&
                      distanceMeters > 1e-6)
                    {
                      const double selfDt = (now - selfPrevIt->second.timestamp).GetSeconds ();
                      const double deletedDt =
                          (now - deletedPrevIt->second.timestamp).GetSeconds ();
                      if (selfDt > 0.0 && deletedDt > 0.0)
                        {
                          const Vector selfDelta = selfPosition - selfPrevIt->second.position;
                          const Vector deletedDelta =
                              deletedPosition - deletedPrevIt->second.position;
                          const Vector selfVelocity (selfDelta.x / selfDt,
                                                     selfDelta.y / selfDt,
                                                     selfDelta.z / selfDt);
                          const Vector deletedVelocity (deletedDelta.x / deletedDt,
                                                        deletedDelta.y / deletedDt,
                                                        deletedDelta.z / deletedDt);
                          const Vector relativePosition = deletedPosition - selfPosition;
                          const Vector relativeVelocity = deletedVelocity - selfVelocity;
                          const double rangeRate =
                              ((relativePosition.x * relativeVelocity.x) +
                               (relativePosition.y * relativeVelocity.y) +
                               (relativePosition.z * relativeVelocity.z)) /
                              distanceMeters;
                          const double closingSpeed = -rangeRate;
                          if (closingSpeed >= g_priorityClosingSpeedThresholdMps)
                            {
                              ++g_thesisStats.rmrDeletedEvalClosingSpeedGeThreshold;
                              if (distanceMeters / closingSpeed <= g_priorityTtcThresholdSeconds)
                                {
                                  ++g_thesisStats.rmrDeletedEvalTtcLeThreshold;
                                }
                            }
                        }
                    }
                }
              if (freshRedundancy200ByStation[resolvedDeletedId] >= 2)
                {
                  ++g_thesisStats.rmrDeletedEvalFreshRedundancyGe2_200;
                }
              if (freshRedundancy500ByStation[resolvedDeletedId] >= 2)
                {
                  ++g_thesisStats.rmrDeletedEvalFreshRedundancyGe2_500;
                }
              if (recognizedStationIds.count (resolvedDeletedId) == 0)
                {
                  ++g_thesisStats.rmrDeletedEvalUnrecognizedTotal;
                  if (highPriority)
                    {
                      ++g_thesisStats.rmrDeletedEvalUnrecognizedHigh;
                    }
                  else
                    {
                      ++g_thesisStats.rmrDeletedEvalUnrecognizedLow;
                    }
                }
            }
        }

      g_thesisStats.recognitionRatioSum +=
          Clamp01 (static_cast<double> (recognizedObjects) /
                   static_cast<double> (expectedStationIds.size ()));
      ++g_thesisStats.recognitionVehicleSamples;
      g_thesisStats.ttlViolationRatioSum +=
          Clamp01 (static_cast<double> (ttlViolationObjects) /
                   static_cast<double> (expectedStationIds.size ()));
      ++g_thesisStats.ttlViolationVehicleSamples;
      g_thesisStats.neverReceivedRatioSum +=
          Clamp01 (static_cast<double> (neverReceivedObjects) /
                   static_cast<double> (expectedStationIds.size ()));
      ++g_thesisStats.neverReceivedVehicleSamples;
      if (!highPriorityExpectedStationIds.empty ())
        {
          g_thesisStats.highPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (highPriorityRecognizedObjects) /
                       static_cast<double> (highPriorityExpectedStationIds.size ()));
          ++g_thesisStats.highPriorityRecognitionVehicleSamples;
          g_thesisStats.highPriorityTtlViolationRatioSum +=
              Clamp01 (static_cast<double> (highPriorityTtlViolationObjects) /
                       static_cast<double> (highPriorityExpectedStationIds.size ()));
          ++g_thesisStats.highPriorityTtlViolationVehicleSamples;
          g_thesisStats.highPriorityNeverReceivedRatioSum +=
              Clamp01 (static_cast<double> (highPriorityNeverReceivedObjects) /
                       static_cast<double> (highPriorityExpectedStationIds.size ()));
          ++g_thesisStats.highPriorityNeverReceivedVehicleSamples;
        }
      const uint32_t lowPriorityExpectedObjects =
          static_cast<uint32_t> (expectedStationIds.size () -
                                 highPriorityExpectedStationIds.size ());
      if (lowPriorityExpectedObjects > 0)
        {
          g_thesisStats.lowPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (lowPriorityRecognizedObjects) /
                       static_cast<double> (lowPriorityExpectedObjects));
          ++g_thesisStats.lowPriorityRecognitionVehicleSamples;
          g_thesisStats.lowPriorityTtlViolationRatioSum +=
              Clamp01 (static_cast<double> (lowPriorityTtlViolationObjects) /
                       static_cast<double> (lowPriorityExpectedObjects));
          ++g_thesisStats.lowPriorityTtlViolationVehicleSamples;
          g_thesisStats.lowPriorityNeverReceivedRatioSum +=
              Clamp01 (static_cast<double> (lowPriorityNeverReceivedObjects) /
                       static_cast<double> (lowPriorityExpectedObjects));
          ++g_thesisStats.lowPriorityNeverReceivedVehicleSamples;
        }

      uint32_t sensorHighPriorityObjects = 0;
      uint32_t sensorLowPriorityObjects = 0;
      for (const uint64_t stationId : sensorRecognizedStationIds)
        {
          if (highPriorityExpectedStationIds.count (stationId) > 0)
            {
              ++sensorHighPriorityObjects;
            }
          else if (expectedStationIds.count (stationId) > 0)
            {
              ++sensorLowPriorityObjects;
            }
        }
      g_thesisStats.sensorRecognitionRatioSum +=
          Clamp01 (static_cast<double> (sensorRecognizedStationIds.size ()) /
                   static_cast<double> (expectedStationIds.size ()));
      ++g_thesisStats.sensorRecognitionVehicleSamples;
      if (!highPriorityExpectedStationIds.empty ())
        {
          g_thesisStats.sensorHighPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (sensorHighPriorityObjects) /
                       static_cast<double> (highPriorityExpectedStationIds.size ()));
          ++g_thesisStats.sensorHighPriorityRecognitionVehicleSamples;
        }
      if (lowPriorityExpectedObjects > 0)
        {
          g_thesisStats.sensorLowPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (sensorLowPriorityObjects) /
                       static_cast<double> (lowPriorityExpectedObjects));
          ++g_thesisStats.sensorLowPriorityRecognitionVehicleSamples;
        }

      if (!cooperativeExpectedStationIds.empty ())
        {
          uint32_t cooperativeRecognizedObjects = 0;
          for (uint64_t stationId : cpmRecognizedStationIds)
            {
              if (cooperativeExpectedStationIds.count (stationId) > 0)
                {
                  ++cooperativeRecognizedObjects;
                }
            }
          g_thesisStats.cooperativeRecognitionRatioSum +=
              Clamp01 (static_cast<double> (cooperativeRecognizedObjects) /
                       static_cast<double> (cooperativeExpectedStationIds.size ()));
          ++g_thesisStats.cooperativeRecognitionVehicleSamples;
        }
    }

  for (const auto& senderEntry : g_vehicleRuntime)
    {
      if (senderEntry.second.nrContainer == nullptr)
        {
          continue;
        }
      const uint64_t senderStationId = VehicleIdToStationId (senderEntry.first);
      const auto senderPositionIt = activeStationPositions.find (senderStationId);
      if (senderPositionIt == activeStationPositions.end ())
        {
          continue;
        }
      const auto& deletedIds =
          senderEntry.second.nrContainer->getCPBasicService ()->getLastRmrDeletedIds ();
      for (const uint64_t deletedId : deletedIds)
        {
          uint64_t resolvedDeletedId = deletedId;
          if (activeStationPositions.find (resolvedDeletedId) == activeStationPositions.end () &&
              activeStationPositions.find (deletedId + 1) != activeStationPositions.end ())
            {
              resolvedDeletedId = deletedId + 1;
            }
          else if (deletedId > 0 &&
                   activeStationPositions.find (resolvedDeletedId) == activeStationPositions.end () &&
                   activeStationPositions.find (deletedId - 1) != activeStationPositions.end ())
            {
              resolvedDeletedId = deletedId - 1;
            }
          const auto deletedPositionIt = activeStationPositions.find (resolvedDeletedId);
          if (deletedPositionIt == activeStationPositions.end ())
            {
              continue;
            }
          const Vector senderPosition = senderPositionIt->second;
          const Vector deletedPosition = deletedPositionIt->second;
          const double dx = deletedPosition.x - senderPosition.x;
          const double dy = deletedPosition.y - senderPosition.y;
          const double dz = deletedPosition.z - senderPosition.z;
          const double distanceMeters = std::sqrt ((dx * dx) + (dy * dy) + (dz * dz));
          const bool highPriority =
              distanceMeters <= g_priorityDistanceThresholdMeters ||
              IsClosingHighPriorityObject (senderStationId,
                                           resolvedDeletedId,
                                           senderPosition,
                                           deletedPosition,
                                           distanceMeters,
                                           now);
          ++g_thesisStats.rmrDeletedSenderEvalTotal;
          if (highPriority)
            {
              ++g_thesisStats.rmrDeletedSenderEvalHigh;
            }
          else
            {
              ++g_thesisStats.rmrDeletedSenderEvalLow;
            }
          g_thesisStats.rmrDeletedSenderEvalDistanceSum += distanceMeters;
          ++g_thesisStats.rmrDeletedSenderEvalDistanceSamples;

          const auto senderPrevIt = g_priorityMotionHistory.find (senderStationId);
          const auto deletedPrevIt = g_priorityMotionHistory.find (resolvedDeletedId);
          if (senderPrevIt == g_priorityMotionHistory.end () ||
              deletedPrevIt == g_priorityMotionHistory.end () || distanceMeters <= 1e-6)
            {
              continue;
            }
          const double senderDt = (now - senderPrevIt->second.timestamp).GetSeconds ();
          const double deletedDt = (now - deletedPrevIt->second.timestamp).GetSeconds ();
          if (senderDt <= 0.0 || deletedDt <= 0.0)
            {
              continue;
            }
          const Vector senderDelta = senderPosition - senderPrevIt->second.position;
          const Vector deletedDelta = deletedPosition - deletedPrevIt->second.position;
          const Vector senderVelocity (senderDelta.x / senderDt,
                                       senderDelta.y / senderDt,
                                       senderDelta.z / senderDt);
          const Vector deletedVelocity (deletedDelta.x / deletedDt,
                                        deletedDelta.y / deletedDt,
                                        deletedDelta.z / deletedDt);
          const Vector relativePosition = deletedPosition - senderPosition;
          const Vector relativeVelocity = deletedVelocity - senderVelocity;
          const double rangeRate =
              ((relativePosition.x * relativeVelocity.x) +
               (relativePosition.y * relativeVelocity.y) +
               (relativePosition.z * relativeVelocity.z)) /
              distanceMeters;
          const double closingSpeed = -rangeRate;
          if (closingSpeed >= g_priorityClosingSpeedThresholdMps)
            {
              ++g_thesisStats.rmrDeletedSenderEvalClosingSpeedGeThreshold;
              if (distanceMeters / closingSpeed <= g_priorityTtcThresholdSeconds)
                {
                  ++g_thesisStats.rmrDeletedSenderEvalTtcLeThreshold;
                }
            }
        }
    }

  g_priorityMotionHistory.clear ();
  for (const auto& entry : activeStationPositions)
    {
      PriorityMotionState state;
      state.position = entry.second;
      state.timestamp = now;
      const auto velocityIt = activeStationVelocities.find (entry.first);
      if (velocityIt != activeStationVelocities.end ())
        {
          state.velocity = velocityIt->second;
          state.hasVelocity = true;
        }
      g_priorityMotionHistory[entry.first] = state;
    }
}

static void
AccumulateThesisPacketLossCounters (Ptr<MetricSupervisor> dsrcMetrics,
                                    Ptr<MetricSupervisor> nrMetrics,
                                    Ptr<MetricSupervisor> mecMetrics,
                                    Ptr<TraciClient> sumoClient,
                                    double baselineMeters,
                                    bool accumulate = true)
{
  const auto cpmType = MetricSupervisor::messageType_cpm;

  const uint64_t dsrcRx = dsrcMetrics->getNumberRx_messagetype (cpmType);
  const uint64_t nrRx = nrMetrics->getNumberRx_messagetype (cpmType);
  const uint64_t mecRx =
      g_useIdealMecLink ? g_idealMecRx : mecMetrics->getNumberRx_messagetype (cpmType);

  const uint64_t dsrcDeltaRx = dsrcRx - g_thesisStats.dsrcLastCpmRx;
  const uint64_t nrDeltaRx = nrRx - g_thesisStats.nrLastCpmRx;
  const uint64_t mecDeltaRx = mecRx - g_thesisStats.mecLastCpmRx;

  g_thesisStats.dsrcLastCpmRx = dsrcRx;
  g_thesisStats.nrLastCpmRx = nrRx;
  g_thesisStats.mecLastCpmRx = mecRx;

  uint64_t dsrcHighIdealDelta = 0;
  uint64_t dsrcLowIdealDelta = 0;
  uint64_t nrHighIdealDelta = 0;
  uint64_t nrLowIdealDelta = 0;
  uint64_t mecHighIdealDelta = 0;
  uint64_t mecLowIdealDelta = 0;
  uint64_t idealMecHighTrueDelta = 0;
  uint64_t idealMecLowTrueDelta = 0;

  if (g_useIdealMecLink)
    {
      mecHighIdealDelta =
          g_idealMecHighAttempts - g_thesisStats.idealMecLastHighAttempts;
      mecLowIdealDelta =
          g_idealMecLowAttempts - g_thesisStats.idealMecLastLowAttempts;
      idealMecHighTrueDelta =
          g_idealMecHighSuccesses - g_thesisStats.idealMecLastHighSuccesses;
      idealMecLowTrueDelta =
          g_idealMecLowSuccesses - g_thesisStats.idealMecLastLowSuccesses;
      g_thesisStats.idealMecLastHighAttempts = g_idealMecHighAttempts;
      g_thesisStats.idealMecLastLowAttempts = g_idealMecLowAttempts;
      g_thesisStats.idealMecLastHighSuccesses = g_idealMecHighSuccesses;
      g_thesisStats.idealMecLastLowSuccesses = g_idealMecLowSuccesses;
    }

  for (const auto& entry : g_vehicleRuntime)
    {
      const ReceiverPriorityCounts receivers =
          CountReceiversByPriorityWithinBaseline (sumoClient, entry.first, baselineMeters);

      const uint64_t dsrcTx =
          entry.second.dsrcContainer->getCPBasicService ()->getCpmSent ();
      const uint64_t dsrcLastTx = g_thesisStats.dsrcLastCpmTxByVehicle[entry.first];
      const uint64_t dsrcDeltaTx = dsrcTx - dsrcLastTx;
      dsrcHighIdealDelta += dsrcDeltaTx * receivers.high;
      dsrcLowIdealDelta += dsrcDeltaTx * receivers.low;
      g_thesisStats.dsrcLastCpmTxByVehicle[entry.first] = dsrcTx;

      if (entry.second.nrContainer != nullptr)
        {
          const uint64_t nrTx =
              entry.second.nrContainer->getCPBasicService ()->getCpmSent ();
          const uint64_t nrLastTx = g_thesisStats.nrLastCpmTxByVehicle[entry.first];
          const uint64_t nrDeltaTx = nrTx - nrLastTx;
          nrHighIdealDelta += nrDeltaTx * receivers.high;
          nrLowIdealDelta += nrDeltaTx * receivers.low;
          g_thesisStats.nrLastCpmTxByVehicle[entry.first] = nrTx;
        }

      if (!g_useIdealMecLink && entry.second.mecContainer != nullptr)
        {
          const uint64_t mecTx =
              entry.second.mecContainer->getCPBasicService ()->getCpmSent ();
          const uint64_t mecLastTx = g_thesisStats.mecLastCpmTxByVehicle[entry.first];
          const uint64_t mecDeltaTx = mecTx - mecLastTx;
          mecHighIdealDelta += mecDeltaTx * receivers.high;
          mecLowIdealDelta += mecDeltaTx * receivers.low;
          g_thesisStats.mecLastCpmTxByVehicle[entry.first] = mecTx;
        }
    }

  if (!accumulate)
    {
      return;
    }

  auto allocateTrueRx = [] (uint64_t trueRx, uint64_t classIdeal, uint64_t routeIdeal) {
    if (trueRx == 0 || classIdeal == 0 || routeIdeal == 0)
      {
        return static_cast<uint64_t> (0);
      }
    const double share = static_cast<double> (classIdeal) / static_cast<double> (routeIdeal);
    return std::min (trueRx, static_cast<uint64_t> (std::llround (trueRx * share)));
  };

  const uint64_t dsrcIdealDelta = dsrcHighIdealDelta + dsrcLowIdealDelta;
  const uint64_t nrIdealDelta = nrHighIdealDelta + nrLowIdealDelta;
  const uint64_t mecIdealDelta = mecHighIdealDelta + mecLowIdealDelta;
  g_thesisStats.dsrcIdealCpmRx += dsrcIdealDelta;
  g_thesisStats.nrIdealCpmRx += nrIdealDelta;
  g_thesisStats.mecIdealCpmRx += mecIdealDelta;
  const uint64_t dsrcTrueDelta = std::min (dsrcDeltaRx, dsrcIdealDelta);
  const uint64_t nrTrueDelta = std::min (nrDeltaRx, nrIdealDelta);
  const uint64_t mecTrueDelta = std::min (mecDeltaRx, mecIdealDelta);
  const uint64_t dsrcHighTrueDelta =
      allocateTrueRx (dsrcTrueDelta, dsrcHighIdealDelta, dsrcIdealDelta);
  const uint64_t nrHighTrueDelta = allocateTrueRx (nrTrueDelta, nrHighIdealDelta, nrIdealDelta);
  const uint64_t mecHighTrueDelta =
      g_useIdealMecLink
          ? std::min (idealMecHighTrueDelta, mecHighIdealDelta)
          : allocateTrueRx (mecTrueDelta, mecHighIdealDelta, mecIdealDelta);
  const uint64_t dsrcLowTrueDelta =
      dsrcLowIdealDelta > 0 ? dsrcTrueDelta - dsrcHighTrueDelta : 0;
  const uint64_t nrLowTrueDelta = nrLowIdealDelta > 0 ? nrTrueDelta - nrHighTrueDelta : 0;
  const uint64_t mecLowTrueDelta =
      g_useIdealMecLink
          ? std::min (idealMecLowTrueDelta, mecLowIdealDelta)
          : (mecLowIdealDelta > 0 ? mecTrueDelta - mecHighTrueDelta : 0);

  g_thesisStats.dsrcTrueCpmRx += dsrcTrueDelta;
  g_thesisStats.nrTrueCpmRx += nrTrueDelta;
  g_thesisStats.mecTrueCpmRx += mecHighTrueDelta + mecLowTrueDelta;
  g_thesisStats.highIdealCpmRx += dsrcHighIdealDelta + nrHighIdealDelta + mecHighIdealDelta;
  g_thesisStats.lowIdealCpmRx += dsrcLowIdealDelta + nrLowIdealDelta + mecLowIdealDelta;
  g_thesisStats.highTrueCpmRx += dsrcHighTrueDelta + nrHighTrueDelta + mecHighTrueDelta;
  g_thesisStats.lowTrueCpmRx += dsrcLowTrueDelta + nrLowTrueDelta + mecLowTrueDelta;
  g_thesisStats.nrHighIdealCpmRx += nrHighIdealDelta;
  g_thesisStats.nrHighTrueCpmRx += nrHighTrueDelta;
  g_thesisStats.nrLowIdealCpmRx += nrLowIdealDelta;
  g_thesisStats.nrLowTrueCpmRx += nrLowTrueDelta;
}

static void
SampleThesisMetrics (Ptr<MetricSupervisor> dsrcMetrics,
                     Ptr<MetricSupervisor> nrMetrics,
                     Ptr<MetricSupervisor> mecMetrics,
                     Ptr<TraciClient> sumoClient,
                     double baselineMeters,
                     Time interval)
{
  const bool accumulate = ShouldAccumulateThesisMetrics ();
  if (accumulate)
    {
      AccumulateThesisRecognitionSample ();
    }
  AccumulateThesisPacketLossCounters (dsrcMetrics,
                                      nrMetrics,
                                      mecMetrics,
                                      sumoClient,
                                      baselineMeters,
                                      accumulate);

  Simulator::Schedule (interval,
                       &SampleThesisMetrics,
                       dsrcMetrics,
                       nrMetrics,
                       mecMetrics,
                       sumoClient,
                       baselineMeters,
                       interval);
}

static double
ThesisRecognitionRatePercent ()
{
  if (g_thesisStats.recognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.recognitionRatioSum /
         static_cast<double> (g_thesisStats.recognitionVehicleSamples);
}

static double
ThesisHighPriorityObjectRecognitionRatePercent ()
{
  if (g_thesisStats.highPriorityRecognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.highPriorityRecognitionRatioSum /
         static_cast<double> (g_thesisStats.highPriorityRecognitionVehicleSamples);
}

static double
ThesisLowPriorityObjectRecognitionRatePercent ()
{
  if (g_thesisStats.lowPriorityRecognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.lowPriorityRecognitionRatioSum /
         static_cast<double> (g_thesisStats.lowPriorityRecognitionVehicleSamples);
}

static double
ThesisSensorObjectRecognitionRatePercent ()
{
  if (g_thesisStats.sensorRecognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.sensorRecognitionRatioSum /
         static_cast<double> (g_thesisStats.sensorRecognitionVehicleSamples);
}

static double
ThesisSensorHighPriorityObjectRecognitionRatePercent ()
{
  if (g_thesisStats.sensorHighPriorityRecognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.sensorHighPriorityRecognitionRatioSum /
         static_cast<double> (g_thesisStats.sensorHighPriorityRecognitionVehicleSamples);
}

static double
ThesisSensorLowPriorityObjectRecognitionRatePercent ()
{
  if (g_thesisStats.sensorLowPriorityRecognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.sensorLowPriorityRecognitionRatioSum /
         static_cast<double> (g_thesisStats.sensorLowPriorityRecognitionVehicleSamples);
}

static double
ThesisCooperativeObjectRecognitionRatePercent ()
{
  if (g_thesisStats.cooperativeRecognitionVehicleSamples == 0)
    {
      return -1.0;
    }
  return 100.0 * g_thesisStats.cooperativeRecognitionRatioSum /
         static_cast<double> (g_thesisStats.cooperativeRecognitionVehicleSamples);
}

static double
PercentFromRatioSum (double ratioSum, uint64_t samples)
{
  if (samples == 0)
    {
      return -1.0;
    }
  return 100.0 * ratioSum / static_cast<double> (samples);
}

static double
AverageFromSum (double sum, uint64_t samples)
{
  if (samples == 0)
    {
      return -1.0;
    }
  return sum / static_cast<double> (samples);
}

static double
ThesisPacketLossRatePercent (uint64_t idealRx, uint64_t trueRx)
{
  if (idealRx == 0)
    {
      return -1.0;
    }
  if (trueRx >= idealRx)
    {
      return 0.0;
    }
  return 100.0 * static_cast<double> (idealRx - trueRx) / static_cast<double> (idealRx);
}

static double
ThesisPacketDeliveryRatioPercent (uint64_t idealRx, uint64_t trueRx)
{
  if (idealRx == 0)
    {
      return -1.0;
    }
  return 100.0 * static_cast<double> (std::min (idealRx, trueRx)) /
         static_cast<double> (idealRx);
}

static std::string
FormatThesisMetric (double value)
{
  if (value < 0.0)
    {
      return "N/A";
    }

  std::ostringstream stream;
  stream << value;
  return stream.str ();
}

static Ptr<BSContainer>
GetPrimaryCpmContainer (const VehicleRuntime& runtime)
{
  return runtime.nrContainer != nullptr ? runtime.nrContainer : runtime.dsrcContainer;
}

static double
AverageLatestPrimaryCpmSizeBytes ()
{
  double sizeSum = 0.0;
  uint32_t samples = 0;
  for (const auto& entry : g_vehicleRuntime)
    {
      Ptr<BSContainer> primaryContainer = GetPrimaryCpmContainer (entry.second);
      if (primaryContainer == nullptr)
        {
          continue;
        }
      const uint32_t sizeBytes = primaryContainer->getCPBasicService ()->getLastCpmSizeBytes ();
      if (sizeBytes > 0)
        {
          sizeSum += static_cast<double> (sizeBytes);
          ++samples;
        }
    }

  if (samples == 0)
    {
      return std::max (g_predictiveRmrConfig.trafficFlowAvgPacketSizeBytes, 1.0);
    }
  return std::max (sizeSum / static_cast<double> (samples), 1.0);
}

static double
AverageVehicleSpeedMetersPerSecond ()
{
  double speedSum = 0.0;
  uint32_t samples = 0;
  for (const auto& entry : g_vehicleRuntime)
    {
      if (g_sumoClient != nullptr)
        {
          try
            {
              const double speed = g_sumoClient->TraCIAPI::vehicle.getSpeed (entry.first);
              if (speed >= 0.0)
                {
                  speedSum += speed;
                  ++samples;
                  continue;
                }
            }
          catch (...)
            {
            }
        }

      Ptr<MobilityModel> mobility =
          entry.second.node != nullptr ? entry.second.node->GetObject<MobilityModel> () : nullptr;
      if (mobility != nullptr)
        {
          const Vector velocity = mobility->GetVelocity ();
          speedSum += std::sqrt ((velocity.x * velocity.x) + (velocity.y * velocity.y) +
                                 (velocity.z * velocity.z));
          ++samples;
        }
    }
  return samples > 0 ? speedSum / static_cast<double> (samples) : 0.0;
}

static double
EstimateTrafficCbrFromVehicleCount (double vehicleCount,
                                    double packetSizeBits,
                                    double messageRateHz)
{
  const double channelRateBps = std::max (g_predictiveChannelRateBps, 1.0);
  const double loadBps =
      std::max (0.0, vehicleCount) * std::max (0.0, messageRateHz) * packetSizeBits;
  return Clamp01 (loadBps / channelRateBps);
}

static double
ComputeRsuPassiveObservationPdr (double referenceCbr)
{
  return referenceCbr >= g_predictiveRmrConfig.rsuPassiveSaturationCbr
             ? 0.0
             : (referenceCbr >= g_hybridRouteConfig.switchCbr
                    ? g_predictiveRmrConfig.rsuPassivePdrMidCbr
                    : g_predictiveRmrConfig.rsuPassivePdrLowCbr);
}

static double
NormalizeTrafficFlowLoopPosition (double positionMeters)
{
  const double roadLengthM = std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters, 1.0);
  double normalized = std::fmod (positionMeters, roadLengthM);
  if (normalized < 0.0)
    {
      normalized += roadLengthM;
    }
  return normalized;
}

static bool
IsTrafficFlowPositionInSegment (double positionMeters, double segmentStartM, double segmentEndM)
{
  const double roadLengthM = std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters, 1.0);
  if (!g_trafficFlowIsLoop)
    {
      return positionMeters >= std::max (0.0, segmentStartM) &&
             positionMeters < std::min (roadLengthM, segmentEndM);
    }

  if (segmentEndM - segmentStartM >= roadLengthM)
    {
      return true;
    }

  const double position = NormalizeTrafficFlowLoopPosition (positionMeters);
  const double start = NormalizeTrafficFlowLoopPosition (segmentStartM);
  const double end = NormalizeTrafficFlowLoopPosition (segmentEndM);
  if (start <= end)
    {
      return position >= start && position < end;
    }
  return position >= start || position < end;
}

static std::vector<TrafficFlowVehicleSnapshot>
ReadTrafficFlowVehicleSnapshots ()
{
  std::vector<TrafficFlowVehicleSnapshot> vehicles;
  if (g_sumoClient == nullptr)
    {
      return vehicles;
    }

  for (const auto& entry : g_allVehicleNodes)
    {
      try
        {
          const libsumo::TraCIPosition position =
              g_sumoClient->TraCIAPI::vehicle.getPosition (entry.first);
          TrafficFlowVehicleSnapshot snapshot;
          snapshot.id = entry.first;
          snapshot.x = position.x;
          snapshot.y = position.y;
          snapshot.loopPositionMeters = ProjectLoopPositionMeters (position);
          snapshot.speedMps = std::max (0.0, g_sumoClient->TraCIAPI::vehicle.getSpeed (entry.first));
          const std::string roadId = g_sumoClient->TraCIAPI::vehicle.getRoadID (entry.first);
          snapshot.direction =
              roadId.rfind ("highway_wb", 0) == 0 || roadId.rfind ("loop_r", 0) == 0 ? -1 : 1;
          vehicles.push_back (snapshot);
        }
      catch (...)
        {
        }
    }
  return vehicles;
}

static TrafficFlowSegmentStats
ComputeTrafficFlowSegmentStats (const std::vector<TrafficFlowVehicleSnapshot>& vehicles,
                                double segmentStartM,
                                double segmentEndM,
                                double packetSizeBits,
                                int directionFilter = 0)
{
  TrafficFlowSegmentStats stats;
  double speedSum = 0.0;
  double cpmSizeSum = 0.0;
  double cpmTxRateSum = 0.0;
  uint32_t cpmSizeSamples = 0;
  uint32_t cpmTxRateSamples = 0;
  for (const auto& vehicle : vehicles)
    {
      if (directionFilter != 0 && vehicle.direction != directionFilter)
        {
          continue;
        }
      if (IsTrafficFlowPositionInSegment (vehicle.loopPositionMeters, segmentStartM, segmentEndM))
        {
          ++stats.vehicleCount;
          speedSum += vehicle.speedMps;

          const auto runtimeIt = g_vehicleRuntime.find (vehicle.id);
          if (runtimeIt != g_vehicleRuntime.end ())
            {
              Ptr<BSContainer> primaryContainer = GetPrimaryCpmContainer (runtimeIt->second);
              if (primaryContainer != nullptr)
                {
                  const uint32_t sizeBytes =
                      primaryContainer->getCPBasicService ()->getLastCpmSizeBytes ();
                  if (sizeBytes > 0)
                    {
                      cpmSizeSum += static_cast<double> (sizeBytes);
                      ++cpmSizeSamples;
                    }
                }

              const auto stateIt = g_predictiveCbrState.find (vehicle.id);
              if (stateIt != g_predictiveCbrState.end () && stateIt->second.initialized &&
                  stateIt->second.dsrcCpmTxRate > 0.0)
                {
                  cpmTxRateSum += stateIt->second.dsrcCpmTxRate;
                  ++cpmTxRateSamples;
                }
            }
        }
    }

  stats.averageSpeedMps =
      stats.vehicleCount > 0 ? speedSum / static_cast<double> (stats.vehicleCount) : 0.0;
  stats.observedCpmSizeBytes =
      cpmSizeSamples > 0
          ? cpmSizeSum / static_cast<double> (cpmSizeSamples)
          : std::max (packetSizeBits / 8.0, g_predictiveRmrConfig.trafficFlowAvgPacketSizeBytes);
  stats.observedCpmTxRateHz =
      cpmTxRateSamples > 0
          ? cpmTxRateSum / static_cast<double> (cpmTxRateSamples)
          : g_predictiveRmrConfig.trafficFlowMessageRateHz;
  stats.cbr = EstimateTrafficCbrFromVehicleCount (stats.vehicleCount,
                                                  stats.observedCpmSizeBytes * 8.0,
                                                  stats.observedCpmTxRateHz);
  return stats;
}

static double
ComputeTrafficFlowRate (double vehicleCount, double speedMps, double segmentLengthM)
{
  return (vehicleCount / std::max (segmentLengthM, 1.0)) * std::max (0.0, speedMps);
}

static double
PredictStraightSegmentPeakVehicles (
    const std::vector<TrafficFlowVehicleSnapshot>& vehicles,
    double segmentStartM,
    double segmentEndM,
    double horizonSeconds,
    double* peakTimeSeconds)
{
  uint32_t peakVehicles = 0;
  const double stepSeconds = 1.0;
  for (double offset = 0.0; offset <= horizonSeconds; offset += stepSeconds)
    {
      uint32_t projectedVehicles = 0;
      for (const auto& vehicle : vehicles)
        {
          const double projectedPosition =
              vehicle.loopPositionMeters +
              static_cast<double> (vehicle.direction) * vehicle.speedMps * offset;
          if (projectedPosition >= segmentStartM && projectedPosition < segmentEndM)
            {
              ++projectedVehicles;
            }
        }
      if (projectedVehicles > peakVehicles)
        {
          peakVehicles = projectedVehicles;
          if (peakTimeSeconds != nullptr)
            {
              *peakTimeSeconds = offset;
            }
        }
    }
  return static_cast<double> (peakVehicles);
}

static bool
UpdateRsuSharedTrafficFlowPrediction (double packetSizeBits,
                                      Ptr<MetricSupervisor> channelMetrics)
{
  if (g_trafficFlowRsus.empty () || g_sumoClient == nullptr)
    {
      return false;
    }

  const std::vector<TrafficFlowVehicleSnapshot> vehicles = ReadTrafficFlowVehicleSnapshots ();
  if (vehicles.empty ())
    {
      return false;
    }

  const double segmentLengthM =
      std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters /
                    std::max (1.0, static_cast<double> (g_trafficFlowRsus.size ())),
                1.0);
  const double halfSegmentM = segmentLengthM / 2.0;
  const double horizonSeconds = std::max (g_predictiveRmrConfig.predictionHorizonSeconds, 0.0);
  double qInSum = 0.0;
  double qOutSum = 0.0;
  double localVehicleSum = 0.0;
  double upstreamVehicleSum = 0.0;
  double futureVehicleSum = 0.0;
  double predictedCbrSum = 0.0;
  double predictedCbrMax = 0.0;
  uint32_t metricCount = 0;
  g_trafficFlowRsuPredictions.clear ();

  for (const auto& rsu : g_trafficFlowRsus)
    {
      libsumo::TraCIPosition rsuPosition;
      rsuPosition.x = rsu.x;
      rsuPosition.y = rsu.y;
      const double rsuLoopPosition = ProjectLoopPositionMeters (rsuPosition);
      const double localStart = rsuLoopPosition - halfSegmentM;
      const double localEnd = rsuLoopPosition + halfSegmentM;
      const double cwUpstreamStart = localStart - segmentLengthM;
      const double cwUpstreamEnd = localStart;
      const double ccwUpstreamStart = localEnd;
      const double ccwUpstreamEnd = localEnd + segmentLengthM;

      const TrafficFlowSegmentStats local =
          ComputeTrafficFlowSegmentStats (vehicles, localStart, localEnd, packetSizeBits);
      const TrafficFlowSegmentStats localCw =
          ComputeTrafficFlowSegmentStats (vehicles, localStart, localEnd, packetSizeBits, 1);
      const TrafficFlowSegmentStats localCcw =
          ComputeTrafficFlowSegmentStats (vehicles, localStart, localEnd, packetSizeBits, -1);
      const TrafficFlowSegmentStats upstreamCw =
          ComputeTrafficFlowSegmentStats (vehicles,
                                          cwUpstreamStart,
                                          cwUpstreamEnd,
                                          packetSizeBits,
                                          1);
      const TrafficFlowSegmentStats upstreamCcw =
          ComputeTrafficFlowSegmentStats (vehicles,
                                          ccwUpstreamStart,
                                          ccwUpstreamEnd,
                                          packetSizeBits,
                                          -1);

      const double qIn =
          ComputeTrafficFlowRate (upstreamCw.vehicleCount,
                                  upstreamCw.averageSpeedMps,
                                  segmentLengthM) +
          ComputeTrafficFlowRate (upstreamCcw.vehicleCount,
                                  upstreamCcw.averageSpeedMps,
                                  segmentLengthM);
      const double qOut =
          ComputeTrafficFlowRate (localCw.vehicleCount, localCw.averageSpeedMps, segmentLengthM) +
          ComputeTrafficFlowRate (localCcw.vehicleCount, localCcw.averageSpeedMps, segmentLengthM);
      double peakTimeSeconds = horizonSeconds;
      const double futureVehicles =
          g_trafficFlowIsLoop
              ? std::max (0.0,
                          static_cast<double> (local.vehicleCount) +
                              (qIn - qOut) * horizonSeconds)
              : PredictStraightSegmentPeakVehicles (vehicles,
                                                    std::max (0.0, localStart),
                                                    std::min (
                                                        g_predictiveRmrConfig
                                                            .trafficFlowRoadLengthMeters,
                                                        localEnd),
                                                    horizonSeconds,
                                                    &peakTimeSeconds);
      const double upstreamObservedCpmSizeBytes =
          (upstreamCw.observedCpmSizeBytes + upstreamCcw.observedCpmSizeBytes) / 2.0;
      const double upstreamObservedCpmTxRateHz =
          (upstreamCw.observedCpmTxRateHz + upstreamCcw.observedCpmTxRateHz) / 2.0;
      const double observedCpmSizeBytes =
          local.observedCpmSizeBytes > 0.0 ? local.observedCpmSizeBytes
                                           : upstreamObservedCpmSizeBytes;
      const double observedCpmTxRateHz =
          local.observedCpmTxRateHz > 0.0 ? local.observedCpmTxRateHz
                                          : upstreamObservedCpmTxRateHz;
      double measuredLocalCbr = 0.0;
      if (channelMetrics != nullptr)
        {
          for (const auto& vehicle : vehicles)
            {
              if (!IsTrafficFlowPositionInSegment (vehicle.loopPositionMeters,
                                                   localStart,
                                                   localEnd))
                {
                  continue;
                }
              measuredLocalCbr =
                  std::max (measuredLocalCbr,
                            std::max (0.0, channelMetrics->getCBRPerItem (vehicle.id)));
            }
        }
      const double referenceCbr =
          std::max ({local.cbr, upstreamCw.cbr, upstreamCcw.cbr, measuredLocalCbr});
      const double passivePdr = ComputeRsuPassiveObservationPdr (referenceCbr);
      const double correctedCpmTxRateHz =
          passivePdr > 0.0 ? observedCpmTxRateHz / passivePdr : observedCpmTxRateHz;
      const double futureTrafficCbr =
          EstimateTrafficCbrFromVehicleCount (futureVehicles,
                                              observedCpmSizeBytes * 8.0,
                                              correctedCpmTxRateHz);
      const double currentTrafficCbr =
          std::max ({local.cbr, upstreamCw.cbr, upstreamCcw.cbr});
      const double trafficFlowIncrease =
          std::max (0.0, futureTrafficCbr - currentTrafficCbr);
      const double predictedCbr =
          referenceCbr >= g_predictiveRmrConfig.rsuPassiveSaturationCbr
              ? 1.0
              : Clamp01 (std::max (measuredLocalCbr, currentTrafficCbr) +
                         trafficFlowIncrease);
      g_trafficFlowRsuPredictions[rsu.id] = {
          measuredLocalCbr,
          predictedCbr,
          peakTimeSeconds,
          local.vehicleCount,
          futureVehicles};

      qInSum += qIn;
      qOutSum += qOut;
      localVehicleSum += static_cast<double> (local.vehicleCount);
      upstreamVehicleSum +=
          static_cast<double> (upstreamCw.vehicleCount + upstreamCcw.vehicleCount);
      futureVehicleSum += futureVehicles;
      predictedCbrSum += predictedCbr;
      predictedCbrMax = std::max (predictedCbrMax, predictedCbr);
      ++metricCount;
    }

  if (metricCount == 0)
    {
      return false;
    }

  const double invCount = 1.0 / static_cast<double> (metricCount);
  g_trafficFlowPrediction.qInVehiclesPerSecond = qInSum * invCount;
  g_trafficFlowPrediction.qOutVehiclesPerSecond = qOutSum * invCount;
  g_trafficFlowPrediction.localVehicleCountAvg = localVehicleSum * invCount;
  g_trafficFlowPrediction.upstreamVehicleCountAvg = upstreamVehicleSum * invCount;
  g_trafficFlowPrediction.futureVehicleCountAvg = futureVehicleSum * invCount;
  g_trafficFlowPrediction.rsuPredictedCbrAvg = predictedCbrSum * invCount;
  g_trafficFlowPrediction.rsuPredictedCbrMax = predictedCbrMax;
  g_trafficFlowPrediction.rsuCount = metricCount;
  g_trafficFlowPrediction.trafficFlowCbr = predictedCbrMax;
  g_trafficFlowPrediction.deltaCbr = 0.0;
  return true;
}

static void
UpdateTrafficFlowCbrPrediction (Ptr<MetricSupervisor> channelMetrics, Time interval)
{
  const Time now = Simulator::Now ();
  const uint32_t activeVehicles = static_cast<uint32_t> (g_vehicleRuntime.size ());
  double elapsedSeconds = interval.GetSeconds ();
  double activeVehicleRate = 0.0;
  if (g_trafficFlowPrediction.initialized)
    {
      elapsedSeconds = std::max ((now - g_trafficFlowPrediction.lastUpdate).GetSeconds (), 1e-9);
      activeVehicleRate =
          (static_cast<double> (activeVehicles) -
           static_cast<double> (g_trafficFlowPrediction.lastActiveVehicles)) /
          elapsedSeconds;
    }

  const double roadLengthMeters =
      std::max (g_predictiveRmrConfig.trafficFlowRoadLengthMeters, 1.0);
  const double meanSpeed = AverageVehicleSpeedMetersPerSecond ();
  const double densityVehiclesPerMeter =
      static_cast<double> (activeVehicles) / roadLengthMeters;
  const double measuredQOut = densityVehiclesPerMeter * meanSpeed;
  double estimatedQIn = measuredQOut;
  if (g_trafficFlowPrediction.initialized)
    {
      estimatedQIn =
          std::max (0.0, g_trafficFlowPrediction.qOutVehiclesPerSecond + activeVehicleRate);
    }

  const double packetSizeBits = AverageLatestPrimaryCpmSizeBytes () * 8.0;
  const bool usedRsuSharedPrediction =
      UpdateRsuSharedTrafficFlowPrediction (packetSizeBits, channelMetrics);
  if (usedRsuSharedPrediction)
    {
      g_trafficFlowPrediction.initialized = true;
      g_trafficFlowPrediction.lastUpdate = now;
      g_trafficFlowPrediction.lastActiveVehicles = activeVehicles;
      g_trafficFlowPrediction.meanSpeedMetersPerSecond = meanSpeed;
      return;
    }

  const double channelRateBps = std::max (g_predictiveChannelRateBps, 1.0);
  const double trafficFlowCbr =
      (static_cast<double> (activeVehicles) *
       g_predictiveRmrConfig.trafficFlowMessageRateHz * packetSizeBits) /
      channelRateBps;
  const double deltaCbr =
      ((estimatedQIn - measuredQOut) *
       g_predictiveRmrConfig.predictionHorizonSeconds *
       g_predictiveRmrConfig.trafficFlowMessageRateHz * packetSizeBits) /
      channelRateBps;

  g_trafficFlowPrediction.initialized = true;
  g_trafficFlowPrediction.lastUpdate = now;
  g_trafficFlowPrediction.lastActiveVehicles = activeVehicles;
  g_trafficFlowPrediction.qInVehiclesPerSecond = estimatedQIn;
  g_trafficFlowPrediction.qOutVehiclesPerSecond = measuredQOut;
  g_trafficFlowPrediction.trafficFlowCbr = Clamp01 (trafficFlowCbr);
  g_trafficFlowPrediction.deltaCbr = deltaCbr;
  g_trafficFlowPrediction.meanSpeedMetersPerSecond = meanSpeed;
}

static bool
IsInMovingBackgroundHighLoadZone (const std::string& vehicleId);

static void
OpenObservationLog (const std::string& path)
{
  if (path.empty ())
    {
      return;
    }

  g_observationLog.open (path, std::ios::out);
  if (!g_observationLog.is_open ())
    {
      NS_FATAL_ERROR ("Unable to open observation log file: " << path);
    }

  g_observationLog
      << "time_s,active_vehicles,legacy_route_count,nr_route_count,mec_route_count,dual_route_count,channel_busy_ratio_avg,channel_busy_ratio_max,"
      << "predicted_channel_busy_ratio_avg,predicted_channel_busy_ratio_max,channel_busy_ratio_rate_avg,"
      << "legacy_cpm_tx_rate_avg,traffic_q_in,traffic_q_out,traffic_flow_cbr,traffic_delta_cbr,"
      << "traffic_mean_speed_mps,rsu_count,rsu_local_vehicle_avg,rsu_upstream_vehicle_avg,"
      << "rsu_future_vehicle_avg,rsu_predicted_cbr_avg,rsu_predicted_cbr_max,"
      << "legacy_prr_avg,legacy_packet_loss_avg,legacy_latency_ms,legacy_tx,legacy_rx,"
      << "nr_prr_avg,nr_packet_loss_avg,nr_latency_ms,nr_tx,nr_rx,"
      << "mec_prr_avg,mec_packet_loss_avg,mec_latency_ms,mec_tx,mec_rx,"
      << "legacy_cpm_objects,legacy_rmr_candidates,legacy_rmr_deleted_last,"
      << "legacy_rmr_deleted_near_last,legacy_rmr_deleted_far_last,"
      << "legacy_cpm_objects_total,legacy_rmr_candidates_total,legacy_rmr_deleted_total,"
      << "legacy_rmr_deleted_near_total,legacy_rmr_deleted_far_total,"
      << "legacy_cpm_size_bytes,legacy_cpm_size_bytes_total,nr_cpm_objects,nr_rmr_candidates,"
      << "nr_rmr_deleted_last,nr_rmr_deleted_near_last,nr_rmr_deleted_far_last,"
      << "nr_cpm_objects_total,nr_rmr_candidates_total,nr_rmr_deleted_total,"
      << "nr_rmr_deleted_near_total,nr_rmr_deleted_far_total,"
      << "nr_rmr_deleted_feature_count,nr_rmr_deleted_distance_mean_m,"
      << "nr_rmr_deleted_frequency_mean,nr_rmr_deleted_position_change_mean_m,"
      << "nr_rmr_deleted_speed_change_mean_mps,nr_rmr_deleted_score_mean,"
      << "nr_cpm_size_bytes,nr_cpm_size_bytes_total,"
      << "mec_cpm_objects,mec_rmr_candidates,mec_rmr_deleted_last,"
      << "mec_rmr_deleted_near_last,mec_rmr_deleted_far_last,"
      << "mec_cpm_objects_total,mec_rmr_candidates_total,mec_rmr_deleted_total,"
      << "mec_rmr_deleted_near_total,mec_rmr_deleted_far_total,"
      << "mec_cpm_size_bytes,mec_cpm_size_bytes_total,"
      << "rmr_deleted_feature_count,rmr_deleted_distance_mean_m,"
      << "rmr_deleted_frequency_mean,rmr_deleted_position_change_mean_m,"
      << "rmr_deleted_speed_change_mean_mps,rmr_deleted_score_mean,"
      << "legacy_cpm_wannabe_sent,legacy_cpm_sent,"
      << "nr_cpm_wannabe_sent,nr_cpm_sent,"
      << "mec_cpm_wannabe_sent,mec_cpm_sent,"
      << "legacy_cpm_start_calls,nr_cpm_start_calls,mec_cpm_start_calls,"
      << "cam_rx,cpm_rx,interference_tx,interference_bytes,interference_drops,"
      << "bg_high_load_active_vehicles,bg_high_load_all_vehicles,"
      << "interference_offered_cbr,"
      << "mec_uplink_packets,mec_uplink_bytes,mec_forwarded_packets,"
      << "mec_forwarded_bytes,mec_bg_ul_bytes,mec_bg_dl_bytes,"
      << "mec_ul_capacity_mbps,mec_dl_capacity_mbps,"
      << "mec_ul_bytes_delta,mec_dl_bytes_delta,"
      << "mec_ul_offered_mbps,mec_dl_offered_mbps,"
      << "mec_ul_busy_ratio,mec_dl_busy_ratio,"
      << "mec_forward_drops,mec_forward_no_receiver,"
      << "v2v_aoi_mean_ms,v2v_aoi_p50_ms,v2v_aoi_p90_ms,v2v_aoi_p99_ms,"
      << "v2v_aoi_le_200_ms_rate,v2v_aoi_le_300_ms_rate,"
      << "v2v_aoi_le_400_ms_rate,v2v_aoi_le_500_ms_rate,"
      << "v2v_aoi_violation_rate_200ms,v2v_aoi_violation_rate_300ms,"
      << "v2v_aoi_violation_rate_400ms,v2v_aoi_violation_rate_500ms,"
      << "v2v_aoi_orr_le_200_ms,v2v_aoi_orr_le_300_ms,"
      << "v2v_aoi_orr_le_400_ms,v2v_aoi_orr_le_500_ms,"
      << "v2v_only_aoi_orr_le_200_ms,v2v_only_aoi_orr_le_300_ms,"
      << "v2v_only_aoi_orr_le_400_ms,v2v_only_aoi_orr_le_500_ms,"
      << "mec_aoi_mean_ms,mec_aoi_p50_ms,mec_aoi_p90_ms,mec_aoi_p99_ms,"
      << "mec_aoi_le_200_ms_rate,mec_aoi_le_300_ms_rate,"
      << "mec_aoi_le_400_ms_rate,mec_aoi_le_500_ms_rate,"
      << "mec_aoi_violation_rate_200ms,mec_aoi_violation_rate_300ms,"
      << "mec_aoi_violation_rate_400ms,mec_aoi_violation_rate_500ms,"
      << "mec_high_aoi_le_200_ms_rate,mec_high_aoi_le_300_ms_rate,"
      << "mec_high_aoi_le_400_ms_rate,mec_high_aoi_le_500_ms_rate,"
      << "mec_low_aoi_le_200_ms_rate,mec_low_aoi_le_300_ms_rate,"
      << "mec_low_aoi_le_400_ms_rate,mec_low_aoi_le_500_ms_rate,"
      << "mec_aoi_orr_le_200_ms,mec_aoi_orr_le_300_ms,"
      << "mec_aoi_orr_le_400_ms,mec_aoi_orr_le_500_ms,"
      << "mec_only_aoi_orr_le_200_ms,mec_only_aoi_orr_le_300_ms,"
      << "mec_only_aoi_orr_le_400_ms,mec_only_aoi_orr_le_500_ms,"
      << "mec_capacity_delay_mean_ms,mec_capacity_delay_p90_ms,"
      << "mec_capacity_delay_p99_ms,mec_capacity_queue_drops,"
      << "mec_ul_first_losses,mec_dl_first_losses,"
      << "mec_ul_retx_recovered,mec_dl_retx_recovered,"
      << "mec_ul_final_losses,mec_dl_final_losses,"
      << "mec_fresh_update_loss_rate,mec_update_failure_rate,"
      << "mec_valid_update_success_rate,mec_ul_final_loss_rate,"
      << "mec_dl_final_loss_rate,mec_radio_final_loss_rate,"
      << "mec_retx_delay_mean_ms,mec_retx_delay_p90_ms,"
      << "orr,high_pdr,low_pdr,high_packet_loss,low_packet_loss,"
      << "v2v_radio_loss_rate,v2v_high_radio_loss_rate,v2v_low_radio_loss_rate,"
      << "v2v_update_failure_rate,"
      << "high_ideal_rx,high_true_rx,low_ideal_rx,low_true_rx,"
      << "ttl_violation_rate,never_received_rate,"
      << "high_ttl_violation_rate,high_never_received_rate,"
      << "low_ttl_violation_rate,low_never_received_rate,"
      << "receiver_fresh_redundancy_200ms_mean,receiver_fresh_redundancy_500ms_mean,"
      << "receiver_fresh_redundancy_ge2_200ms_rate,"
      << "receiver_fresh_redundancy_ge2_500ms_rate,"
      << "receiver_high_fresh_redundancy_200ms_mean,"
      << "receiver_low_fresh_redundancy_200ms_mean,"
      << "receiver_rv_200ms_score,"
      << "receiver_rl_mean,receiver_rl_median,"
      << "receiver_rl_lt1_rate,receiver_rl_ge1_rate,receiver_rl_ge2_rate,"
      << "receiver_rv_delooz_mean,receiver_rv_delooz_median,"
      << "receiver_high_rl_mean,receiver_low_rl_mean,"
      << "rmr_deleted_eval_expected_total,rmr_deleted_eval_expected_high,"
      << "rmr_deleted_eval_expected_low,rmr_deleted_eval_unrecognized_total,"
      << "rmr_deleted_eval_unrecognized_high,rmr_deleted_eval_unrecognized_low,"
      << "rmr_deleted_eval_distance_mean_m,"
      << "rmr_deleted_eval_fresh_redundancy_ge2_200ms_rate,"
      << "rmr_deleted_eval_fresh_redundancy_ge2_500ms_rate,"
      << "rmr_deleted_eval_closing_speed_ge_threshold_rate,"
      << "rmr_deleted_eval_ttc_le_threshold_rate,"
      << "rmr_deleted_sender_eval_total,rmr_deleted_sender_eval_high,"
      << "rmr_deleted_sender_eval_low,rmr_deleted_sender_eval_distance_mean_m,"
      << "rmr_deleted_sender_eval_closing_speed_ge_threshold_rate,"
      << "rmr_deleted_sender_eval_ttc_le_threshold_rate,"
      << "active_all_vehicles,rsu_i2v_vehicle_count,rsu_i2v_predicted_cbr_avg,"
      << "rsu_i2v_prediction_lead_time_avg" << std::endl;
}

static void
OpenCpmInputDiagLog (const std::string& path)
{
  if (path.empty ())
    {
      return;
    }

  g_cpmInputDiagLog.open (path, std::ios::out);
  if (!g_cpmInputDiagLog.is_open ())
    {
      NS_FATAL_ERROR ("Unable to open CPM input diagnostic log file: " << path);
    }

  g_cpmInputDiagLog
      << "time_s,active_vehicles,position_samples,speed_min_mps,speed_avg_mps,"
      << "speed_max_mps,heading_min_deg,heading_max_deg,nearest_distance_min_m,"
      << "neighbors_within_sensor_avg,neighbors_within_orr_avg,"
      << "nr_ldm_pos,nr_ldm_perceived_by_self,"
      << "nr_ldm_x_abs_min_cm,nr_ldm_x_abs_max_cm,"
      << "nr_ldm_y_abs_min_cm,nr_ldm_y_abs_max_cm,"
      << "nr_ldm_speed_abs_min_cms,nr_ldm_speed_abs_max_cms,"
      << "nr_ldm_acc_abs_min_cms2,nr_ldm_acc_abs_max_cms2,"
      << "nr_ldm_acc_unavailable,"
      << "nr_ldm_heading_min_deg,nr_ldm_heading_max_deg,"
      << "nr_ldm_length_min_dm,nr_ldm_length_max_dm,"
      << "nr_ldm_width_min_dm,nr_ldm_width_max_dm,"
      << "nr_ldm_measurement_delta_min_ms,nr_ldm_measurement_delta_max_ms,"
      << "nr_ldm_measurement_delta_negative,nr_ldm_measurement_delta_gt1500,"
      << "nr_ldm_distance_abs_gt131071,"
      << "nr_vdp_lat_min,nr_vdp_lat_max,nr_vdp_lon_min,nr_vdp_lon_max,"
      << "nr_vdp_heading_min,nr_vdp_heading_max,nr_vdp_speed_min,nr_vdp_speed_max,"
      << "legacy_cpm_objects,nr_cpm_objects,mec_cpm_objects,"
      << "legacy_cpm_wannabe_sent,legacy_cpm_sent,"
      << "nr_cpm_wannabe_sent,nr_cpm_sent,"
      << "mec_cpm_wannabe_sent,mec_cpm_sent,"
      << "legacy_cpm_start_calls,nr_cpm_start_calls,mec_cpm_start_calls"
      << std::endl;
}

static void
WriteCpmInputDiagLog (uint32_t activeVehicles,
                      uint32_t dsrcCpmObjects,
                      uint32_t nrCpmObjects,
                      uint32_t mecCpmObjects,
                      uint64_t dsrcCpmWannabeSent,
                      uint64_t dsrcCpmSent,
                      uint64_t nrCpmWannabeSent,
                      uint64_t nrCpmSent,
                      uint64_t mecCpmWannabeSent,
                      uint64_t mecCpmSent)
{
  if (!g_cpmInputDiagLog.is_open () || g_sumoClient == nullptr)
    {
      return;
    }

  struct Snapshot
  {
    double x;
    double y;
    double speed;
    double angle;
  };

  std::vector<Snapshot> snapshots;
  snapshots.reserve (g_vehicleRuntime.size ());
  double speedMin = std::numeric_limits<double>::max ();
  double speedMax = 0.0;
  double speedSum = 0.0;
  double angleMin = std::numeric_limits<double>::max ();
  double angleMax = -std::numeric_limits<double>::max ();

  for (const auto& entry : g_vehicleRuntime)
    {
      try
        {
          const auto pos = g_sumoClient->TraCIAPI::vehicle.getPosition (entry.first);
          const double speed = g_sumoClient->TraCIAPI::vehicle.getSpeed (entry.first);
          const double angle = g_sumoClient->TraCIAPI::vehicle.getAngle (entry.first);
          snapshots.push_back ({pos.x, pos.y, speed, angle});
          speedMin = std::min (speedMin, speed);
          speedMax = std::max (speedMax, speed);
          speedSum += speed;
          angleMin = std::min (angleMin, angle);
          angleMax = std::max (angleMax, angle);
        }
      catch (...)
        {
        }
    }

  double nearestMin = -1.0;
  uint64_t sensorNeighbors = 0;
  uint64_t orrNeighbors = 0;
  for (uint32_t i = 0; i < snapshots.size (); ++i)
    {
      double localNearest = std::numeric_limits<double>::max ();
      for (uint32_t j = 0; j < snapshots.size (); ++j)
        {
          if (i == j)
            {
              continue;
            }
          const double dx = snapshots[i].x - snapshots[j].x;
          const double dy = snapshots[i].y - snapshots[j].y;
          const double distance = std::sqrt (dx * dx + dy * dy);
          localNearest = std::min (localNearest, distance);
          if (distance <= g_sensorRangeMeters)
            {
              ++sensorNeighbors;
            }
          if (distance <= g_orrRangeMeters)
            {
              ++orrNeighbors;
            }
        }
      if (localNearest < std::numeric_limits<double>::max ())
        {
          nearestMin = nearestMin < 0.0 ? localNearest : std::min (nearestMin, localNearest);
        }
    }

  const double sampleCount = static_cast<double> (snapshots.size ());

  uint64_t nrLdmPos = 0;
  uint64_t nrLdmPerceivedBySelf = 0;
  uint64_t nrLdmMeasurementDeltaNegative = 0;
  uint64_t nrLdmMeasurementDeltaGt1500 = 0;
  uint64_t nrLdmDistanceAbsGt131071 = 0;
  uint64_t nrVdpSamples = 0;
  long nrXMin = std::numeric_limits<long>::max ();
  long nrXMax = std::numeric_limits<long>::min ();
  long nrYMin = std::numeric_limits<long>::max ();
  long nrYMax = std::numeric_limits<long>::min ();
  long nrSpeedMin = std::numeric_limits<long>::max ();
  long nrSpeedMax = std::numeric_limits<long>::min ();
  long nrAccMin = std::numeric_limits<long>::max ();
  long nrAccMax = std::numeric_limits<long>::min ();
  uint64_t nrAccUnavailable = 0;
  double nrHeadingMin = std::numeric_limits<double>::max ();
  double nrHeadingMax = -std::numeric_limits<double>::max ();
  long nrLengthMin = std::numeric_limits<long>::max ();
  long nrLengthMax = std::numeric_limits<long>::min ();
  long nrWidthMin = std::numeric_limits<long>::max ();
  long nrWidthMax = std::numeric_limits<long>::min ();
  int64_t nrDeltaMinMs = std::numeric_limits<int64_t>::max ();
  int64_t nrDeltaMaxMs = std::numeric_limits<int64_t>::min ();
  long nrLatMin = std::numeric_limits<long>::max ();
  long nrLatMax = std::numeric_limits<long>::min ();
  long nrLonMin = std::numeric_limits<long>::max ();
  long nrLonMax = std::numeric_limits<long>::min ();
  long nrHeadingValueMin = std::numeric_limits<long>::max ();
  long nrHeadingValueMax = std::numeric_limits<long>::min ();
  long nrSpeedValueMin = std::numeric_limits<long>::max ();
  long nrSpeedValueMax = std::numeric_limits<long>::min ();
  const int64_t nowUs = Simulator::Now ().GetMicroSeconds ();

  auto updateLongRange = [] (long value, long& minValue, long& maxValue) {
    minValue = std::min (minValue, value);
    maxValue = std::max (maxValue, value);
  };
  auto longOrMinusOne = [] (uint64_t count, long value) {
    return count == 0 ? -1 : value;
  };
  auto int64OrMinusOne = [] (uint64_t count, int64_t value) {
    return count == 0 ? static_cast<int64_t> (-1) : value;
  };
  auto doubleOrMinusOne = [] (uint64_t count, double value) {
    return count == 0 ? -1.0 : value;
  };

  for (auto& entry : g_vehicleRuntime)
    {
      if (entry.second.nrContainer == nullptr || entry.second.nrContainer->getLDM () == nullptr)
        {
          continue;
        }

      if (entry.second.nrContainer->getVDP () != nullptr)
        {
          auto cpmData = entry.second.nrContainer->getVDP ()->getCPMMandatoryData ();
          ++nrVdpSamples;
          updateLongRange (cpmData.latitude, nrLatMin, nrLatMax);
          updateLongRange (cpmData.longitude, nrLonMin, nrLonMax);
          updateLongRange (cpmData.heading.getValue (), nrHeadingValueMin, nrHeadingValueMax);
          updateLongRange (cpmData.speed.getValue (), nrSpeedValueMin, nrSpeedValueMax);
        }

      std::vector<LDM::returnedVehicleData_t> ldmPos;
      if (!entry.second.nrContainer->getLDM ()->getAllPOs (ldmPos))
        {
          continue;
        }

      const uint64_t selfStationId = VehicleIdToStationId (entry.first);
      nrLdmPos += ldmPos.size ();
      for (auto& po : ldmPos)
        {
          auto& data = po.vehData;
          if (!data.perceivedBy.isAvailable () ||
              static_cast<uint64_t> (data.perceivedBy.getData ()) != selfStationId)
            {
              continue;
            }
          ++nrLdmPerceivedBySelf;

          if (data.xDistAbs.isAvailable ())
            {
              const long value = data.xDistAbs.getData ();
              updateLongRange (value, nrXMin, nrXMax);
              if (std::labs (value) > 131071)
                {
                  ++nrLdmDistanceAbsGt131071;
                }
            }
          if (data.yDistAbs.isAvailable ())
            {
              const long value = data.yDistAbs.getData ();
              updateLongRange (value, nrYMin, nrYMax);
              if (std::labs (value) > 131071)
                {
                  ++nrLdmDistanceAbsGt131071;
                }
            }
          if (data.xSpeedAbs.isAvailable ())
            {
              updateLongRange (data.xSpeedAbs.getData (), nrSpeedMin, nrSpeedMax);
            }
          if (data.ySpeedAbs.isAvailable ())
            {
              updateLongRange (data.ySpeedAbs.getData (), nrSpeedMin, nrSpeedMax);
            }
          if (data.xAccAbs.isAvailable ())
            {
              updateLongRange (data.xAccAbs.getData (), nrAccMin, nrAccMax);
            }
          else
            {
              ++nrAccUnavailable;
            }
          if (data.yAccAbs.isAvailable ())
            {
              updateLongRange (data.yAccAbs.getData (), nrAccMin, nrAccMax);
            }
          else
            {
              ++nrAccUnavailable;
            }
          nrHeadingMin = std::min (nrHeadingMin, data.heading);
          nrHeadingMax = std::max (nrHeadingMax, data.heading);
          if (data.vehicleLength.isAvailable ())
            {
              updateLongRange (data.vehicleLength.getData (), nrLengthMin, nrLengthMax);
            }
          if (data.vehicleWidth.isAvailable ())
            {
              updateLongRange (data.vehicleWidth.getData (), nrWidthMin, nrWidthMax);
            }

          const int64_t deltaMs =
              (nowUs - static_cast<int64_t> (data.timestamp_us)) / 1000;
          nrDeltaMinMs = std::min (nrDeltaMinMs, deltaMs);
          nrDeltaMaxMs = std::max (nrDeltaMaxMs, deltaMs);
          if (deltaMs < 0)
            {
              ++nrLdmMeasurementDeltaNegative;
            }
          if (deltaMs > 1500)
            {
              ++nrLdmMeasurementDeltaGt1500;
            }
        }
    }

  g_cpmInputDiagLog
      << Simulator::Now ().GetSeconds () << "," << activeVehicles << ","
      << snapshots.size () << ","
      << (snapshots.empty () ? -1.0 : speedMin) << ","
      << (snapshots.empty () ? -1.0 : speedSum / sampleCount) << ","
      << (snapshots.empty () ? -1.0 : speedMax) << ","
      << (snapshots.empty () ? -1.0 : angleMin) << ","
      << (snapshots.empty () ? -1.0 : angleMax) << ","
      << nearestMin << ","
      << (snapshots.empty () ? -1.0 : static_cast<double> (sensorNeighbors) / sampleCount)
      << ","
      << (snapshots.empty () ? -1.0 : static_cast<double> (orrNeighbors) / sampleCount)
      << "," << nrLdmPos << "," << nrLdmPerceivedBySelf << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrXMin) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrXMax) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrYMin) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrYMax) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrSpeedMin) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrSpeedMax) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrAccMin) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrAccMax) << ","
      << nrAccUnavailable << ","
      << doubleOrMinusOne (nrLdmPerceivedBySelf, nrHeadingMin) << ","
      << doubleOrMinusOne (nrLdmPerceivedBySelf, nrHeadingMax) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrLengthMin) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrLengthMax) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrWidthMin) << ","
      << longOrMinusOne (nrLdmPerceivedBySelf, nrWidthMax) << ","
      << int64OrMinusOne (nrLdmPerceivedBySelf, nrDeltaMinMs) << ","
      << int64OrMinusOne (nrLdmPerceivedBySelf, nrDeltaMaxMs) << ","
      << nrLdmMeasurementDeltaNegative << "," << nrLdmMeasurementDeltaGt1500 << ","
      << nrLdmDistanceAbsGt131071 << ","
      << longOrMinusOne (nrVdpSamples, nrLatMin) << ","
      << longOrMinusOne (nrVdpSamples, nrLatMax) << ","
      << longOrMinusOne (nrVdpSamples, nrLonMin) << ","
      << longOrMinusOne (nrVdpSamples, nrLonMax) << ","
      << longOrMinusOne (nrVdpSamples, nrHeadingValueMin) << ","
      << longOrMinusOne (nrVdpSamples, nrHeadingValueMax) << ","
      << longOrMinusOne (nrVdpSamples, nrSpeedValueMin) << ","
      << longOrMinusOne (nrVdpSamples, nrSpeedValueMax) << ","
      << dsrcCpmObjects << "," << nrCpmObjects << "," << mecCpmObjects << ","
      << dsrcCpmWannabeSent << "," << dsrcCpmSent << ","
      << nrCpmWannabeSent << "," << nrCpmSent << ","
      << mecCpmWannabeSent << "," << mecCpmSent << ","
      << g_dsrcCpmStartCalls << "," << g_nrCpmStartCalls << ","
      << g_mecCpmStartCalls << std::endl;
}

static void
WriteObservationLog (Ptr<MetricSupervisor> channelMetrics,
                     Ptr<MetricSupervisor> dsrcMetrics,
                     Ptr<MetricSupervisor> nrMetrics,
                     Ptr<MetricSupervisor> mecMetrics,
                     Time interval)
{
  uint32_t activeVehicles = 0;
  uint32_t bgHighLoadActiveVehicles = 0;
  uint32_t bgHighLoadAllVehicles = 0;
  uint32_t dsrcRouteCount = 0;
  uint32_t nrRouteCount = 0;
  uint32_t mecRouteCount = 0;
  uint32_t dualRouteCount = 0;
  uint32_t cbrSamples = 0;
  double cbrSum = 0.0;
  double cbrMax = -1.0;
  uint32_t predictedCbrSamples = 0;
  double predictedCbrSum = 0.0;
  double predictedCbrMax = -1.0;
  double cbrRateSum = 0.0;
  double cpmTxRateSum = 0.0;
  uint64_t rmrDeletedFeatureCount = 0;
  double rmrDeletedDistanceSum = 0.0;
  double rmrDeletedFrequencySum = 0.0;
  double rmrDeletedPositionChangeSum = 0.0;
  double rmrDeletedSpeedChangeSum = 0.0;
  double rmrDeletedScoreSum = 0.0;
  uint32_t dsrcCpmObjects = 0;
  uint32_t dsrcRmrCandidates = 0;
  uint32_t dsrcRmrDeletedLast = 0;
  uint32_t dsrcRmrDeletedNearLast = 0;
  uint32_t dsrcRmrDeletedFarLast = 0;
  uint32_t dsrcCpmSizeBytes = 0;
  uint64_t dsrcCpmObjectsTotal = 0;
  uint64_t dsrcRmrCandidatesTotal = 0;
  uint64_t dsrcRmrDeletedTotal = 0;
  uint64_t dsrcRmrDeletedNearTotal = 0;
  uint64_t dsrcRmrDeletedFarTotal = 0;
  uint64_t dsrcCpmSizeBytesTotal = 0;
  uint64_t dsrcCpmWannabeSent = 0;
  uint64_t dsrcCpmSent = 0;
  uint32_t nrCpmObjects = 0;
  uint32_t nrRmrCandidates = 0;
  uint32_t nrRmrDeletedLast = 0;
  uint32_t nrRmrDeletedNearLast = 0;
  uint32_t nrRmrDeletedFarLast = 0;
  uint32_t nrCpmSizeBytes = 0;
  uint64_t nrCpmObjectsTotal = 0;
  uint64_t nrRmrCandidatesTotal = 0;
  uint64_t nrRmrDeletedTotal = 0;
  uint64_t nrRmrDeletedNearTotal = 0;
  uint64_t nrRmrDeletedFarTotal = 0;
  uint64_t nrRmrDeletedFeatureCount = 0;
  double nrRmrDeletedDistanceSum = 0.0;
  double nrRmrDeletedFrequencySum = 0.0;
  double nrRmrDeletedPositionChangeSum = 0.0;
  double nrRmrDeletedSpeedChangeSum = 0.0;
  double nrRmrDeletedScoreSum = 0.0;
  uint64_t nrCpmSizeBytesTotal = 0;
  uint64_t nrCpmWannabeSent = 0;
  uint64_t nrCpmSent = 0;
  uint32_t mecCpmObjects = 0;
  uint32_t mecRmrCandidates = 0;
  uint32_t mecRmrDeletedLast = 0;
  uint32_t mecRmrDeletedNearLast = 0;
  uint32_t mecRmrDeletedFarLast = 0;
  uint32_t mecCpmSizeBytes = 0;
  uint64_t mecCpmObjectsTotal = 0;
  uint64_t mecRmrCandidatesTotal = 0;
  uint64_t mecRmrDeletedTotal = 0;
  uint64_t mecRmrDeletedNearTotal = 0;
  uint64_t mecRmrDeletedFarTotal = 0;
  uint64_t mecCpmSizeBytesTotal = 0;
  uint64_t mecCpmWannabeSent = 0;
  uint64_t mecCpmSent = 0;

  for (const auto& entry : g_vehicleRuntime)
    {
      ++activeVehicles;
      if (IsInMovingBackgroundHighLoadZone (entry.first))
        {
          ++bgHighLoadActiveVehicles;
        }
      uint32_t activeRouteCount = 0;
      if (entry.second.dsrcTxActive)
        {
          ++dsrcRouteCount;
          ++activeRouteCount;
        }
      if (entry.second.nrTxActive)
        {
          ++nrRouteCount;
          ++activeRouteCount;
        }
      if (entry.second.mecTxActive)
        {
          ++mecRouteCount;
          ++activeRouteCount;
        }
      if (activeRouteCount > 1)
        {
          ++dualRouteCount;
        }

      const double cbr = channelMetrics->getCBRPerItem (entry.first);
      if (cbr >= 0.0)
        {
          cbrSum += cbr;
          cbrMax = std::max (cbrMax, cbr);
          ++cbrSamples;
        }

      const auto predictiveIt = g_predictiveCbrState.find (entry.first);
      if (predictiveIt != g_predictiveCbrState.end () && predictiveIt->second.initialized)
        {
          predictedCbrSum += predictiveIt->second.predictedCbr;
          predictedCbrMax = std::max (predictedCbrMax, predictiveIt->second.predictedCbr);
          cbrRateSum += predictiveIt->second.cbrRatePerSecond;
          cpmTxRateSum += predictiveIt->second.dsrcCpmTxRate;
          ++predictedCbrSamples;
        }

      Ptr<CPBasicService> dsrcCp = entry.second.dsrcContainer->getCPBasicService ();
      dsrcCpmObjects += dsrcCp->getLastRmrIncludedObjectCount ();
      dsrcRmrCandidates += dsrcCp->getLastRmrCandidateCount ();
      dsrcRmrDeletedLast += dsrcCp->getLastRmrDeletedObjectCount ();
      dsrcCpmObjectsTotal += dsrcCp->getTotalRmrIncludedObjectCount ();
      dsrcRmrCandidatesTotal += dsrcCp->getTotalRmrCandidateCount ();
      dsrcRmrDeletedTotal += dsrcCp->getTotalRmrDeletedObjectCount ();
      rmrDeletedFeatureCount += dsrcCp->getTotalRmrDeletedFeatureCount ();
      rmrDeletedDistanceSum += dsrcCp->getTotalRmrDeletedDistanceSum ();
      rmrDeletedFrequencySum += dsrcCp->getTotalRmrDeletedFrequencySum ();
      rmrDeletedPositionChangeSum += dsrcCp->getTotalRmrDeletedPositionChangeSum ();
      rmrDeletedSpeedChangeSum += dsrcCp->getTotalRmrDeletedSpeedChangeSum ();
      rmrDeletedScoreSum += dsrcCp->getTotalRmrDeletedScoreSum ();
      dsrcCpmSizeBytes += dsrcCp->getLastCpmSizeBytes ();
      dsrcCpmSizeBytesTotal += dsrcCp->getTotalCpmSizeBytes ();
      dsrcCpmWannabeSent += dsrcCp->getWannabeSent ();
      dsrcCpmSent += dsrcCp->getCpmSent ();

      if (entry.second.nrContainer != nullptr)
        {
          Ptr<CPBasicService> nrCp = entry.second.nrContainer->getCPBasicService ();
          nrCpmObjects += nrCp->getLastRmrIncludedObjectCount ();
          nrRmrCandidates += nrCp->getLastRmrCandidateCount ();
          nrRmrDeletedLast += nrCp->getLastRmrDeletedObjectCount ();
          nrCpmObjectsTotal += nrCp->getTotalRmrIncludedObjectCount ();
          nrRmrCandidatesTotal += nrCp->getTotalRmrCandidateCount ();
          nrRmrDeletedTotal += nrCp->getTotalRmrDeletedObjectCount ();
          nrRmrDeletedFeatureCount += nrCp->getTotalRmrDeletedFeatureCount ();
          rmrDeletedFeatureCount += nrCp->getTotalRmrDeletedFeatureCount ();
          nrRmrDeletedDistanceSum += nrCp->getTotalRmrDeletedDistanceSum ();
          rmrDeletedDistanceSum += nrCp->getTotalRmrDeletedDistanceSum ();
          nrRmrDeletedFrequencySum += nrCp->getTotalRmrDeletedFrequencySum ();
          rmrDeletedFrequencySum += nrCp->getTotalRmrDeletedFrequencySum ();
          nrRmrDeletedPositionChangeSum += nrCp->getTotalRmrDeletedPositionChangeSum ();
          rmrDeletedPositionChangeSum += nrCp->getTotalRmrDeletedPositionChangeSum ();
          nrRmrDeletedSpeedChangeSum += nrCp->getTotalRmrDeletedSpeedChangeSum ();
          rmrDeletedSpeedChangeSum += nrCp->getTotalRmrDeletedSpeedChangeSum ();
          nrRmrDeletedScoreSum += nrCp->getTotalRmrDeletedScoreSum ();
          rmrDeletedScoreSum += nrCp->getTotalRmrDeletedScoreSum ();
          nrCpmSizeBytes += nrCp->getLastCpmSizeBytes ();
          nrCpmSizeBytesTotal += nrCp->getTotalCpmSizeBytes ();
          nrCpmWannabeSent += nrCp->getWannabeSent ();
          nrCpmSent += nrCp->getCpmSent ();
        }

      if (entry.second.mecContainer != nullptr)
        {
          Ptr<CPBasicService> mecCp = entry.second.mecContainer->getCPBasicService ();
          mecCpmObjects += mecCp->getLastRmrIncludedObjectCount ();
          mecRmrCandidates += mecCp->getLastRmrCandidateCount ();
          mecRmrDeletedLast += mecCp->getLastRmrDeletedObjectCount ();
          mecCpmObjectsTotal += mecCp->getTotalRmrIncludedObjectCount ();
          mecRmrCandidatesTotal += mecCp->getTotalRmrCandidateCount ();
          mecRmrDeletedTotal += mecCp->getTotalRmrDeletedObjectCount ();
          rmrDeletedFeatureCount += mecCp->getTotalRmrDeletedFeatureCount ();
          rmrDeletedDistanceSum += mecCp->getTotalRmrDeletedDistanceSum ();
          rmrDeletedFrequencySum += mecCp->getTotalRmrDeletedFrequencySum ();
          rmrDeletedPositionChangeSum += mecCp->getTotalRmrDeletedPositionChangeSum ();
          rmrDeletedSpeedChangeSum += mecCp->getTotalRmrDeletedSpeedChangeSum ();
          rmrDeletedScoreSum += mecCp->getTotalRmrDeletedScoreSum ();
          mecCpmSizeBytes += mecCp->getLastCpmSizeBytes ();
          mecCpmSizeBytesTotal += mecCp->getTotalCpmSizeBytes ();
          mecCpmWannabeSent += mecCp->getWannabeSent ();
          mecCpmSent += mecCp->getCpmSent ();
        }
    }
  for (const auto& entry : g_allVehicleNodes)
    {
      if (IsInMovingBackgroundHighLoadZone (entry.first))
        {
          ++bgHighLoadAllVehicles;
        }
    }

  const double cbrAvg = cbrSamples > 0 ? cbrSum / static_cast<double> (cbrSamples) : -1.0;
  const double predictedCbrAvg =
      predictedCbrSamples > 0
          ? predictedCbrSum / static_cast<double> (predictedCbrSamples)
          : -1.0;
  const double cbrRateAvg =
      predictedCbrSamples > 0 ? cbrRateSum / static_cast<double> (predictedCbrSamples) : 0.0;
  const double cpmTxRateAvg =
      predictedCbrSamples > 0 ? cpmTxRateSum / static_cast<double> (predictedCbrSamples) : 0.0;
  const uint64_t dsrcTx = dsrcMetrics->getNumberTx_overall ();
  const uint64_t nrTx = nrMetrics->getNumberTx_overall ();
  const uint64_t mecTx = mecMetrics->getNumberTx_overall ();
  const double dsrcPrr = dsrcMetrics->getAveragePRR_overall ();
  const double nrPrr = nrMetrics->getAveragePRR_overall ();
  const double mecPrr = mecMetrics->getAveragePRR_overall ();
  const double orr = ThesisRecognitionRatePercent ();
  const double highPdr =
      ThesisPacketDeliveryRatioPercent (g_thesisStats.highIdealCpmRx,
                                        g_thesisStats.highTrueCpmRx);
  const double lowPdr =
      ThesisPacketDeliveryRatioPercent (g_thesisStats.lowIdealCpmRx,
                                        g_thesisStats.lowTrueCpmRx);
  const double highLoss =
      ThesisPacketLossRatePercent (g_thesisStats.highIdealCpmRx, g_thesisStats.highTrueCpmRx);
  const double lowLoss =
      ThesisPacketLossRatePercent (g_thesisStats.lowIdealCpmRx, g_thesisStats.lowTrueCpmRx);
  const double v2vRadioLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrIdealCpmRx, g_thesisStats.nrTrueCpmRx);
  const double v2vHighRadioLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrHighIdealCpmRx,
                                   g_thesisStats.nrHighTrueCpmRx);
  const double v2vLowRadioLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrLowIdealCpmRx, g_thesisStats.nrLowTrueCpmRx);
  const double v2vUpdateFailureRate =
      ThesisPacketLossRatePercent (g_thesisStats.nrUpdateExpected,
                                   g_thesisStats.nrUpdateSuccess);
  const double mecUpdateFailureRate =
      ThesisPacketLossRatePercent (g_thesisStats.mecUpdateExpected,
                                   g_thesisStats.mecUpdateSuccess);
  const double mecValidUpdateSuccessRate =
      PercentRate (g_thesisStats.mecUpdateSuccess, g_thesisStats.mecUpdateExpected);
  const double mecUlFinalLossRate =
      PercentRate (g_idealMecUlFinalLosses, g_mecUplinkPackets);
  const double mecDlFinalLossRate =
      PercentRate (g_idealMecDlFinalLosses, g_mecForwardedPackets);
  const double mecRadioFinalLossRate =
      PercentRate (g_idealMecUlFinalLosses + g_idealMecDlFinalLosses,
                   g_mecUplinkPackets + g_mecForwardedPackets);
  const double ttlViolationRate =
      PercentFromRatioSum (g_thesisStats.ttlViolationRatioSum,
                           g_thesisStats.ttlViolationVehicleSamples);
  const double neverReceivedRate =
      PercentFromRatioSum (g_thesisStats.neverReceivedRatioSum,
                           g_thesisStats.neverReceivedVehicleSamples);
  const double highTtlViolationRate =
      PercentFromRatioSum (g_thesisStats.highPriorityTtlViolationRatioSum,
                           g_thesisStats.highPriorityTtlViolationVehicleSamples);
  const double highNeverReceivedRate =
      PercentFromRatioSum (g_thesisStats.highPriorityNeverReceivedRatioSum,
                           g_thesisStats.highPriorityNeverReceivedVehicleSamples);
  const double lowTtlViolationRate =
      PercentFromRatioSum (g_thesisStats.lowPriorityTtlViolationRatioSum,
                           g_thesisStats.lowPriorityTtlViolationVehicleSamples);
  const double lowNeverReceivedRate =
      PercentFromRatioSum (g_thesisStats.lowPriorityNeverReceivedRatioSum,
                           g_thesisStats.lowPriorityNeverReceivedVehicleSamples);
  const double v2vAoiMeanMs = Mean (g_v2vAoiSamplesMs);
  const double v2vAoiP50Ms = Percentile (g_v2vAoiSamplesMs, 50.0);
  const double v2vAoiP90Ms = Percentile (g_v2vAoiSamplesMs, 90.0);
  const double v2vAoiP99Ms = Percentile (g_v2vAoiSamplesMs, 99.0);
  const double mecAoiMeanMs = Mean (g_idealMecAoiSamplesMs);
  const double mecAoiP50Ms = Percentile (g_idealMecAoiSamplesMs, 50.0);
  const double mecAoiP90Ms = Percentile (g_idealMecAoiSamplesMs, 90.0);
  const double mecAoiP99Ms = Percentile (g_idealMecAoiSamplesMs, 99.0);
  const double mecCapacityDelayMeanMs = Mean (g_idealMecCapacityDelaySamplesMs);
  const double mecCapacityDelayP90Ms = Percentile (g_idealMecCapacityDelaySamplesMs, 90.0);
  const double mecCapacityDelayP99Ms = Percentile (g_idealMecCapacityDelaySamplesMs, 99.0);
  const double mecRetxDelayMeanMs = Mean (g_idealMecRetxDelaySamplesMs);
  const double mecRetxDelayP90Ms = Percentile (g_idealMecRetxDelaySamplesMs, 90.0);
  const double observationWindowSeconds = std::max (interval.GetSeconds (), 1.0e-9);
  const uint64_t mecUlBytesForBusyRatio = g_mecUplinkBytes + g_mecBackgroundUplinkBytes;
  const uint64_t mecDlBytesForBusyRatio = g_mecForwardedBytes + g_mecBackgroundDownlinkBytes;
  const uint64_t mecUlBytesDelta =
      mecUlBytesForBusyRatio >= g_lastMecUplinkBytesForBusyRatio
          ? mecUlBytesForBusyRatio - g_lastMecUplinkBytesForBusyRatio
          : 0;
  const uint64_t mecDlBytesDelta =
      mecDlBytesForBusyRatio >= g_lastMecForwardedBytesForBusyRatio
          ? mecDlBytesForBusyRatio - g_lastMecForwardedBytesForBusyRatio
          : 0;
  g_lastMecUplinkBytesForBusyRatio = mecUlBytesForBusyRatio;
  g_lastMecForwardedBytesForBusyRatio = mecDlBytesForBusyRatio;
  const double mecUlOfferedMbps =
      (static_cast<double> (mecUlBytesDelta) * 8.0) / observationWindowSeconds / 1.0e6;
  const double mecDlOfferedMbps =
      (static_cast<double> (mecDlBytesDelta) * 8.0) / observationWindowSeconds / 1.0e6;
  const double mecUlBusyRatio =
      g_idealMecUplinkCapacityMbps > 0.0
          ? Clamp01 (mecUlOfferedMbps / g_idealMecUplinkCapacityMbps)
          : 0.0;
  const double mecDlBusyRatio =
      g_idealMecDownlinkCapacityMbps > 0.0
          ? Clamp01 (mecDlOfferedMbps / g_idealMecDownlinkCapacityMbps)
          : 0.0;
  g_mecUlBusyRatioSamples.push_back (mecUlBusyRatio);
  g_mecDlBusyRatioSamples.push_back (mecDlBusyRatio);

  if (g_observationLog.is_open ())
    {
      g_observationLog << Simulator::Now ().GetSeconds () << "," << activeVehicles << ","
                       << dsrcRouteCount << "," << nrRouteCount << "," << mecRouteCount << ","
                       << dualRouteCount << "," << cbrAvg << "," << cbrMax << ","
                       << predictedCbrAvg << "," << predictedCbrMax << ","
                       << cbrRateAvg << "," << cpmTxRateAvg << ","
                       << g_trafficFlowPrediction.qInVehiclesPerSecond << ","
                       << g_trafficFlowPrediction.qOutVehiclesPerSecond << ","
                       << g_trafficFlowPrediction.trafficFlowCbr << ","
                       << g_trafficFlowPrediction.deltaCbr << ","
                       << g_trafficFlowPrediction.meanSpeedMetersPerSecond << ","
                       << g_trafficFlowPrediction.rsuCount << ","
                       << g_trafficFlowPrediction.localVehicleCountAvg << ","
                       << g_trafficFlowPrediction.upstreamVehicleCountAvg << ","
                       << g_trafficFlowPrediction.futureVehicleCountAvg << ","
                       << g_trafficFlowPrediction.rsuPredictedCbrAvg << ","
                       << g_trafficFlowPrediction.rsuPredictedCbrMax << "," << dsrcPrr << ","
                       << PacketLossFromPrr (dsrcPrr, dsrcTx) << ","
                       << dsrcMetrics->getAverageLatency_overall () << "," << dsrcTx << ","
                       << dsrcMetrics->getNumberRx_overall () << "," << nrPrr << ","
                       << PacketLossFromPrr (nrPrr, nrTx) << ","
                       << nrMetrics->getAverageLatency_overall () << "," << nrTx << ","
                       << nrMetrics->getNumberRx_overall () << "," << mecPrr << ","
                       << PacketLossFromPrr (mecPrr, mecTx) << ","
                       << mecMetrics->getAverageLatency_overall () << "," << mecTx << ","
                       << mecMetrics->getNumberRx_overall () << "," << dsrcCpmObjects << ","
                       << dsrcRmrCandidates << "," << dsrcRmrDeletedLast << ","
                       << dsrcRmrDeletedNearLast << "," << dsrcRmrDeletedFarLast << ","
                       << dsrcCpmObjectsTotal << "," << dsrcRmrCandidatesTotal << ","
                       << dsrcRmrDeletedTotal << "," << dsrcRmrDeletedNearTotal << ","
                       << dsrcRmrDeletedFarTotal << "," << dsrcCpmSizeBytes << ","
                       << dsrcCpmSizeBytesTotal << "," << nrCpmObjects << ","
                       << nrRmrCandidates << "," << nrRmrDeletedLast << ","
                       << nrRmrDeletedNearLast << "," << nrRmrDeletedFarLast << ","
                       << nrCpmObjectsTotal << "," << nrRmrCandidatesTotal << ","
                       << nrRmrDeletedTotal << "," << nrRmrDeletedNearTotal << ","
                       << nrRmrDeletedFarTotal << "," << nrRmrDeletedFeatureCount << ","
                       << FormatThesisMetric (AverageFromSum (nrRmrDeletedDistanceSum,
                                                               nrRmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (nrRmrDeletedFrequencySum,
                                                               nrRmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (nrRmrDeletedPositionChangeSum,
                                                               nrRmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (nrRmrDeletedSpeedChangeSum,
                                                               nrRmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (nrRmrDeletedScoreSum,
                                                               nrRmrDeletedFeatureCount))
                       << "," << nrCpmSizeBytes << ","
                       << nrCpmSizeBytesTotal << "," << mecCpmObjects << ","
                       << mecRmrCandidates << "," << mecRmrDeletedLast << ","
                       << mecRmrDeletedNearLast << "," << mecRmrDeletedFarLast << ","
                       << mecCpmObjectsTotal << "," << mecRmrCandidatesTotal << ","
                       << mecRmrDeletedTotal << "," << mecRmrDeletedNearTotal << ","
                       << mecRmrDeletedFarTotal << "," << mecCpmSizeBytes << ","
                       << mecCpmSizeBytesTotal << "," << rmrDeletedFeatureCount << ","
                       << FormatThesisMetric (AverageFromSum (rmrDeletedDistanceSum,
                                                               rmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (rmrDeletedFrequencySum,
                                                               rmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (rmrDeletedPositionChangeSum,
                                                               rmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (rmrDeletedSpeedChangeSum,
                                                               rmrDeletedFeatureCount))
                       << ","
                       << FormatThesisMetric (AverageFromSum (rmrDeletedScoreSum,
                                                               rmrDeletedFeatureCount))
                       << ","
                       << dsrcCpmWannabeSent << "," << dsrcCpmSent << ","
                       << nrCpmWannabeSent << "," << nrCpmSent << ","
                       << mecCpmWannabeSent << "," << mecCpmSent << ","
                       << g_dsrcCpmStartCalls << "," << g_nrCpmStartCalls << ","
                       << g_mecCpmStartCalls << ","
                       << g_camRx << "," << g_cpmRx << ","
                       << g_interferenceTx << "," << g_interferenceBytes << ","
                       << g_interferenceDrops << "," << bgHighLoadActiveVehicles << ","
                       << bgHighLoadAllVehicles << "," << g_dsrcInterferenceOfferedCbr << ","
                       << g_mecUplinkPackets << ","
                       << g_mecUplinkBytes << "," << g_mecForwardedPackets << ","
                       << g_mecForwardedBytes << ","
                       << g_mecBackgroundUplinkBytes << ","
                       << g_mecBackgroundDownlinkBytes << ","
                       << FormatThesisMetric (g_idealMecUplinkCapacityMbps) << ","
                       << FormatThesisMetric (g_idealMecDownlinkCapacityMbps) << ","
                       << mecUlBytesDelta << "," << mecDlBytesDelta << ","
                       << FormatThesisMetric (mecUlOfferedMbps) << ","
                       << FormatThesisMetric (mecDlOfferedMbps) << ","
                       << FormatThesisMetric (mecUlBusyRatio) << ","
                       << FormatThesisMetric (mecDlBusyRatio) << ","
                       << g_mecForwardDrops << ","
                       << g_mecForwardNoReceiver << ","
                       << FormatThesisMetric (v2vAoiMeanMs) << ","
                       << FormatThesisMetric (v2vAoiP50Ms) << ","
                       << FormatThesisMetric (v2vAoiP90Ms) << ","
                       << FormatThesisMetric (v2vAoiP99Ms) << ","
                       << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[0],
                                                           g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[1],
                                                           g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[2],
                                                           g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[3],
                                                           g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[0],
                                                               g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[1],
                                                               g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[2],
                                                               g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[3],
                                                               g_v2vAoiSamples)) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vAoiRecognitionRatioSum[0],
                              g_thesisStats.v2vAoiRecognitionVehicleSamples[0])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vAoiRecognitionRatioSum[1],
                              g_thesisStats.v2vAoiRecognitionVehicleSamples[1])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vAoiRecognitionRatioSum[2],
                              g_thesisStats.v2vAoiRecognitionVehicleSamples[2])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vAoiRecognitionRatioSum[3],
                              g_thesisStats.v2vAoiRecognitionVehicleSamples[3])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vOnlyAoiRecognitionRatioSum[0],
                              g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[0])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vOnlyAoiRecognitionRatioSum[1],
                              g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[1])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vOnlyAoiRecognitionRatioSum[2],
                              g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[2])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.v2vOnlyAoiRecognitionRatioSum[3],
                              g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[3])) << ","
                       << FormatThesisMetric (mecAoiMeanMs) << ","
                       << FormatThesisMetric (mecAoiP50Ms) << ","
                       << FormatThesisMetric (mecAoiP90Ms) << ","
                       << FormatThesisMetric (mecAoiP99Ms) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[0],
                                                           g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[1],
                                                           g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[2],
                                                           g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[3],
                                                           g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[0],
                                                               g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[1],
                                                               g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[2],
                                                               g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[3],
                                                               g_idealMecAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[0],
                                                           g_idealMecHighAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[1],
                                                           g_idealMecHighAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[2],
                                                           g_idealMecHighAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[3],
                                                           g_idealMecHighAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[0],
                                                           g_idealMecLowAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[1],
                                                           g_idealMecLowAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[2],
                                                           g_idealMecLowAoiSamples)) << ","
                       << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[3],
                                                           g_idealMecLowAoiSamples)) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecAoiRecognitionRatioSum[0],
                              g_thesisStats.mecAoiRecognitionVehicleSamples[0])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecAoiRecognitionRatioSum[1],
                              g_thesisStats.mecAoiRecognitionVehicleSamples[1])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecAoiRecognitionRatioSum[2],
                              g_thesisStats.mecAoiRecognitionVehicleSamples[2])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecAoiRecognitionRatioSum[3],
                              g_thesisStats.mecAoiRecognitionVehicleSamples[3])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecOnlyAoiRecognitionRatioSum[0],
                              g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[0])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecOnlyAoiRecognitionRatioSum[1],
                              g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[1])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecOnlyAoiRecognitionRatioSum[2],
                              g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[2])) << ","
                       << FormatThesisMetric (PercentFromRatioSum (
                              g_thesisStats.mecOnlyAoiRecognitionRatioSum[3],
                              g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[3])) << ","
                       << FormatThesisMetric (mecCapacityDelayMeanMs) << ","
                       << FormatThesisMetric (mecCapacityDelayP90Ms) << ","
                       << FormatThesisMetric (mecCapacityDelayP99Ms) << ","
                       << g_idealMecCapacityQueueDrops << ","
                       << g_idealMecUlFirstLosses << "," << g_idealMecDlFirstLosses << ","
                       << g_idealMecUlRetxRecovered << "," << g_idealMecDlRetxRecovered
                       << "," << g_idealMecUlFinalLosses << ","
                       << g_idealMecDlFinalLosses << ","
                       << FormatThesisMetric (mecUpdateFailureRate) << ","
                       << FormatThesisMetric (mecUpdateFailureRate) << ","
                       << FormatThesisMetric (mecValidUpdateSuccessRate) << ","
                       << FormatThesisMetric (mecUlFinalLossRate) << ","
                       << FormatThesisMetric (mecDlFinalLossRate) << ","
                       << FormatThesisMetric (mecRadioFinalLossRate) << ","
                       << FormatThesisMetric (mecRetxDelayMeanMs) << ","
                       << FormatThesisMetric (mecRetxDelayP90Ms) << ","
                       << FormatThesisMetric (orr) << ","
                       << FormatThesisMetric (highPdr) << ","
                       << FormatThesisMetric (lowPdr) << ","
                       << FormatThesisMetric (highLoss) << ","
                       << FormatThesisMetric (lowLoss) << ","
                       << FormatThesisMetric (v2vRadioLoss) << ","
                       << FormatThesisMetric (v2vHighRadioLoss) << ","
                       << FormatThesisMetric (v2vLowRadioLoss) << ","
                       << FormatThesisMetric (v2vUpdateFailureRate) << ","
                       << g_thesisStats.highIdealCpmRx << "," << g_thesisStats.highTrueCpmRx
                       << "," << g_thesisStats.lowIdealCpmRx << ","
                       << g_thesisStats.lowTrueCpmRx << ","
                       << FormatThesisMetric (ttlViolationRate) << ","
                       << FormatThesisMetric (neverReceivedRate) << ","
                       << FormatThesisMetric (highTtlViolationRate) << ","
                       << FormatThesisMetric (highNeverReceivedRate) << ","
                       << FormatThesisMetric (lowTtlViolationRate) << ","
                       << FormatThesisMetric (lowNeverReceivedRate) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverFreshRedundancy200Sum,
                              g_thesisStats.receiverFreshRedundancySamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverFreshRedundancy500Sum,
                              g_thesisStats.receiverFreshRedundancySamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.receiverFreshRedundancyGe2_200,
                              g_thesisStats.receiverFreshRedundancySamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.receiverFreshRedundancyGe2_500,
                              g_thesisStats.receiverFreshRedundancySamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverFreshRedundancyHigh200Sum,
                              g_thesisStats.receiverFreshRedundancyHighSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverFreshRedundancyLow200Sum,
                              g_thesisStats.receiverFreshRedundancyLowSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverRv200Sum,
                              g_thesisStats.receiverFreshRedundancySamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverRlSum,
                              g_thesisStats.receiverRlSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverRlMedianSum,
                              g_thesisStats.receiverRlVehicleSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.receiverRlLt1,
                              g_thesisStats.receiverRlSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.receiverRlGe1,
                              g_thesisStats.receiverRlSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.receiverRlGe2,
                              g_thesisStats.receiverRlSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverRvDeloozSum,
                              g_thesisStats.receiverRlSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverRvDeloozMedianSum,
                              g_thesisStats.receiverRlVehicleSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverHighRlSum,
                              g_thesisStats.receiverHighRlSamples)) << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.receiverLowRlSum,
                              g_thesisStats.receiverLowRlSamples)) << ","
                       << g_thesisStats.rmrDeletedEvalExpectedTotal << ","
                       << g_thesisStats.rmrDeletedEvalExpectedHigh << ","
                       << g_thesisStats.rmrDeletedEvalExpectedLow << ","
                       << g_thesisStats.rmrDeletedEvalUnrecognizedTotal << ","
                       << g_thesisStats.rmrDeletedEvalUnrecognizedHigh << ","
                       << g_thesisStats.rmrDeletedEvalUnrecognizedLow << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.rmrDeletedEvalDistanceSum,
                              g_thesisStats.rmrDeletedEvalDistanceSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.rmrDeletedEvalFreshRedundancyGe2_200,
                              g_thesisStats.rmrDeletedEvalExpectedTotal)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.rmrDeletedEvalFreshRedundancyGe2_500,
                              g_thesisStats.rmrDeletedEvalExpectedTotal)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.rmrDeletedEvalClosingSpeedGeThreshold,
                              g_thesisStats.rmrDeletedEvalDistanceSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.rmrDeletedEvalTtcLeThreshold,
                              g_thesisStats.rmrDeletedEvalDistanceSamples)) << ","
                       << g_thesisStats.rmrDeletedSenderEvalTotal << ","
                       << g_thesisStats.rmrDeletedSenderEvalHigh << ","
                       << g_thesisStats.rmrDeletedSenderEvalLow << ","
                       << FormatThesisMetric (AverageFromSum (
                              g_thesisStats.rmrDeletedSenderEvalDistanceSum,
                              g_thesisStats.rmrDeletedSenderEvalDistanceSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.rmrDeletedSenderEvalClosingSpeedGeThreshold,
                              g_thesisStats.rmrDeletedSenderEvalDistanceSamples)) << ","
                       << FormatThesisMetric (PercentRate (
                              g_thesisStats.rmrDeletedSenderEvalTtcLeThreshold,
                              g_thesisStats.rmrDeletedSenderEvalDistanceSamples)) << ","
                       << g_allVehicleNodes.size () << ","
                       << g_trafficFlowPrediction.i2vVehicleCount << ","
                       << g_trafficFlowPrediction.i2vPredictedCbrAvg << ","
                       << g_trafficFlowPrediction.i2vPredictionLeadTimeAvg << std::endl;
    }

  WriteCpmInputDiagLog (activeVehicles,
                        dsrcCpmObjects,
                        nrCpmObjects,
                        mecCpmObjects,
                        dsrcCpmWannabeSent,
                        dsrcCpmSent,
                        nrCpmWannabeSent,
                        nrCpmSent,
                        mecCpmWannabeSent,
                        mecCpmSent);

  Simulator::Schedule (interval,
                       &WriteObservationLog,
                       channelMetrics,
                       dsrcMetrics,
                       nrMetrics,
                       mecMetrics,
                       interval);
}

static void
UpdateReactiveRmrCbr (Ptr<MetricSupervisor> channelMetrics, Time interval)
{
  for (auto& entry : g_vehicleRuntime)
    {
      const double cbr = channelMetrics->getCBRPerItem (entry.first);
      const double effectiveCbr = cbr >= 0.0 ? cbr : 0.0;
      const double actionProbability = ComputePredictiveRmrActionProbability (effectiveCbr);
      SetRmrActionProbability (entry.second, actionProbability);
      entry.second.dsrcContainer->getCPBasicService ()->setRmrCurrentCbr (effectiveCbr);
      if (entry.second.nrContainer != nullptr)
        {
          entry.second.nrContainer->getCPBasicService ()->setRmrCurrentCbr (effectiveCbr);
        }
      if (entry.second.mecContainer != nullptr)
        {
          entry.second.mecContainer->getCPBasicService ()->setRmrCurrentCbr (effectiveCbr);
        }
    }

  Simulator::Schedule (interval, &UpdateReactiveRmrCbr, channelMetrics, interval);
}

static bool
GetVehicleRsuPrediction (const std::string& vehicleId,
                         double* predictedCbr,
                         double* peakTimeSeconds)
{
  if (g_sumoClient == nullptr || g_trafficFlowRsus.empty ())
    {
      return false;
    }
  try
    {
      const libsumo::TraCIPosition position =
          g_sumoClient->TraCIAPI::vehicle.getPosition (vehicleId);
      const std::string roadId = g_sumoClient->TraCIAPI::vehicle.getRoadID (vehicleId);
      const int direction =
          roadId.rfind ("highway_wb", 0) == 0 || roadId.rfind ("loop_r", 0) == 0 ? -1 : 1;
      const TrafficFlowRsuInfo* selected = nullptr;
      std::size_t selectedIndex = 0;
      double selectedDistance = std::numeric_limits<double>::infinity ();
      for (std::size_t index = 0; index < g_trafficFlowRsus.size (); ++index)
        {
          const auto& rsu = g_trafficFlowRsus[index];
          const double signedDistance = (rsu.x - position.x) * direction;
          const double distance = std::abs (rsu.x - position.x);
          const bool eligible =
              g_fixedRsuCbrMode == "current"
                  ? distance <= selectedDistance
                  : signedDistance >= 0.0 &&
                        distance <= g_predictiveRmrConfig.rsuI2vRangeMeters &&
                        distance < selectedDistance;
          if (!eligible ||
              distance >= selectedDistance)
            {
              continue;
            }
          selected = &rsu;
          selectedIndex = index;
          selectedDistance = distance;
        }
      if (selected == nullptr)
        {
          return false;
        }
      if (g_fixedRsuCbrMode != "disabled")
        {
          *predictedCbr =
              g_fixedRsuCbrValues[std::min (selectedIndex, g_fixedRsuCbrValues.size () - 1)];
          *peakTimeSeconds =
              g_fixedRsuCbrMode == "next"
                  ? selectedDistance /
                        std::max (g_sumoClient->TraCIAPI::vehicle.getSpeed (vehicleId), 1.0)
                  : 0.0;
          ++g_trafficFlowRsuI2vCounts[selected->id];
          return true;
        }
      const auto predictionIt = g_trafficFlowRsuPredictions.find (selected->id);
      if (predictionIt == g_trafficFlowRsuPredictions.end ())
        {
          return false;
        }
      *predictedCbr = predictionIt->second.predictedCbr;
      *peakTimeSeconds = predictionIt->second.peakTimeSeconds;
      ++g_trafficFlowRsuI2vCounts[selected->id];
      return true;
    }
  catch (...)
    {
      return false;
    }
}

static void
UpdatePredictiveRmrCbr (Ptr<MetricSupervisor> channelMetrics, Time interval)
{
  const Time now = Simulator::Now ();
  UpdateTrafficFlowCbrPrediction (channelMetrics, interval);
  const double activeVehicleFeature =
      g_predictiveRmrConfig.activeVehicleNorm > 0.0
          ? static_cast<double> (g_vehicleRuntime.size ()) /
                g_predictiveRmrConfig.activeVehicleNorm
          : 0.0;
  uint32_t i2vPredictionCount = 0;
  double i2vPredictionSum = 0.0;
  double i2vLeadTimeSum = 0.0;
  g_trafficFlowRsuI2vCounts.clear ();

  for (auto& entry : g_vehicleRuntime)
    {
      const std::string& vehicleId = entry.first;
      const double measuredCbr = channelMetrics->getCBRPerItem (vehicleId);
      const double currentCbr = measuredCbr >= 0.0 ? measuredCbr : 0.0;

      Ptr<BSContainer> primaryContainer = GetPrimaryCpmContainer (entry.second);
      Ptr<CPBasicService> primaryCp = primaryContainer->getCPBasicService ();
      const uint64_t cpmTx = primaryCp->getCpmSent ();
      const uint32_t cpmSizeBytes = primaryCp->getLastCpmSizeBytes ();

      PredictiveCbrState& state = g_predictiveCbrState[vehicleId];
      double elapsedSeconds = interval.GetSeconds ();
      uint64_t cpmTxDelta = 0;
      if (state.initialized)
        {
          elapsedSeconds = std::max ((now - state.lastUpdate).GetSeconds (), 1e-9);
          state.cbrRatePerSecond = (currentCbr - state.lastCbr) / elapsedSeconds;
          cpmTxDelta = cpmTx - state.lastDsrcCpmTx;
        }
      else
        {
          state.cbrRatePerSecond = 0.0;
        }

      state.dsrcCpmTxRate = elapsedSeconds > 0.0
                                ? static_cast<double> (cpmTxDelta) / elapsedSeconds
                                : 0.0;
      state.lastDsrcCpmSizeBytes = cpmSizeBytes;

      const double cpmSizeFeature =
          g_predictiveRmrConfig.cpmSizeNormBytes > 0.0
              ? static_cast<double> (cpmSizeBytes) /
                    g_predictiveRmrConfig.cpmSizeNormBytes
              : 0.0;
      const double cpmTxRateFeature =
          g_predictiveRmrConfig.cpmTxRateNorm > 0.0
              ? state.dsrcCpmTxRate / g_predictiveRmrConfig.cpmTxRateNorm
              : 0.0;

      double trafficFlowCbr = currentCbr;
      double predictionLeadTimeSeconds = 0.0;
      if (GetVehicleRsuPrediction (vehicleId,
                                   &trafficFlowCbr,
                                   &predictionLeadTimeSeconds))
        {
          ++i2vPredictionCount;
          i2vPredictionSum += trafficFlowCbr;
          i2vLeadTimeSum += predictionLeadTimeSeconds;
        }
      const double predictedCbr =
          Clamp01 (std::max (currentCbr, trafficFlowCbr) +
                   (g_predictiveRmrConfig.cpmSizeWeight * cpmSizeFeature) +
                   (g_predictiveRmrConfig.cpmTxRateWeight * cpmTxRateFeature) +
                   (g_predictiveRmrConfig.activeVehicleWeight * activeVehicleFeature));

      state.predictedCbr = predictedCbr;
      state.lastCbr = currentCbr;
      state.lastUpdate = now;
      state.lastDsrcCpmTx = cpmTx;
      state.initialized = true;

      if (g_enablePredictiveRmrControl)
        {
          const double actionProbability = ComputePredictiveRmrActionProbability (predictedCbr);
          SetRmrActionProbability (entry.second, actionProbability);
          entry.second.dsrcContainer->getCPBasicService ()->setRmrCurrentCbr (predictedCbr);
          if (entry.second.nrContainer != nullptr)
            {
              entry.second.nrContainer->getCPBasicService ()->setRmrCurrentCbr (predictedCbr);
            }
          if (entry.second.mecContainer != nullptr)
            {
              entry.second.mecContainer->getCPBasicService ()->setRmrCurrentCbr (predictedCbr);
            }
        }
    }
  g_trafficFlowPrediction.i2vVehicleCount = i2vPredictionCount;
  g_trafficFlowPrediction.i2vPredictedCbrAvg =
      i2vPredictionCount > 0 ? i2vPredictionSum / i2vPredictionCount : 0.0;
  g_trafficFlowPrediction.i2vPredictionLeadTimeAvg =
      i2vPredictionCount > 0 ? i2vLeadTimeSum / i2vPredictionCount : 0.0;
  if (g_rsuPredictionLog.is_open ())
    {
      for (const auto& rsu : g_trafficFlowRsus)
        {
          const auto predictionIt = g_trafficFlowRsuPredictions.find (rsu.id);
          if (predictionIt == g_trafficFlowRsuPredictions.end ())
            {
              continue;
            }
          const auto countIt = g_trafficFlowRsuI2vCounts.find (rsu.id);
          const uint32_t i2vVehicles =
              countIt != g_trafficFlowRsuI2vCounts.end () ? countIt->second : 0;
          const auto& prediction = predictionIt->second;
          g_rsuPredictionLog
              << now.GetSeconds () << "," << rsu.id << "," << rsu.x << "," << rsu.y << ","
              << prediction.measuredCbr << "," << prediction.predictedCbr << ","
              << prediction.peakTimeSeconds << "," << prediction.localVehicles << ","
              << prediction.futureVehicles << "," << i2vVehicles << std::endl;
        }
    }

  Simulator::Schedule (interval,
                       &UpdatePredictiveRmrCbr,
                       channelMetrics,
                       interval);
}

static void
GenerateDsrcInterference (Ptr<Socket> socket,
                          std::string sourceVehicleId,
                          MovingBackgroundConfig config)
{
  const auto generationIt = g_backgroundGenerationByVehicle.find (sourceVehicleId);
  if (Simulator::Now () >= config.stopTime ||
      g_vehicleRuntime.find (sourceVehicleId) == g_vehicleRuntime.end () ||
      generationIt == g_backgroundGenerationByVehicle.end () ||
      generationIt->second != config.generation)
    {
      return;
    }

  double factor = 1.0;
  if (config.enabled && g_sumoClient != nullptr)
    {
      try
        {
          const double roadLength = std::max (config.roadLengthMeters, 1.0);
          const double rawX =
              g_sumoClient->TraCIAPI::vehicle.getPosition (sourceVehicleId).x;
          const double x = std::fmod (std::fmod (rawX, roadLength) + roadLength, roadLength);
          const double center =
              std::fmod (config.speedMps * Simulator::Now ().GetSeconds (), roadLength);
          const double directDistance = std::abs (x - center);
          const double wrappedDistance = std::max (0.0, roadLength - directDistance);
          const double distance = std::min (directDistance, wrappedDistance);
          factor = distance <= config.widthMeters / 2.0
                       ? 1.0 + config.amplitude
                       : config.minFactor;
        }
      catch (...)
        {
        }
    }
  else if (config.fixedZones && g_sumoClient != nullptr && !g_trafficFlowRsus.empty ())
    {
      try
        {
          const double x = g_sumoClient->TraCIAPI::vehicle.getPosition (sourceVehicleId).x;
          std::size_t nearestIndex = 0;
          double nearestDistance = std::numeric_limits<double>::infinity ();
          for (std::size_t index = 0; index < g_trafficFlowRsus.size (); ++index)
            {
              const double distance = std::abs (g_trafficFlowRsus[index].x - x);
              if (distance < nearestDistance)
                {
                  nearestDistance = distance;
                  nearestIndex = index;
                }
            }
          factor =
              g_fixedRsuCbrValues[std::min (nearestIndex, g_fixedRsuCbrValues.size () - 1)] /
              0.6;
        }
      catch (...)
        {
        }
    }
  const uint32_t currentPktSize =
      std::max (1u,
                static_cast<uint32_t> (std::lround (config.packetSize * factor)));
  const int sent = socket->Send (Create<Packet> (currentPktSize));
  if (sent >= 0)
    {
      ++g_interferenceTx;
      g_interferenceBytes += currentPktSize;
    }
  else
    {
      ++g_interferenceDrops;
    }

  Simulator::Schedule (config.packetInterval,
                       &GenerateDsrcInterference,
                       socket,
                       sourceVehicleId,
                       config);
}

static bool
IsInMovingBackgroundHighLoadZone (const std::string& vehicleId)
{
  if (!g_nrBgMovingWaveForLog || g_sumoClient == nullptr)
    {
      return false;
    }
  try
    {
      const double roadLength = std::max (g_trafficFlowRoadLengthMetersForLog, 1.0);
      const double rawX = g_sumoClient->TraCIAPI::vehicle.getPosition (vehicleId).x;
      const double x = std::fmod (std::fmod (rawX, roadLength) + roadLength, roadLength);
      const double center =
          std::fmod (g_nrBgWaveSpeedMpsForLog * Simulator::Now ().GetSeconds (), roadLength);
      const double directDistance = std::abs (x - center);
      const double wrappedDistance = std::max (0.0, roadLength - directDistance);
      const double distance = std::min (directDistance, wrappedDistance);
      return distance <= g_nrBgWaveWidthMetersForLog / 2.0;
    }
  catch (...)
    {
      return false;
    }
}

static void
StartRouteTx (VehicleRuntime& runtime, ActiveRoute route, double desync)
{
  if (route == ActiveRoute::MecV2n2v && g_useIdealMecLink)
    {
      runtime.mecTxActive = true;
      return;
    }

  Ptr<BSContainer> container = GetRouteContainer (runtime, route);
  if (container == nullptr)
    {
      NS_FATAL_ERROR ("Attempted to activate an unavailable route: " << RouteName (route));
    }

  container->getCABasicService ()->startCamDissemination (desync);
  if (runtime.sendCpm)
    {
      if (route == ActiveRoute::DsrcV2v)
        {
          ++g_dsrcCpmStartCalls;
        }
      else if (route == ActiveRoute::NrSidelinkV2v)
        {
          ++g_nrCpmStartCalls;
        }
      else
        {
          ++g_mecCpmStartCalls;
        }
      if (g_cpmStartDelaySeconds > 0.0)
        {
          Ptr<CPBasicService> cp = container->getCPBasicService ();
          Simulator::Schedule (Seconds (g_cpmStartDelaySeconds),
                               &CPBasicService::startCpmDissemination,
                               cp);
        }
      else
        {
          container->getCPBasicService ()->startCpmDissemination ();
        }
    }

  if (route == ActiveRoute::DsrcV2v)
    {
      runtime.dsrcTxActive = true;
    }
  else if (route == ActiveRoute::NrSidelinkV2v)
    {
      runtime.nrTxActive = true;
    }
  else
    {
      runtime.mecTxActive = true;
    }
}

static void
StopRouteTx (VehicleRuntime& runtime, ActiveRoute route)
{
  if (route == ActiveRoute::MecV2n2v && g_useIdealMecLink)
    {
      runtime.mecTxActive = false;
      return;
    }

  Ptr<BSContainer> container = GetRouteContainer (runtime, route);
  if (container == nullptr)
    {
      return;
    }

  container->getCABasicService ()->terminateDissemination ();
  if (runtime.sendCpm)
    {
      container->getCPBasicService ()->terminateDissemination ();
    }

  if (route == ActiveRoute::DsrcV2v)
    {
      runtime.dsrcTxActive = false;
    }
  else if (route == ActiveRoute::NrSidelinkV2v)
    {
      runtime.nrTxActive = false;
    }
  else
    {
      runtime.mecTxActive = false;
    }
}

static void
ActivateRoute (const std::string& vehicleId,
               ActiveRoute route,
               double cbr,
               const std::string& reason,
               bool force = false)
{
  auto it = g_vehicleRuntime.find (vehicleId);
  if (it == g_vehicleRuntime.end ())
    {
      return;
    }

  VehicleRuntime& runtime = it->second;
  if (!force && runtime.activeRoute == route)
    {
      return;
    }

  if (runtime.dsrcTxActive)
    {
      StopRouteTx (runtime, ActiveRoute::DsrcV2v);
    }
  if (runtime.nrTxActive)
    {
      StopRouteTx (runtime, ActiveRoute::NrSidelinkV2v);
    }
  if (runtime.mecTxActive)
    {
      StopRouteTx (runtime, ActiveRoute::MecV2n2v);
    }

  const double desync = (VehicleIdToStationId (vehicleId) % 100) / 100.0;
  StartRouteTx (runtime, route, desync);
  runtime.activeRoute = route;
  WriteRouteLog (vehicleId, cbr, route, reason);
}

static void
SetRouteTxState (const std::string& vehicleId,
                 bool dsrcActive,
                 bool nrActive,
                 bool mecActive,
                 double cbr,
                 const std::string& reason,
                 bool force = false)
{
  auto it = g_vehicleRuntime.find (vehicleId);
  if (it == g_vehicleRuntime.end ())
    {
      return;
    }
  if (!dsrcActive && !nrActive && !mecActive)
    {
      NS_FATAL_ERROR ("At least one route must be active for " << vehicleId);
    }

  VehicleRuntime& runtime = it->second;
  if (nrActive && runtime.nrContainer == nullptr)
    {
      NS_FATAL_ERROR ("Attempted to activate unavailable NR sidelink route");
    }
  if (mecActive && runtime.mecContainer == nullptr && !g_useIdealMecLink)
    {
      NS_FATAL_ERROR ("Attempted to activate unavailable MEC V2N2V route");
    }

  if (!force && runtime.dsrcTxActive == dsrcActive && runtime.nrTxActive == nrActive &&
      runtime.mecTxActive == mecActive)
    {
      return;
    }

  if (runtime.dsrcTxActive && !dsrcActive)
    {
      StopRouteTx (runtime, ActiveRoute::DsrcV2v);
    }
  if (runtime.nrTxActive && !nrActive)
    {
      StopRouteTx (runtime, ActiveRoute::NrSidelinkV2v);
    }
  if (runtime.mecTxActive && !mecActive)
    {
      StopRouteTx (runtime, ActiveRoute::MecV2n2v);
    }

  const double desync = (VehicleIdToStationId (vehicleId) % 100) / 100.0;
  if (!runtime.dsrcTxActive && dsrcActive)
    {
      StartRouteTx (runtime, ActiveRoute::DsrcV2v, desync);
    }
  if (!runtime.nrTxActive && nrActive)
    {
      StartRouteTx (runtime, ActiveRoute::NrSidelinkV2v, desync);
    }
  if (!runtime.mecTxActive && mecActive)
    {
      StartRouteTx (runtime, ActiveRoute::MecV2n2v, desync);
    }

  if (mecActive)
    {
      runtime.activeRoute = ActiveRoute::MecV2n2v;
      if (g_hybridRouteConfig.mecMinHoldTimeSeconds > 0.0)
        {
          runtime.mecHoldUntil =
              std::max (runtime.mecHoldUntil,
                        Simulator::Now () +
                            Seconds (g_hybridRouteConfig.mecMinHoldTimeSeconds));
        }
    }
  else if (nrActive)
    {
      runtime.activeRoute = ActiveRoute::NrSidelinkV2v;
    }
  else
    {
      runtime.activeRoute = ActiveRoute::DsrcV2v;
    }
  WriteRouteStateLog (vehicleId, cbr, dsrcActive, nrActive, mecActive, reason);
}

static void
SetRmrForwardingModes (VehicleRuntime& runtime,
                       CPBasicService::RmrForwardingMode dsrcMode,
                       CPBasicService::RmrForwardingMode mecMode)
{
  if (runtime.dsrcContainer != nullptr)
    {
      runtime.dsrcContainer->getCPBasicService ()->setRmrForwardingMode (dsrcMode);
    }
  if (runtime.mecContainer != nullptr)
    {
      runtime.mecContainer->getCPBasicService ()->setRmrForwardingMode (mecMode);
    }
  if (runtime.nrContainer != nullptr)
    {
      runtime.nrContainer->getCPBasicService ()->setRmrForwardingMode (dsrcMode);
    }
}

static double
ComputePredictiveRmrActionProbability (double predictedCbr)
{
  if (predictedCbr < g_hybridRouteConfig.switchCbr)
    {
      return 0.0;
    }

  const double span = std::max (g_hybridRouteConfig.maxCbr - g_hybridRouteConfig.switchCbr, 1e-9);
  const double pressure =
      std::max (0.0, std::min (1.0, (predictedCbr - g_hybridRouteConfig.switchCbr) / span));
  const double maxActionProbability =
      std::max (0.0, std::min (1.0, 1.0 - g_hybridRouteConfig.lowV2vMinProbability));
  return std::max (0.0,
                   std::min (1.0,
                             g_hybridRouteConfig.lowV2vAlpha * pressure * maxActionProbability));
}

static void
SetRmrActionProbability (VehicleRuntime& runtime, double probability)
{
  if (runtime.dsrcContainer != nullptr)
    {
      runtime.dsrcContainer->getCPBasicService ()->setRmrActionProbability (probability);
    }
  if (runtime.mecContainer != nullptr)
    {
      runtime.mecContainer->getCPBasicService ()->setRmrActionProbability (probability);
    }
  if (runtime.nrContainer != nullptr)
    {
      runtime.nrContainer->getCPBasicService ()->setRmrActionProbability (probability);
    }
}

static Ptr<Socket>
CreateNrGroupcastSocket (Ptr<Node> node, Ipv4Address groupAddress, uint16_t port)
{
  Ptr<Socket> socket = Socket::CreateSocket (node, TypeId::LookupByName ("ns3::UdpSocketFactory"));
  if (socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), port)) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind NR-V2X UDP socket");
    }
  socket->Connect (InetSocketAddress (groupAddress, port));
  return socket;
}

static Ptr<Socket>
CreateNrGroupcastTxSocket (Ptr<Node> node, Ipv4Address groupAddress, uint16_t port)
{
  Ptr<Socket> socket = Socket::CreateSocket (node, TypeId::LookupByName ("ns3::UdpSocketFactory"));
  socket->Connect (InetSocketAddress (groupAddress, port));
  return socket;
}

static Ptr<Socket>
CreateMecUnicastSocket (Ptr<Node> node,
                        Ipv4Address mecAddress,
                        uint16_t mecPort,
                        uint16_t vehiclePort)
{
  Ptr<Socket> socket = Socket::CreateSocket (node, TypeId::LookupByName ("ns3::UdpSocketFactory"));
  if (socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), vehiclePort)) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind MEC V2N2V vehicle UDP socket");
    }
  socket->Connect (InetSocketAddress (mecAddress, mecPort));
  return socket;
}

static void
CheckRoutesByCbr (Ptr<MetricSupervisor> dsrcMetrics,
                  double switchCbr,
                  double releaseCbr,
                  Time interval)
{
  for (auto& entry : g_vehicleRuntime)
    {
      const std::string& vehicleId = entry.first;
      VehicleRuntime& runtime = entry.second;
      const double cbr = dsrcMetrics->getCBRPerItem (vehicleId);

      if (cbr < 0.0)
        {
          continue;
        }

      if (runtime.activeRoute == ActiveRoute::DsrcV2v && cbr >= switchCbr)
        {
          ActivateRoute (vehicleId, ActiveRoute::NrSidelinkV2v, cbr, "cbr_above_switch_threshold");
        }
      else if (runtime.activeRoute == ActiveRoute::NrSidelinkV2v && cbr <= releaseCbr)
        {
          ActivateRoute (vehicleId, ActiveRoute::DsrcV2v, cbr, "cbr_below_release_threshold");
        }
    }

  Simulator::Schedule (interval,
                       &CheckRoutesByCbr,
                       dsrcMetrics,
                       switchCbr,
                       releaseCbr,
                       interval);
}

static void
CheckHybridRoutesByPredictedCbrToMec (Ptr<MetricSupervisor> dsrcMetrics,
                                      Time interval)
{
  for (auto& entry : g_vehicleRuntime)
    {
      const std::string& vehicleId = entry.first;
      VehicleRuntime& runtime = entry.second;
      if (runtime.mecContainer == nullptr && !g_useIdealMecLink)
        {
          continue;
        }

      double cbr = dsrcMetrics->getCBRPerItem (vehicleId);
      const auto predictiveIt = g_predictiveCbrState.find (vehicleId);
      if (predictiveIt != g_predictiveCbrState.end () && predictiveIt->second.initialized)
        {
          cbr = predictiveIt->second.predictedCbr;
        }

      if (cbr < 0.0)
        {
          continue;
        }

      if (cbr <= g_hybridRouteConfig.releaseCbr)
        {
          if (runtime.mecTxActive && Simulator::Now () < runtime.mecHoldUntil)
            {
              continue;
            }
          SetRmrForwardingModes (runtime,
                                 CPBasicService::RMR_FORWARDING_PRIMARY,
                                 CPBasicService::RMR_FORWARDING_PRIMARY);
          SetRouteTxState (vehicleId,
                           false,
                           true,
                           false,
                           cbr,
                           "predicted_cbr_normal_nr_sidelink");
          continue;
        }

      if (cbr >= g_hybridRouteConfig.maxCbr)
        {
          if (g_hybridRouteConfig.mecRecoveryPolicy == MecRecoveryPolicy::OffloadOnly)
            {
              SetRmrForwardingModes (runtime,
                                     CPBasicService::RMR_FORWARDING_PRIMARY,
                                     CPBasicService::RMR_FORWARDING_OFFLOAD);
              SetRouteTxState (vehicleId,
                               false,
                               true,
                               true,
                               cbr,
                               "predicted_cbr_congested_offload_only");
            }
          else if (g_hybridRouteConfig.highPriorityDualTx)
            {
              SetRmrForwardingModes (runtime,
                                     CPBasicService::RMR_FORWARDING_PRIMARY,
                                     CPBasicService::RMR_FORWARDING_DUPLICATE);
              SetRouteTxState (vehicleId,
                               false,
                               true,
                               true,
                               cbr,
                               "predicted_cbr_congested_high_dual");
            }
          else
            {
              SetRmrForwardingModes (runtime,
                                     CPBasicService::RMR_FORWARDING_PRIMARY,
                                     CPBasicService::RMR_FORWARDING_DUPLICATE);
              SetRouteTxState (vehicleId,
                               false,
                               false,
                               true,
                               cbr,
                               "predicted_cbr_congested_mec");
            }
          continue;
        }

      SetRmrForwardingModes (runtime,
                             CPBasicService::RMR_FORWARDING_PRIMARY,
                             CPBasicService::RMR_FORWARDING_DUPLICATE);
      SetRouteTxState (vehicleId,
                       false,
                       true,
                       true,
                       cbr,
                       "predicted_cbr_warning_object_split");
    }

  Simulator::Schedule (interval,
                       &CheckHybridRoutesByPredictedCbrToMec,
                       dsrcMetrics,
                       interval);
}

static void
GetSlBitmapFromString (std::string slBitMapString,
                       std::vector<std::bitset<1>>& slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable = {
      {"0", 0},
      {"1", 1},
  };

  std::stringstream ss (slBitMapString);
  std::string token;
  while (std::getline (ss, token, '|'))
    {
      auto it = lookupTable.find (token);
      if (it == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Invalid sidelink bitmap item: " << token);
        }
      slBitMapVector.push_back (it->second & 0x01);
    }
}

int
main (int argc, char* argv[])
{
  std::string phyMode = "OfdmRate6MbpsBW10MHz";
  int userPriority = 0;
  int dsrcInterferenceUserPriority = 0;
  bool realtime = false;
  bool verbose = false;
  bool sumoGui = true;
  std::string sumoScreenshotDir;
  std::string sumoScreenshotView = "View #0";
  double sumoScreenshotIntervalSeconds = 0.0;
  double sumoScreenshotStartSeconds = 0.0;
  double sumoScreenshotStopSeconds = 0.0;
  double sumoScreenshotCenterX = 1500.0;
  double sumoScreenshotCenterY = 38.0;
  double sumoScreenshotSpanX = 900.0;
  double sumoScreenshotSpanY = 180.0;
  int sumoScreenshotWidth = 1600;
  int sumoScreenshotHeight = 900;
  bool sendCpm = true;
  bool enableDcc = false;
  bool enableRouteControl = true;
  bool enablePcap = false;
  bool enableDsrcInterference = false;
  bool nrBgMovingWave = false;
  bool nrBgFixedZones = false;
  bool enableNrSensing = false;
  bool enableChannelRandomness = false;
  bool enableReactiveRmr = false;
  bool enablePredictiveRmr = false;
  bool enableTrafficFlowRsuPredictor = false;
  bool enableMecV2n2v = false;
  bool enableMecRouteControl = false;
  bool enableMecOnly = false;
  double simTime = 120.0;
  double baselinePrR = 150.0;
  double txPower = 30.0;
  double sensitivity = -93.0;
  double snrThreshold = 10.0;
  double sinrThreshold = 10.0;
  double switchCbr = 0.60;
  double releaseCbr = 0.50;
  double routeCheckInterval = 1.0;
  double dccTarget = 0.63;
  uint32_t dccIntervalMs = 200;
  double cbrWindowMs = 200.0;
  double cbrAlpha = 0.1;
  double sumoWaitForSocket = 5.0;
  double sumoSyncInterval = 0.01;
  uint32_t sumoPort = 3400;
  int32_t sumoSeed = 10;
  uint64_t rngRun = 1;
  double observationLogInterval = 1.0;
  double thesisEvalInterval = 1.0;
  uint32_t thesisEvalStartMinVehicles = 0;
  bool thesisEvalStartUseAllVehicles = true;
  bool holdTrafficUntilEvaluationStart = false;
  double thesisEvalWarmupSeconds = 5.0;
  double dsrcDccBitRateMbps = 6.0;
  double dsrcInterferenceStart = 1.0;
  double dsrcInterferenceStop = 0.0;
  double dsrcInterferenceIntervalMs = 100.0;
  uint32_t dsrcInterferencePacketSize = 1500;
  uint32_t dsrcInterferenceSourceVehicle = 3;
  uint32_t dsrcInterferenceNodeCount = 6;
  bool dsrcInterferencePerVehicle = false;
  double nrBgWavePeriodSeconds = 20.0;
  double nrBgWaveAmplitude = 0.8;
  double nrBgWaveSpeedMps = 25.0;
  double nrBgWaveMinFactor = 0.2;
  double nrBgWaveWidthMeters = 500.0;
  double rmrCbrLow = 0.33;
  double rmrCbrHigh = 0.67;
  double rmrWeightFrequency = 1.0;
  double rmrWeightDynamics = 1.0;
  double rmrWeightDistance = 1.0;
  uint32_t rmrDeleteLow = 10;
  uint32_t rmrDeleteMiddle = 20;
  uint32_t rmrDeleteHigh = 40;
  uint32_t rmrWindowMs = 1000;
  bool rmrProtectNearObjects = false;
  double predictionHorizonSeconds = 20.0;
  double predictorCpmSizeNormBytes = 1200.0;
  double predictorCpmTxRateNorm = 10.0;
  double predictorActiveVehicleNorm = 100.0;
  double trafficFlowRoadLengthMeters = 2000.0;
  std::string trafficFlowTopology = "loop";
  double trafficFlowMessageRateHz = 10.0;
  double trafficFlowAvgPacketSizeBytes = 500.0;
  double trafficFlowChannelRateMbps = 6.0;
  double rsuPassivePdrLowCbr = 0.95;
  double rsuPassivePdrMidCbr = 0.90;
  double rsuPassiveSaturationCbr = 0.90;
  double rsuI2vRangeMeters = 300.0;
  std::string fixedRsuCbrMode = "disabled";
  double predictorCpmSizeWeight = 0.0;
  double predictorCpmTxRateWeight = 0.0;
  double predictorActiveVehicleWeight = 0.0;
  double hybridCbrMax = 0.80;
  double hybridLowV2vMinProbability = 0.20;
  double hybridLowV2vAlpha = 1.0;
  bool hybridHighPriorityDualTx = true;
  bool mecDuplicateRecovery = false;
  std::string mecRecoveryPolicyName = "staged";
  std::string mecObjectPolicyName = "all-objects";
  double mecAdaptiveLowMaxProbability = 0.5;
  double mecMinHoldTimeSeconds = 5.0;
  int64_t hybridRoutingStream = 7;
  double priorityDistanceMeters = 100.0;
  double priorityClosingSpeedMps = 3.0;
  double priorityTtcSeconds = 5.0;
  double sensorRangeMeters = 100.0;
  double cpmRecognitionTtlSeconds = 1.0;
  double highPriorityCpmRecognitionTtlSeconds = 0.5;
  double lowPriorityCpmRecognitionTtlSeconds = 1.0;
  double orrRangeMeters = 100.0;
  bool sensorExternalEventsEnabled = false;
  double sensorExternalEventProbability = 0.20;
  double sensorExternalEventActivationRangeMeters = 400.0;
  uint16_t nrSocketPort = 19;
  Time slBearersActivationTime = Seconds (2.0);
  std::string mecRsuFile = "stations.xml";
  uint16_t mecServerPort = 49000;
  uint16_t mecVehiclePort = 49001;
  double mecBackhaulDelayMs = 10.0;
  double mecProcessingDelayMs = 0.0;
  double mecForwardRangeMeters = 0.0;
  uint32_t mecSrsPeriodicity = 320;
  bool mecIdealLink = true;
  std::string mecIdealDelayModel = "empirical";
  double mecIdealLatencyMs = 39.4;
  double mecIdealLatencyStddevMs = 20.0;
  double mecIdealLatencyMinMs = 6.1;
  double mecIdealLatencyP90Ms = 59.86;
  double mecIdealLatencyP99Ms = 120.33;
  double mecIdealLatencyMaxMs = 201.0;
  double mecIdealCpmIntervalMs = 100.0;
  uint32_t mecIdealPacketSizeBytes = 500;
  double mecIdealDownlinkPdr = 1.0;
  bool mecIdealCapacityModel = false;
  double mecIdealBandwidthMHz = 20.0;
  double mecIdealSpectralEfficiencyBpsHz = 4.5234;
  double mecIdealMaxQueueDelayMs = 0.0;
  double mecIdealUlFirstLossRate = 0.10;
  double mecIdealDlFirstLossRate = 0.10;
  double mecIdealRetxSuccessProbability = 0.53;
  uint32_t mecIdealMaxRetransmissions = 4;
  double mecIdealRetxDelayMs = 3.0;
  bool mecIdealAoiFilter = false;
  double mecIdealAoiFilterThresholdMs = 200.0;
  bool mecBackgroundLoad = false;
  bool mecBackgroundPerVehicle = true;
  uint32_t mecBackgroundPacketSizeBytes = 500;
  double mecBackgroundIntervalMs = 100.0;
  uint32_t maxCommunicationVehicles = 0;

  double centralFrequencyBandSl = 5.89e9;
  uint16_t bandwidthBandSl = 100;
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  uint16_t slSensingWindow = 100;
  uint16_t slSelectionWindow = 5;
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  int slThresPsschRsrp = -128;
  uint16_t channelUpdatePeriod = 500;
  uint8_t mcs = 14;

  std::string sumoFolder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mobTrace = "cars_7.rou.xml";
  std::string sumoConfig = "src/automotive/examples/sumo_files_v2v_map/map_7.sumo.cfg";
  std::string sumoAdditionalOptions = "";
  std::string routeLogPath = "hybrid-cbr-route-log.csv";
  std::string cbrLogPath = "hybrid-nr-channel-log.csv";
  std::string observationLogPath = "hybrid-observation-log.csv";
  std::string cpmInputDiagLogPath = "";
  std::string rsuPredictionLogPath = "";
  std::string summaryCsvPath = "summary.csv";
  std::string method = "legacy";

  CommandLine cmd (__FILE__);
  cmd.AddValue ("phyMode", "Legacy V2V PHY mode; unused in NR-only evaluation", phyMode);
  cmd.AddValue ("userpriority", "EDCA User Priority for ETSI messages", userPriority);
  cmd.AddValue ("realtime", "Run with the realtime scheduler", realtime);
  cmd.AddValue ("verbose", "Enable verbose legacy V2V logging", verbose);
  cmd.AddValue ("sumo-gui", "Show SUMO GUI", sumoGui);
  cmd.AddValue ("sumo-screenshot-dir",
                "Directory for periodic SUMO GUI screenshots; empty disables screenshots",
                sumoScreenshotDir);
  cmd.AddValue ("sumo-screenshot-view", "SUMO GUI view id for screenshots", sumoScreenshotView);
  cmd.AddValue ("sumo-screenshot-interval",
                "SUMO GUI screenshot interval [s]; <=0 disables screenshots",
                sumoScreenshotIntervalSeconds);
  cmd.AddValue ("sumo-screenshot-start", "First SUMO GUI screenshot time [s]", sumoScreenshotStartSeconds);
  cmd.AddValue ("sumo-screenshot-stop", "Last SUMO GUI screenshot time [s]; <=0 uses sim-time", sumoScreenshotStopSeconds);
  cmd.AddValue ("sumo-screenshot-center-x", "SUMO GUI screenshot boundary center x [m]", sumoScreenshotCenterX);
  cmd.AddValue ("sumo-screenshot-center-y", "SUMO GUI screenshot boundary center y [m]", sumoScreenshotCenterY);
  cmd.AddValue ("sumo-screenshot-span-x", "SUMO GUI screenshot boundary width [m]", sumoScreenshotSpanX);
  cmd.AddValue ("sumo-screenshot-span-y", "SUMO GUI screenshot boundary height [m]", sumoScreenshotSpanY);
  cmd.AddValue ("sumo-screenshot-width", "SUMO GUI screenshot pixel width", sumoScreenshotWidth);
  cmd.AddValue ("sumo-screenshot-height", "SUMO GUI screenshot pixel height", sumoScreenshotHeight);
  cmd.AddValue ("send-cpm", "Enable CPM dissemination in addition to CAM", sendCpm);
  cmd.AddValue ("cpm-start-delay",
                "Delay CPM dissemination start after route activation [s]",
                g_cpmStartDelaySeconds);
  cmd.AddValue ("enable-dcc", "Enable ETSI DCC on the legacy V2V route", enableDcc);
  cmd.AddValue ("route-control", "Enable legacy route-control mode; unused in NR-only evaluation", enableRouteControl);
  cmd.AddValue ("dsrc-dcc-bitrate-mbps", "Legacy DCC bitrate; unused in NR-only evaluation", dsrcDccBitRateMbps);
  cmd.AddValue ("dsrc-interference", "Deprecated alias for --nr-bg", enableDsrcInterference);
  cmd.AddValue ("dsrc-interference-source", "Deprecated alias for --nr-bg-source", dsrcInterferenceSourceVehicle);
  cmd.AddValue ("dsrc-interference-nodes", "Deprecated alias for --nr-bg-nodes", dsrcInterferenceNodeCount);
  cmd.AddValue ("dsrc-interference-per-vehicle",
                "If true, every active SUMO vehicle emits background traffic while it is active",
                dsrcInterferencePerVehicle);
  cmd.AddValue ("dsrc-interference-size", "Background packet size [bytes]", dsrcInterferencePacketSize);
  cmd.AddValue ("dsrc-interference-interval-ms", "Background traffic interval [ms]", dsrcInterferenceIntervalMs);
  cmd.AddValue ("dsrc-interference-start", "Background traffic start time [s]", dsrcInterferenceStart);
  cmd.AddValue ("dsrc-interference-stop", "Background traffic stop time [s]; 0 means sim-time", dsrcInterferenceStop);
  cmd.AddValue ("dsrc-interference-userpriority", "Deprecated alias for --nr-bg-userpriority", dsrcInterferenceUserPriority);
  cmd.AddValue ("nr-bg", "Enable NR sidelink UDP background traffic", enableDsrcInterference);
  cmd.AddValue ("nr-bg-source", "Deprecated single-vehicle background source option", dsrcInterferenceSourceVehicle);
  cmd.AddValue ("nr-bg-nodes", "Number of SUMO vehicle nodes sending NR sidelink background traffic", dsrcInterferenceNodeCount);
  cmd.AddValue ("nr-bg-per-vehicle",
                "If true, every active SUMO vehicle emits background traffic while it is active",
                dsrcInterferencePerVehicle);
  cmd.AddValue ("nr-bg-size", "Background packet size [bytes]", dsrcInterferencePacketSize);
  cmd.AddValue ("nr-bg-interval-ms", "Background traffic interval [ms]", dsrcInterferenceIntervalMs);
  cmd.AddValue ("nr-bg-start", "Background traffic start time [s]", dsrcInterferenceStart);
  cmd.AddValue ("nr-bg-stop", "Background traffic stop time [s]; 0 means sim-time", dsrcInterferenceStop);
  cmd.AddValue ("nr-bg-userpriority", "User Priority for NR background traffic", dsrcInterferenceUserPriority);
  cmd.AddValue ("nr-bg-moving-wave",
                "Modulate NR background packet size as a spatially moving sine wave",
                nrBgMovingWave);
  cmd.AddValue ("nr-bg-fixed-zones",
                "Scale NR background by fixed RSU zone CBR profile",
                nrBgFixedZones);
  cmd.AddValue ("nr-bg-wave-period", "Moving background wave period [s]", nrBgWavePeriodSeconds);
  cmd.AddValue ("nr-bg-wave-amplitude",
                "Moving background fractional amplitude [0,1]",
                nrBgWaveAmplitude);
  cmd.AddValue ("nr-bg-wave-speed", "Moving background wave speed [m/s]", nrBgWaveSpeedMps);
  cmd.AddValue ("nr-bg-wave-min-factor",
                "Minimum moving background packet-size factor",
                nrBgWaveMinFactor);
  cmd.AddValue ("nr-bg-wave-width",
                "Width [m] of the moving high-load background region",
                nrBgWaveWidthMeters);
  cmd.AddValue ("method",
                "Run method: legacy, no-control, reactive-rmr, predictive-rmr-v2v, predictive-rmr-v2n2v, hybrid-v2v-v2n2v, v2n2v-only, cbr-route-nr-sidelink",
                method);
  cmd.AddValue ("reactive-rmr", "Enable thesis-style CBR-reactive CPM object deletion", enableReactiveRmr);
  cmd.AddValue ("predictive-rmr", "Use predicted CBR instead of measured CBR as the RMR input", enablePredictiveRmr);
  cmd.AddValue ("traffic-flow-rsu-predictor",
                "Enable traffic-flow RSU CBR prediction for hybrid route control without enabling RMR",
                enableTrafficFlowRsuPredictor);
  cmd.AddValue ("rmr-cbr-low", "CBR threshold for low/middle RMR deletion phases", rmrCbrLow);
  cmd.AddValue ("rmr-cbr-high", "CBR threshold for middle/high RMR deletion phases", rmrCbrHigh);
  cmd.AddValue ("rmr-delete-low", "Objects deleted per CPM in low CBR phase", rmrDeleteLow);
  cmd.AddValue ("rmr-delete-middle", "Objects deleted per CPM in middle CBR phase", rmrDeleteMiddle);
  cmd.AddValue ("rmr-delete-high", "Objects deleted per CPM in high CBR phase", rmrDeleteHigh);
  cmd.AddValue ("rmr-window-ms", "Time window used for Frequency-based RMR observation count", rmrWindowMs);
  cmd.AddValue ("rmr-protect-near-objects",
                "Exclude objects within --priority-distance from RMR deletion candidates",
                rmrProtectNearObjects);
  cmd.AddValue ("rmr-weight-frequency", "Weight for Frequency-based RMR term", rmrWeightFrequency);
  cmd.AddValue ("rmr-weight-dynamics", "Weight for Dynamics-based RMR terms", rmrWeightDynamics);
  cmd.AddValue ("rmr-weight-distance", "Weight for Distance-based RMR term", rmrWeightDistance);
  cmd.AddValue ("prediction-horizon",
                "Prediction horizon for predictive RMR [s]",
                predictionHorizonSeconds);
  cmd.AddValue ("traffic-flow-road-length",
                "Road segment length [m] used by the RSU traffic-flow CBR predictor",
                trafficFlowRoadLengthMeters);
  cmd.AddValue ("traffic-flow-topology",
                "Traffic-flow topology: loop or straight",
                trafficFlowTopology);
  cmd.AddValue ("traffic-flow-message-rate",
                "Message generation rate lambda [Hz] used by the RSU traffic-flow CBR predictor",
                trafficFlowMessageRateHz);
  cmd.AddValue ("traffic-flow-avg-packet-size",
                "Fallback average CPM packet size [bytes] used by the RSU traffic-flow CBR predictor",
                trafficFlowAvgPacketSizeBytes);
  cmd.AddValue ("traffic-flow-channel-rate-mbps",
                "Channel-rate denominator [Mbps] used by the RSU traffic-flow CBR predictor",
                trafficFlowChannelRateMbps);
  cmd.AddValue ("rsu-passive-pdr-low-cbr",
                "Pseudo passive-listening PDR below --switch-cbr",
                rsuPassivePdrLowCbr);
  cmd.AddValue ("rsu-passive-pdr-mid-cbr",
                "Pseudo passive-listening PDR between --switch-cbr and --rsu-passive-saturation-cbr",
                rsuPassivePdrMidCbr);
  cmd.AddValue ("rsu-passive-saturation-cbr",
                "CBR where RSU prediction saturates to 1.0 instead of relying on passive-listening precision",
                rsuPassiveSaturationCbr);
  cmd.AddValue ("rsu-i2v-range",
                "Maximum distance [m] for using the next RSU prediction",
                rsuI2vRangeMeters);
  cmd.AddValue ("fixed-rsu-cbr-mode",
                "Fixed RSU CBR input: disabled, current, or next",
                fixedRsuCbrMode);
  cmd.AddValue ("predictor-cpm-size-norm",
                "Normalization value for the CPM size predictor feature [bytes]",
                predictorCpmSizeNormBytes);
  cmd.AddValue ("predictor-cpm-tx-rate-norm",
                "Normalization value for the CPM TX-rate predictor feature [packets/s]",
                predictorCpmTxRateNorm);
  cmd.AddValue ("predictor-active-vehicle-norm",
                "Normalization value for the active-vehicle-count predictor feature",
                predictorActiveVehicleNorm);
  cmd.AddValue ("predictor-cpm-size-weight",
                "Optional weight for the normalized CPM size predictor feature",
                predictorCpmSizeWeight);
  cmd.AddValue ("predictor-cpm-tx-rate-weight",
                "Optional weight for the normalized CPM TX-rate predictor feature",
                predictorCpmTxRateWeight);
  cmd.AddValue ("predictor-active-vehicle-weight",
                "Optional weight for the normalized active-vehicle-count predictor feature",
                predictorActiveVehicleWeight);
  cmd.AddValue ("hybrid-cbr-max",
                "Predicted CBR threshold for congested hybrid V2V+V2N2V transmission",
                hybridCbrMax);
  cmd.AddValue ("hybrid-low-v2v-min-prob",
                "Minimum V2V probability for low-priority CPMs in warning/congested states",
                hybridLowV2vMinProbability);
  cmd.AddValue ("hybrid-low-v2v-alpha",
                "Slope for reducing low-priority V2V probability after --switch-cbr",
                hybridLowV2vAlpha);
  cmd.AddValue ("hybrid-high-dual-tx",
                "Use NR sidelink V2V and MEC V2N2V together for high-priority congested CPMs",
                hybridHighPriorityDualTx);
  cmd.AddValue ("mec-duplicate-recovery",
                "Send a full CPM copy through MEC while NR sidelink sends the RMR-reduced CPM under predictive congestion",
                mecDuplicateRecovery);
  cmd.AddValue ("mec-recovery-policy",
                "MEC recovery policy: offload-only, staged, duplicate-always",
                mecRecoveryPolicyName);
  cmd.AddValue ("mec-object-policy",
                "Objects forwarded by abstract MEC: all-objects, high-priority-only, or adaptive-probability",
                mecObjectPolicyName);
  cmd.AddValue ("mec-adaptive-low-max-prob",
                "Maximum MEC forwarding probability for low-priority objects under adaptive-probability",
                mecAdaptiveLowMaxProbability);
  cmd.AddValue ("mec-min-hold-time",
                "Minimum time [s] to keep MEC V2N2V active after switching to MEC",
                mecMinHoldTimeSeconds);
  cmd.AddValue ("hybrid-routing-stream",
                "Random stream index for probabilistic hybrid route decisions",
                hybridRoutingStream);
  cmd.AddValue ("priority-distance",
                "Distance threshold [m] used to classify CPM evaluation objects as high priority",
                priorityDistanceMeters);
  cmd.AddValue ("priority-closing-speed",
                "Minimum closing speed [m/s] for approaching objects to become high priority",
                priorityClosingSpeedMps);
  cmd.AddValue ("priority-ttc",
                "Time-to-collision threshold [s] for approaching objects to become high priority",
                priorityTtcSeconds);
  cmd.AddValue ("sensor-range",
                "Distance threshold [m] where ego sensors directly recognize objects for sensor+CPM ORR",
                sensorRangeMeters);
  cmd.AddValue ("cpm-recognition-ttl",
                "Legacy common time [s] that a received CPM keeps the sender vehicle recognized",
                cpmRecognitionTtlSeconds);
  cmd.AddValue ("high-priority-cpm-recognition-ttl",
                "Time [s] that a received CPM keeps a high-priority vehicle recognized",
                highPriorityCpmRecognitionTtlSeconds);
  cmd.AddValue ("low-priority-cpm-recognition-ttl",
                "Time [s] that a received CPM keeps a low-priority vehicle recognized",
                lowPriorityCpmRecognitionTtlSeconds);
  cmd.AddValue ("orr-range",
                "Distance threshold [m] used by the ORR/recognition evaluation",
                orrRangeMeters);
  cmd.AddValue ("sensor-external-events",
                "Enable reproducible non-CAV events outside sensor range",
                sensorExternalEventsEnabled);
  cmd.AddValue ("sensor-external-event-probability",
                "Non-CAV event selection probability",
                sensorExternalEventProbability);
  cmd.AddValue ("sensor-external-event-activation-range",
                "Maximum distance [m] from any CAV when a selected non-CAV event starts",
                sensorExternalEventActivationRangeMeters);
  cmd.AddValue ("mec-rsu-file",
                "SUMO additional POI file used as LTE eNB/RSU positions for MEC V2N2V",
                mecRsuFile);
  cmd.AddValue ("mec-server-port", "UDP port listened by the MEC RemoteHost", mecServerPort);
  cmd.AddValue ("mec-vehicle-port", "UDP port used by vehicle MEC sockets", mecVehiclePort);
  cmd.AddValue ("mec-backhaul-delay-ms",
                "One-way PGW-to-MEC point-to-point backhaul delay [ms]",
                mecBackhaulDelayMs);
  cmd.AddValue ("mec-processing-delay-ms",
                "Extra MEC application processing delay before forwarding [ms]",
                mecProcessingDelayMs);
  cmd.AddValue ("mec-forward-range",
                "SUMO XY range used by MEC to choose forwarding targets [m]; 0 uses --baseline",
                mecForwardRangeMeters);
  cmd.AddValue ("mec-ideal-link",
                "Use fixed-delay lossless abstract V2N/N2V instead of detailed NR Uu",
                mecIdealLink);
  cmd.AddValue ("mec-ideal-latency-ms",
                "Mean V2N2V latency [ms] used by --mec-ideal-link",
                mecIdealLatencyMs);
  cmd.AddValue ("mec-ideal-delay-model",
                "Abstract MEC V2N2V delay model: empirical or normal",
                mecIdealDelayModel);
  cmd.AddValue ("mec-ideal-latency-stddev-ms",
                "Standard deviation [ms] for abstract MEC V2N2V latency; 0 keeps it fixed",
                mecIdealLatencyStddevMs);
  cmd.AddValue ("mec-ideal-latency-min-ms",
                "Minimum clipped abstract MEC V2N2V latency [ms]",
                mecIdealLatencyMinMs);
  cmd.AddValue ("mec-ideal-latency-p90-ms",
                "90th percentile [ms] for empirical abstract MEC V2N2V latency",
                mecIdealLatencyP90Ms);
  cmd.AddValue ("mec-ideal-latency-p99-ms",
                "99th percentile [ms] for empirical abstract MEC V2N2V latency",
                mecIdealLatencyP99Ms);
  cmd.AddValue ("mec-ideal-latency-max-ms",
                "Maximum clipped abstract MEC V2N2V latency [ms]",
                mecIdealLatencyMaxMs);
  cmd.AddValue ("mec-ideal-cpm-interval-ms",
                "CPM generation interval [ms] used by --mec-ideal-link",
                mecIdealCpmIntervalMs);
  cmd.AddValue ("mec-ideal-packet-size",
                "Abstract MEC CPM packet size [bytes] used for V2N load accounting",
                mecIdealPacketSizeBytes);
  cmd.AddValue ("mec-ideal-dl-pdr",
                "Abstract MEC downlink packet delivery ratio; 1.0 means lossless",
                mecIdealDownlinkPdr);
  cmd.AddValue ("mec-ideal-capacity-model",
                "Serialize abstract MEC UL/DL packets by configured Uu capacities",
                mecIdealCapacityModel);
  cmd.AddValue ("mec-ideal-bandwidth-mhz",
                "Abstract MEC Uu bandwidth [MHz] used for UL/DL capacity proxy",
                mecIdealBandwidthMHz);
  cmd.AddValue ("mec-ideal-spectral-efficiency-bpshz",
                "Spectral efficiency [bit/s/Hz] used for the MEC Uu capacity proxy",
                mecIdealSpectralEfficiencyBpsHz);
  cmd.AddValue ("mec-ideal-max-queue-delay-ms",
                "Drop abstract MEC packets whose capacity delay exceeds this value; 0 disables",
                mecIdealMaxQueueDelayMs);
  cmd.AddValue ("mec-ideal-ul-first-loss-rate",
                "Abstract Uu uplink first-transmission loss probability before retransmission",
                mecIdealUlFirstLossRate);
  cmd.AddValue ("mec-ideal-dl-first-loss-rate",
                "Abstract Uu downlink first-transmission loss probability before retransmission",
                mecIdealDlFirstLossRate);
  cmd.AddValue ("mec-ideal-retx-success-prob",
                "Success probability of each abstract HARQ/RLC-style retransmission",
                mecIdealRetxSuccessProbability);
  cmd.AddValue ("mec-ideal-max-retx",
                "Maximum abstract retransmissions after a first Uu transmission loss",
                mecIdealMaxRetransmissions);
  cmd.AddValue ("mec-ideal-retx-delay-ms",
                "Additional delay [ms] per abstract retransmission attempt",
                mecIdealRetxDelayMs);
  cmd.AddValue ("mec-ideal-aoi-filter",
                "If true, stale abstract MEC CPMs do not update ORR recognition state",
                mecIdealAoiFilter);
  cmd.AddValue ("mec-ideal-aoi-filter-threshold-ms",
                "AoI threshold [ms] used by --mec-ideal-aoi-filter",
                mecIdealAoiFilterThresholdMs);
  cmd.AddValue ("mec-bg",
                "Add abstract MEC UL/DL background traffic to the capacity model",
                mecBackgroundLoad);
  cmd.AddValue ("mec-bg-per-vehicle",
                "If true, every active communication vehicle contributes MEC background load",
                mecBackgroundPerVehicle);
  cmd.AddValue ("mec-bg-size",
                "Abstract MEC background packet size [bytes]",
                mecBackgroundPacketSizeBytes);
  cmd.AddValue ("mec-bg-interval-ms",
                "Abstract MEC background packet interval [ms]",
                mecBackgroundIntervalMs);
  cmd.AddValue ("max-communication-vehicles",
                "Maximum number of SUMO vehicles that run CAM/CPM/V2X applications; 0 means all",
                maxCommunicationVehicles);
  cmd.AddValue ("mec-srs-periodicity",
                "LTE eNB RRC SRS periodicity [ms] for MEC Uu; use 320 for many UEs",
                mecSrsPeriodicity);
  cmd.AddValue ("pcap", "Enable legacy V2V PCAP output", enablePcap);
  cmd.AddValue ("sim-time", "Simulation time [s]", simTime);
  cmd.AddValue ("baseline", "PRR baseline [m]", baselinePrR);
  cmd.AddValue ("tx-power", "Tx power [dBm]", txPower);
  cmd.AddValue ("switch-cbr", "CBR threshold to start MEC-assisted NR operation", switchCbr);
  cmd.AddValue ("release-cbr", "CBR threshold to release MEC-assisted NR operation", releaseCbr);
  cmd.AddValue ("route-check-interval", "Route control interval [s]", routeCheckInterval);
  cmd.AddValue ("cbr-window-ms", "MetricSupervisor CBR window [ms]", cbrWindowMs);
  cmd.AddValue ("cbr-alpha", "MetricSupervisor CBR exponential average alpha", cbrAlpha);
  cmd.AddValue ("cbr-log", "CSV file for MetricSupervisor CBR values", cbrLogPath);
  cmd.AddValue ("route-log", "CSV file for route switching events", routeLogPath);
  cmd.AddValue ("observation-log", "CSV file for CBR/PRR/loss/route-count time series", observationLogPath);
  cmd.AddValue ("cpm-input-diag-log",
                "Optional CSV file for CPM input diagnostics visible from the example",
                cpmInputDiagLogPath);
  cmd.AddValue ("rsu-prediction-log",
                "CSV file for per-RSU measured/predicted CBR time series",
                rsuPredictionLogPath);
  cmd.AddValue ("summary-csv", "CSV file for one-line thesis summary metrics", summaryCsvPath);
  cmd.AddValue ("observation-log-interval", "Observation CSV write interval [s]", observationLogInterval);
  cmd.AddValue ("thesis-eval-interval", "Evaluation interval for thesis 4.3 metrics [s]", thesisEvalInterval);
  cmd.AddValue ("thesis-eval-start-min-vehicles",
                "Start thesis metric accumulation after this vehicle-count threshold; 0 starts immediately",
                thesisEvalStartMinVehicles);
  cmd.AddValue ("thesis-eval-start-use-all-vehicles",
                "If true, the thesis evaluation start threshold counts all active SUMO vehicles; otherwise it counts communication vehicles only",
                thesisEvalStartUseAllVehicles);
  cmd.AddValue ("hold-traffic-until-eval-start",
                "Keep inserted SUMO vehicles stopped until the evaluation vehicle threshold is reached",
                holdTrafficUntilEvaluationStart);
  cmd.AddValue ("thesis-eval-warmup-seconds",
                "Delay between traffic release and thesis metric accumulation [s]",
                thesisEvalWarmupSeconds);
  cmd.AddValue ("sumo-folder", "Folder containing SUMO mobility trace", sumoFolder);
  cmd.AddValue ("mob-trace", "SUMO route file name", mobTrace);
  cmd.AddValue ("sumo-config", "SUMO configuration file", sumoConfig);
  cmd.AddValue ("sumo-wait-socket", "Seconds to wait for SUMO/TraCI socket startup", sumoWaitForSocket);
  cmd.AddValue ("sumo-sync-interval", "SUMO/ns-3 synchronization interval [s]", sumoSyncInterval);
  cmd.AddValue ("sumo-port", "TCP port used by SUMO/TraCI", sumoPort);
  cmd.AddValue ("sumo-seed", "SUMO random seed", sumoSeed);
  cmd.AddValue ("rng-run", "ns-3 RNG run number", rngRun);
  cmd.AddValue ("sumo-extra-options", "Additional command line options passed to SUMO", sumoAdditionalOptions);
  cmd.AddValue ("nr-sensing", "Enable NR-V2X sidelink sensing", enableNrSensing);
  cmd.AddValue ("nr-channel-randomness", "Enable NR channel randomness/shadowing", enableChannelRandomness);
  cmd.AddValue ("nr-mcs", "Fixed NR sidelink MCS", mcs);
  cmd.Parse (argc, argv);
  RngSeedManager::SetRun (rngRun);

  if (method == "no-control")
    {
      enablePredictiveRmr = false;
      enableRouteControl = false;
    }
  else if (method == "reactive-rmr")
    {
      enableReactiveRmr = true;
      enablePredictiveRmr = false;
      enableRouteControl = false;
    }
  else if (method == "predictive-rmr-v2v")
    {
      enableReactiveRmr = true;
      enablePredictiveRmr = true;
      enableRouteControl = false;
    }
  else if (method == "predictive-rmr-v2n2v")
    {
      enableReactiveRmr = true;
      enablePredictiveRmr = true;
      enableRouteControl = false;
      enableMecV2n2v = true;
      enableMecRouteControl = true;
    }
  else if (method == "v2n2v-only")
    {
      enableReactiveRmr = false;
      enablePredictiveRmr = false;
      enableRouteControl = false;
      enableMecV2n2v = true;
      enableMecRouteControl = false;
      enableMecOnly = true;
    }
  else if (method == "hybrid-v2v-v2n2v")
    {
      enablePredictiveRmr = false;
      enableRouteControl = false;
      enableMecV2n2v = true;
      enableMecRouteControl = true;
    }
  else if (method == "cbr-route-nr-sidelink")
    {
      enableRouteControl = true;
    }
  else if (method != "legacy")
    {
      NS_FATAL_ERROR ("Unsupported method: " << method);
    }

  if (enablePredictiveRmr)
    {
      enableReactiveRmr = true;
      enableTrafficFlowRsuPredictor = true;
    }
  g_enablePredictiveRmrControl = enablePredictiveRmr;

  if (releaseCbr > switchCbr)
    {
      NS_FATAL_ERROR ("release-cbr must be lower than or equal to switch-cbr");
    }
  if (enableMecRouteControl && hybridCbrMax < switchCbr)
    {
      NS_FATAL_ERROR ("hybrid-cbr-max must be greater than or equal to switch-cbr");
    }
  if (hybridLowV2vMinProbability < 0.0 || hybridLowV2vMinProbability > 1.0)
    {
      NS_FATAL_ERROR ("hybrid-low-v2v-min-prob must be within [0, 1]");
    }
  if (hybridLowV2vAlpha < 0.0)
    {
      NS_FATAL_ERROR ("hybrid-low-v2v-alpha must be greater than or equal to 0");
    }
  if (priorityDistanceMeters < 0.0)
    {
      NS_FATAL_ERROR ("priority-distance must be greater than or equal to 0");
    }
  if (priorityClosingSpeedMps < 0.0)
    {
      NS_FATAL_ERROR ("priority-closing-speed must be greater than or equal to 0");
    }
  if (priorityTtcSeconds < 0.0)
    {
      NS_FATAL_ERROR ("priority-ttc must be greater than or equal to 0");
    }
  if (sensorRangeMeters < 0.0)
    {
      NS_FATAL_ERROR ("sensor-range must be greater than or equal to 0");
    }
  if (cpmRecognitionTtlSeconds <= 0.0)
    {
      NS_FATAL_ERROR ("cpm-recognition-ttl must be greater than 0");
    }
  if (highPriorityCpmRecognitionTtlSeconds <= 0.0)
    {
      NS_FATAL_ERROR ("high-priority-cpm-recognition-ttl must be greater than 0");
    }
  if (lowPriorityCpmRecognitionTtlSeconds <= 0.0)
    {
      NS_FATAL_ERROR ("low-priority-cpm-recognition-ttl must be greater than 0");
    }
  if (orrRangeMeters <= 0.0)
    {
      NS_FATAL_ERROR ("orr-range must be greater than 0");
    }
  if (sensorExternalEventProbability < 0.0 || sensorExternalEventProbability > 1.0)
    {
      NS_FATAL_ERROR ("sensor-external-event-probability must be in [0,1]");
    }
  if (sensorExternalEventActivationRangeMeters <= orrRangeMeters)
    {
      NS_FATAL_ERROR (
          "sensor-external-event-activation-range must be greater than orr-range");
    }
  if (observationLogInterval <= 0.0)
    {
      NS_FATAL_ERROR ("observation-log-interval must be greater than 0");
    }
  if (thesisEvalInterval <= 0.0)
    {
      NS_FATAL_ERROR ("thesis-eval-interval must be greater than 0");
    }
  g_thesisEvalStartMinVehicles = thesisEvalStartMinVehicles;
  g_thesisEvalStartUseAllVehicles = thesisEvalStartUseAllVehicles;
  g_holdTrafficUntilEvaluationStart =
      holdTrafficUntilEvaluationStart && thesisEvalStartMinVehicles > 0;
  g_thesisEvalWarmupSeconds = std::max (0.0, thesisEvalWarmupSeconds);
  if (sumoSyncInterval <= 0.0)
    {
      NS_FATAL_ERROR ("sumo-sync-interval must be greater than 0");
    }
  if (dsrcInterferenceIntervalMs <= 0.0)
    {
      NS_FATAL_ERROR ("dsrc-interference-interval-ms must be greater than 0");
    }
  if (dsrcInterferencePacketSize == 0)
    {
      NS_FATAL_ERROR ("dsrc-interference-size must be greater than 0");
    }
  if (mecIdealAoiFilterThresholdMs <= 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-aoi-filter-threshold-ms must be greater than 0");
    }
  if (mecBackgroundPacketSizeBytes == 0)
    {
      NS_FATAL_ERROR ("mec-bg-size must be greater than 0");
    }
  if (mecBackgroundIntervalMs <= 0.0)
    {
      NS_FATAL_ERROR ("mec-bg-interval-ms must be greater than 0");
    }
  if (nrBgWavePeriodSeconds <= 0.0 || nrBgWaveSpeedMps <= 0.0 ||
      nrBgWaveMinFactor <= 0.0 || nrBgWaveAmplitude < 0.0 ||
      nrBgWaveAmplitude > 1.0 || nrBgWaveWidthMeters <= 0.0)
    {
      NS_FATAL_ERROR ("invalid NR moving-wave background parameters");
    }
  g_nrBgMovingWaveForLog = nrBgMovingWave;
  g_nrBgWaveSpeedMpsForLog = nrBgWaveSpeedMps;
  g_nrBgWaveWidthMetersForLog = nrBgWaveWidthMeters;
  g_trafficFlowRoadLengthMetersForLog = trafficFlowRoadLengthMeters;
  if (enableDsrcInterference && !dsrcInterferencePerVehicle && dsrcInterferenceNodeCount < 2)
    {
      NS_FATAL_ERROR ("dsrc-interference-nodes must be at least 2 for unicast traffic");
    }
  if (rmrCbrLow > rmrCbrHigh)
    {
      NS_FATAL_ERROR ("rmr-cbr-low must be lower than or equal to rmr-cbr-high");
    }
  if (rmrWindowMs == 0)
    {
      NS_FATAL_ERROR ("rmr-window-ms must be greater than 0");
    }
  if (predictionHorizonSeconds < 0.0)
    {
      NS_FATAL_ERROR ("prediction-horizon must be greater than or equal to 0");
    }
  if (trafficFlowRoadLengthMeters <= 0.0)
    {
      NS_FATAL_ERROR ("traffic-flow-road-length must be greater than 0");
    }
  if (rsuI2vRangeMeters <= 0.0)
    {
      NS_FATAL_ERROR ("rsu-i2v-range must be greater than 0");
    }
  if (fixedRsuCbrMode != "disabled" && fixedRsuCbrMode != "current" &&
      fixedRsuCbrMode != "next")
    {
      NS_FATAL_ERROR ("fixed-rsu-cbr-mode must be disabled, current, or next");
    }
  if (trafficFlowTopology != "loop" && trafficFlowTopology != "straight")
    {
      NS_FATAL_ERROR ("traffic-flow-topology must be loop or straight");
    }
  if (trafficFlowMessageRateHz < 0.0)
    {
      NS_FATAL_ERROR ("traffic-flow-message-rate must be greater than or equal to 0");
    }
  if (trafficFlowAvgPacketSizeBytes <= 0.0)
    {
      NS_FATAL_ERROR ("traffic-flow-avg-packet-size must be greater than 0");
    }
  if (trafficFlowChannelRateMbps <= 0.0)
    {
      NS_FATAL_ERROR ("traffic-flow-channel-rate-mbps must be greater than 0");
    }
  if (mecBackhaulDelayMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-backhaul-delay-ms must be greater than or equal to 0");
    }
  if (mecProcessingDelayMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-processing-delay-ms must be greater than or equal to 0");
    }
  if (mecForwardRangeMeters < 0.0)
    {
      NS_FATAL_ERROR ("mec-forward-range must be greater than or equal to 0");
    }
  if (mecIdealLatencyMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-ms must be greater than or equal to 0");
    }
  if (mecIdealDelayModel != "empirical" && mecIdealDelayModel != "normal")
    {
      NS_FATAL_ERROR ("mec-ideal-delay-model must be empirical or normal");
    }
  if (mecIdealLatencyStddevMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-stddev-ms must be greater than or equal to 0");
    }
  if (mecIdealLatencyMinMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-min-ms must be greater than or equal to 0");
    }
  if (mecIdealLatencyP90Ms < mecIdealLatencyMinMs)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-p90-ms must be greater than or equal to min");
    }
  if (mecIdealLatencyP99Ms < mecIdealLatencyP90Ms)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-p99-ms must be greater than or equal to p90");
    }
  if (mecIdealLatencyMaxMs < mecIdealLatencyMinMs)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-max-ms must be greater than or equal to min");
    }
  if (mecIdealLatencyMaxMs < mecIdealLatencyP99Ms)
    {
      NS_FATAL_ERROR ("mec-ideal-latency-max-ms must be greater than or equal to p99");
    }
  if (mecIdealCpmIntervalMs <= 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-cpm-interval-ms must be greater than 0");
    }
  if (mecIdealPacketSizeBytes == 0)
    {
      NS_FATAL_ERROR ("mec-ideal-packet-size must be greater than 0");
    }
  if (mecIdealDownlinkPdr < 0.0 || mecIdealDownlinkPdr > 1.0)
    {
      NS_FATAL_ERROR ("mec-ideal-dl-pdr must be within [0, 1]");
    }
  if (mecIdealBandwidthMHz <= 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-bandwidth-mhz must be greater than 0");
    }
  if (mecIdealSpectralEfficiencyBpsHz <= 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-spectral-efficiency-bpshz must be greater than 0");
    }
  if (mecIdealMaxQueueDelayMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-max-queue-delay-ms must be greater than or equal to 0");
    }
  if (mecIdealUlFirstLossRate < 0.0 || mecIdealUlFirstLossRate > 1.0)
    {
      NS_FATAL_ERROR ("mec-ideal-ul-first-loss-rate must be within [0, 1]");
    }
  if (mecIdealDlFirstLossRate < 0.0 || mecIdealDlFirstLossRate > 1.0)
    {
      NS_FATAL_ERROR ("mec-ideal-dl-first-loss-rate must be within [0, 1]");
    }
  if (mecIdealRetxSuccessProbability < 0.0 || mecIdealRetxSuccessProbability > 1.0)
    {
      NS_FATAL_ERROR ("mec-ideal-retx-success-prob must be within [0, 1]");
    }
  if (mecIdealRetxDelayMs < 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-retx-delay-ms must be greater than or equal to 0");
    }
  if (mecAdaptiveLowMaxProbability < 0.0 || mecAdaptiveLowMaxProbability > 1.0)
    {
      NS_FATAL_ERROR ("mec-adaptive-low-max-prob must be within [0, 1]");
    }
  if (mecServerPort == mecVehiclePort)
    {
      NS_FATAL_ERROR ("mec-server-port and mec-vehicle-port must be different");
    }
  if (enableMecRouteControl && !enableMecV2n2v)
    {
      NS_FATAL_ERROR ("MEC route control requires MEC V2N2V to be enabled");
    }
  const bool enableNrSidelinkV2v = true;
  if (mecForwardRangeMeters == 0.0)
    {
      mecForwardRangeMeters = baselinePrR;
    }
  g_mecForwardRangeMeters = mecForwardRangeMeters;
  if (mecSrsPeriodicity != 2 && mecSrsPeriodicity != 5 && mecSrsPeriodicity != 10 &&
      mecSrsPeriodicity != 20 && mecSrsPeriodicity != 40 && mecSrsPeriodicity != 80 &&
      mecSrsPeriodicity != 160 && mecSrsPeriodicity != 320)
    {
      NS_FATAL_ERROR ("mec-srs-periodicity must be one of 2,5,10,20,40,80,160,320");
    }

  g_predictiveRmrConfig.predictionHorizonSeconds = predictionHorizonSeconds;
  g_predictiveRmrConfig.cpmSizeNormBytes = predictorCpmSizeNormBytes;
  g_predictiveRmrConfig.cpmTxRateNorm = predictorCpmTxRateNorm;
  g_predictiveRmrConfig.activeVehicleNorm = predictorActiveVehicleNorm;
  g_predictiveRmrConfig.trafficFlowRoadLengthMeters = trafficFlowRoadLengthMeters;
  g_trafficFlowIsLoop = trafficFlowTopology == "loop";
  g_predictiveRmrConfig.trafficFlowMessageRateHz = trafficFlowMessageRateHz;
  g_predictiveRmrConfig.trafficFlowAvgPacketSizeBytes = trafficFlowAvgPacketSizeBytes;
  g_predictiveRmrConfig.rsuPassivePdrLowCbr = Clamp01 (rsuPassivePdrLowCbr);
  g_predictiveRmrConfig.rsuPassivePdrMidCbr = Clamp01 (rsuPassivePdrMidCbr);
  g_predictiveRmrConfig.rsuPassiveSaturationCbr = Clamp01 (rsuPassiveSaturationCbr);
  g_predictiveRmrConfig.rsuI2vRangeMeters = rsuI2vRangeMeters;
  g_fixedRsuCbrMode = fixedRsuCbrMode;
  g_predictiveRmrConfig.cpmSizeWeight = predictorCpmSizeWeight;
  g_predictiveRmrConfig.cpmTxRateWeight = predictorCpmTxRateWeight;
  g_predictiveRmrConfig.activeVehicleWeight = predictorActiveVehicleWeight;
  g_predictiveChannelRateBps = trafficFlowChannelRateMbps * 1e6;
  g_hybridRouteConfig.switchCbr = switchCbr;
  g_hybridRouteConfig.releaseCbr = releaseCbr;
  g_hybridRouteConfig.maxCbr = hybridCbrMax;
  g_hybridRouteConfig.lowV2vMinProbability = hybridLowV2vMinProbability;
  g_hybridRouteConfig.lowV2vAlpha = hybridLowV2vAlpha;
  g_hybridRouteConfig.highPriorityDualTx = hybridHighPriorityDualTx;
  if (mecDuplicateRecovery && mecRecoveryPolicyName == "staged")
    {
      mecRecoveryPolicyName = "duplicate-always";
    }
  g_hybridRouteConfig.mecRecoveryPolicy = ParseMecRecoveryPolicy (mecRecoveryPolicyName);
  g_hybridRouteConfig.mecMinHoldTimeSeconds = mecMinHoldTimeSeconds;
  g_useIdealMecLink = mecIdealLink;
  g_mecObjectPolicy = ParseMecObjectPolicy (mecObjectPolicyName);
  g_mecAdaptiveLowMaxProbability = mecAdaptiveLowMaxProbability;
  g_idealMecLatency = MilliSeconds (mecIdealLatencyMs);
  g_idealMecDelayModel = mecIdealDelayModel;
  g_idealMecLatencyMeanMs = mecIdealLatencyMs;
  g_idealMecLatencyStddevMs = mecIdealLatencyStddevMs;
  g_idealMecLatencyMinMs = mecIdealLatencyMinMs;
  g_idealMecLatencyP90Ms = mecIdealLatencyP90Ms;
  g_idealMecLatencyP99Ms = mecIdealLatencyP99Ms;
  g_idealMecLatencyMaxMs = mecIdealLatencyMaxMs;
  g_idealMecLatencyRv = CreateObject<NormalRandomVariable> ();
  g_idealMecLatencyRv->SetAttribute ("Mean", DoubleValue (g_idealMecLatencyMeanMs));
  g_idealMecLatencyRv->SetAttribute ("Variance",
                                     DoubleValue (g_idealMecLatencyStddevMs *
                                                  g_idealMecLatencyStddevMs));
  g_idealMecLatencyUniformRv = CreateObject<UniformRandomVariable> ();
  g_idealMecLatencyUniformRv->SetAttribute ("Min", DoubleValue (0.0));
  g_idealMecLatencyUniformRv->SetAttribute ("Max", DoubleValue (1.0));
  g_idealMecLatencyUniformRv->SetStream (hybridRoutingStream + 3);
  g_idealMecInterval = MilliSeconds (mecIdealCpmIntervalMs);
  g_idealMecPacketSizeBytes = mecIdealPacketSizeBytes;
  g_idealMecDownlinkPdr = mecIdealDownlinkPdr;
  g_idealMecCapacityModel = mecIdealCapacityModel;
  g_idealMecBandwidthMHz = mecIdealBandwidthMHz;
  g_idealMecSpectralEfficiencyBpsHz = mecIdealSpectralEfficiencyBpsHz;
  g_idealMecUplinkCapacityMbps =
      g_idealMecBandwidthMHz * g_idealMecSpectralEfficiencyBpsHz;
  g_idealMecDownlinkCapacityMbps =
      g_idealMecBandwidthMHz * g_idealMecSpectralEfficiencyBpsHz;
  g_idealMecMaxQueueDelayMs = mecIdealMaxQueueDelayMs;
  g_idealMecUlFirstLossRate = mecIdealUlFirstLossRate;
  g_idealMecDlFirstLossRate = mecIdealDlFirstLossRate;
  g_idealMecRetxSuccessProbability = mecIdealRetxSuccessProbability;
  g_idealMecMaxRetransmissions = mecIdealMaxRetransmissions;
  g_idealMecRetxDelayMs = mecIdealRetxDelayMs;
  g_idealMecAoiFilter = mecIdealAoiFilter;
  g_idealMecAoiFilterThresholdMs = mecIdealAoiFilterThresholdMs;
  g_mecBackgroundLoad = mecBackgroundLoad;
  g_mecBackgroundPerVehicle = mecBackgroundPerVehicle;
  g_mecBackgroundPacketSizeBytes = mecBackgroundPacketSizeBytes;
  g_mecBackgroundInterval = MilliSeconds (mecBackgroundIntervalMs);
  g_maxCommunicationVehicles = maxCommunicationVehicles;
  g_priorityDistanceThresholdMeters = priorityDistanceMeters;
  g_priorityClosingSpeedThresholdMps = priorityClosingSpeedMps;
  g_priorityTtcThresholdSeconds = priorityTtcSeconds;
  g_sensorRangeMeters = sensorRangeMeters;
  if (cpmRecognitionTtlSeconds != 1.0)
    {
      highPriorityCpmRecognitionTtlSeconds = cpmRecognitionTtlSeconds;
      lowPriorityCpmRecognitionTtlSeconds = cpmRecognitionTtlSeconds;
    }
  g_highPriorityCpmRecognitionTtlSeconds = highPriorityCpmRecognitionTtlSeconds;
  g_lowPriorityCpmRecognitionTtlSeconds = lowPriorityCpmRecognitionTtlSeconds;
  g_orrRangeMeters = orrRangeMeters;
  g_sensorExternalEventsEnabled = sensorExternalEventsEnabled;
  g_sensorExternalEventProbability = sensorExternalEventProbability;
  g_sensorExternalEventActivationRangeMeters = sensorExternalEventActivationRangeMeters;
  g_dsrcInterferencePerVehicle = dsrcInterferencePerVehicle;
  if (enableDsrcInterference && dsrcInterferenceIntervalMs > 0.0 && dsrcDccBitRateMbps > 0.0)
    {
      const double nodeOfferedBitsPerSecond =
          static_cast<double> (dsrcInterferencePacketSize) * 8.0 /
          (dsrcInterferenceIntervalMs / 1000.0);
      g_dsrcInterferenceNodeOfferedCbr =
          nodeOfferedBitsPerSecond / (dsrcDccBitRateMbps * 1e6);
      g_dsrcInterferenceOfferedCbr =
          dsrcInterferencePerVehicle
              ? 0.0
              : g_dsrcInterferenceNodeOfferedCbr *
                    static_cast<double> (dsrcInterferenceNodeCount);
    }
  else
    {
      g_dsrcInterferenceNodeOfferedCbr = 0.0;
      g_dsrcInterferenceOfferedCbr = 0.0;
    }
  g_hybridRouteRandom = CreateObject<UniformRandomVariable> ();
  g_hybridRouteRandom->SetAttribute ("Min", DoubleValue (0.0));
  g_hybridRouteRandom->SetAttribute ("Max", DoubleValue (1.0));
  g_hybridRouteRandom->SetStream (hybridRoutingStream);
  g_idealMecDownlinkRandom = CreateObject<UniformRandomVariable> ();
  g_idealMecDownlinkRandom->SetAttribute ("Min", DoubleValue (0.0));
  g_idealMecDownlinkRandom->SetAttribute ("Max", DoubleValue (1.0));
  g_idealMecDownlinkRandom->SetStream (hybridRoutingStream + 1);
  g_idealMecRadioRandom = CreateObject<UniformRandomVariable> ();
  g_idealMecRadioRandom->SetAttribute ("Min", DoubleValue (0.0));
  g_idealMecRadioRandom->SetAttribute ("Max", DoubleValue (1.0));
  g_idealMecRadioRandom->SetStream (hybridRoutingStream + 2);

  if (realtime)
    {
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    }

  xmlInitParser ();
  std::string routeFile = sumoFolder + mobTrace;
  xmlDocPtr rouXmlFile = xmlParseFile (routeFile.c_str ());
  if (rouXmlFile == nullptr)
    {
      NS_FATAL_ERROR ("Unable to parse SUMO route file: " << routeFile);
    }

  int numberOfNodes = XML_rou_count_vehicles (rouXmlFile);
  xmlFreeDoc (rouXmlFile);
  xmlCleanupParser ();

  if (numberOfNodes <= 0)
    {
      NS_FATAL_ERROR ("No vehicles found in SUMO route file: " << routeFile);
    }
  if (enableDsrcInterference &&
      dsrcInterferenceNodeCount > static_cast<uint32_t> (numberOfNodes))
    {
      NS_FATAL_ERROR ("dsrc-interference-nodes=" << dsrcInterferenceNodeCount
                                                 << " exceeds vehicle count "
                                                 << numberOfNodes);
    }
  std::vector<std::tuple<std::string, float, float>> mecRsuData;
  if (enableMecV2n2v || enablePredictiveRmr)
    {
      const std::string mecRsuPath = sumoFolder + mecRsuFile;
      std::ifstream mecRsuFileStream (mecRsuPath.c_str ());
      if (!mecRsuFileStream.is_open ())
        {
          if (enableMecV2n2v)
            {
              NS_FATAL_ERROR ("Unable to open MEC RSU/eNB station file: " << mecRsuPath);
            }
          std::cout << "Traffic-flow RSU predictor disabled: unable to open " << mecRsuPath
                    << std::endl;
        }
      else
        {
          mecRsuData = XML_poli_count_stations (mecRsuFileStream);
          if (mecRsuData.empty () && enableMecV2n2v)
            {
              NS_FATAL_ERROR ("No MEC RSU/eNB stations found in: " << mecRsuPath);
            }
          g_trafficFlowRsus.clear ();
          for (const auto& rsu : mecRsuData)
            {
              TrafficFlowRsuInfo info;
              info.id = std::get<0> (rsu);
              info.x = std::get<1> (rsu);
              info.y = std::get<2> (rsu);
              g_trafficFlowRsus.push_back (info);
            }
          std::sort (g_trafficFlowRsus.begin (),
                     g_trafficFlowRsus.end (),
                     [] (const TrafficFlowRsuInfo& left, const TrafficFlowRsuInfo& right) {
                       return left.x < right.x;
                     });
        }
    }

  std::cout << "Hybrid CBR route-control example" << std::endl;
  std::cout << "Vehicles=" << numberOfNodes
            << ", communication vehicles="
            << (g_maxCommunicationVehicles == 0
                    ? std::string ("all")
                    : std::to_string (g_maxCommunicationVehicles))
            << ", initial route=NR-V2X sidelink"
            << ", optional fallback route="
            << (enableMecV2n2v ? "MEC V2N2V over NR Uu/EPC"
                                : (enableNrSidelinkV2v ? "NR-V2X sidelink V2V"
                                                       : "disabled"))
            << std::endl;
  std::cout << "Method: " << method << std::endl;
  std::cout << "NR sidelink background traffic: "
            << (enableDsrcInterference ? "enabled" : "disabled");
  if (enableDsrcInterference)
    {
      std::cout << " nodes=" << dsrcInterferenceNodeCount
                << ", per-vehicle=" << (dsrcInterferencePerVehicle ? "enabled" : "disabled")
                << ", packet=" << dsrcInterferencePacketSize << " bytes"
                << ", interval=" << dsrcInterferenceIntervalMs << " ms";
    }
  std::cout << std::endl;
  std::cout << "Route thresholds: switch-cbr=" << switchCbr
            << ", release-cbr=" << releaseCbr
            << ", route-control=" << (enableRouteControl ? "enabled" : "disabled")
            << std::endl;
  std::cout << "Reactive RMR: " << (enableReactiveRmr ? "enabled" : "disabled");
  if (enableReactiveRmr)
    {
      std::cout << " cbr-low=" << rmrCbrLow << ", cbr-high=" << rmrCbrHigh
                << ", delete=" << rmrDeleteLow << "/" << rmrDeleteMiddle << "/"
                << rmrDeleteHigh;
    }
  std::cout << std::endl;
  std::cout << "Predictive RMR input: " << (enablePredictiveRmr ? "enabled" : "disabled");
  if (enablePredictiveRmr)
    {
      std::cout << " horizon=" << predictionHorizonSeconds << "s"
                << ", optional-feature-weights="
                << predictorCpmSizeWeight << "/" << predictorCpmTxRateWeight << "/"
                << predictorActiveVehicleWeight;
    }
  std::cout << std::endl;
  std::cout << "Priority metrics: high-distance=" << priorityDistanceMeters
            << " m, closing-speed=" << priorityClosingSpeedMps
            << " m/s, ttc=" << priorityTtcSeconds << " s, sensor-range="
            << sensorRangeMeters << " m, ORR range=" << orrRangeMeters << " m"
            << std::endl;
  std::cout << "Hybrid route policy: cbr-start=" << switchCbr
            << ", cbr-release=" << releaseCbr << ", cbr-max=" << hybridCbrMax
            << ", low-v2v-min-prob=" << hybridLowV2vMinProbability
            << ", low-v2v-alpha=" << hybridLowV2vAlpha
            << ", high-dual-tx=" << (hybridHighPriorityDualTx ? "enabled" : "disabled")
            << ", mec-recovery-policy="
            << MecRecoveryPolicyName (g_hybridRouteConfig.mecRecoveryPolicy)
            << ", mec-object-policy=" << mecObjectPolicyName
            << ", mec-min-hold-time=" << mecMinHoldTimeSeconds << " s"
            << std::endl;
  std::cout << "MEC V2N2V: " << (enableMecV2n2v ? "enabled" : "disabled");
  if (enableMecV2n2v && !g_useIdealMecLink)
    {
      std::cout << " rsu-file=" << mecRsuFile << ", gNBs=" << mecRsuData.size ()
                << ", backhaul-delay=" << mecBackhaulDelayMs << " ms"
                << ", processing-delay=" << mecProcessingDelayMs << " ms"
                << ", forward-range=" << mecForwardRangeMeters << " m";
    }
  std::cout << std::endl;
  std::cout << "Traffic-flow RSU sharing predictor: "
            << (enableTrafficFlowRsuPredictor && !g_trafficFlowRsus.empty () ? "enabled"
                                                                             : "disabled")
            << ", rsus=" << g_trafficFlowRsus.size () << std::endl;
  std::cout << "CSV logs: CBR=" << cbrLogPath << ", route=" << routeLogPath
            << ", observation=" << observationLogPath << ", summary=" << summaryCsvPath
            << std::endl;
  if (sumoGui)
    {
      const char* display = std::getenv ("DISPLAY");
      std::cout << "SUMO GUI enabled";
      if (display != nullptr && std::string (display).size () > 0)
        {
          std::cout << " (DISPLAY=" << display << ")";
        }
      else
        {
          std::cout << " (DISPLAY is not set)";
        }
      std::cout << std::endl;
    }
  std::cout << "NR-only migration mode: V2V CPM uses NR sidelink." << std::endl;

  NodeContainer vehicleNodes;
  vehicleNodes.Create (numberOfNodes);
  std::cout << "Setup progress: vehicle nodes created" << std::endl;

  MobilityHelper mobility;
  mobility.Install (vehicleNodes);
  std::cout << "Setup progress: mobility installed" << std::endl;

  Ptr<NrHelper> mecNrHelper;
  Ptr<NrPointToPointEpcHelper> mecEpcHelper;
  NodeContainer mecRemoteHostContainer;
  Ptr<Node> mecRemoteHost;
  Ipv4Address mecRemoteHostAddr;
  NodeContainer mecEnbNodes;
  NetDeviceContainer mecEnbDevices;
  NetDeviceContainer mecUeDevices;
  Ipv4InterfaceContainer mecUeIpIfaces;
  BandwidthPartInfoPtrVector mecBwps;

  if (enableMecV2n2v && !g_useIdealMecLink)
    {
      mecEpcHelper = CreateObject<NrPointToPointEpcHelper> ();
      Ptr<IdealBeamformingHelper> mecBeamformingHelper = CreateObject<IdealBeamformingHelper> ();
      mecNrHelper = CreateObject<NrHelper> ();
      mecNrHelper->SetBeamformingHelper (mecBeamformingHelper);
      mecNrHelper->SetEpcHelper (mecEpcHelper);

      Ptr<Node> pgw = mecEpcHelper->GetPgwNode ();
      mecRemoteHostContainer.Create (1);
      mecRemoteHost = mecRemoteHostContainer.Get (0);
      InternetStackHelper mecRemoteInternet;
      mecRemoteInternet.Install (mecRemoteHostContainer);

      PointToPointHelper mecBackhaul;
      mecBackhaul.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("10Gb/s")));
      mecBackhaul.SetDeviceAttribute ("Mtu", UintegerValue (1500));
      mecBackhaul.SetChannelAttribute ("Delay",
                                       TimeValue (MilliSeconds (mecBackhaulDelayMs)));
      NetDeviceContainer mecInternetDevices = mecBackhaul.Install (pgw, mecRemoteHost);
      Ipv4AddressHelper mecIpv4;
      mecIpv4.SetBase ("10.0.0.0", "255.0.0.0");
      Ipv4InterfaceContainer mecInternetIpIfaces = mecIpv4.Assign (mecInternetDevices);
      mecRemoteHostAddr = mecInternetIpIfaces.GetAddress (1);

      Ipv4StaticRoutingHelper ipv4RoutingHelper;
      Ptr<Ipv4StaticRouting> mecRemoteHostRouting =
          ipv4RoutingHelper.GetStaticRouting (mecRemoteHost->GetObject<Ipv4> ());
      mecRemoteHostRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"),
                                               Ipv4Mask ("255.0.0.0"),
                                               1);

      mecEnbNodes.Create (mecRsuData.size ());
      MobilityHelper mecEnbMobility;
      mecEnbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      mecEnbMobility.Install (mecEnbNodes);
      for (uint32_t i = 0; i < mecEnbNodes.GetN (); ++i)
        {
          Ptr<ConstantPositionMobilityModel> mob =
              mecEnbNodes.Get (i)->GetObject<ConstantPositionMobilityModel> ();
          mob->SetPosition (Vector (std::get<1> (mecRsuData[i]),
                                    std::get<2> (mecRsuData[i]),
                                    0.0));
        }

      CcBwpCreator mecCcBwpCreator;
      const uint8_t mecNumCcPerBand = 1;
      CcBwpCreator::SimpleOperationBandConf mecBandConf (3.5e9,
                                                          100e6,
                                                          mecNumCcPerBand,
                                                          BandwidthPartInfo::UMi_StreetCanyon);
      OperationBandInfo mecBand = mecCcBwpCreator.CreateOperationBandContiguousCc (mecBandConf);
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod",
                          TimeValue (MilliSeconds (channelUpdatePeriod)));
      mecNrHelper->SetChannelConditionModelAttribute ("UpdatePeriod",
                                                      TimeValue (MilliSeconds (channelUpdatePeriod)));
      mecNrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
      mecNrHelper->InitializeOperationBand (&mecBand);
      mecBwps = CcBwpCreator::GetAllBwps ({mecBand});
      mecNrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (4));
      mecNrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (8));
      mecNrHelper->SetGnbAntennaAttribute (
          "AntennaElement",
          PointerValue (CreateObject<IsotropicAntennaModel> ()));
      mecNrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));
      mecNrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
      mecNrHelper->SetUeAntennaAttribute (
          "AntennaElement",
          PointerValue (CreateObject<IsotropicAntennaModel> ()));
      mecNrHelper->SetGnbPhyAttribute ("Numerology", UintegerValue (numerologyBwpSl));
      mecNrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));
      mecEnbDevices = mecNrHelper->InstallGnbDevice (mecEnbNodes, mecBwps);
      for (auto it = mecEnbDevices.Begin (); it != mecEnbDevices.End (); ++it)
        {
          DynamicCast<NrGnbNetDevice> (*it)->UpdateConfig ();
        }
    }

  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  wifiPhy.SetPreambleDetectionModel ("ns3::ThresholdPreambleDetectionModel",
                                     "MinimumRssi",
                                     DoubleValue (sensitivity),
                                     "Threshold",
                                     DoubleValue (snrThreshold));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  channel->SetAttribute ("PropagationLossModel",
                         StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
  wifiPhy.SetChannel (channel);
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();
    }
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",
                                      StringValue (phyMode),
                                      "ControlMode",
                                      StringValue (phyMode),
                                      "NonUnicastMode",
                                      StringValue (phyMode));
  NetDeviceContainer dsrcDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, vehicleNodes);
  std::cout << "Setup progress: 802.11p devices installed" << std::endl;
  if (enablePcap)
    {
      wifiPhy.EnablePcap ("v2v-hybrid-cbr-80211p", dsrcDevices);
    }
  if (enableMecV2n2v && !g_useIdealMecLink)
    {
      mecUeDevices = mecNrHelper->InstallUeDevice (vehicleNodes, mecBwps);
      for (auto it = mecUeDevices.Begin (); it != mecUeDevices.End (); ++it)
        {
          DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
        }
    }

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));

  Ptr<NrPointToPointEpcHelper> nrEpcHelper;
  Ptr<NrHelper> nrHelper;
  Ptr<NrSlHelper> nrSlHelper;
  NetDeviceContainer nrDevices;

  if (enableNrSidelinkV2v)
    {
  std::cout << "Setup progress: NR sidelink setup begin" << std::endl;
  nrEpcHelper = CreateObject<NrPointToPointEpcHelper> ();
  nrHelper = CreateObject<NrHelper> ();
  nrHelper->SetEpcHelper (nrEpcHelper);

  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;
  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl,
                                                    bandwidthBandSl,
                                                    numCcPerBand,
                                                    BandwidthPartInfo::V2V_Highway);
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod",
                          TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod",
                                                   TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod",
                          TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod",
                                                   TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  nrHelper->InitializeOperationBand (&bandSl);
  BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps ({bandSl});
  std::cout << "Setup progress: NR operation band initialized" << std::endl;

  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement",
                                   PointerValue (CreateObject<IsotropicAntennaModel> ()));
  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold1", DoubleValue (sinrThreshold));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold2", DoubleValue (sinrThreshold));
  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableNrSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;
  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK",
                                               UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  nrDevices = nrHelper->InstallUeDevice (vehicleNodes, allBwps);
  std::cout << "Setup progress: NR UE devices installed" << std::endl;
  for (auto it = nrDevices.Begin (); it != nrDevices.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  nrSlHelper = CreateObject<NrSlHelper> ();
  nrSlHelper->SetEpcHelper (nrEpcHelper);
  nrSlHelper->SetSlErrorModel ("ns3::NrLteMiErrorModel");
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));
  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId ());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));
  nrSlHelper->PrepareUeForSidelink (nrDevices, bwpIdContainer);
  std::cout << "Setup progress: NR sidelink UEs prepared" << std::endl;

  Ptr<NrSlCommPreconfigResourcePoolFactory> poolFactory =
      Create<NrSlCommPreconfigResourcePoolFactory> ();
  std::vector<std::bitset<1>> slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "Failed to generate sidelink bitmap");
  poolFactory->SetSlTimeResources (slBitMapVector);
  poolFactory->SetSlSensingWindow (slSensingWindow);
  poolFactory->SetSlSelectionWindow (slSelectionWindow);
  poolFactory->SetSlFreqResourcePscch (10);
  poolFactory->SetSlSubchannelSize (slSubchannelSize);
  poolFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);

  LteRrcSap::SlResourcePoolConfigNr slResourcePoolConfig;
  slResourcePoolConfig.haveSlResourcePoolConfigNr = true;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolId;
  slResourcePoolId.id = 0;
  slResourcePoolConfig.slResourcePoolId = slResourcePoolId;
  slResourcePoolConfig.slResourcePool = poolFactory->CreatePool ();

  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfig;
  slBwpPoolConfig.slTxPoolSelectedNormal[slResourcePoolId.id] = slResourcePoolConfig;

  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  LteRrcSap::SlBwpConfigCommonNr slBwpConfig;
  slBwpConfig.haveSlBwpGeneric = true;
  slBwpConfig.slBwpGeneric = slBwpGeneric;
  slBwpConfig.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfig.slBwpPoolConfigCommonNr = slBwpPoolConfig;

  LteRrcSap::SlFreqConfigCommonNr slFreqConfig;
  for (const auto& bwpId : bwpIdContainer)
    {
      slFreqConfig.slBwpList[bwpId] = slBwpConfig;
    }

  LteRrcSap::TddUlDlConfigCommon tddUlDlConfig;
  tddUlDlConfig.tddPattern = tddPattern;

  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneral;
  slPreconfigGeneral.slTddConfig = tddUlDlConfig;

  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0,
                       "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters[0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  LteRrcSap::SidelinkPreconfigNr slPreConfig;
  slPreConfig.slPreconfigGeneral = slPreconfigGeneral;
  slPreConfig.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfig.slPreconfigFreqInfoList[0] = slFreqConfig;
  nrSlHelper->InstallNrSlPreConfiguration (nrDevices, slPreConfig);
  std::cout << "Setup progress: NR sidelink preconfiguration installed" << std::endl;

  int64_t stream = 1;
  stream += nrHelper->AssignStreams (nrDevices, stream);
  nrSlHelper->AssignStreams (nrDevices, stream);
    }

  InternetStackHelper internet;
  internet.Install (vehicleNodes);
  std::cout << "Setup progress: internet stack installed" << std::endl;

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  if (enableMecV2n2v && !g_useIdealMecLink)
    {
      mecUeIpIfaces = mecEpcHelper->AssignUeIpv4Address (NetDeviceContainer (mecUeDevices));
      mecNrHelper->AttachToClosestEnb (mecUeDevices, mecEnbDevices);
      for (uint32_t i = 0; i < vehicleNodes.GetN (); ++i)
        {
          Ptr<Ipv4StaticRouting> ueStaticRouting =
              ipv4RoutingHelper.GetStaticRouting (vehicleNodes.Get (i)->GetObject<Ipv4> ());
          const int32_t interfaceIndex =
              vehicleNodes.Get (i)->GetObject<Ipv4> ()->GetInterfaceForDevice (mecUeDevices.Get (i));
          if (interfaceIndex < 0)
            {
              NS_FATAL_ERROR ("Unable to find LTE UE IPv4 interface for MEC route");
            }
          ueStaticRouting->SetDefaultRoute (mecEpcHelper->GetUeDefaultGatewayAddress (),
                                            static_cast<uint32_t> (interfaceIndex));
        }
    }
  if (enableNrSidelinkV2v)
    {
      Ipv4InterfaceContainer nrIpIfaces = nrEpcHelper->AssignUeIpv4Address (nrDevices);
      (void) nrIpIfaces;
      std::cout << "Setup progress: NR IPv4 addresses assigned" << std::endl;
      for (uint32_t i = 0; i < vehicleNodes.GetN (); ++i)
        {
          Ptr<Ipv4StaticRouting> ueStaticRouting =
              ipv4RoutingHelper.GetStaticRouting (vehicleNodes.Get (i)->GetObject<Ipv4> ());
          const int32_t interfaceIndex =
              vehicleNodes.Get (i)->GetObject<Ipv4> ()->GetInterfaceForDevice (nrDevices.Get (i));
          if (interfaceIndex < 0)
            {
              NS_FATAL_ERROR ("Unable to find NR UE IPv4 interface for sidelink route");
            }
          ueStaticRouting->AddNetworkRouteTo (Ipv4Address ("225.0.0.0"),
                                              Ipv4Mask ("255.0.0.0"),
                                              static_cast<uint32_t> (interfaceIndex));
          if (!enableMecV2n2v)
            {
              ueStaticRouting->SetDefaultRoute (nrEpcHelper->GetUeDefaultGatewayAddress (),
                                                static_cast<uint32_t> (interfaceIndex));
            }
        }
    }

  Ipv4Address nrGroupAddress ("225.0.0.0");
  uint32_t dstL2Id = 255;
  if (enableNrSidelinkV2v)
    {
      Ptr<LteSlTft> tft =
          Create<LteSlTft> (LteSlTft::Direction::TRANSMIT,
                            LteSlTft::CommType::GroupCast,
                            nrGroupAddress,
                            dstL2Id);
      nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, nrDevices, tft);
      tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE,
                              LteSlTft::CommType::GroupCast,
                              nrGroupAddress,
                              dstL2Id);
      nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, nrDevices, tft);
      std::cout << "Setup progress: NR sidelink bearers activated" << std::endl;
    }

  std::vector<Ptr<Socket>> dsrcInterferenceSockets;
  if (enableDsrcInterference)
    {
      const uint32_t interferenceSocketCount =
          dsrcInterferencePerVehicle ? numberOfNodes : dsrcInterferenceNodeCount;
      dsrcInterferenceSockets.reserve (interferenceSocketCount);
      for (uint32_t i = 0; i < interferenceSocketCount; ++i)
        {
          dsrcInterferenceSockets.push_back (
              CreateNrGroupcastTxSocket (vehicleNodes.Get (i), nrGroupAddress, nrSocketPort));
        }
    }

  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  g_sumoClient = sumoClient;
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumoConfig));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumoSyncInterval)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumoGui));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumoAdditionalOptions));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (sumoPort));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (sumoSeed));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (sumoWaitForSocket)));

  if (enableMecV2n2v && !g_useIdealMecLink)
    {
      for (uint32_t i = 0; i < mecRsuData.size (); ++i)
        {
          sumoClient->AddStation (std::get<0> (mecRsuData[i]),
                                  std::get<1> (mecRsuData[i]),
                                  std::get<2> (mecRsuData[i]),
                                  0.0,
                                  mecEnbNodes.Get (i));
        }
    }

  MetricSupervisor dsrcMetricsObj (baselinePrR);
  Ptr<MetricSupervisor> dsrcMetrics = &dsrcMetricsObj;
  dsrcMetrics->setTraCIClient (sumoClient);
  dsrcMetrics->setChannelTechnology ("80211p");
  dsrcMetrics->setCBRWindowValue (cbrWindowMs);
  dsrcMetrics->setCBRAlphaValue (cbrAlpha);
  dsrcMetrics->setSimulationTimeValue (simTime);
  dsrcMetrics->setNodeContainer (vehicleNodes);
  if (!enableNrSidelinkV2v && !cbrLogPath.empty ())
    {
      dsrcMetrics->writeCBRtoCSV (cbrLogPath);
      dsrcMetrics->enableCBRWriteToFile ();
    }
  if (!enableNrSidelinkV2v)
    {
      dsrcMetrics->startCheckCBR (numberOfNodes);
    }

  MetricSupervisor nrMetricsObj (baselinePrR);
  Ptr<MetricSupervisor> nrMetrics = &nrMetricsObj;
  nrMetrics->setTraCIClient (sumoClient);
  nrMetrics->setChannelTechnology ("Nr");
  nrMetrics->setCBRWindowValue (cbrWindowMs);
  nrMetrics->setCBRAlphaValue (cbrAlpha);
  nrMetrics->setSimulationTimeValue (simTime);
  nrMetrics->setNodeContainer (vehicleNodes);
  if (enableNrSidelinkV2v && !cbrLogPath.empty ())
    {
      nrMetrics->writeCBRtoCSV (cbrLogPath);
      nrMetrics->enableCBRWriteToFile ();
    }
  if (enableNrSidelinkV2v)
    {
      std::cout << "Setup progress: NR CBR monitor start" << std::endl;
      nrMetrics->startCheckCBR (numberOfNodes);
      std::cout << "Setup progress: NR CBR monitor ready" << std::endl;
    }

  Ptr<MetricSupervisor> channelMetrics = enableNrSidelinkV2v ? nrMetrics : dsrcMetrics;

  MetricSupervisor mecMetricsObj (baselinePrR);
  Ptr<MetricSupervisor> mecMetrics = &mecMetricsObj;
  mecMetrics->setTraCIClient (sumoClient);
  mecMetrics->setChannelTechnology ("Lte");

  Ptr<MecV2n2vForwarder> mecForwarder;
  if (enableMecV2n2v && !g_useIdealMecLink)
    {
      mecForwarder = CreateObject<MecV2n2vForwarder> ();
      mecForwarder->Configure (sumoClient,
                                mecServerPort,
                                mecVehiclePort,
                                mecForwardRangeMeters,
                                MilliSeconds (mecProcessingDelayMs));
      mecRemoteHost->AddApplication (mecForwarder);
      mecForwarder->SetStartTime (Seconds (0.0));
      mecForwarder->SetStopTime (Seconds (simTime) - MilliSeconds (1));
    }

  OpenRouteLog (routeLogPath);
  OpenObservationLog (observationLogPath);
  OpenCpmInputDiagLog (cpmInputDiagLogPath);
  std::cout << "Setup progress: CSV logs opened" << std::endl;
  if (!rsuPredictionLogPath.empty ())
    {
      g_rsuPredictionLog.open (rsuPredictionLogPath, std::ios::out);
      if (!g_rsuPredictionLog.is_open ())
        {
          NS_FATAL_ERROR ("Unable to open RSU prediction log file: " << rsuPredictionLogPath);
        }
      g_rsuPredictionLog
          << "time_s,rsu_id,rsu_x,rsu_y,measured_cbr,predicted_cbr,"
          << "peak_lead_time_s,local_vehicles,future_vehicles,i2v_vehicles" << std::endl;
    }

  STARTUP_FCN setupNewVehicle = [&] (std::string vehicleId,
                                     TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    (void) stationType;

    const uint64_t stationId = VehicleIdToStationId (vehicleId);
    const uint32_t nodeIndex = static_cast<uint32_t> (stationId - 1);
    if (nodeIndex >= vehicleNodes.GetN ())
      {
        NS_FATAL_ERROR ("SUMO vehicle id is outside the pre-created node range: " << vehicleId);
      }

    Ptr<Node> node = vehicleNodes.Get (nodeIndex);
    g_allVehicleNodes[vehicleId] = node;
    g_vehicleIdByStationId[stationId] = vehicleId;

    bool communicationVehicle = true;
    try
      {
        const std::string typeId = sumoClient->TraCIAPI::vehicle.getTypeID (vehicleId);
        communicationVehicle = typeId.rfind ("cav", 0) == 0;
      }
    catch (...)
      {
      }
    if (!communicationVehicle)
      {
        g_nonCavStationIds.insert (stationId);
        return node;
      }
    if (g_maxCommunicationVehicles > 0 && g_communicationVehicleCount >= g_maxCommunicationVehicles)
      {
        return node;
      }
    ++g_communicationVehicleCount;

    Ptr<Socket> dsrcSocket = GeoNet::createGNPacketSocket (node);
    dsrcSocket->SetPriority (userPriority);

    Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (dsrcDevices.Get (nodeIndex));
    wifiDevice->GetPhy ()->SetRxSensitivity (sensitivity);

    Ptr<Socket> nrSocket;
    if (enableNrSidelinkV2v)
      {
        nrSocket = CreateNrGroupcastSocket (node, nrGroupAddress, nrSocketPort);
        Ptr<NrUeNetDevice> nrDevice = DynamicCast<NrUeNetDevice> (nrDevices.Get (nodeIndex));
        nrDevice->GetPhy (0)->GetSpectrumPhy ()->GetSpectrumChannel ()->SetAttribute (
            "MaxLossDb",
            DoubleValue (120.0));
      }

    Ptr<BSContainer> dsrcContainer =
        CreateObject<BSContainer> (stationId, StationType_passengerCar, sumoClient, false, dsrcSocket);
    dsrcContainer->linkMetricSupervisor (dsrcMetrics);
    dsrcContainer->disablePRRSupervisorForGNBeacons ();
    dsrcContainer->addCAMRxCallback (std::bind (&ReceiveCAM,
                                                std::placeholders::_1,
                                                std::placeholders::_2,
                                                std::placeholders::_3,
                                                std::placeholders::_4,
                                                std::placeholders::_5));
    dsrcContainer->addCPMRxCallback (std::bind (&ReceiveCPM,
                                                std::placeholders::_1,
                                                std::placeholders::_2,
                                                std::placeholders::_3,
                                                std::placeholders::_4,
                                                std::placeholders::_5,
                                                ActiveRoute::DsrcV2v));
    dsrcContainer->setupContainer (true, false, false, sendCpm, false, false);
    if (enableReactiveRmr)
      {
        Ptr<CPBasicService> cp = dsrcContainer->getCPBasicService ();
        cp->setCbrAdaptiveRmr (true);
        cp->configureCbrAdaptiveRmr (rmrCbrLow,
                                     rmrCbrHigh,
                                     rmrDeleteLow,
                                     rmrDeleteMiddle,
                                     rmrDeleteHigh);
        cp->configureRmrWeights (rmrWeightFrequency, rmrWeightDynamics, rmrWeightDistance);
        cp->setRmrWindowMs (rmrWindowMs);
      }

    Ptr<DCC> dcc = CreateObject<DCC> ();
    if (enableDcc)
      {
        dcc->SetupDCC (vehicleId, dsrcMetrics, node, "adaptive", dccIntervalMs, dccTarget);
        dcc->setBitRate (dsrcDccBitRateMbps * 1e6);
        dsrcContainer->getGeoNet ()->setDCC (dcc);
        dcc->StartDCC ();
      }

    Ptr<BSContainer> nrContainer;
    if (enableNrSidelinkV2v)
      {
        nrContainer =
            CreateObject<BSContainer> (stationId, StationType_passengerCar, sumoClient, false, nrSocket);
        nrContainer->linkMetricSupervisor (nrMetrics);
        nrContainer->disablePRRSupervisorForGNBeacons ();
        nrContainer->addCAMRxCallback (std::bind (&ReceiveCAM,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2,
                                                  std::placeholders::_3,
                                                  std::placeholders::_4,
                                                  std::placeholders::_5));
        nrContainer->addCPMRxCallback (std::bind (&ReceiveCPM,
                                                  std::placeholders::_1,
                                                  std::placeholders::_2,
                                                  std::placeholders::_3,
                                                  std::placeholders::_4,
                                                  std::placeholders::_5,
                                                  ActiveRoute::NrSidelinkV2v));
        nrContainer->setupContainer (true, false, false, sendCpm, false, false);
        if (enableReactiveRmr)
          {
            Ptr<CPBasicService> cp = nrContainer->getCPBasicService ();
            cp->setCbrAdaptiveRmr (true);
            cp->configureCbrAdaptiveRmr (rmrCbrLow,
                                         rmrCbrHigh,
                                         rmrDeleteLow,
                                         rmrDeleteMiddle,
                                         rmrDeleteHigh);
            cp->configureRmrWeights (rmrWeightFrequency, rmrWeightDynamics, rmrWeightDistance);
            cp->setRmrWindowMs (rmrWindowMs);
          }
      }

    Ptr<BSContainer> mecContainer;
    if (enableMecV2n2v && !g_useIdealMecLink)
      {
        Ptr<Socket> mecSocket =
            CreateMecUnicastSocket (node, mecRemoteHostAddr, mecServerPort, mecVehiclePort);
        const Ipv4Address mecVehicleIp = mecUeIpIfaces.GetAddress (nodeIndex);
        g_mecVehicleIdByIp[mecVehicleIp.Get ()] = vehicleId;
        g_mecIpByVehicleId[vehicleId] = mecVehicleIp;

        mecContainer = CreateObject<BSContainer> (stationId,
                                                  StationType_passengerCar,
                                                  sumoClient,
                                                  false,
                                                  mecSocket);
        mecContainer->linkMetricSupervisor (mecMetrics);
        mecContainer->disablePRRSupervisorForGNBeacons ();
        mecContainer->addCAMRxCallback (std::bind (&ReceiveCAM,
                                                   std::placeholders::_1,
                                                   std::placeholders::_2,
                                                   std::placeholders::_3,
                                                   std::placeholders::_4,
                                                   std::placeholders::_5));
        mecContainer->addCPMRxCallback (std::bind (&ReceiveCPM,
                                                   std::placeholders::_1,
                                                   std::placeholders::_2,
                                                   std::placeholders::_3,
                                                   std::placeholders::_4,
                                                   std::placeholders::_5,
                                                   ActiveRoute::MecV2n2v));
        mecContainer->setupContainer (true, false, false, sendCpm, false, false);
        if (enableReactiveRmr)
          {
            Ptr<CPBasicService> cp = mecContainer->getCPBasicService ();
            cp->setCbrAdaptiveRmr (true);
            cp->configureCbrAdaptiveRmr (rmrCbrLow,
                                         rmrCbrHigh,
                                         rmrDeleteLow,
                                         rmrDeleteMiddle,
                                         rmrDeleteHigh);
            cp->configureRmrWeights (rmrWeightFrequency, rmrWeightDynamics, rmrWeightDistance);
            cp->setRmrWindowMs (rmrWindowMs);
          }
      }

    VehicleRuntime runtime;
    runtime.node = node;
    runtime.dsrcContainer = dsrcContainer;
    runtime.nrContainer = nrContainer;
    runtime.mecContainer = mecContainer;
    runtime.dcc = dcc;
    runtime.sendCpm = sendCpm;
    g_vehicleRuntime[vehicleId] = runtime;
    UpdateDsrcInterferenceOfferedCbr ();

    if (enableMecOnly)
      {
        ActivateRoute (vehicleId, ActiveRoute::MecV2n2v, -1.0, "initial_mec_v2n2v_only", true);
      }
    else
      {
        ActivateRoute (vehicleId, ActiveRoute::NrSidelinkV2v, -1.0, "initial_nr_sidelink_route", true);
      }
    if (enableDsrcInterference &&
        (dsrcInterferencePerVehicle || stationId <= dsrcInterferenceNodeCount))
      {
        const uint64_t backgroundGeneration =
            ++g_backgroundGenerationByVehicle[vehicleId];
        const uint32_t sourceIndex = static_cast<uint32_t> (stationId - 1);
        const Time stopTime =
            Seconds (dsrcInterferenceStop > 0.0 ? dsrcInterferenceStop : simTime);
        const MovingBackgroundConfig backgroundConfig = {
            dsrcInterferencePacketSize,
            MilliSeconds (dsrcInterferenceIntervalMs),
            stopTime,
            nrBgMovingWave,
            nrBgFixedZones,
            nrBgWavePeriodSeconds,
            nrBgWaveAmplitude,
            nrBgWaveSpeedMps,
            nrBgWaveMinFactor,
            nrBgWaveWidthMeters,
            trafficFlowRoadLengthMeters,
            backgroundGeneration};
        const double phaseStepMs =
            dsrcInterferenceIntervalMs /
            static_cast<double> (dsrcInterferencePerVehicle ? numberOfNodes
                                                            : dsrcInterferenceNodeCount);
        Simulator::ScheduleWithContext (node->GetId (),
                                        Seconds (dsrcInterferenceStart) +
                                            MilliSeconds (phaseStepMs * sourceIndex),
                                        &GenerateDsrcInterference,
                                        dsrcInterferenceSockets[sourceIndex],
                                        vehicleId,
                                        backgroundConfig);
      }
    return node;
  };

  SHUTDOWN_FCN shutdownVehicle = [] (Ptr<Node> exNode, std::string vehicleId)
  {
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel> ();
    mob->SetPosition (Vector (-1000.0 + (rand () % 25), 320.0 + (rand () % 25), 250.0));

    const uint64_t stationId = VehicleIdToStationId (vehicleId);
    ++g_backgroundGenerationByVehicle[vehicleId];
    g_allVehicleNodes.erase (vehicleId);
    g_vehicleIdByStationId.erase (stationId);
    g_nonCavStationIds.erase (stationId);
    g_priorityMotionHistory.erase (stationId);
    g_latestCpmRxByReceiver.erase (stationId);
    g_latestNrCpmRxByReceiver.erase (stationId);
    g_latestNrCpmGenerationTimeByReceiver.erase (stationId);
    g_latestMecCpmRxByReceiver.erase (stationId);
    for (auto& updates : g_latestMecCpmRxByReceiverAoiThreshold)
      {
        updates.erase (stationId);
      }
    g_latestMecCpmGenerationTimeByReceiver.erase (stationId);
    g_latestActualCpmObjectRxByReceiver.erase (stationId);
    g_cpmObjectUpdateHistoryByReceiver.erase (stationId);
    for (auto& entry : g_latestCpmRxByReceiver)
      {
        entry.second.erase (stationId);
      }
    for (auto& entry : g_latestNrCpmRxByReceiver)
      {
        entry.second.erase (stationId);
      }
    for (auto& entry : g_latestNrCpmGenerationTimeByReceiver)
      {
        entry.second.erase (stationId);
      }
    for (auto& entry : g_latestMecCpmRxByReceiver)
      {
        entry.second.erase (stationId);
      }
    for (auto& updates : g_latestMecCpmRxByReceiverAoiThreshold)
      {
        for (auto& entry : updates)
          {
            entry.second.erase (stationId);
          }
      }
    for (auto& entry : g_latestMecCpmGenerationTimeByReceiver)
      {
        entry.second.erase (stationId);
      }
    for (auto& entry : g_latestActualCpmObjectRxByReceiver)
      {
        entry.second.erase (stationId);
      }
    for (auto& entry : g_cpmObjectUpdateHistoryByReceiver)
      {
        entry.second.erase (stationId);
      }

    auto it = g_vehicleRuntime.find (vehicleId);
    if (it == g_vehicleRuntime.end ())
      {
        return;
      }

    it->second.dsrcContainer->cleanup ();
    if (it->second.nrContainer != nullptr)
      {
        it->second.nrContainer->cleanup ();
      }
    if (it->second.mecContainer != nullptr)
      {
        it->second.mecContainer->cleanup ();
      }
    auto ipIt = g_mecIpByVehicleId.find (vehicleId);
    if (ipIt != g_mecIpByVehicleId.end ())
      {
        g_mecVehicleIdByIp.erase (ipIt->second.Get ());
        g_mecIpByVehicleId.erase (ipIt);
      }
    g_vehicleRuntime.erase (it);
    UpdateDsrcInterferenceOfferedCbr ();
  };

  std::cout << "Setup progress: SUMO setup begin" << std::endl;
  sumoClient->SumoSetup (setupNewVehicle, shutdownVehicle);
  std::cout << "Setup progress: SUMO setup complete" << std::endl;
  if (sumoGui && !sumoScreenshotDir.empty () && sumoScreenshotIntervalSeconds > 0.0)
    {
      SumoScreenshotConfig screenshotConfig;
      screenshotConfig.dir = sumoScreenshotDir;
      screenshotConfig.viewId = sumoScreenshotView;
      screenshotConfig.centerX = sumoScreenshotCenterX;
      screenshotConfig.centerY = sumoScreenshotCenterY;
      screenshotConfig.spanX = sumoScreenshotSpanX;
      screenshotConfig.spanY = sumoScreenshotSpanY;
      screenshotConfig.width = sumoScreenshotWidth;
      screenshotConfig.height = sumoScreenshotHeight;
      screenshotConfig.intervalSeconds = sumoScreenshotIntervalSeconds;
      screenshotConfig.stopSeconds =
          sumoScreenshotStopSeconds > 0.0 ? sumoScreenshotStopSeconds : simTime;
      Simulator::Schedule (Seconds (std::max (0.0, sumoScreenshotStartSeconds)),
                           &CaptureSumoGuiScreenshot,
                           sumoClient,
                           screenshotConfig);
    }
  if (g_holdTrafficUntilEvaluationStart)
    {
      Simulator::Schedule (MilliSeconds (100), &HoldTrafficUntilEvaluationStart);
    }
  if (enableMecV2n2v && g_useIdealMecLink)
    {
      Simulator::Schedule (g_idealMecInterval, &GenerateIdealMecCpm);
      if (g_mecBackgroundLoad)
        {
          Simulator::Schedule (g_mecBackgroundInterval, &GenerateIdealMecBackgroundLoad);
        }
    }
  if (enableRouteControl)
    {
      Simulator::Schedule (Seconds (routeCheckInterval),
                           &CheckRoutesByCbr,
                           channelMetrics,
                           switchCbr,
                           releaseCbr,
                           Seconds (routeCheckInterval));
    }
  if (enableTrafficFlowRsuPredictor)
    {
      Simulator::Schedule (Seconds (routeCheckInterval),
                           &UpdatePredictiveRmrCbr,
                           channelMetrics,
                           Seconds (routeCheckInterval));
    }
  else if (enableReactiveRmr)
    {
      Simulator::Schedule (Seconds (routeCheckInterval),
                           &UpdateReactiveRmrCbr,
                           channelMetrics,
                           Seconds (routeCheckInterval));
    }
  if (enableMecRouteControl)
    {
      Simulator::Schedule (Seconds (routeCheckInterval) + MicroSeconds (1),
                           &CheckHybridRoutesByPredictedCbrToMec,
                           channelMetrics,
                           Seconds (routeCheckInterval));
    }
  Simulator::Schedule (Seconds (observationLogInterval),
                       &WriteObservationLog,
                       channelMetrics,
                       dsrcMetrics,
                       nrMetrics,
                       mecMetrics,
                       Seconds (observationLogInterval));
  Simulator::Schedule (Seconds (thesisEvalInterval),
                       &SampleThesisMetrics,
                       dsrcMetrics,
                       nrMetrics,
                       mecMetrics,
                       sumoClient,
                       baselinePrR,
                       Seconds (thesisEvalInterval));
  if (g_sensorExternalEventsEnabled)
    {
      Simulator::Schedule (MilliSeconds (100), &MonitorSensorExternalEvents);
    }

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  if (g_thesisEvaluationStarted || g_thesisEvalStartMinVehicles == 0)
    {
      AccumulateThesisPacketLossCounters (dsrcMetrics,
                                          nrMetrics,
                                          mecMetrics,
                                          sumoClient,
                                          baselinePrR,
                                          true);
    }

  std::cout << "Run terminated." << std::endl;
  std::cout << "CAM RX callbacks: " << g_camRx << std::endl;
  std::cout << "CPM RX callbacks: " << g_cpmRx << std::endl;
  const uint64_t thesisIdealCpmRx =
      g_thesisStats.dsrcIdealCpmRx + g_thesisStats.nrIdealCpmRx +
      g_thesisStats.mecIdealCpmRx;
  const uint64_t thesisTrueCpmRx =
      g_thesisStats.dsrcTrueCpmRx + g_thesisStats.nrTrueCpmRx +
      g_thesisStats.mecTrueCpmRx;
  const double recognitionRate = ThesisRecognitionRatePercent ();
  const double highPriorityObjectRecognitionRate =
      ThesisHighPriorityObjectRecognitionRatePercent ();
  const double lowPriorityObjectRecognitionRate =
      ThesisLowPriorityObjectRecognitionRatePercent ();
  const double sensorObjectRecognitionRate = ThesisSensorObjectRecognitionRatePercent ();
  const double sensorHighPriorityObjectRecognitionRate =
      ThesisSensorHighPriorityObjectRecognitionRatePercent ();
  const double sensorLowPriorityObjectRecognitionRate =
      ThesisSensorLowPriorityObjectRecognitionRatePercent ();
  const double cooperativeObjectRecognitionRate = ThesisCooperativeObjectRecognitionRatePercent ();
  const double packetLossRate = ThesisPacketLossRatePercent (thesisIdealCpmRx, thesisTrueCpmRx);
  const double highPdr =
      ThesisPacketDeliveryRatioPercent (g_thesisStats.highIdealCpmRx,
                                        g_thesisStats.highTrueCpmRx);
  const double lowPdr =
      ThesisPacketDeliveryRatioPercent (g_thesisStats.lowIdealCpmRx,
                                        g_thesisStats.lowTrueCpmRx);
  const double highPacketLoss =
      ThesisPacketLossRatePercent (g_thesisStats.highIdealCpmRx, g_thesisStats.highTrueCpmRx);
  const double lowPacketLoss =
      ThesisPacketLossRatePercent (g_thesisStats.lowIdealCpmRx, g_thesisStats.lowTrueCpmRx);
  const double dsrcRouteLoss =
      ThesisPacketLossRatePercent (g_thesisStats.dsrcIdealCpmRx, g_thesisStats.dsrcTrueCpmRx);
  const double nrRouteLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrUpdateExpected,
                                   g_thesisStats.nrUpdateSuccess);
  const double v2vRadioLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrIdealCpmRx, g_thesisStats.nrTrueCpmRx);
  const double v2vHighRadioLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrHighIdealCpmRx,
                                   g_thesisStats.nrHighTrueCpmRx);
  const double v2vLowRadioLoss =
      ThesisPacketLossRatePercent (g_thesisStats.nrLowIdealCpmRx, g_thesisStats.nrLowTrueCpmRx);
  const double mecRouteLoss =
      ThesisPacketLossRatePercent (g_thesisStats.mecUpdateExpected,
                                   g_thesisStats.mecUpdateSuccess);
  const double mecValidUpdateSuccessRate =
      PercentRate (g_thesisStats.mecUpdateSuccess, g_thesisStats.mecUpdateExpected);
  const double mecUlFinalLossRate =
      PercentRate (g_idealMecUlFinalLosses, g_mecUplinkPackets);
  const double mecDlFinalLossRate =
      PercentRate (g_idealMecDlFinalLosses, g_mecForwardedPackets);
  const double mecRadioFinalLossRate =
      PercentRate (g_idealMecUlFinalLosses + g_idealMecDlFinalLosses,
                   g_mecUplinkPackets + g_mecForwardedPackets);
  const double ttlViolationRate =
      PercentFromRatioSum (g_thesisStats.ttlViolationRatioSum,
                           g_thesisStats.ttlViolationVehicleSamples);
  const double neverReceivedRate =
      PercentFromRatioSum (g_thesisStats.neverReceivedRatioSum,
                           g_thesisStats.neverReceivedVehicleSamples);
  const double highTtlViolationRate =
      PercentFromRatioSum (g_thesisStats.highPriorityTtlViolationRatioSum,
                           g_thesisStats.highPriorityTtlViolationVehicleSamples);
  const double highNeverReceivedRate =
      PercentFromRatioSum (g_thesisStats.highPriorityNeverReceivedRatioSum,
                           g_thesisStats.highPriorityNeverReceivedVehicleSamples);
  const double lowTtlViolationRate =
      PercentFromRatioSum (g_thesisStats.lowPriorityTtlViolationRatioSum,
                           g_thesisStats.lowPriorityTtlViolationVehicleSamples);
  const double lowNeverReceivedRate =
      PercentFromRatioSum (g_thesisStats.lowPriorityNeverReceivedRatioSum,
                           g_thesisStats.lowPriorityNeverReceivedVehicleSamples);
  const uint64_t sensorExternalEventCompleted =
      g_sensorExternalEventRecognized + g_sensorExternalEventMissed;
  const uint64_t sensorExternalEventCensored =
      g_sensorExternalEventCount >= sensorExternalEventCompleted
          ? g_sensorExternalEventCount - sensorExternalEventCompleted
          : 0;
  const double sensorExternalEventRecognitionRate =
      sensorExternalEventCompleted > 0
          ? 100.0 * static_cast<double> (g_sensorExternalEventRecognized) /
                static_cast<double> (sensorExternalEventCompleted)
          : -1.0;
  const double sensorExternalEventDelayMeanMs =
      g_sensorExternalEventDelaysMs.empty ()
          ? -1.0
          : std::accumulate (g_sensorExternalEventDelaysMs.begin (),
                             g_sensorExternalEventDelaysMs.end (),
                             0.0) /
                static_cast<double> (g_sensorExternalEventDelaysMs.size ());
  const double sensorExternalEventDelayP50Ms =
      g_sensorExternalEventDelaysMs.empty ()
          ? -1.0
          : Percentile (g_sensorExternalEventDelaysMs, 50.0);
  const auto cpmType = MetricSupervisor::messageType_cpm;
  const double nrLatencyP50 = nrMetrics->getLatencyPercentile_messagetype (cpmType, 50.0);
  const double nrLatencyP90 = nrMetrics->getLatencyPercentile_messagetype (cpmType, 90.0);
  const double nrLatencyP95 = nrMetrics->getLatencyPercentile_messagetype (cpmType, 95.0);
  const double nrLatencyP99 = nrMetrics->getLatencyPercentile_messagetype (cpmType, 99.0);
  const uint64_t mecTxOverall =
      g_useIdealMecLink ? g_idealMecTx : mecMetrics->getNumberTx_overall ();
  const uint64_t mecRxOverall =
      g_useIdealMecLink ? g_idealMecRx : mecMetrics->getNumberRx_overall ();
  const double mecLatencyAvg =
      g_useIdealMecLink && !g_idealMecLatencySamplesMs.empty ()
          ? std::accumulate (g_idealMecLatencySamplesMs.begin (),
                             g_idealMecLatencySamplesMs.end (),
                             0.0) /
                static_cast<double> (g_idealMecLatencySamplesMs.size ())
          : mecMetrics->getAverageLatency_overall ();
  const double mecLatencyP50 =
      g_useIdealMecLink ? Percentile (g_idealMecLatencySamplesMs, 50.0)
                        : mecMetrics->getLatencyPercentile_messagetype (cpmType, 50.0);
  const double mecLatencyP90 =
      g_useIdealMecLink ? Percentile (g_idealMecLatencySamplesMs, 90.0)
                        : mecMetrics->getLatencyPercentile_messagetype (cpmType, 90.0);
  const double mecLatencyP95 =
      g_useIdealMecLink ? Percentile (g_idealMecLatencySamplesMs, 95.0)
                        : mecMetrics->getLatencyPercentile_messagetype (cpmType, 95.0);
  const double mecLatencyP99 =
      g_useIdealMecLink ? Percentile (g_idealMecLatencySamplesMs, 99.0)
                        : mecMetrics->getLatencyPercentile_messagetype (cpmType, 99.0);
  const double v2vAoiMeanMs = Mean (g_v2vAoiSamplesMs);
  const double v2vAoiP50Ms = Percentile (g_v2vAoiSamplesMs, 50.0);
  const double v2vAoiP90Ms = Percentile (g_v2vAoiSamplesMs, 90.0);
  const double v2vAoiP99Ms = Percentile (g_v2vAoiSamplesMs, 99.0);
  const double mecAoiMeanMs = Mean (g_idealMecAoiSamplesMs);
  const double mecAoiP50Ms = Percentile (g_idealMecAoiSamplesMs, 50.0);
  const double mecAoiP90Ms = Percentile (g_idealMecAoiSamplesMs, 90.0);
  const double mecAoiP99Ms = Percentile (g_idealMecAoiSamplesMs, 99.0);
  const double mecCapacityDelayMeanMs = Mean (g_idealMecCapacityDelaySamplesMs);
  const double mecCapacityDelayP90Ms = Percentile (g_idealMecCapacityDelaySamplesMs, 90.0);
  const double mecCapacityDelayP99Ms = Percentile (g_idealMecCapacityDelaySamplesMs, 99.0);
  const double mecRetxDelayMeanMs = Mean (g_idealMecRetxDelaySamplesMs);
  const double mecRetxDelayP90Ms = Percentile (g_idealMecRetxDelaySamplesMs, 90.0);
  const double mecUlBusyRatioMean = Mean (g_mecUlBusyRatioSamples);
  const double mecUlBusyRatioP90 = Percentile (g_mecUlBusyRatioSamples, 90.0);
  const double mecUlBusyRatioP99 = Percentile (g_mecUlBusyRatioSamples, 99.0);
  const double mecUlBusyRatioMax = Percentile (g_mecUlBusyRatioSamples, 100.0);
  const double mecDlBusyRatioMean = Mean (g_mecDlBusyRatioSamples);
  const double mecDlBusyRatioP90 = Percentile (g_mecDlBusyRatioSamples, 90.0);
  const double mecDlBusyRatioP99 = Percentile (g_mecDlBusyRatioSamples, 99.0);
  const double mecDlBusyRatioMax = Percentile (g_mecDlBusyRatioSamples, 100.0);
  const double effectiveChannelBusyRatio =
      static_cast<double> (channelMetrics->getAverageCBROverall ());
  std::cout << "Thesis 4.3 recognition rate (%): "
            << FormatThesisMetric (recognitionRate) << std::endl;
  std::cout << "Thesis 4.3 object recognition rate (%): "
            << FormatThesisMetric (recognitionRate) << std::endl;
  std::cout << "Thesis 4.3 high priority object recognition rate (%): "
            << FormatThesisMetric (highPriorityObjectRecognitionRate) << std::endl;
  std::cout << "Thesis 4.3 low priority object recognition rate (%): "
            << FormatThesisMetric (lowPriorityObjectRecognitionRate) << std::endl;
  std::cout << "Thesis 4.3 sensor object recognition rate (%): "
            << FormatThesisMetric (sensorObjectRecognitionRate) << std::endl;
  std::cout << "Thesis 4.3 sensor high priority object recognition rate (%): "
            << FormatThesisMetric (sensorHighPriorityObjectRecognitionRate) << std::endl;
  std::cout << "Thesis 4.3 sensor low priority object recognition rate (%): "
            << FormatThesisMetric (sensorLowPriorityObjectRecognitionRate) << std::endl;
  std::cout << "Thesis 4.3 cooperative object recognition rate (%): "
            << FormatThesisMetric (cooperativeObjectRecognitionRate) << std::endl;
  std::cout << "Thesis 4.3 ORR (%): " << FormatThesisMetric (recognitionRate)
            << std::endl;
  std::cout << "Thesis 4.3 CPM packet loss rate (%): "
            << FormatThesisMetric (packetLossRate) << std::endl;
  std::cout << "Thesis 4.3 CPM ideal/true RX: " << thesisIdealCpmRx << "/"
            << thesisTrueCpmRx << std::endl;
  std::cout << "Thesis 4.3 importance PDR (%): High="
            << FormatThesisMetric (highPdr) << ", Low=" << FormatThesisMetric (lowPdr)
            << std::endl;
  std::cout << "Thesis 4.3 importance packet loss (%): High="
            << FormatThesisMetric (highPacketLoss)
            << ", Low=" << FormatThesisMetric (lowPacketLoss) << std::endl;
  std::cout << "Thesis 4.3 importance ideal/true RX: High="
            << g_thesisStats.highIdealCpmRx << "/" << g_thesisStats.highTrueCpmRx
            << ", Low=" << g_thesisStats.lowIdealCpmRx << "/"
            << g_thesisStats.lowTrueCpmRx << std::endl;
  std::cout << "Thesis 4.3 update failure by route (%): Legacy V2V="
            << FormatThesisMetric (dsrcRouteLoss)
            << ", NR-V2X sidelink V2V=" << FormatThesisMetric (nrRouteLoss)
            << ", MEC V2N2V=" << FormatThesisMetric (mecRouteLoss)
            << std::endl;
  std::cout << "MEC V2N2V valid-update/radio final loss (%): fresh-update-loss="
            << FormatThesisMetric (mecRouteLoss)
            << ", valid-update-success=" << FormatThesisMetric (mecValidUpdateSuccessRate)
            << ", UL-final=" << FormatThesisMetric (mecUlFinalLossRate)
            << ", DL-final=" << FormatThesisMetric (mecDlFinalLossRate)
            << ", combined-final=" << FormatThesisMetric (mecRadioFinalLossRate)
            << std::endl;
  std::cout << "Legacy V2V average CBR: " << dsrcMetrics->getAverageCBROverall () << std::endl;
  std::cout << "Legacy V2V average PRR: " << dsrcMetrics->getAveragePRR_overall () << std::endl;
  std::cout << "Legacy V2V average packet loss: "
            << PacketLossFromPrr (dsrcMetrics->getAveragePRR_overall (),
                                  dsrcMetrics->getNumberTx_overall ())
            << std::endl;
  std::cout << "Legacy V2V TX/RX: " << dsrcMetrics->getNumberTx_overall () << "/"
            << dsrcMetrics->getNumberRx_overall () << std::endl;
  std::cout << "Legacy V2V average latency (ms): " << dsrcMetrics->getAverageLatency_overall ()
            << std::endl;
  std::cout << "NR-V2X sidelink V2V legacy packet PDR (%): "
            << FormatThesisMetric (
                   ThesisPacketDeliveryRatioPercent (g_thesisStats.nrIdealCpmRx,
                                                     g_thesisStats.nrTrueCpmRx))
            << std::endl;
  std::cout << "NR-V2X sidelink V2V AoI update failure (%): "
            << FormatThesisMetric (nrRouteLoss)
            << std::endl;
  std::cout << "NR-V2X sidelink V2V average latency (ms): "
            << nrMetrics->getAverageLatency_overall () << std::endl;
  std::cout << "NR-V2X sidelink V2V CPM latency percentiles (ms): p50="
            << nrLatencyP50 << ", p90=" << nrLatencyP90 << ", p95=" << nrLatencyP95
            << ", p99=" << nrLatencyP99 << std::endl;
  std::cout << "NR-V2X sidelink V2V AoI from TX timestamp (ms): mean="
            << v2vAoiMeanMs << ", p50=" << v2vAoiP50Ms << ", p90=" << v2vAoiP90Ms
            << ", p99=" << v2vAoiP99Ms << std::endl;
  std::cout << "NR-V2X sidelink V2V AoI <=200/300/400/500 ms (%): "
            << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[0],
                                                g_v2vAoiSamples))
            << "/"
            << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[1],
                                                g_v2vAoiSamples))
            << "/"
            << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[2],
                                                g_v2vAoiSamples))
            << "/"
            << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[3],
                                                g_v2vAoiSamples))
            << std::endl;
  std::cout << "MEC V2N2V packet delivery (%): "
            << FormatThesisMetric (
                   ThesisPacketDeliveryRatioPercent (g_thesisStats.mecIdealCpmRx,
                                                     g_thesisStats.mecTrueCpmRx))
            << std::endl;
  std::cout << "MEC V2N2V AoI update failure (%): "
            << FormatThesisMetric (mecRouteLoss)
            << std::endl;
  std::cout << "MEC V2N2V TX/RX: " << mecTxOverall << "/" << mecRxOverall << std::endl;
  std::cout << "MEC V2N2V average latency (ms): " << mecLatencyAvg << std::endl;
  std::cout << "MEC V2N2V CPM latency percentiles (ms): p50="
            << mecLatencyP50 << ", p90=" << mecLatencyP90 << ", p95=" << mecLatencyP95
            << ", p99=" << mecLatencyP99 << std::endl;
  std::cout << "MEC V2N2V AoI from CPM generation (ms): mean="
            << mecAoiMeanMs << ", p50=" << mecAoiP50Ms << ", p90=" << mecAoiP90Ms
            << ", p99=" << mecAoiP99Ms << std::endl;
  std::cout << "MEC V2N2V AoI threshold success (%): <=200="
            << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[0],
                                                g_idealMecAoiSamples))
            << ", <=300="
            << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[1],
                                                g_idealMecAoiSamples))
            << ", <=400="
            << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[2],
                                                g_idealMecAoiSamples))
            << ", <=500="
            << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[3],
                                                g_idealMecAoiSamples))
            << std::endl;
  std::cout << "MEC V2N2V capacity delay (ms): mean="
            << mecCapacityDelayMeanMs << ", p90=" << mecCapacityDelayP90Ms
            << ", p99=" << mecCapacityDelayP99Ms
            << ", queue-drops=" << g_idealMecCapacityQueueDrops << std::endl;
  std::cout << "MEC V2N2V radio loss/retransmission: UL first/final/recovered="
            << g_idealMecUlFirstLosses << "/" << g_idealMecUlFinalLosses << "/"
            << g_idealMecUlRetxRecovered << ", DL first/final/recovered="
            << g_idealMecDlFirstLosses << "/" << g_idealMecDlFinalLosses << "/"
            << g_idealMecDlRetxRecovered << ", retx-delay mean/p90(ms)="
            << mecRetxDelayMeanMs << "/" << mecRetxDelayP90Ms << std::endl;
  std::cout << "MEC uplink/forward/drop/no-receiver: " << g_mecUplinkPackets << "/"
            << g_mecForwardedPackets << "/" << g_mecForwardDrops << "/"
            << g_mecForwardNoReceiver << std::endl;
  std::cout << "NR background TX/drops/bytes: " << g_interferenceTx << "/"
            << g_interferenceDrops << "/" << g_interferenceBytes << std::endl;
  std::cout << "Sensor-external candidate/selected/events/recognized/missed/censored: "
            << g_sensorExternalEventCandidateVehicles << "/"
            << g_sensorExternalEventSelectedVehicles << "/" << g_sensorExternalEventCount << "/"
            << g_sensorExternalEventRecognized << "/" << g_sensorExternalEventMissed << "/"
            << sensorExternalEventCensored << std::endl;
  std::cout << "Sensor-external recognition (%), delay mean/p50 (ms): "
            << FormatThesisMetric (sensorExternalEventRecognitionRate) << ","
            << FormatThesisMetric (sensorExternalEventDelayMeanMs) << "/"
            << FormatThesisMetric (sensorExternalEventDelayP50Ms) << std::endl;

  if (!summaryCsvPath.empty ())
    {
      std::ofstream summaryCsv (summaryCsvPath, std::ios::out);
      if (!summaryCsv.is_open ())
        {
          NS_FATAL_ERROR ("Unable to open summary CSV file: " << summaryCsvPath);
        }
      summaryCsv
          << "method,object_recognition_rate,high_priority_object_recognition_rate,"
          << "low_priority_object_recognition_rate,sensor_object_recognition_rate,"
          << "sensor_high_priority_object_recognition_rate,"
          << "sensor_low_priority_object_recognition_rate,"
          << "cooperative_object_recognition_rate,recognition_rate,packet_loss_rate,ideal_rx,true_rx,"
          << "orr,high_pdr,low_pdr,high_packet_loss,low_packet_loss,"
          << "high_ideal_rx,high_true_rx,low_ideal_rx,low_true_rx,"
          << "legacy_route_loss,nr_route_loss,mec_route_loss,channel_busy_ratio,legacy_tx,legacy_rx,"
          << "v2v_radio_loss_rate,v2v_high_radio_loss_rate,v2v_low_radio_loss_rate,"
          << "v2v_update_failure_rate,"
          << "mec_fresh_update_loss_rate,mec_update_failure_rate,mec_valid_update_success_rate,"
          << "mec_ul_final_loss_rate,mec_dl_final_loss_rate,mec_radio_final_loss_rate,"
          << "nr_latency_ms,nr_latency_p50_ms,nr_latency_p90_ms,nr_latency_p95_ms,nr_latency_p99_ms,"
          << "mec_tx,mec_rx,mec_latency_ms,mec_latency_p50_ms,mec_latency_p90_ms,"
          << "mec_latency_p95_ms,mec_latency_p99_ms,interference_tx,interference_drops,"
          << "interference_bytes,mec_bg_ul_bytes,mec_bg_dl_bytes,"
          << "mec_aoi_filter,mec_aoi_filter_threshold_ms,"
          << "ttl_violation_rate,never_received_rate,"
          << "high_ttl_violation_rate,high_never_received_rate,"
          << "low_ttl_violation_rate,low_never_received_rate,"
          << "receiver_fresh_redundancy_200ms_mean,receiver_fresh_redundancy_500ms_mean,"
          << "receiver_fresh_redundancy_ge2_200ms_rate,"
          << "receiver_fresh_redundancy_ge2_500ms_rate,"
          << "receiver_high_fresh_redundancy_200ms_mean,"
          << "receiver_low_fresh_redundancy_200ms_mean,"
          << "receiver_rv_200ms_score,"
          << "receiver_rl_mean,receiver_rl_median,"
          << "receiver_rl_lt1_rate,receiver_rl_ge1_rate,receiver_rl_ge2_rate,"
          << "receiver_rv_delooz_mean,receiver_rv_delooz_median,"
          << "receiver_high_rl_mean,receiver_low_rl_mean,"
          << "rmr_deleted_eval_expected_total,rmr_deleted_eval_expected_high,"
          << "rmr_deleted_eval_expected_low,rmr_deleted_eval_unrecognized_total,"
          << "rmr_deleted_eval_unrecognized_high,rmr_deleted_eval_unrecognized_low,"
          << "rmr_deleted_eval_distance_mean_m,"
          << "rmr_deleted_eval_fresh_redundancy_ge2_200ms_rate,"
          << "rmr_deleted_eval_fresh_redundancy_ge2_500ms_rate,"
          << "rmr_deleted_eval_closing_speed_ge_threshold_rate,"
          << "rmr_deleted_eval_ttc_le_threshold_rate,"
          << "rmr_deleted_sender_eval_total,rmr_deleted_sender_eval_high,"
          << "rmr_deleted_sender_eval_low,rmr_deleted_sender_eval_distance_mean_m,"
          << "rmr_deleted_sender_eval_closing_speed_ge_threshold_rate,"
          << "rmr_deleted_sender_eval_ttc_le_threshold_rate,"
          << "sensor_external_event_candidate_vehicles,sensor_external_event_selected_vehicles,"
          << "sensor_external_event_count,sensor_external_event_recognized,"
          << "sensor_external_event_missed,sensor_external_event_censored,"
          << "sensor_external_event_recognition_rate,"
          << "sensor_external_event_delay_mean_ms,sensor_external_event_delay_p50_ms,"
          << "v2v_aoi_mean_ms,v2v_aoi_p50_ms,v2v_aoi_p90_ms,v2v_aoi_p99_ms,"
          << "v2v_aoi_le_200_ms_rate,v2v_aoi_le_300_ms_rate,"
          << "v2v_aoi_le_400_ms_rate,v2v_aoi_le_500_ms_rate,"
          << "v2v_aoi_violation_rate_200ms,v2v_aoi_violation_rate_300ms,"
          << "v2v_aoi_violation_rate_400ms,v2v_aoi_violation_rate_500ms,"
          << "v2v_aoi_orr_le_200_ms,v2v_aoi_orr_le_300_ms,"
          << "v2v_aoi_orr_le_400_ms,v2v_aoi_orr_le_500_ms,"
          << "v2v_only_aoi_orr_le_200_ms,v2v_only_aoi_orr_le_300_ms,"
          << "v2v_only_aoi_orr_le_400_ms,v2v_only_aoi_orr_le_500_ms,"
          << "mec_aoi_mean_ms,mec_aoi_p50_ms,mec_aoi_p90_ms,mec_aoi_p99_ms,"
          << "mec_aoi_le_200_ms_rate,mec_aoi_le_300_ms_rate,"
          << "mec_aoi_le_400_ms_rate,mec_aoi_le_500_ms_rate,"
          << "mec_aoi_violation_rate_200ms,mec_aoi_violation_rate_300ms,"
          << "mec_aoi_violation_rate_400ms,mec_aoi_violation_rate_500ms,"
          << "mec_high_aoi_le_200_ms_rate,mec_high_aoi_le_300_ms_rate,"
          << "mec_high_aoi_le_400_ms_rate,mec_high_aoi_le_500_ms_rate,"
          << "mec_low_aoi_le_200_ms_rate,mec_low_aoi_le_300_ms_rate,"
          << "mec_low_aoi_le_400_ms_rate,mec_low_aoi_le_500_ms_rate,"
          << "mec_aoi_orr_le_200_ms,mec_aoi_orr_le_300_ms,"
          << "mec_aoi_orr_le_400_ms,mec_aoi_orr_le_500_ms,"
          << "mec_only_aoi_orr_le_200_ms,mec_only_aoi_orr_le_300_ms,"
          << "mec_only_aoi_orr_le_400_ms,mec_only_aoi_orr_le_500_ms,"
          << "mec_capacity_delay_mean_ms,mec_capacity_delay_p90_ms,"
          << "mec_capacity_delay_p99_ms,mec_capacity_queue_drops,"
          << "mec_ul_capacity_mbps,mec_dl_capacity_mbps,"
          << "mec_ul_busy_ratio_mean,mec_ul_busy_ratio_p90,"
          << "mec_ul_busy_ratio_p99,mec_ul_busy_ratio_max,"
          << "mec_dl_busy_ratio_mean,mec_dl_busy_ratio_p90,"
          << "mec_dl_busy_ratio_p99,mec_dl_busy_ratio_max,"
          << "mec_ul_first_losses,mec_dl_first_losses,"
          << "mec_ul_retx_recovered,mec_dl_retx_recovered,"
          << "mec_ul_final_losses,mec_dl_final_losses,"
          << "mec_retx_delay_mean_ms,mec_retx_delay_p90_ms"
          << std::endl;
      summaryCsv << method << "," << FormatThesisMetric (recognitionRate) << ","
                 << FormatThesisMetric (highPriorityObjectRecognitionRate) << ","
                 << FormatThesisMetric (lowPriorityObjectRecognitionRate) << ","
                 << FormatThesisMetric (sensorObjectRecognitionRate) << ","
                 << FormatThesisMetric (sensorHighPriorityObjectRecognitionRate) << ","
                 << FormatThesisMetric (sensorLowPriorityObjectRecognitionRate) << ","
                 << FormatThesisMetric (cooperativeObjectRecognitionRate) << ","
                 << FormatThesisMetric (recognitionRate) << ","
                 << FormatThesisMetric (packetLossRate) << "," << thesisIdealCpmRx << ","
                 << thesisTrueCpmRx << "," << FormatThesisMetric (recognitionRate) << ","
                 << FormatThesisMetric (highPdr) << "," << FormatThesisMetric (lowPdr)
                 << "," << FormatThesisMetric (highPacketLoss) << ","
                 << FormatThesisMetric (lowPacketLoss) << ","
                 << g_thesisStats.highIdealCpmRx << "," << g_thesisStats.highTrueCpmRx
                 << "," << g_thesisStats.lowIdealCpmRx << ","
                 << g_thesisStats.lowTrueCpmRx << ","
                 << FormatThesisMetric (dsrcRouteLoss) << ","
                 << FormatThesisMetric (nrRouteLoss) << "," << FormatThesisMetric (mecRouteLoss)
                 << "," << effectiveChannelBusyRatio << ","
                 << dsrcMetrics->getNumberTx_overall () << ","
                 << dsrcMetrics->getNumberRx_overall () << ","
                 << FormatThesisMetric (v2vRadioLoss) << ","
                 << FormatThesisMetric (v2vHighRadioLoss) << ","
                 << FormatThesisMetric (v2vLowRadioLoss) << ","
                 << FormatThesisMetric (nrRouteLoss) << ","
                 << FormatThesisMetric (mecRouteLoss) << ","
                 << FormatThesisMetric (mecRouteLoss) << ","
                 << FormatThesisMetric (mecValidUpdateSuccessRate) << ","
                 << FormatThesisMetric (mecUlFinalLossRate) << ","
                 << FormatThesisMetric (mecDlFinalLossRate) << ","
                 << FormatThesisMetric (mecRadioFinalLossRate) << ","
                 << nrMetrics->getAverageLatency_overall () << ","
                 << nrLatencyP50 << "," << nrLatencyP90 << "," << nrLatencyP95 << ","
                 << nrLatencyP99 << ","
                 << mecTxOverall << ","
                 << mecRxOverall << ","
                 << mecLatencyAvg << ","
                 << mecLatencyP50 << "," << mecLatencyP90 << "," << mecLatencyP95 << ","
                 << mecLatencyP99 << "," << g_interferenceTx << ","
                 << g_interferenceDrops << "," << g_interferenceBytes << ","
                 << g_mecBackgroundUplinkBytes << "," << g_mecBackgroundDownlinkBytes << ","
                 << (g_idealMecAoiFilter ? 1 : 0) << ","
                 << FormatThesisMetric (g_idealMecAoiFilterThresholdMs) << ","
                 << FormatThesisMetric (ttlViolationRate) << ","
                 << FormatThesisMetric (neverReceivedRate) << ","
                 << FormatThesisMetric (highTtlViolationRate) << ","
                 << FormatThesisMetric (highNeverReceivedRate) << ","
                 << FormatThesisMetric (lowTtlViolationRate) << ","
                 << FormatThesisMetric (lowNeverReceivedRate) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverFreshRedundancy200Sum,
                        g_thesisStats.receiverFreshRedundancySamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverFreshRedundancy500Sum,
                        g_thesisStats.receiverFreshRedundancySamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.receiverFreshRedundancyGe2_200,
                        g_thesisStats.receiverFreshRedundancySamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.receiverFreshRedundancyGe2_500,
                        g_thesisStats.receiverFreshRedundancySamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverFreshRedundancyHigh200Sum,
                        g_thesisStats.receiverFreshRedundancyHighSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverFreshRedundancyLow200Sum,
                        g_thesisStats.receiverFreshRedundancyLowSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverRv200Sum,
                        g_thesisStats.receiverFreshRedundancySamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverRlSum,
                        g_thesisStats.receiverRlSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverRlMedianSum,
                        g_thesisStats.receiverRlVehicleSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.receiverRlLt1,
                        g_thesisStats.receiverRlSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.receiverRlGe1,
                        g_thesisStats.receiverRlSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.receiverRlGe2,
                        g_thesisStats.receiverRlSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverRvDeloozSum,
                        g_thesisStats.receiverRlSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverRvDeloozMedianSum,
                        g_thesisStats.receiverRlVehicleSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverHighRlSum,
                        g_thesisStats.receiverHighRlSamples)) << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.receiverLowRlSum,
                        g_thesisStats.receiverLowRlSamples)) << ","
                 << g_thesisStats.rmrDeletedEvalExpectedTotal << ","
                 << g_thesisStats.rmrDeletedEvalExpectedHigh << ","
                 << g_thesisStats.rmrDeletedEvalExpectedLow << ","
                 << g_thesisStats.rmrDeletedEvalUnrecognizedTotal << ","
                 << g_thesisStats.rmrDeletedEvalUnrecognizedHigh << ","
                 << g_thesisStats.rmrDeletedEvalUnrecognizedLow << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.rmrDeletedEvalDistanceSum,
                        g_thesisStats.rmrDeletedEvalDistanceSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.rmrDeletedEvalFreshRedundancyGe2_200,
                        g_thesisStats.rmrDeletedEvalExpectedTotal)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.rmrDeletedEvalFreshRedundancyGe2_500,
                        g_thesisStats.rmrDeletedEvalExpectedTotal)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.rmrDeletedEvalClosingSpeedGeThreshold,
                        g_thesisStats.rmrDeletedEvalDistanceSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.rmrDeletedEvalTtcLeThreshold,
                        g_thesisStats.rmrDeletedEvalDistanceSamples)) << ","
                 << g_thesisStats.rmrDeletedSenderEvalTotal << ","
                 << g_thesisStats.rmrDeletedSenderEvalHigh << ","
                 << g_thesisStats.rmrDeletedSenderEvalLow << ","
                 << FormatThesisMetric (AverageFromSum (
                        g_thesisStats.rmrDeletedSenderEvalDistanceSum,
                        g_thesisStats.rmrDeletedSenderEvalDistanceSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.rmrDeletedSenderEvalClosingSpeedGeThreshold,
                        g_thesisStats.rmrDeletedSenderEvalDistanceSamples)) << ","
                 << FormatThesisMetric (PercentRate (
                        g_thesisStats.rmrDeletedSenderEvalTtcLeThreshold,
                        g_thesisStats.rmrDeletedSenderEvalDistanceSamples)) << ","
                 << g_sensorExternalEventCandidateVehicles << ","
                 << g_sensorExternalEventSelectedVehicles << ","
                 << g_sensorExternalEventCount << ","
                 << g_sensorExternalEventRecognized << ","
                 << g_sensorExternalEventMissed << ","
                 << sensorExternalEventCensored << ","
                 << FormatThesisMetric (sensorExternalEventRecognitionRate) << ","
                 << FormatThesisMetric (sensorExternalEventDelayMeanMs) << ","
                 << FormatThesisMetric (sensorExternalEventDelayP50Ms) << ","
                 << FormatThesisMetric (v2vAoiMeanMs) << ","
                 << FormatThesisMetric (v2vAoiP50Ms) << ","
                 << FormatThesisMetric (v2vAoiP90Ms) << ","
                 << FormatThesisMetric (v2vAoiP99Ms) << ","
                 << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[0],
                                                     g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[1],
                                                     g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[2],
                                                     g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_v2vAoiWithinThreshold[3],
                                                     g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[0],
                                                         g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[1],
                                                         g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[2],
                                                         g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_v2vAoiWithinThreshold[3],
                                                         g_v2vAoiSamples)) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vAoiRecognitionRatioSum[0],
                        g_thesisStats.v2vAoiRecognitionVehicleSamples[0])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vAoiRecognitionRatioSum[1],
                        g_thesisStats.v2vAoiRecognitionVehicleSamples[1])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vAoiRecognitionRatioSum[2],
                        g_thesisStats.v2vAoiRecognitionVehicleSamples[2])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vAoiRecognitionRatioSum[3],
                        g_thesisStats.v2vAoiRecognitionVehicleSamples[3])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vOnlyAoiRecognitionRatioSum[0],
                        g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[0])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vOnlyAoiRecognitionRatioSum[1],
                        g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[1])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vOnlyAoiRecognitionRatioSum[2],
                        g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[2])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.v2vOnlyAoiRecognitionRatioSum[3],
                        g_thesisStats.v2vOnlyAoiRecognitionVehicleSamples[3])) << ","
                 << FormatThesisMetric (mecAoiMeanMs) << ","
                 << FormatThesisMetric (mecAoiP50Ms) << ","
                 << FormatThesisMetric (mecAoiP90Ms) << ","
                 << FormatThesisMetric (mecAoiP99Ms) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[0],
                                                     g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[1],
                                                     g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[2],
                                                     g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecAoiWithinThreshold[3],
                                                     g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[0],
                                                         g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[1],
                                                         g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[2],
                                                         g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentMissRate (g_idealMecAoiWithinThreshold[3],
                                                         g_idealMecAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[0],
                                                     g_idealMecHighAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[1],
                                                     g_idealMecHighAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[2],
                                                     g_idealMecHighAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecHighAoiWithinThreshold[3],
                                                     g_idealMecHighAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[0],
                                                     g_idealMecLowAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[1],
                                                     g_idealMecLowAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[2],
                                                     g_idealMecLowAoiSamples)) << ","
                 << FormatThesisMetric (PercentRate (g_idealMecLowAoiWithinThreshold[3],
                                                     g_idealMecLowAoiSamples)) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecAoiRecognitionRatioSum[0],
                        g_thesisStats.mecAoiRecognitionVehicleSamples[0])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecAoiRecognitionRatioSum[1],
                        g_thesisStats.mecAoiRecognitionVehicleSamples[1])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecAoiRecognitionRatioSum[2],
                        g_thesisStats.mecAoiRecognitionVehicleSamples[2])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecAoiRecognitionRatioSum[3],
                        g_thesisStats.mecAoiRecognitionVehicleSamples[3])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecOnlyAoiRecognitionRatioSum[0],
                        g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[0])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecOnlyAoiRecognitionRatioSum[1],
                        g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[1])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecOnlyAoiRecognitionRatioSum[2],
                        g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[2])) << ","
                 << FormatThesisMetric (PercentFromRatioSum (
                        g_thesisStats.mecOnlyAoiRecognitionRatioSum[3],
                        g_thesisStats.mecOnlyAoiRecognitionVehicleSamples[3])) << ","
                 << FormatThesisMetric (mecCapacityDelayMeanMs) << ","
                 << FormatThesisMetric (mecCapacityDelayP90Ms) << ","
                 << FormatThesisMetric (mecCapacityDelayP99Ms) << ","
                 << g_idealMecCapacityQueueDrops << ","
                 << FormatThesisMetric (g_idealMecUplinkCapacityMbps) << ","
                 << FormatThesisMetric (g_idealMecDownlinkCapacityMbps) << ","
                 << FormatThesisMetric (mecUlBusyRatioMean) << ","
                 << FormatThesisMetric (mecUlBusyRatioP90) << ","
                 << FormatThesisMetric (mecUlBusyRatioP99) << ","
                 << FormatThesisMetric (mecUlBusyRatioMax) << ","
                 << FormatThesisMetric (mecDlBusyRatioMean) << ","
                 << FormatThesisMetric (mecDlBusyRatioP90) << ","
                 << FormatThesisMetric (mecDlBusyRatioP99) << ","
                 << FormatThesisMetric (mecDlBusyRatioMax) << ","
                 << g_idealMecUlFirstLosses << "," << g_idealMecDlFirstLosses << ","
                 << g_idealMecUlRetxRecovered << "," << g_idealMecDlRetxRecovered << ","
                 << g_idealMecUlFinalLosses << "," << g_idealMecDlFinalLosses << ","
                 << FormatThesisMetric (mecRetxDelayMeanMs) << ","
                 << FormatThesisMetric (mecRetxDelayP90Ms) << std::endl;
    }

  if (g_routeLog.is_open ())
    {
      g_routeLog.close ();
    }
  if (g_observationLog.is_open ())
    {
      g_observationLog.close ();
    }
  if (g_rsuPredictionLog.is_open ())
    {
      g_rsuPredictionLog.close ();
    }

  Simulator::Destroy ();
  return 0;
}
