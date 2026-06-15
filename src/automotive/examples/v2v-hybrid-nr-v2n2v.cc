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
#include <bitset>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
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
  uint64_t dsrcLastCpmRx = 0;
  uint64_t nrLastCpmRx = 0;
  uint64_t mecLastCpmRx = 0;
  uint64_t dsrcIdealCpmRx = 0;
  uint64_t dsrcTrueCpmRx = 0;
  uint64_t nrIdealCpmRx = 0;
  uint64_t nrTrueCpmRx = 0;
  uint64_t mecIdealCpmRx = 0;
  uint64_t mecTrueCpmRx = 0;
  uint64_t highIdealCpmRx = 0;
  uint64_t highTrueCpmRx = 0;
  uint64_t lowIdealCpmRx = 0;
  uint64_t lowTrueCpmRx = 0;
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
};

struct TrafficFlowRsuInfo
{
  std::string id;
  double x = 0.0;
  double y = 0.0;
};

struct TrafficFlowVehicleSnapshot
{
  std::string id;
  double x = 0.0;
  double y = 0.0;
  double speedMps = 0.0;
};

struct TrafficFlowSegmentStats
{
  uint32_t vehicleCount = 0;
  double averageSpeedMps = 0.0;
  double cbr = 0.0;
};

struct PriorityMotionState
{
  Vector position;
  Time timestamp = Seconds (0.0);
};

struct PredictiveRmrConfig
{
  double predictionHorizonSeconds = 2.0;
  double cpmSizeNormBytes = 1200.0;
  double cpmTxRateNorm = 10.0;
  double activeVehicleNorm = 100.0;
  double trafficFlowRoadLengthMeters = 2000.0;
  double trafficFlowMessageRateHz = 10.0;
  double trafficFlowAvgPacketSizeBytes = 500.0;
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
static std::unordered_map<std::string, PredictiveCbrState> g_predictiveCbrState;
static TrafficFlowPredictionState g_trafficFlowPrediction;
static PredictiveRmrConfig g_predictiveRmrConfig;
static HybridRouteConfig g_hybridRouteConfig;
static std::vector<TrafficFlowRsuInfo> g_trafficFlowRsus;
static std::ofstream g_routeLog;
static std::ofstream g_observationLog;
static ThesisEvaluationStats g_thesisStats;
static uint64_t g_camRx = 0;
static uint64_t g_cpmRx = 0;
static std::unordered_map<uint64_t, std::unordered_map<uint64_t, Time>> g_latestCpmRxByReceiver;
static uint64_t g_interferenceTx = 0;
static uint64_t g_interferenceBytes = 0;
static uint64_t g_interferenceDrops = 0;
static uint64_t g_mecUplinkPackets = 0;
static uint64_t g_mecUplinkBytes = 0;
static uint64_t g_mecForwardedPackets = 0;
static uint64_t g_mecForwardedBytes = 0;
static uint64_t g_mecForwardDrops = 0;
static uint64_t g_mecForwardNoReceiver = 0;
static uint64_t g_idealMecTx = 0;
static uint64_t g_idealMecRx = 0;
static uint32_t g_idealMecPacketSizeBytes = 500;
static Time g_idealMecLatency = MilliSeconds (50);
static Time g_idealMecInterval = MilliSeconds (100);
static double g_mecForwardRangeMeters = 150.0;
static bool g_useIdealMecLink = true;
static std::vector<double> g_idealMecLatencySamplesMs;
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
static Ptr<UniformRandomVariable> g_hybridRouteRandom;

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
      return "DSRC_V2V";
    }
  if (route == ActiveRoute::NrSidelinkV2v)
    {
      return "NR_SIDELINK_V2V";
    }
  return "MEC_V2N2V";
}

static std::string
RouteStateName (bool dsrcActive, bool nrActive, bool mecActive)
{
  std::string name;
  if (dsrcActive)
    {
      name += "DSRC_V2V";
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
ReceiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
            Address from,
            StationID_t myStationId,
            StationType_t myStationType,
            SignalInfo phyInfo)
{
  (void) from;
  (void) myStationType;
  (void) phyInfo;
  if (cpm->header.stationId > 0 &&
      static_cast<uint64_t> (cpm->header.stationId) != static_cast<uint64_t> (myStationId))
    {
      g_latestCpmRxByReceiver[static_cast<uint64_t> (myStationId)]
                             [static_cast<uint64_t> (cpm->header.stationId)] = Simulator::Now ();
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

static double
DistanceMeters2d (const libsumo::TraCIPosition& a, const libsumo::TraCIPosition& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt ((dx * dx) + (dy * dy));
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
          const uint64_t senderStationId = VehicleIdToStationId (senderVehicleId);
          const uint64_t receiverStationId = VehicleIdToStationId (entry.first);
          const Vector senderVector (senderPos.x, senderPos.y, 0.0);
          const Vector receiverVector (receiverPos.x, receiverPos.y, 0.0);
          if (distanceMeters <= g_priorityDistanceThresholdMeters ||
              IsClosingHighPriorityObject (senderStationId,
                                           receiverStationId,
                                           senderVector,
                                           receiverVector,
                                           distanceMeters,
                                           Simulator::Now ()))
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

static void
DeliverIdealMecCpm (uint64_t receiverStationId, uint64_t senderStationId)
{
  g_latestCpmRxByReceiver[receiverStationId][senderStationId] = Simulator::Now ();
  ++g_idealMecRx;
  g_idealMecLatencySamplesMs.push_back (g_idealMecLatency.GetMilliSeconds ());
}

static void
GenerateIdealMecCpm ()
{
  if (g_sumoClient == nullptr)
    {
      Simulator::Schedule (g_idealMecInterval, &GenerateIdealMecCpm);
      return;
    }

  std::vector<std::pair<std::string, libsumo::TraCIPosition>> vehiclePositions;
  std::vector<std::pair<std::string, libsumo::TraCIPosition>> activeSenders;
  vehiclePositions.reserve (g_vehicleRuntime.size ());
  activeSenders.reserve (g_vehicleRuntime.size ());

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

      vehiclePositions.emplace_back (entry.first, pos);
      if (entry.second.mecTxActive)
        {
          activeSenders.emplace_back (entry.first, pos);
        }
    }

  for (const auto& senderEntry : activeSenders)
    {
      ++g_idealMecTx;
      ++g_mecUplinkPackets;
      g_mecUplinkBytes += g_idealMecPacketSizeBytes;

      uint32_t targets = 0;
      for (const auto& receiverEntry : vehiclePositions)
        {
          if (receiverEntry.first == senderEntry.first)
            {
              continue;
            }

          if (DistanceMeters2d (senderEntry.second, receiverEntry.second) > g_mecForwardRangeMeters)
            {
              continue;
            }

          ++targets;
          ++g_mecForwardedPackets;
          g_mecForwardedBytes += g_idealMecPacketSizeBytes;
          Simulator::Schedule (g_idealMecLatency,
                               &DeliverIdealMecCpm,
                               VehicleIdToStationId (receiverEntry.first),
                               VehicleIdToStationId (senderEntry.first));
        }

      if (targets == 0)
        {
          ++g_mecForwardNoReceiver;
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

static void
AccumulateThesisRecognitionSample ()
{
  const uint32_t activeVehicles = static_cast<uint32_t> (g_vehicleRuntime.size ());
  if (activeVehicles <= 1)
    {
      return;
    }

  std::unordered_map<uint64_t, Ptr<MobilityModel>> activeStationMobility;
  std::unordered_map<uint64_t, Vector> activeStationPositions;
  for (const auto& entry : g_vehicleRuntime)
    {
      Ptr<MobilityModel> mobility =
          entry.second.node != nullptr ? entry.second.node->GetObject<MobilityModel> () : nullptr;
      if (mobility != nullptr)
        {
          const uint64_t stationId = VehicleIdToStationId (entry.first);
          activeStationMobility[stationId] = mobility;
          activeStationPositions[stationId] = mobility->GetPosition ();
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

      std::set<uint64_t> cpmRecognizedStationIds;
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

      std::set<uint64_t> recognizedStationIds = sensorRecognizedStationIds;
      recognizedStationIds.insert (cpmRecognizedStationIds.begin (), cpmRecognizedStationIds.end ());
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

      g_thesisStats.recognitionRatioSum +=
          Clamp01 (static_cast<double> (recognizedObjects) /
                   static_cast<double> (expectedStationIds.size ()));
      ++g_thesisStats.recognitionVehicleSamples;
      if (!highPriorityExpectedStationIds.empty ())
        {
          g_thesisStats.highPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (highPriorityRecognizedObjects) /
                       static_cast<double> (highPriorityExpectedStationIds.size ()));
          ++g_thesisStats.highPriorityRecognitionVehicleSamples;
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
        }

      g_thesisStats.sensorRecognitionRatioSum +=
          Clamp01 (static_cast<double> (recognizedStationIds.size ()) /
                   static_cast<double> (expectedStationIds.size ()));
      ++g_thesisStats.sensorRecognitionVehicleSamples;
      if (!highPriorityExpectedStationIds.empty ())
        {
          g_thesisStats.sensorHighPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (highPriorityRecognizedObjects) /
                       static_cast<double> (highPriorityExpectedStationIds.size ()));
          ++g_thesisStats.sensorHighPriorityRecognitionVehicleSamples;
        }
      if (lowPriorityExpectedObjects > 0)
        {
          g_thesisStats.sensorLowPriorityRecognitionRatioSum +=
              Clamp01 (static_cast<double> (lowPriorityRecognizedObjects) /
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

  g_priorityMotionHistory.clear ();
  for (const auto& entry : activeStationPositions)
    {
      PriorityMotionState state;
      state.position = entry.second;
      state.timestamp = now;
      g_priorityMotionHistory[entry.first] = state;
    }
}

static void
AccumulateThesisPacketLossCounters (Ptr<MetricSupervisor> dsrcMetrics,
                                    Ptr<MetricSupervisor> nrMetrics,
                                    Ptr<MetricSupervisor> mecMetrics,
                                    Ptr<TraciClient> sumoClient,
                                    double baselineMeters)
{
  const auto cpmType = MetricSupervisor::messageType_cpm;

  const uint64_t dsrcRx = dsrcMetrics->getNumberRx_messagetype (cpmType);
  const uint64_t nrRx = nrMetrics->getNumberRx_messagetype (cpmType);
  const uint64_t mecRx = mecMetrics->getNumberRx_messagetype (cpmType);

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

  for (const auto& entry : g_vehicleRuntime)
    {
      const ReceiverPriorityCounts receivers =
          CountReceiversByPriorityWithinBaseline (sumoClient, entry.first, baselineMeters);
      const uint64_t totalReceivers = receivers.Total ();

      const uint64_t dsrcTx =
          entry.second.dsrcContainer->getCPBasicService ()->getCpmSent ();
      const uint64_t dsrcLastTx = g_thesisStats.dsrcLastCpmTxByVehicle[entry.first];
      const uint64_t dsrcDeltaTx = dsrcTx - dsrcLastTx;
      g_thesisStats.dsrcIdealCpmRx += dsrcDeltaTx * totalReceivers;
      dsrcHighIdealDelta += dsrcDeltaTx * receivers.high;
      dsrcLowIdealDelta += dsrcDeltaTx * receivers.low;
      g_thesisStats.dsrcLastCpmTxByVehicle[entry.first] = dsrcTx;

      if (entry.second.nrContainer != nullptr)
        {
          const uint64_t nrTx =
              entry.second.nrContainer->getCPBasicService ()->getCpmSent ();
          const uint64_t nrLastTx = g_thesisStats.nrLastCpmTxByVehicle[entry.first];
          const uint64_t nrDeltaTx = nrTx - nrLastTx;
          g_thesisStats.nrIdealCpmRx += nrDeltaTx * totalReceivers;
          nrHighIdealDelta += nrDeltaTx * receivers.high;
          nrLowIdealDelta += nrDeltaTx * receivers.low;
          g_thesisStats.nrLastCpmTxByVehicle[entry.first] = nrTx;
        }

      if (entry.second.mecContainer != nullptr)
        {
          const uint64_t mecTx =
              entry.second.mecContainer->getCPBasicService ()->getCpmSent ();
          const uint64_t mecLastTx = g_thesisStats.mecLastCpmTxByVehicle[entry.first];
          const uint64_t mecDeltaTx = mecTx - mecLastTx;
          g_thesisStats.mecIdealCpmRx += mecDeltaTx * totalReceivers;
          mecHighIdealDelta += mecDeltaTx * receivers.high;
          mecLowIdealDelta += mecDeltaTx * receivers.low;
          g_thesisStats.mecLastCpmTxByVehicle[entry.first] = mecTx;
        }
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
  const uint64_t dsrcTrueDelta = std::min (dsrcDeltaRx, dsrcIdealDelta);
  const uint64_t nrTrueDelta = std::min (nrDeltaRx, nrIdealDelta);
  const uint64_t mecTrueDelta = std::min (mecDeltaRx, mecIdealDelta);
  const uint64_t dsrcHighTrueDelta =
      allocateTrueRx (dsrcTrueDelta, dsrcHighIdealDelta, dsrcIdealDelta);
  const uint64_t nrHighTrueDelta = allocateTrueRx (nrTrueDelta, nrHighIdealDelta, nrIdealDelta);
  const uint64_t mecHighTrueDelta =
      allocateTrueRx (mecTrueDelta, mecHighIdealDelta, mecIdealDelta);
  const uint64_t dsrcLowTrueDelta =
      dsrcLowIdealDelta > 0 ? dsrcTrueDelta - dsrcHighTrueDelta : 0;
  const uint64_t nrLowTrueDelta = nrLowIdealDelta > 0 ? nrTrueDelta - nrHighTrueDelta : 0;
  const uint64_t mecLowTrueDelta = mecLowIdealDelta > 0 ? mecTrueDelta - mecHighTrueDelta : 0;

  g_thesisStats.dsrcTrueCpmRx += dsrcTrueDelta;
  g_thesisStats.nrTrueCpmRx += nrTrueDelta;
  g_thesisStats.mecTrueCpmRx += mecTrueDelta;
  g_thesisStats.highIdealCpmRx += dsrcHighIdealDelta + nrHighIdealDelta + mecHighIdealDelta;
  g_thesisStats.lowIdealCpmRx += dsrcLowIdealDelta + nrLowIdealDelta + mecLowIdealDelta;
  g_thesisStats.highTrueCpmRx += dsrcHighTrueDelta + nrHighTrueDelta + mecHighTrueDelta;
  g_thesisStats.lowTrueCpmRx += dsrcLowTrueDelta + nrLowTrueDelta + mecLowTrueDelta;
}

static void
SampleThesisMetrics (Ptr<MetricSupervisor> dsrcMetrics,
                     Ptr<MetricSupervisor> nrMetrics,
                     Ptr<MetricSupervisor> mecMetrics,
                     Ptr<TraciClient> sumoClient,
                     double baselineMeters,
                     Time interval)
{
  AccumulateThesisRecognitionSample ();
  AccumulateThesisPacketLossCounters (dsrcMetrics,
                                      nrMetrics,
                                      mecMetrics,
                                      sumoClient,
                                      baselineMeters);

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

static double
AverageLatestDsrcCpmSizeBytes ()
{
  double sizeSum = 0.0;
  uint32_t samples = 0;
  for (const auto& entry : g_vehicleRuntime)
    {
      if (entry.second.dsrcContainer == nullptr)
        {
          continue;
        }
      const uint32_t sizeBytes =
          entry.second.dsrcContainer->getCPBasicService ()->getLastCpmSizeBytes ();
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
EstimateTrafficCbrFromVehicleCount (double vehicleCount, double packetSizeBits)
{
  const double channelRateBps = std::max (g_predictiveChannelRateBps, 1.0);
  const double loadBps =
      std::max (0.0, vehicleCount) * g_predictiveRmrConfig.trafficFlowMessageRateHz *
      packetSizeBits;
  return Clamp01 (loadBps / channelRateBps);
}

static std::vector<TrafficFlowVehicleSnapshot>
ReadTrafficFlowVehicleSnapshots ()
{
  std::vector<TrafficFlowVehicleSnapshot> vehicles;
  if (g_sumoClient == nullptr)
    {
      return vehicles;
    }

  for (const auto& entry : g_vehicleRuntime)
    {
      try
        {
          const libsumo::TraCIPosition position =
              g_sumoClient->TraCIAPI::vehicle.getPosition (entry.first);
          TrafficFlowVehicleSnapshot snapshot;
          snapshot.id = entry.first;
          snapshot.x = position.x;
          snapshot.y = position.y;
          snapshot.speedMps = std::max (0.0, g_sumoClient->TraCIAPI::vehicle.getSpeed (entry.first));
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
                                double packetSizeBits)
{
  TrafficFlowSegmentStats stats;
  double speedSum = 0.0;
  for (const auto& vehicle : vehicles)
    {
      if (vehicle.x >= segmentStartM && vehicle.x < segmentEndM)
        {
          ++stats.vehicleCount;
          speedSum += vehicle.speedMps;
        }
    }

  stats.averageSpeedMps =
      stats.vehicleCount > 0 ? speedSum / static_cast<double> (stats.vehicleCount) : 0.0;
  stats.cbr = EstimateTrafficCbrFromVehicleCount (stats.vehicleCount, packetSizeBits);
  return stats;
}

static double
ComputeTrafficFlowRate (double vehicleCount, double speedMps, double segmentLengthM)
{
  return (vehicleCount / std::max (segmentLengthM, 1.0)) * std::max (0.0, speedMps);
}

static bool
UpdateRsuSharedTrafficFlowPrediction (double packetSizeBits)
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

  for (const auto& rsu : g_trafficFlowRsus)
    {
      const double localStart = std::max (0.0, rsu.x - halfSegmentM);
      const double localEnd = rsu.x + halfSegmentM;
      const double upstreamStart = std::max (0.0, localStart - segmentLengthM);
      const double upstreamEnd = localStart;

      const TrafficFlowSegmentStats local =
          ComputeTrafficFlowSegmentStats (vehicles, localStart, localEnd, packetSizeBits);
      TrafficFlowSegmentStats upstream;
      if (upstreamEnd > upstreamStart)
        {
          upstream =
              ComputeTrafficFlowSegmentStats (vehicles, upstreamStart, upstreamEnd, packetSizeBits);
        }
      else
        {
          upstream = local;
        }

      const double qIn =
          ComputeTrafficFlowRate (upstream.vehicleCount, upstream.averageSpeedMps, segmentLengthM);
      const double qOut =
          ComputeTrafficFlowRate (local.vehicleCount, local.averageSpeedMps, segmentLengthM);
      const double futureVehicles =
          std::max (0.0, static_cast<double> (local.vehicleCount) + (qIn - qOut) * horizonSeconds);
      const double predictedCbr =
          EstimateTrafficCbrFromVehicleCount (futureVehicles, packetSizeBits);

      qInSum += qIn;
      qOutSum += qOut;
      localVehicleSum += static_cast<double> (local.vehicleCount);
      upstreamVehicleSum += static_cast<double> (upstream.vehicleCount);
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
  g_trafficFlowPrediction.deltaCbr =
      std::max (0.0, predictedCbrMax - EstimateTrafficCbrFromVehicleCount (localVehicleSum * invCount,
                                                                           packetSizeBits));
  return true;
}

static void
UpdateTrafficFlowCbrPrediction (Time interval)
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

  const double packetSizeBits = AverageLatestDsrcCpmSizeBytes () * 8.0;
  const bool usedRsuSharedPrediction = UpdateRsuSharedTrafficFlowPrediction (packetSizeBits);
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
      << "legacy_cpm_objects_total,legacy_rmr_candidates_total,legacy_rmr_deleted_total,"
      << "legacy_cpm_size_bytes,legacy_cpm_size_bytes_total,nr_cpm_objects,nr_rmr_candidates,"
      << "nr_rmr_deleted_last,nr_cpm_objects_total,nr_rmr_candidates_total,"
      << "nr_rmr_deleted_total,nr_cpm_size_bytes,nr_cpm_size_bytes_total,"
      << "mec_cpm_objects,mec_rmr_candidates,mec_rmr_deleted_last,"
      << "mec_cpm_objects_total,mec_rmr_candidates_total,mec_rmr_deleted_total,"
      << "mec_cpm_size_bytes,mec_cpm_size_bytes_total,"
      << "cam_rx,cpm_rx,interference_tx,interference_bytes,interference_drops,"
      << "interference_offered_cbr,"
      << "mec_uplink_packets,mec_uplink_bytes,mec_forwarded_packets,"
      << "mec_forwarded_bytes,mec_forward_drops,mec_forward_no_receiver,"
      << "orr,high_pdr,low_pdr,high_packet_loss,low_packet_loss,"
      << "high_ideal_rx,high_true_rx,low_ideal_rx,low_true_rx" << std::endl;
}

static void
WriteObservationLog (Ptr<MetricSupervisor> dsrcMetrics,
                     Ptr<MetricSupervisor> nrMetrics,
                     Ptr<MetricSupervisor> mecMetrics,
                     Time interval)
{
  uint32_t activeVehicles = 0;
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
  uint32_t dsrcCpmObjects = 0;
  uint32_t dsrcRmrCandidates = 0;
  uint32_t dsrcRmrDeletedLast = 0;
  uint32_t dsrcCpmSizeBytes = 0;
  uint64_t dsrcCpmObjectsTotal = 0;
  uint64_t dsrcRmrCandidatesTotal = 0;
  uint64_t dsrcRmrDeletedTotal = 0;
  uint64_t dsrcCpmSizeBytesTotal = 0;
  uint32_t nrCpmObjects = 0;
  uint32_t nrRmrCandidates = 0;
  uint32_t nrRmrDeletedLast = 0;
  uint32_t nrCpmSizeBytes = 0;
  uint64_t nrCpmObjectsTotal = 0;
  uint64_t nrRmrCandidatesTotal = 0;
  uint64_t nrRmrDeletedTotal = 0;
  uint64_t nrCpmSizeBytesTotal = 0;
  uint32_t mecCpmObjects = 0;
  uint32_t mecRmrCandidates = 0;
  uint32_t mecRmrDeletedLast = 0;
  uint32_t mecCpmSizeBytes = 0;
  uint64_t mecCpmObjectsTotal = 0;
  uint64_t mecRmrCandidatesTotal = 0;
  uint64_t mecRmrDeletedTotal = 0;
  uint64_t mecCpmSizeBytesTotal = 0;

  for (const auto& entry : g_vehicleRuntime)
    {
      ++activeVehicles;
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

      const double cbr = dsrcMetrics->getCBRPerItem (entry.first);
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
      dsrcCpmSizeBytes += dsrcCp->getLastCpmSizeBytes ();
      dsrcCpmSizeBytesTotal += dsrcCp->getTotalCpmSizeBytes ();

      if (entry.second.nrContainer != nullptr)
        {
          Ptr<CPBasicService> nrCp = entry.second.nrContainer->getCPBasicService ();
          nrCpmObjects += nrCp->getLastRmrIncludedObjectCount ();
          nrRmrCandidates += nrCp->getLastRmrCandidateCount ();
          nrRmrDeletedLast += nrCp->getLastRmrDeletedObjectCount ();
          nrCpmObjectsTotal += nrCp->getTotalRmrIncludedObjectCount ();
          nrRmrCandidatesTotal += nrCp->getTotalRmrCandidateCount ();
          nrRmrDeletedTotal += nrCp->getTotalRmrDeletedObjectCount ();
          nrCpmSizeBytes += nrCp->getLastCpmSizeBytes ();
          nrCpmSizeBytesTotal += nrCp->getTotalCpmSizeBytes ();
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
          mecCpmSizeBytes += mecCp->getLastCpmSizeBytes ();
          mecCpmSizeBytesTotal += mecCp->getTotalCpmSizeBytes ();
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
                       << dsrcCpmObjectsTotal << "," << dsrcRmrCandidatesTotal << ","
                       << dsrcRmrDeletedTotal << "," << dsrcCpmSizeBytes << ","
                       << dsrcCpmSizeBytesTotal << "," << nrCpmObjects << ","
                       << nrRmrCandidates << "," << nrRmrDeletedLast << ","
                       << nrCpmObjectsTotal << "," << nrRmrCandidatesTotal << ","
                       << nrRmrDeletedTotal << "," << nrCpmSizeBytes << ","
                       << nrCpmSizeBytesTotal << "," << mecCpmObjects << ","
                       << mecRmrCandidates << "," << mecRmrDeletedLast << ","
                       << mecCpmObjectsTotal << "," << mecRmrCandidatesTotal << ","
                       << mecRmrDeletedTotal << "," << mecCpmSizeBytes << ","
                       << mecCpmSizeBytesTotal << "," << g_camRx << "," << g_cpmRx << ","
                       << g_interferenceTx << "," << g_interferenceBytes << ","
                       << g_interferenceDrops << "," << g_dsrcInterferenceOfferedCbr << ","
                       << g_mecUplinkPackets << ","
                       << g_mecUplinkBytes << "," << g_mecForwardedPackets << ","
                       << g_mecForwardedBytes << "," << g_mecForwardDrops << ","
                       << g_mecForwardNoReceiver << "," << FormatThesisMetric (orr) << ","
                       << FormatThesisMetric (highPdr) << ","
                       << FormatThesisMetric (lowPdr) << ","
                       << FormatThesisMetric (highLoss) << ","
                       << FormatThesisMetric (lowLoss) << ","
                       << g_thesisStats.highIdealCpmRx << "," << g_thesisStats.highTrueCpmRx
                       << "," << g_thesisStats.lowIdealCpmRx << ","
                       << g_thesisStats.lowTrueCpmRx << std::endl;
    }

  Simulator::Schedule (interval, &WriteObservationLog, dsrcMetrics, nrMetrics, mecMetrics, interval);
}

static void
UpdateReactiveRmrCbr (Ptr<MetricSupervisor> dsrcMetrics, Time interval)
{
  for (auto& entry : g_vehicleRuntime)
    {
      const double cbr = dsrcMetrics->getCBRPerItem (entry.first);
      const double effectiveCbr = cbr >= 0.0 ? cbr : 0.0;
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

  Simulator::Schedule (interval, &UpdateReactiveRmrCbr, dsrcMetrics, interval);
}

static void
UpdatePredictiveRmrCbr (Ptr<MetricSupervisor> dsrcMetrics, Time interval)
{
  const Time now = Simulator::Now ();
  UpdateTrafficFlowCbrPrediction (interval);
  const double activeVehicleFeature =
      g_predictiveRmrConfig.activeVehicleNorm > 0.0
          ? static_cast<double> (g_vehicleRuntime.size ()) /
                g_predictiveRmrConfig.activeVehicleNorm
          : 0.0;

  for (auto& entry : g_vehicleRuntime)
    {
      const std::string& vehicleId = entry.first;
      const double measuredCbr = dsrcMetrics->getCBRPerItem (vehicleId);
      const double currentCbr =
          std::max (measuredCbr >= 0.0 ? measuredCbr : 0.0, g_dsrcInterferenceOfferedCbr);

      Ptr<CPBasicService> dsrcCp = entry.second.dsrcContainer->getCPBasicService ();
      const uint64_t cpmTx = dsrcCp->getCpmSent ();
      const uint32_t cpmSizeBytes = dsrcCp->getLastCpmSizeBytes ();

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

      const double trafficFlowCbr =
          std::max (g_trafficFlowPrediction.trafficFlowCbr, g_dsrcInterferenceOfferedCbr);
      const double predictedCbr =
          Clamp01 (std::max (currentCbr, trafficFlowCbr) +
                   g_trafficFlowPrediction.deltaCbr +
                   (g_predictiveRmrConfig.cpmSizeWeight * cpmSizeFeature) +
                   (g_predictiveRmrConfig.cpmTxRateWeight * cpmTxRateFeature) +
                   (g_predictiveRmrConfig.activeVehicleWeight * activeVehicleFeature));

      state.predictedCbr = predictedCbr;
      state.lastCbr = currentCbr;
      state.lastUpdate = now;
      state.lastDsrcCpmTx = cpmTx;
      state.initialized = true;

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

  Simulator::Schedule (interval,
                       &UpdatePredictiveRmrCbr,
                       dsrcMetrics,
                       interval);
}

static void
GenerateDsrcInterference (Ptr<Socket> socket,
                          uint32_t pktSize,
                          Time pktInterval,
                          Time stopTime)
{
  if (Simulator::Now () >= stopTime)
    {
      socket->Close ();
      return;
    }

  const int sent = socket->Send (Create<Packet> (pktSize));
  if (sent >= 0)
    {
      ++g_interferenceTx;
      g_interferenceBytes += pktSize;
    }
  else
    {
      ++g_interferenceDrops;
    }

  Simulator::Schedule (pktInterval,
                       &GenerateDsrcInterference,
                       socket,
                       pktSize,
                       pktInterval,
                       stopTime);
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
      container->getCPBasicService ()->startCpmDissemination ();
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
  bool sendCpm = true;
  bool enableDcc = false;
  bool enableRouteControl = true;
  bool enablePcap = false;
  bool enableDsrcInterference = false;
  bool enableNrSensing = false;
  bool enableChannelRandomness = false;
  bool enableReactiveRmr = false;
  bool enablePredictiveRmr = false;
  bool enableMecV2n2v = false;
  bool enableMecRouteControl = false;
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
  double observationLogInterval = 1.0;
  double thesisEvalInterval = 1.0;
  double dsrcDccBitRateMbps = 6.0;
  double dsrcInterferenceStart = 1.0;
  double dsrcInterferenceStop = 0.0;
  double dsrcInterferenceIntervalMs = 20.0;
  uint32_t dsrcInterferencePacketSize = 1500;
  uint32_t dsrcInterferenceSourceVehicle = 3;
  uint32_t dsrcInterferenceNodeCount = 6;
  bool dsrcInterferencePerVehicle = false;
  double rmrCbrLow = 0.33;
  double rmrCbrHigh = 0.67;
  double rmrWeightFrequency = 1.0;
  double rmrWeightDynamics = 1.0;
  double rmrWeightDistance = 1.0;
  uint32_t rmrDeleteLow = 10;
  uint32_t rmrDeleteMiddle = 20;
  uint32_t rmrDeleteHigh = 40;
  uint32_t rmrWindowMs = 1000;
  double predictionHorizonSeconds = 2.0;
  double predictorCpmSizeNormBytes = 1200.0;
  double predictorCpmTxRateNorm = 10.0;
  double predictorActiveVehicleNorm = 100.0;
  double trafficFlowRoadLengthMeters = 2000.0;
  double trafficFlowMessageRateHz = 10.0;
  double trafficFlowAvgPacketSizeBytes = 500.0;
  double predictorCpmSizeWeight = 0.0;
  double predictorCpmTxRateWeight = 0.0;
  double predictorActiveVehicleWeight = 0.0;
  double hybridCbrMax = 0.80;
  double hybridLowV2vMinProbability = 0.20;
  double hybridLowV2vAlpha = 1.0;
  bool hybridHighPriorityDualTx = true;
  bool mecDuplicateRecovery = false;
  std::string mecRecoveryPolicyName = "staged";
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
  double mecIdealLatencyMs = 50.0;
  double mecIdealCpmIntervalMs = 100.0;
  uint32_t mecIdealPacketSizeBytes = 500;

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
  std::string summaryCsvPath = "summary.csv";
  std::string method = "legacy";

  CommandLine cmd (__FILE__);
  cmd.AddValue ("phyMode", "802.11p PHY mode", phyMode);
  cmd.AddValue ("userpriority", "EDCA User Priority for ETSI messages", userPriority);
  cmd.AddValue ("realtime", "Run with the realtime scheduler", realtime);
  cmd.AddValue ("verbose", "Enable verbose 802.11p logging", verbose);
  cmd.AddValue ("sumo-gui", "Show SUMO GUI", sumoGui);
  cmd.AddValue ("send-cpm", "Enable CPM dissemination in addition to CAM", sendCpm);
  cmd.AddValue ("enable-dcc", "Enable ETSI DCC on the 802.11p route", enableDcc);
  cmd.AddValue ("route-control", "Enable CBR-based DSRC-to-NR route switching", enableRouteControl);
  cmd.AddValue ("dsrc-dcc-bitrate-mbps", "DCC bitrate value matching the 802.11p PHY mode", dsrcDccBitRateMbps);
  cmd.AddValue ("dsrc-interference", "Enable NR sidelink UDP background traffic", enableDsrcInterference);
  cmd.AddValue ("dsrc-interference-source", "Deprecated single-vehicle background source option", dsrcInterferenceSourceVehicle);
  cmd.AddValue ("dsrc-interference-nodes", "Number of SUMO vehicle nodes sending NR sidelink background traffic", dsrcInterferenceNodeCount);
  cmd.AddValue ("dsrc-interference-per-vehicle",
                "If true, every active SUMO vehicle emits background traffic while it is active",
                dsrcInterferencePerVehicle);
  cmd.AddValue ("dsrc-interference-size", "Background packet size [bytes]", dsrcInterferencePacketSize);
  cmd.AddValue ("dsrc-interference-interval-ms", "Background traffic interval [ms]", dsrcInterferenceIntervalMs);
  cmd.AddValue ("dsrc-interference-start", "Background traffic start time [s]", dsrcInterferenceStart);
  cmd.AddValue ("dsrc-interference-stop", "Background traffic stop time [s]; 0 means sim-time", dsrcInterferenceStop);
  cmd.AddValue ("dsrc-interference-userpriority", "User Priority for background 802.11p traffic", dsrcInterferenceUserPriority);
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
  cmd.AddValue ("method",
                "Run method: legacy, no-control, reactive-rmr, predictive-rmr-v2v, predictive-rmr-v2n2v, cbr-route-nr-sidelink",
                method);
  cmd.AddValue ("reactive-rmr", "Enable thesis-style CBR-reactive CPM object deletion", enableReactiveRmr);
  cmd.AddValue ("predictive-rmr", "Use predicted CBR instead of measured CBR as the RMR input", enablePredictiveRmr);
  cmd.AddValue ("rmr-cbr-low", "CBR threshold for low/middle RMR deletion phases", rmrCbrLow);
  cmd.AddValue ("rmr-cbr-high", "CBR threshold for middle/high RMR deletion phases", rmrCbrHigh);
  cmd.AddValue ("rmr-delete-low", "Objects deleted per CPM in low CBR phase", rmrDeleteLow);
  cmd.AddValue ("rmr-delete-middle", "Objects deleted per CPM in middle CBR phase", rmrDeleteMiddle);
  cmd.AddValue ("rmr-delete-high", "Objects deleted per CPM in high CBR phase", rmrDeleteHigh);
  cmd.AddValue ("rmr-window-ms", "Time window used for Frequency-based RMR observation count", rmrWindowMs);
  cmd.AddValue ("rmr-weight-frequency", "Weight for Frequency-based RMR term", rmrWeightFrequency);
  cmd.AddValue ("rmr-weight-dynamics", "Weight for Dynamics-based RMR terms", rmrWeightDynamics);
  cmd.AddValue ("rmr-weight-distance", "Weight for Distance-based RMR term", rmrWeightDistance);
  cmd.AddValue ("prediction-horizon",
                "Prediction horizon for predictive RMR [s]",
                predictionHorizonSeconds);
  cmd.AddValue ("traffic-flow-road-length",
                "Road segment length [m] used by the RSU traffic-flow CBR predictor",
                trafficFlowRoadLengthMeters);
  cmd.AddValue ("traffic-flow-message-rate",
                "Message generation rate lambda [Hz] used by the RSU traffic-flow CBR predictor",
                trafficFlowMessageRateHz);
  cmd.AddValue ("traffic-flow-avg-packet-size",
                "Fallback average CPM packet size [bytes] used by the RSU traffic-flow CBR predictor",
                trafficFlowAvgPacketSizeBytes);
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
                "Use DSRC V2V and MEC V2N2V together for high-priority congested CPMs",
                hybridHighPriorityDualTx);
  cmd.AddValue ("mec-duplicate-recovery",
                "Send a full CPM copy through MEC while DSRC sends the RMR-reduced CPM under predictive congestion",
                mecDuplicateRecovery);
  cmd.AddValue ("mec-recovery-policy",
                "MEC recovery policy: offload-only, staged, duplicate-always",
                mecRecoveryPolicyName);
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
                "Fixed V2N2V latency [ms] used by --mec-ideal-link",
                mecIdealLatencyMs);
  cmd.AddValue ("mec-ideal-cpm-interval-ms",
                "CPM generation interval [ms] used by --mec-ideal-link",
                mecIdealCpmIntervalMs);
  cmd.AddValue ("mec-ideal-packet-size",
                "Abstract MEC CPM packet size [bytes] used for V2N load accounting",
                mecIdealPacketSizeBytes);
  cmd.AddValue ("mec-srs-periodicity",
                "LTE eNB RRC SRS periodicity [ms] for MEC Uu; use 320 for many UEs",
                mecSrsPeriodicity);
  cmd.AddValue ("pcap", "Enable 802.11p PCAP output", enablePcap);
  cmd.AddValue ("sim-time", "Simulation time [s]", simTime);
  cmd.AddValue ("baseline", "PRR baseline [m]", baselinePrR);
  cmd.AddValue ("tx-power", "Tx power [dBm]", txPower);
  cmd.AddValue ("switch-cbr", "CBR threshold to switch DSRC V2V to NR route", switchCbr);
  cmd.AddValue ("release-cbr", "CBR threshold to release NR route back to DSRC V2V", releaseCbr);
  cmd.AddValue ("route-check-interval", "Route control interval [s]", routeCheckInterval);
  cmd.AddValue ("cbr-window-ms", "MetricSupervisor CBR window [ms]", cbrWindowMs);
  cmd.AddValue ("cbr-alpha", "MetricSupervisor CBR exponential average alpha", cbrAlpha);
  cmd.AddValue ("cbr-log", "CSV file for MetricSupervisor CBR values", cbrLogPath);
  cmd.AddValue ("route-log", "CSV file for route switching events", routeLogPath);
  cmd.AddValue ("observation-log", "CSV file for CBR/PRR/loss/route-count time series", observationLogPath);
  cmd.AddValue ("summary-csv", "CSV file for one-line thesis summary metrics", summaryCsvPath);
  cmd.AddValue ("observation-log-interval", "Observation CSV write interval [s]", observationLogInterval);
  cmd.AddValue ("thesis-eval-interval", "Evaluation interval for thesis 4.3 metrics [s]", thesisEvalInterval);
  cmd.AddValue ("sumo-folder", "Folder containing SUMO mobility trace", sumoFolder);
  cmd.AddValue ("mob-trace", "SUMO route file name", mobTrace);
  cmd.AddValue ("sumo-config", "SUMO configuration file", sumoConfig);
  cmd.AddValue ("sumo-wait-socket", "Seconds to wait for SUMO/TraCI socket startup", sumoWaitForSocket);
  cmd.AddValue ("sumo-sync-interval", "SUMO/ns-3 synchronization interval [s]", sumoSyncInterval);
  cmd.AddValue ("sumo-port", "TCP port used by SUMO/TraCI", sumoPort);
  cmd.AddValue ("sumo-extra-options", "Additional command line options passed to SUMO", sumoAdditionalOptions);
  cmd.AddValue ("nr-sensing", "Enable NR-V2X sidelink sensing", enableNrSensing);
  cmd.AddValue ("nr-channel-randomness", "Enable NR channel randomness/shadowing", enableChannelRandomness);
  cmd.AddValue ("nr-mcs", "Fixed NR sidelink MCS", mcs);
  cmd.Parse (argc, argv);

  if (method == "no-control")
    {
      enableReactiveRmr = false;
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
    }

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
  if (observationLogInterval <= 0.0)
    {
      NS_FATAL_ERROR ("observation-log-interval must be greater than 0");
    }
  if (thesisEvalInterval <= 0.0)
    {
      NS_FATAL_ERROR ("thesis-eval-interval must be greater than 0");
    }
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
  if (trafficFlowMessageRateHz < 0.0)
    {
      NS_FATAL_ERROR ("traffic-flow-message-rate must be greater than or equal to 0");
    }
  if (trafficFlowAvgPacketSizeBytes <= 0.0)
    {
      NS_FATAL_ERROR ("traffic-flow-avg-packet-size must be greater than 0");
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
  if (mecIdealCpmIntervalMs <= 0.0)
    {
      NS_FATAL_ERROR ("mec-ideal-cpm-interval-ms must be greater than 0");
    }
  if (mecIdealPacketSizeBytes == 0)
    {
      NS_FATAL_ERROR ("mec-ideal-packet-size must be greater than 0");
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
  g_predictiveRmrConfig.trafficFlowMessageRateHz = trafficFlowMessageRateHz;
  g_predictiveRmrConfig.trafficFlowAvgPacketSizeBytes = trafficFlowAvgPacketSizeBytes;
  g_predictiveRmrConfig.cpmSizeWeight = predictorCpmSizeWeight;
  g_predictiveRmrConfig.cpmTxRateWeight = predictorCpmTxRateWeight;
  g_predictiveRmrConfig.activeVehicleWeight = predictorActiveVehicleWeight;
  g_predictiveChannelRateBps = dsrcDccBitRateMbps * 1e6;
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
  g_idealMecLatency = MilliSeconds (mecIdealLatencyMs);
  g_idealMecInterval = MilliSeconds (mecIdealCpmIntervalMs);
  g_idealMecPacketSizeBytes = mecIdealPacketSizeBytes;
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
  std::cout << "Vehicles=" << numberOfNodes << ", initial route=NR-V2X sidelink"
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
            << (enablePredictiveRmr && !g_trafficFlowRsus.empty () ? "enabled" : "disabled")
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

  MobilityHelper mobility;
  mobility.Install (vehicleNodes);

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

  int64_t stream = 1;
  stream += nrHelper->AssignStreams (nrDevices, stream);
  nrSlHelper->AssignStreams (nrDevices, stream);
    }

  InternetStackHelper internet;
  internet.Install (vehicleNodes);

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
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
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
  if (!cbrLogPath.empty ())
    {
      dsrcMetrics->writeCBRtoCSV (cbrLogPath);
      dsrcMetrics->enableCBRWriteToFile ();
    }
  dsrcMetrics->startCheckCBR (numberOfNodes);

  MetricSupervisor nrMetricsObj (baselinePrR);
  Ptr<MetricSupervisor> nrMetrics = &nrMetricsObj;
  nrMetrics->setTraCIClient (sumoClient);

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
                                                std::placeholders::_5));
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
                                                  std::placeholders::_5));
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
                                                   std::placeholders::_5));
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

    ActivateRoute (vehicleId, ActiveRoute::NrSidelinkV2v, -1.0, "initial_nr_sidelink_route", true);
    if (enableDsrcInterference &&
        (dsrcInterferencePerVehicle || stationId <= dsrcInterferenceNodeCount))
      {
        const uint32_t sourceIndex = static_cast<uint32_t> (stationId - 1);
        const Time stopTime =
            Seconds (dsrcInterferenceStop > 0.0 ? dsrcInterferenceStop : simTime);
        const double phaseStepMs =
            dsrcInterferenceIntervalMs /
            static_cast<double> (dsrcInterferencePerVehicle ? numberOfNodes
                                                            : dsrcInterferenceNodeCount);
        Simulator::ScheduleWithContext (node->GetId (),
                                        Seconds (dsrcInterferenceStart) +
                                            MilliSeconds (phaseStepMs * sourceIndex),
                                        &GenerateDsrcInterference,
                                        dsrcInterferenceSockets[sourceIndex],
                                        dsrcInterferencePacketSize,
                                        MilliSeconds (dsrcInterferenceIntervalMs),
                                        stopTime);
      }
    return node;
  };

  SHUTDOWN_FCN shutdownVehicle = [] (Ptr<Node> exNode, std::string vehicleId)
  {
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel> ();
    mob->SetPosition (Vector (-1000.0 + (rand () % 25), 320.0 + (rand () % 25), 250.0));

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

  sumoClient->SumoSetup (setupNewVehicle, shutdownVehicle);
  if (enableMecV2n2v && g_useIdealMecLink)
    {
      Simulator::Schedule (g_idealMecInterval, &GenerateIdealMecCpm);
    }
  if (enableRouteControl)
    {
      Simulator::Schedule (Seconds (routeCheckInterval),
                           &CheckRoutesByCbr,
                           dsrcMetrics,
                           switchCbr,
                           releaseCbr,
                           Seconds (routeCheckInterval));
    }
  if (enablePredictiveRmr)
    {
      Simulator::Schedule (Seconds (routeCheckInterval),
                           &UpdatePredictiveRmrCbr,
                           dsrcMetrics,
                           Seconds (routeCheckInterval));
      if (enableMecRouteControl)
        {
          Simulator::Schedule (Seconds (routeCheckInterval) + MicroSeconds (1),
                               &CheckHybridRoutesByPredictedCbrToMec,
                               dsrcMetrics,
                               Seconds (routeCheckInterval));
        }
    }
  else if (enableReactiveRmr)
    {
      Simulator::Schedule (Seconds (routeCheckInterval),
                           &UpdateReactiveRmrCbr,
                           dsrcMetrics,
                           Seconds (routeCheckInterval));
    }
  Simulator::Schedule (Seconds (observationLogInterval),
                       &WriteObservationLog,
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

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  AccumulateThesisPacketLossCounters (dsrcMetrics,
                                      nrMetrics,
                                      mecMetrics,
                                      sumoClient,
                                      baselinePrR);

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
      ThesisPacketLossRatePercent (g_thesisStats.nrIdealCpmRx, g_thesisStats.nrTrueCpmRx);
  const double mecRouteLoss =
      ThesisPacketLossRatePercent (g_thesisStats.mecIdealCpmRx, g_thesisStats.mecTrueCpmRx);
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
          ? static_cast<double> (g_idealMecLatency.GetMilliSeconds ())
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
  const double effectiveChannelBusyRatio =
      std::max (static_cast<double> (dsrcMetrics->getAverageCBROverall ()),
                g_dsrcInterferenceOfferedCbr);
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
  std::cout << "Thesis 4.3 CPM packet loss by route (%): 802.11p="
            << FormatThesisMetric (dsrcRouteLoss)
            << ", NR-V2X sidelink V2V=" << FormatThesisMetric (nrRouteLoss)
            << ", MEC V2N2V=" << FormatThesisMetric (mecRouteLoss)
            << std::endl;
  std::cout << "802.11p average CBR: " << dsrcMetrics->getAverageCBROverall () << std::endl;
  std::cout << "802.11p average PRR: " << dsrcMetrics->getAveragePRR_overall () << std::endl;
  std::cout << "802.11p average packet loss: "
            << PacketLossFromPrr (dsrcMetrics->getAveragePRR_overall (),
                                  dsrcMetrics->getNumberTx_overall ())
            << std::endl;
  std::cout << "802.11p TX/RX: " << dsrcMetrics->getNumberTx_overall () << "/"
            << dsrcMetrics->getNumberRx_overall () << std::endl;
  std::cout << "802.11p average latency (ms): " << dsrcMetrics->getAverageLatency_overall ()
            << std::endl;
  std::cout << "NR-V2X sidelink V2V average PRR: " << nrMetrics->getAveragePRR_overall ()
            << std::endl;
  std::cout << "NR-V2X sidelink V2V average packet loss: "
            << PacketLossFromPrr (nrMetrics->getAveragePRR_overall (),
                                  nrMetrics->getNumberTx_overall ())
            << std::endl;
  std::cout << "NR-V2X sidelink V2V average latency (ms): "
            << nrMetrics->getAverageLatency_overall () << std::endl;
  std::cout << "NR-V2X sidelink V2V CPM latency percentiles (ms): p50="
            << nrLatencyP50 << ", p90=" << nrLatencyP90 << ", p95=" << nrLatencyP95
            << ", p99=" << nrLatencyP99 << std::endl;
  std::cout << "MEC V2N2V average PRR: " << mecMetrics->getAveragePRR_overall ()
            << std::endl;
  std::cout << "MEC V2N2V average packet loss: "
            << PacketLossFromPrr (mecMetrics->getAveragePRR_overall (),
                                  mecMetrics->getNumberTx_overall ())
            << std::endl;
  std::cout << "MEC V2N2V TX/RX: " << mecTxOverall << "/" << mecRxOverall << std::endl;
  std::cout << "MEC V2N2V average latency (ms): " << mecLatencyAvg << std::endl;
  std::cout << "MEC V2N2V CPM latency percentiles (ms): p50="
            << mecLatencyP50 << ", p90=" << mecLatencyP90 << ", p95=" << mecLatencyP95
            << ", p99=" << mecLatencyP99 << std::endl;
  std::cout << "MEC uplink/forward/drop/no-receiver: " << g_mecUplinkPackets << "/"
            << g_mecForwardedPackets << "/" << g_mecForwardDrops << "/"
            << g_mecForwardNoReceiver << std::endl;
  std::cout << "DSRC interference TX/drops/bytes: " << g_interferenceTx << "/"
            << g_interferenceDrops << "/" << g_interferenceBytes << std::endl;

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
          << "nr_latency_ms,nr_latency_p50_ms,nr_latency_p90_ms,nr_latency_p95_ms,nr_latency_p99_ms,"
          << "mec_tx,mec_rx,mec_latency_ms,mec_latency_p50_ms,mec_latency_p90_ms,"
          << "mec_latency_p95_ms,mec_latency_p99_ms,interference_tx,interference_drops,"
          << "interference_bytes" << std::endl;
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
                 << nrMetrics->getAverageLatency_overall () << ","
                 << nrLatencyP50 << "," << nrLatencyP90 << "," << nrLatencyP95 << ","
                 << nrLatencyP99 << ","
                 << mecTxOverall << ","
                 << mecRxOverall << ","
                 << mecLatencyAvg << ","
                 << mecLatencyP50 << "," << mecLatencyP90 << "," << mecLatencyP95 << ","
                 << mecLatencyP99 << "," << g_interferenceTx << ","
                 << g_interferenceDrops << "," << g_interferenceBytes << std::endl;
    }

  if (g_routeLog.is_open ())
    {
      g_routeLog.close ();
    }
  if (g_observationLog.is_open ())
    {
      g_observationLog.close ();
    }

  Simulator::Destroy ();
  return 0;
}
