// ============================================================================
// 研究用シミュレーションスクリプト: highway MEC / V2X の土台
// 目的:
//   1. SUMO GUI の delay 値で表示速度を調整できるようにする
//   2. SUMO GUI 上に RSU を見える形で描画する
//   3. deltaT ごとに RSU/車両ごとの CBR を CSV ログへ出力する
// ============================================================================

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/traci-module.h"
#include "ns3/wave-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("HighwayMecSimulation");

// CBR の状態を 3 段階で表す。
enum class CbrPhase
{
  Normal,
  Transition,
  Congested
};

// 評価対象パケットの重要度。
enum class PriorityClass
{
  Low,
  High
};

// 送信に使う経路。
enum class RouteType
{
  V2V,
  V2N2V
};

// 実験で使う経路モード。
enum class RouteMode
{
  HybridAdaptive,
  V2vOnly,
  V2n2vOnly,
  Redundant
};

// 車両側の送信方針を表す。
struct TransmissionDecision
{
  double lowPriorityV2vProbability;
  double lowPriorityOffloadProbability;
  bool lowPriorityV2n;
  bool highPriorityV2v;
  bool highPriorityV2n;
};

// MEC 側の簡易制御結果を表す。
struct MecDecision
{
  double estimatedDownlinkLoadBps;
  double geocastRadiusM;
};

// 経路ごとの到達・遅延モデル。
struct RouteAttempt
{
  bool attempted = false;
  bool delivered = false;
  double latencyMs = 0.0;
  double pdr = 0.0;
  double sinrDb = 0.0;
  double macCollisionProbability = 0.0;
  double resourceBusyRatio = 0.0;
  double queueDelayMs = 0.0;
  double schedulingDelayMs = 0.0;
  double backhaulDelayMs = 0.0;
  double coreNetworkDelayMs = 0.0;
  uint32_t payloadBytes = 0;
};

// CPMに含まれる物標情報の価値を評価するための特徴量。
struct CpmContentFeatures
{
  uint32_t groundTruthObjects;
  uint32_t visibleObjects;
  double averageObjectDistanceM;
  double nearestObjectDistanceM;
  double redundancyRatio;
  double averageAoiSec;
  double maxAoiSec;
  double blindSpotMissRate;
  double occlusionRisk;
  double importanceScore;
  std::vector<std::string> visibleObjectIds;
};

// ORR計算用。PDRとは別に、センサ視野・遮蔽・重複を反映した認識数を持つ。
struct RecognitionEstimate
{
  double groundTruthObjects;
  double visibleObjects;
  double recognizedObjects;
  double blindSpotMissedObjects;
  double duplicateSuppressedObjects;
};

// 評価モデルのパラメータ。
struct EvaluationConfig
{
  bool enableV2v;
  bool enableV2n2v;
  RouteMode routeMode;
  uint32_t randomSeed;
  uint32_t objectsPerMessage;
  double v2vPdrNormal;
  double v2vPdrCongested;
  double v2n2vPdr;
  double v2vBaseLatencyMs;
  double v2n2vBaseLatencyMs;
  double mecProcessingLatencyMs;
  double e2eTargetLatencyMs;
  std::string communicationLogPath;
  std::string evaluationSummaryPath;
};

// RSU の位置情報。ns-3 ノードと SUMO GUI 上のマーカーで同じ座標を使う。
struct RsuInfo
{
  uint32_t index;
  double x;
  double y;
  double z;
};

// SUMO から取得した車両情報。
struct VehicleSnapshot
{
  std::string id;
  double x;
  double y;
  double speedMps;
};

// ある道路区間に含まれる車両数・平均速度・推定 CBR。
struct SegmentStats
{
  uint32_t vehicleCount;
  double averageSpeedMps;
  double cbr;
};

// RSU ごとの CBR 計算結果。
struct RsuMetric
{
  uint32_t index;
  double x;
  SegmentStats upstream;
  SegmentStats local;
  double cbrEst;
  double cbrTrendPerSec;
  CbrPhase phase;
  RouteType preferredRoute;
  bool switchedToV2n2v;
  bool switchedToV2v;
};

// ============================================================================
// RSU 側: 交通流連続の式に基づく CBR 予測器
// ============================================================================
class CbrPredictor
{
public:
  double
  PredictCbr (double cbrLocal,
              double upstreamVehicles,
              double upstreamSpeedMps,
              double localVehicles,
              double localSpeedMps,
              double segmentLengthM,
              double deltaT,
              double lambdaHz,
              double packetSizeBits,
              double channelRateBps) const
  {
    const double safeSegmentLength = std::max (segmentLengthM, 1.0);
    const double upstreamFlow = ComputeFlow (upstreamVehicles, upstreamSpeedMps, safeSegmentLength);
    const double localFlow = ComputeFlow (localVehicles, localSpeedMps, safeSegmentLength);
    const double trafficDifference = upstreamFlow - localFlow;
    const double denominator = std::max (channelRateBps, 1.0);
    const double deltaCbr = trafficDifference * deltaT * lambdaHz * packetSizeBits / denominator;

    return Clamp01 (cbrLocal + deltaCbr);
  }

private:
  // 車両数を区間長で割って密度にし、平均速度を掛けて交通流量に変換する。
  static double
  ComputeFlow (double vehicleCount, double speedMps, double segmentLengthM)
  {
    const double densityVehiclesPerM = vehicleCount / segmentLengthM;
    return densityVehiclesPerM * speedMps;
  }

  static double
  Clamp01 (double value)
  {
    return std::max (0.0, std::min (1.0, value));
  }
};

static double
Clamp01 (double value)
{
  return std::max (0.0, std::min (1.0, value));
}

static double
SafeLog10 (double value)
{
  return std::log10 (std::max (value, 1e-12));
}

static double
DbmToMw (double dbm)
{
  return std::pow (10.0, dbm / 10.0);
}

static double
MwToDbm (double mw)
{
  return 10.0 * SafeLog10 (mw);
}

static double
Logistic (double value, double midpoint, double slope)
{
  return 1.0 / (1.0 + std::exp (-(value - midpoint) / std::max (slope, 1e-9)));
}

static std::string
ToString (CbrPhase phase)
{
  switch (phase)
    {
    case CbrPhase::Normal:
      return "Normal";
    case CbrPhase::Transition:
      return "Transition";
    case CbrPhase::Congested:
      return "Congested";
    }

  return "Unknown";
}

static std::string
ToString (PriorityClass priority)
{
  return priority == PriorityClass::High ? "High" : "Low";
}

static std::string
ToString (RouteType route)
{
  return route == RouteType::V2V ? "V2V" : "V2N2V";
}

static std::string
ToString (RouteMode mode)
{
  switch (mode)
    {
    case RouteMode::HybridAdaptive:
      return "hybrid";
    case RouteMode::V2vOnly:
      return "v2v-only";
    case RouteMode::V2n2vOnly:
      return "v2n2v-only";
    case RouteMode::Redundant:
      return "redundant";
    }

  return "unknown";
}

static RouteMode
ParseRouteMode (const std::string& mode)
{
  if (mode == "hybrid")
    {
      return RouteMode::HybridAdaptive;
    }
  if (mode == "v2v-only")
    {
      return RouteMode::V2vOnly;
    }
  if (mode == "v2n2v-only")
    {
      return RouteMode::V2n2vOnly;
    }
  if (mode == "redundant")
    {
      return RouteMode::Redundant;
    }

  NS_FATAL_ERROR ("Invalid route-mode: " << mode
                                         << ". Use hybrid, v2v-only, v2n2v-only, or redundant.");
}

static CbrPhase
ClassifyCbrPhase (double cbrEst, double cbrMin, double cbrMax)
{
  if (cbrEst < cbrMin)
    {
      return CbrPhase::Normal;
    }

  if (cbrEst < cbrMax)
    {
      return CbrPhase::Transition;
    }

  return CbrPhase::Congested;
}

static TransmissionDecision
BuildTransmissionDecision (const RsuMetric& metric,
                           double cbrMin,
                           double cbrMax,
                           double pMin)
{
  TransmissionDecision decision;
  decision.lowPriorityV2vProbability = 1.0;
  decision.lowPriorityOffloadProbability = 0.0;
  decision.lowPriorityV2n = false;
  decision.highPriorityV2v = true;
  decision.highPriorityV2n = false;

  if (metric.preferredRoute == RouteType::V2V)
    {
      return decision;
    }

  const double transitionWidth = std::max (cbrMax - cbrMin, 1e-9);
  const double congestionRatio = Clamp01 ((metric.cbrEst - cbrMin) / transitionWidth);
  const double trendBoost = Clamp01 (metric.cbrTrendPerSec / 0.05);

  double offloadProbability = 0.30 + 0.55 * congestionRatio + 0.15 * trendBoost;
  if (metric.phase == CbrPhase::Congested)
    {
      offloadProbability = std::max (offloadProbability, 0.90);
    }

  decision.lowPriorityOffloadProbability = Clamp01 (offloadProbability);
  decision.lowPriorityV2vProbability =
      std::max (Clamp01 (pMin), 1.0 - decision.lowPriorityOffloadProbability);
  decision.lowPriorityV2n = true;
  decision.highPriorityV2n = true;
  return decision;
}

// ============================================================================
// 実験終了時に出力する評価指標
// ============================================================================
class EvaluationMetrics
{
public:
  explicit EvaluationMetrics (uint32_t objectsPerMessage = 1)
    : m_objectsPerMessage (std::max (1u, objectsPerMessage))
  {
  }

  void
  RecordMessage (PriorityClass priority,
                 bool delivered,
                 double bestLatencyMs,
                 bool duplicatedDelivery,
                 const CpmContentFeatures& cpm,
                 const RecognitionEstimate& recognition)
  {
    ++m_generatedPackets;
    const double generatedObjects = recognition.groundTruthObjects;
    m_generatedObjects += generatedObjects;
    m_visibleObjects += recognition.visibleObjects;
    m_blindSpotMissedObjects += recognition.blindSpotMissedObjects;
    m_duplicateSuppressedObjects += recognition.duplicateSuppressedObjects;
    m_redundancySum += cpm.redundancyRatio;
    m_aoiSumSec += cpm.averageAoiSec;
    ++m_cpmSamples;

    if (priority == PriorityClass::High)
      {
        ++m_highGeneratedPackets;
      }
    else
      {
        ++m_lowGeneratedPackets;
      }

    if (delivered)
      {
        ++m_deliveredPackets;
        m_recognizedObjects += recognition.recognizedObjects;
        m_e2eLatencySumMs += bestLatencyMs;

        if (bestLatencyMs <= m_e2eTargetLatencyMs)
          {
            ++m_withinTargetPackets;
          }

        if (priority == PriorityClass::High)
          {
            ++m_highDeliveredPackets;
          }
        else
          {
            ++m_lowDeliveredPackets;
          }
      }

    if (duplicatedDelivery)
      {
        ++m_duplicateDeliveredPackets;
      }
  }

  void
  RecordRouteAttempt (RouteType route,
                      PriorityClass priority,
                      bool delivered,
                      double latencyMs,
                      uint32_t payloadBits)
  {
    RouteStats& stats = route == RouteType::V2V ? m_v2v : m_v2n2v;
    ++stats.attemptedPackets;
    stats.attemptedBits += payloadBits;

    if (priority == PriorityClass::High)
      {
        ++stats.highAttemptedPackets;
      }
    else
      {
        ++stats.lowAttemptedPackets;
      }

    if (delivered)
      {
        ++stats.deliveredPackets;
        stats.latencySumMs += latencyMs;
        if (latencyMs <= m_e2eTargetLatencyMs)
          {
            ++stats.withinTargetPackets;
          }

        if (priority == PriorityClass::High)
          {
            ++stats.highDeliveredPackets;
          }
        else
          {
            ++stats.lowDeliveredPackets;
          }
      }
  }

  void
  RecordRouteSwitch (RouteType from, RouteType to)
  {
    if (from == RouteType::V2V && to == RouteType::V2N2V)
      {
        ++m_v2vToV2n2vSwitchEvents;
      }
    else if (from == RouteType::V2N2V && to == RouteType::V2V)
      {
        ++m_v2n2vToV2vSwitchEvents;
      }
  }

  void
  SetE2eTargetLatencyMs (double targetLatencyMs)
  {
    m_e2eTargetLatencyMs = std::max (0.0, targetLatencyMs);
  }

  void
  PrintSummary (std::ostream& os, double simTime, RouteMode routeMode) const
  {
    os << "--- 評価結果 ---" << std::endl;
    os << "route_mode=" << ToString (routeMode) << ", simulation_time_s=" << simTime << std::endl;
    os << "ORR=" << Ratio (m_recognizedObjects, m_generatedObjects)
       << ", overall_PDR=" << Ratio (m_deliveredPackets, m_generatedPackets)
       << ", avg_E2E_latency_ms=" << Average (m_e2eLatencySumMs, m_deliveredPackets)
       << ", E2E_target_ratio=" << Ratio (m_withinTargetPackets, m_deliveredPackets)
       << std::endl;
    os << "High priority PDR=" << Ratio (m_highDeliveredPackets, m_highGeneratedPackets)
       << " (" << m_highDeliveredPackets << "/" << m_highGeneratedPackets << ")"
       << ", Low priority PDR=" << Ratio (m_lowDeliveredPackets, m_lowGeneratedPackets)
       << " (" << m_lowDeliveredPackets << "/" << m_lowGeneratedPackets << ")" << std::endl;
    os << "V2V PDR=" << Ratio (m_v2v.deliveredPackets, m_v2v.attemptedPackets)
       << ", avg_latency_ms=" << Average (m_v2v.latencySumMs, m_v2v.deliveredPackets)
       << ", attempts=" << m_v2v.attemptedPackets << std::endl;
    os << "V2N2V PDR=" << Ratio (m_v2n2v.deliveredPackets, m_v2n2v.attemptedPackets)
       << ", avg_latency_ms=" << Average (m_v2n2v.latencySumMs, m_v2n2v.deliveredPackets)
       << ", attempts=" << m_v2n2v.attemptedPackets << std::endl;
    os << "redundant_success_packets=" << m_duplicateDeliveredPackets << std::endl;
    os << "v2v_to_v2n2v_switch_events=" << m_v2vToV2n2vSwitchEvents
       << ", v2n2v_to_v2v_switch_events=" << m_v2n2vToV2vSwitchEvents << std::endl;
    os << "visible_object_ratio=" << Ratio (m_visibleObjects, m_generatedObjects)
       << ", blind_spot_missed_objects=" << m_blindSpotMissedObjects
       << ", duplicate_suppressed_objects=" << m_duplicateSuppressedObjects << std::endl;
    os << "----------------" << std::endl;
  }

  void
  WriteCsv (const std::string& path, double simTime, RouteMode routeMode) const
  {
    std::ofstream out (path);
    if (!out.is_open ())
      {
        NS_FATAL_ERROR ("Evaluation summary file could not be opened: " << path);
      }

    out << "metric,value\n";
    WriteMetric (out, "route_mode", ToString (routeMode));
    WriteMetric (out, "simulation_time_s", simTime);
    WriteMetric (out, "generated_packets", m_generatedPackets);
    WriteMetric (out, "delivered_packets", m_deliveredPackets);
    WriteMetric (out, "overall_pdr", Ratio (m_deliveredPackets, m_generatedPackets));
    WriteMetric (out, "generated_objects", m_generatedObjects);
    WriteMetric (out, "recognized_objects", m_recognizedObjects);
    WriteMetric (out, "orr", Ratio (m_recognizedObjects, m_generatedObjects));
    WriteMetric (out, "visible_objects", m_visibleObjects);
    WriteMetric (out, "visible_object_ratio", Ratio (m_visibleObjects, m_generatedObjects));
    WriteMetric (out, "blind_spot_missed_objects", m_blindSpotMissedObjects);
    WriteMetric (out, "duplicate_suppressed_objects", m_duplicateSuppressedObjects);
    WriteMetric (out, "avg_cpm_redundancy", Average (m_redundancySum, m_cpmSamples));
    WriteMetric (out, "avg_cpm_aoi_s", Average (m_aoiSumSec, m_cpmSamples));
    WriteMetric (out, "high_generated_packets", m_highGeneratedPackets);
    WriteMetric (out, "high_delivered_packets", m_highDeliveredPackets);
    WriteMetric (out, "high_priority_pdr", Ratio (m_highDeliveredPackets, m_highGeneratedPackets));
    WriteMetric (out, "low_generated_packets", m_lowGeneratedPackets);
    WriteMetric (out, "low_delivered_packets", m_lowDeliveredPackets);
    WriteMetric (out, "low_priority_pdr", Ratio (m_lowDeliveredPackets, m_lowGeneratedPackets));
    WriteMetric (out, "avg_e2e_latency_ms", Average (m_e2eLatencySumMs, m_deliveredPackets));
    WriteMetric (out, "e2e_target_latency_ms", m_e2eTargetLatencyMs);
    WriteMetric (out, "e2e_target_ratio", Ratio (m_withinTargetPackets, m_deliveredPackets));
    WriteMetric (out, "redundant_success_packets", m_duplicateDeliveredPackets);
    WriteMetric (out, "v2v_to_v2n2v_switch_events", m_v2vToV2n2vSwitchEvents);
    WriteMetric (out, "v2n2v_to_v2v_switch_events", m_v2n2vToV2vSwitchEvents);
    WriteRouteMetrics (out, "v2v", m_v2v);
    WriteRouteMetrics (out, "v2n2v", m_v2n2v);
  }

private:
  struct RouteStats
  {
    uint64_t attemptedPackets = 0;
    uint64_t deliveredPackets = 0;
    uint64_t highAttemptedPackets = 0;
    uint64_t highDeliveredPackets = 0;
    uint64_t lowAttemptedPackets = 0;
    uint64_t lowDeliveredPackets = 0;
    uint64_t withinTargetPackets = 0;
    uint64_t attemptedBits = 0;
    double latencySumMs = 0.0;
  };

  static double
  Ratio (double numerator, double denominator)
  {
    return denominator == 0 ? 0.0 : static_cast<double> (numerator) / denominator;
  }

  static double
  Average (double numerator, double denominator)
  {
    return denominator == 0 ? 0.0 : numerator / denominator;
  }

  template <typename T>
  static void
  WriteMetric (std::ofstream& out, const std::string& key, const T& value)
  {
    out << key << "," << value << "\n";
  }

  static void
  WriteRouteMetrics (std::ofstream& out, const std::string& prefix, const RouteStats& stats)
  {
    WriteMetric (out, prefix + "_attempted_packets", stats.attemptedPackets);
    WriteMetric (out, prefix + "_delivered_packets", stats.deliveredPackets);
    WriteMetric (out, prefix + "_pdr", Ratio (stats.deliveredPackets, stats.attemptedPackets));
    WriteMetric (out, prefix + "_avg_latency_ms", Average (stats.latencySumMs, stats.deliveredPackets));
    WriteMetric (out, prefix + "_e2e_target_ratio",
                 Ratio (stats.withinTargetPackets, stats.deliveredPackets));
    WriteMetric (out, prefix + "_attempted_bits", stats.attemptedBits);
    WriteMetric (out, prefix + "_high_attempted_packets", stats.highAttemptedPackets);
    WriteMetric (out, prefix + "_high_delivered_packets", stats.highDeliveredPackets);
    WriteMetric (out, prefix + "_low_attempted_packets", stats.lowAttemptedPackets);
    WriteMetric (out, prefix + "_low_delivered_packets", stats.lowDeliveredPackets);
  }

  uint32_t m_objectsPerMessage;
  double m_e2eTargetLatencyMs = 50.0;
  uint64_t m_generatedPackets = 0;
  uint64_t m_deliveredPackets = 0;
  uint64_t m_highGeneratedPackets = 0;
  uint64_t m_highDeliveredPackets = 0;
  uint64_t m_lowGeneratedPackets = 0;
  uint64_t m_lowDeliveredPackets = 0;
  double m_generatedObjects = 0.0;
  double m_recognizedObjects = 0.0;
  double m_visibleObjects = 0.0;
  double m_blindSpotMissedObjects = 0.0;
  double m_duplicateSuppressedObjects = 0.0;
  double m_redundancySum = 0.0;
  double m_aoiSumSec = 0.0;
  uint64_t m_cpmSamples = 0;
  uint64_t m_withinTargetPackets = 0;
  uint64_t m_duplicateDeliveredPackets = 0;
  uint64_t m_v2vToV2n2vSwitchEvents = 0;
  uint64_t m_v2n2vToV2vSwitchEvents = 0;
  double m_e2eLatencySumMs = 0.0;
  RouteStats m_v2v;
  RouteStats m_v2n2v;
};

static MecDecision
BuildMecDecision (CbrPhase phase,
                  const TransmissionDecision& transmission,
                  double localVehicles,
                  double lambdaHz,
                  double packetSizeBits,
                  double highPriorityTrafficRatio,
                  double downlinkCapacityBps,
                  double normalGeocastRadiusM,
                  double congestedGeocastRadiusM)
{
  const double lowPriorityTrafficRatio = 1.0 - highPriorityTrafficRatio;
  const double lowPriorityOffloadRatio = transmission.lowPriorityV2n
                                           ? 1.0 - transmission.lowPriorityV2vProbability
                                           : 0.0;
  const double highPriorityOffloadRatio = transmission.highPriorityV2n ? 1.0 : 0.0;
  const double offloadedTrafficRatio =
      lowPriorityTrafficRatio * lowPriorityOffloadRatio +
      highPriorityTrafficRatio * highPriorityOffloadRatio;
  const double estimatedLoadBps = localVehicles * offloadedTrafficRatio * lambdaHz * packetSizeBits;
  const double loadRatio = estimatedLoadBps / std::max (downlinkCapacityBps, 1.0);

  MecDecision decision;
  decision.estimatedDownlinkLoadBps = estimatedLoadBps;
  decision.geocastRadiusM = normalGeocastRadiusM;

  if (phase == CbrPhase::Congested && loadRatio >= 0.8)
    {
      decision.geocastRadiusM = congestedGeocastRadiusM;
    }
  else if (phase == CbrPhase::Congested)
    {
      decision.geocastRadiusM = std::max (congestedGeocastRadiusM, normalGeocastRadiusM * 0.5);
    }
  else if (phase == CbrPhase::Transition)
    {
      decision.geocastRadiusM = std::max (congestedGeocastRadiusM, normalGeocastRadiusM * 0.75);
    }

  return decision;
}

static libsumo::TraCIColor
MakeColor (uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
  libsumo::TraCIColor color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

static libsumo::TraCIPosition
MakePosition (double x, double y)
{
  libsumo::TraCIPosition position;
  position.x = x;
  position.y = y;
  position.z = 0.0;
  return position;
}

// SUMO GUI で見やすくするため、RSU を POI と四角形ポリゴンの両方で描画する。
static void
AddVisibleRsusToSumo (Ptr<TraciClient> sumoClient, const std::vector<RsuInfo>& rsus, double markerSizeM)
{
  const libsumo::TraCIColor poiColor = MakeColor (0, 96, 255, 255);
  const libsumo::TraCIColor markerColor = MakeColor (0, 96, 255, 180);
  const double halfSize = std::max (markerSizeM, 1.0) / 2.0;

  std::vector<std::string> existingPois = sumoClient->TraCIAPI::poi.getIDList ();
  std::vector<std::string> existingPolygons = sumoClient->TraCIAPI::polygon.getIDList ();

  for (const auto& rsu : rsus)
    {
      const std::string id = "RSU_" + std::to_string (rsu.index);
      const std::string polygonId = id + "_marker";

      if (std::find (existingPois.begin (), existingPois.end (), id) != existingPois.end ())
        {
          sumoClient->TraCIAPI::poi.remove (id);
        }
      if (std::find (existingPolygons.begin (), existingPolygons.end (), polygonId) != existingPolygons.end ())
        {
          sumoClient->TraCIAPI::polygon.remove (polygonId);
        }

      sumoClient->TraCIAPI::poi.add (id, rsu.x, rsu.y, poiColor, "RSU", 20);

      libsumo::TraCIPositionVector square;
      square.push_back (MakePosition (rsu.x - halfSize, rsu.y - halfSize));
      square.push_back (MakePosition (rsu.x - halfSize, rsu.y + halfSize));
      square.push_back (MakePosition (rsu.x + halfSize, rsu.y + halfSize));
      square.push_back (MakePosition (rsu.x + halfSize, rsu.y - halfSize));
      square.push_back (MakePosition (rsu.x - halfSize, rsu.y - halfSize));
      sumoClient->TraCIAPI::polygon.add (polygonId, square, markerColor, true, "rsu.visible", 19);
    }
}

static double
EstimateCbrFromVehicleCount (uint32_t vehicleCount,
                             double lambdaHz,
                             double packetSizeBits,
                             double channelRateBps)
{
  const double loadBps = vehicleCount * lambdaHz * packetSizeBits;
  return Clamp01 (loadBps / std::max (channelRateBps, 1.0));
}

static SegmentStats
ComputeSegmentStats (const std::vector<VehicleSnapshot>& vehicles,
                     double segmentStartM,
                     double segmentEndM,
                     double lambdaHz,
                     double packetSizeBits,
                     double channelRateBps)
{
  uint32_t count = 0;
  double speedSum = 0.0;

  for (const auto& vehicle : vehicles)
    {
      if (vehicle.x >= segmentStartM && vehicle.x < segmentEndM)
        {
          ++count;
          speedSum += vehicle.speedMps;
        }
    }

  SegmentStats stats;
  stats.vehicleCount = count;
  stats.averageSpeedMps = count > 0 ? speedSum / count : 0.0;
  stats.cbr = EstimateCbrFromVehicleCount (count, lambdaHz, packetSizeBits, channelRateBps);
  return stats;
}

static uint32_t
FindNearestRsuIndex (const std::vector<RsuMetric>& metrics, double vehicleX)
{
  uint32_t nearestIndex = 0;
  double nearestDistance = std::numeric_limits<double>::max ();

  for (uint32_t i = 0; i < metrics.size (); ++i)
    {
      const double distance = std::abs (metrics[i].x - vehicleX);
      if (distance < nearestDistance)
        {
          nearestDistance = distance;
          nearestIndex = i;
        }
    }

  return nearestIndex;
}

static double
Distance2d (const VehicleSnapshot& a, const VehicleSnapshot& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt (dx * dx + dy * dy);
}

static bool
IsInForwardFov (const VehicleSnapshot& observer, const VehicleSnapshot& target, double fovDeg)
{
  const double dx = target.x - observer.x;
  const double dy = target.y - observer.y;
  if (dx <= 0.0)
    {
      return false;
    }

  const double halfFovRad = fovDeg * 0.5 * 3.14159265358979323846 / 180.0;
  return std::abs (std::atan2 (dy, dx)) <= halfFovRad;
}

static bool
IsInRearNearField (const VehicleSnapshot& observer, const VehicleSnapshot& target, double rearRangeM)
{
  return target.x < observer.x && Distance2d (observer, target) <= rearRangeM;
}

static bool
IsOccludedOnLane (const VehicleSnapshot& observer,
                  const VehicleSnapshot& target,
                  const std::vector<VehicleSnapshot>& vehicles)
{
  const double minX = std::min (observer.x, target.x);
  const double maxX = std::max (observer.x, target.x);
  const double laneToleranceM = 2.4;

  for (const auto& blocker : vehicles)
    {
      if (blocker.id == observer.id || blocker.id == target.id)
        {
          continue;
        }
      if (blocker.x > minX && blocker.x < maxX && std::abs (blocker.y - target.y) <= laneToleranceM)
        {
          return true;
        }
    }

  return false;
}

// ============================================================================
// 分散型通信基盤: NetDevice/UDP Socketへ段階移行するための足場
// ============================================================================
class DistributedV2xRuntime
{
public:
  struct Config
  {
    bool enabled = true;
    bool installWaveDevices = true;
    uint16_t udpPort = 49000;
    double txPowerDbm = 20.0;
    double pc5RateBps = 6000000.0;
    double cbrWindowSec = 2.0;
    std::string phyMode = "OfdmRate6MbpsBW10MHz";
  };

  struct RadioState
  {
    double windowStartSec = 0.0;
    double busyTimeSec = 0.0;
    double localCbr = 0.0;
    double previousLocalCbr = 0.0;
    double localCbrTrendPerSec = 0.0;
    uint64_t rxPackets = 0;
    uint64_t rxBytes = 0;
    uint64_t txPackets = 0;
    uint64_t txBytes = 0;
    bool hasMeasurement = false;
  };

  void
  Configure (NodeContainer vehicleNodes,
             NodeContainer rsuNodes,
             std::shared_ptr<std::unordered_map<std::string, uint32_t>> vehicleNodeIndex,
             const Config& config)
  {
    m_vehicleNodes = vehicleNodes;
    m_rsuNodes = rsuNodes;
    m_vehicleNodeIndex = std::move (vehicleNodeIndex);
    m_config = config;
    m_enabled = config.enabled;

    if (!m_enabled)
      {
        return;
      }

    NodeContainer communicationNodes;
    communicationNodes.Add (m_vehicleNodes);
    communicationNodes.Add (m_rsuNodes);

    if (m_config.installWaveDevices)
      {
        YansWifiPhyHelper wifiPhy;
        wifiPhy.Set ("TxPowerStart", DoubleValue (m_config.txPowerDbm));
        wifiPhy.Set ("TxPowerEnd", DoubleValue (m_config.txPowerDbm));
        YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
        Ptr<YansWifiChannel> channel = wifiChannel.Create ();
        wifiPhy.SetChannel (channel);
        wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

        NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
        Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
        wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                            "DataMode",
                                            StringValue (m_config.phyMode),
                                            "ControlMode",
                                            StringValue (m_config.phyMode),
                                            "NonUnicastMode",
                                            StringValue (m_config.phyMode));
        m_netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, communicationNodes);
      }

    InternetStackHelper internet;
    internet.Install (communicationNodes);

    Ipv4AddressHelper address;
    address.SetBase ("10.77.0.0", "255.255.0.0");
    m_interfaces = address.Assign (m_netDevices);

    InstallSockets (communicationNodes);
  }

  void
  UpdateLocalCbr (double now)
  {
    if (!m_enabled)
      {
        return;
      }

    const double windowSec = std::max (m_config.cbrWindowSec, 1e-6);
    for (auto& kv : m_radioStates)
      {
        RadioState& state = kv.second;
        const double elapsedSec = now - state.windowStartSec;
        if (elapsedSec + 1e-9 < windowSec)
          {
            continue;
          }

        state.previousLocalCbr = state.localCbr;
        state.localCbr = Clamp01 (state.busyTimeSec / std::max (elapsedSec, 1e-6));
        state.localCbrTrendPerSec =
            (state.localCbr - state.previousLocalCbr) / std::max (elapsedSec, 1e-6);
        state.busyTimeSec = 0.0;
        state.windowStartSec = now;
        state.hasMeasurement = true;
      }
  }

  void
  SendPacketProbe (const std::string& vehicleId,
                   RouteType route,
                   uint32_t payloadBytes,
                   double queueDelayMs)
  {
    if (!m_enabled || route != RouteType::V2V || m_vehicleNodeIndex == nullptr)
      {
        return;
      }

    const auto indexIt = m_vehicleNodeIndex->find (vehicleId);
    if (indexIt == m_vehicleNodeIndex->end () || indexIt->second >= m_vehicleSockets.size ())
      {
        return;
      }

    Ptr<Socket> socket = m_vehicleSockets[indexIt->second];
    if (socket == nullptr)
      {
        return;
      }

    const uint32_t nodeId = m_vehicleNodes.Get (indexIt->second)->GetId ();
    AccountChannelBusy (nodeId, payloadBytes, true);
    Address broadcast = InetSocketAddress (Ipv4Address ("255.255.255.255"), m_config.udpPort);
    Simulator::Schedule (MilliSeconds (std::max (0.0, queueDelayMs)),
                         &DistributedV2xRuntime::SendPacketNow,
                         this,
                         socket,
                         payloadBytes,
                         broadcast);
  }

  void
  AddSensedBusyTime (const std::string& vehicleId, double busyTimeSec)
  {
    if (!m_enabled || m_vehicleNodeIndex == nullptr)
      {
        return;
      }

    const auto indexIt = m_vehicleNodeIndex->find (vehicleId);
    if (indexIt == m_vehicleNodeIndex->end () || indexIt->second >= m_vehicleNodes.GetN ())
      {
        return;
      }

    const uint32_t nodeId = m_vehicleNodes.Get (indexIt->second)->GetId ();
    m_radioStates[nodeId].busyTimeSec += std::max (0.0, busyTimeSec);
  }

  double
  GetLocalCbr (const std::string& vehicleId, double fallback) const
  {
    const RadioState* state = GetVehicleRadioState (vehicleId);
    if (state == nullptr || !state->hasMeasurement)
      {
        return fallback;
      }
    return state->localCbr;
  }

  double
  GetLocalCbrTrend (const std::string& vehicleId, double fallback) const
  {
    const RadioState* state = GetVehicleRadioState (vehicleId);
    if (state == nullptr || !state->hasMeasurement)
      {
        return fallback;
      }
    return state->localCbrTrendPerSec;
  }

private:
  void
  InstallSockets (const NodeContainer& communicationNodes)
  {
    const TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    m_vehicleSockets.resize (m_vehicleNodes.GetN ());

    for (uint32_t i = 0; i < communicationNodes.GetN (); ++i)
      {
        Ptr<Node> node = communicationNodes.Get (i);
        Ptr<Socket> socket = Socket::CreateSocket (node, tid);
        if (socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_config.udpPort)) == -1)
          {
            NS_FATAL_ERROR ("Failed to bind V2X UDP socket on node " << node->GetId ());
          }
        socket->SetAllowBroadcast (true);
        socket->SetRecvCallback (MakeCallback (&DistributedV2xRuntime::HandleReceive, this));

        m_radioStates[node->GetId ()].windowStartSec = Simulator::Now ().GetSeconds ();
        if (i < m_vehicleNodes.GetN ())
          {
            m_vehicleSockets[i] = socket;
          }
        else
          {
            m_rsuSockets.push_back (socket);
          }
      }
  }

  void
  SendPacketNow (Ptr<Socket> socket, uint32_t payloadBytes, Address destination)
  {
    Ptr<Packet> packet = Create<Packet> (payloadBytes);
    socket->SendTo (packet, 0, destination);
  }

  void
  HandleReceive (Ptr<Socket> socket)
  {
    Address from;
    Ptr<Packet> packet;
    while ((packet = socket->RecvFrom (from)) != nullptr)
      {
        const uint32_t nodeId = socket->GetNode ()->GetId ();
        AccountChannelBusy (nodeId, packet->GetSize (), false);
      }
  }

  void
  AccountChannelBusy (uint32_t nodeId, uint32_t payloadBytes, bool tx)
  {
    RadioState& state = m_radioStates[nodeId];
    const double airtimeSec =
        (static_cast<double> (std::max (payloadBytes, 1u)) * 8.0) /
        std::max (m_config.pc5RateBps, 1.0);
    state.busyTimeSec += airtimeSec;
    if (tx)
      {
        ++state.txPackets;
        state.txBytes += payloadBytes;
      }
    else
      {
        ++state.rxPackets;
        state.rxBytes += payloadBytes;
      }
  }

  const RadioState*
  GetVehicleRadioState (const std::string& vehicleId) const
  {
    if (m_vehicleNodeIndex == nullptr)
      {
        return nullptr;
      }

    const auto indexIt = m_vehicleNodeIndex->find (vehicleId);
    if (indexIt == m_vehicleNodeIndex->end () || indexIt->second >= m_vehicleNodes.GetN ())
      {
        return nullptr;
      }

    const uint32_t nodeId = m_vehicleNodes.Get (indexIt->second)->GetId ();
    const auto stateIt = m_radioStates.find (nodeId);
    return stateIt == m_radioStates.end () ? nullptr : &stateIt->second;
  }

  bool m_enabled = false;
  Config m_config;
  NodeContainer m_vehicleNodes;
  NodeContainer m_rsuNodes;
  NetDeviceContainer m_netDevices;
  Ipv4InterfaceContainer m_interfaces;
  std::vector<Ptr<Socket>> m_vehicleSockets;
  std::vector<Ptr<Socket>> m_rsuSockets;
  std::shared_ptr<std::unordered_map<std::string, uint32_t>> m_vehicleNodeIndex;
  std::unordered_map<uint32_t, RadioState> m_radioStates;
};

// ============================================================================
// deltaT ごとの CBR ロガー
// ============================================================================
class CbrLogger
{
public:
  CbrLogger (Ptr<TraciClient> sumoClient,
             std::vector<RsuInfo> rsus,
             double roadLengthM,
             double segmentLengthM,
             double deltaT,
             double lambdaHz,
             double packetSizeBits,
             double pc5RateBps,
             double cbrMin,
             double cbrMax,
             double pMin,
             double routeSwitchCbr,
             double routeReleaseCbr,
             double routeSwitchTrendPerSec,
             double highPriorityTrafficRatio,
             double downlinkCapacityBps,
             double normalGeocastRadiusM,
             double congestedGeocastRadiusM,
             double perceptionRangeM,
             double rearSensingRangeM,
             double sensorFovDeg,
             double aoiHighThresholdSec,
             double aoiCriticalThresholdSec,
             double importanceThreshold,
             uint32_t ipUdpHeaderBytes,
             uint32_t securityOverheadBytes,
             uint32_t macOverheadBytes,
             double baseStationSchedulingIntervalMs,
             double backhaulDelayMs,
             double coreNetworkDelayMs,
             double channelSensingGain,
             double v2vTxPowerDbm,
             std::shared_ptr<DistributedV2xRuntime> distributedRuntime,
             double simTime,
             std::string rsuLogPath,
             std::string vehicleLogPath,
             EvaluationConfig evaluationConfig)
    : m_sumoClient (sumoClient),
      m_rsus (std::move (rsus)),
      m_roadLengthM (roadLengthM),
      m_segmentLengthM (segmentLengthM),
      m_deltaT (deltaT),
      m_lambdaHz (lambdaHz),
      m_packetSizeBits (packetSizeBits),
      m_pc5RateBps (pc5RateBps),
      m_cbrMin (cbrMin),
      m_cbrMax (cbrMax),
      m_pMin (pMin),
      m_routeSwitchCbr (routeSwitchCbr),
      m_routeReleaseCbr (routeReleaseCbr),
      m_routeSwitchTrendPerSec (routeSwitchTrendPerSec),
      m_highPriorityTrafficRatio (highPriorityTrafficRatio),
      m_downlinkCapacityBps (downlinkCapacityBps),
      m_normalGeocastRadiusM (normalGeocastRadiusM),
      m_congestedGeocastRadiusM (congestedGeocastRadiusM),
      m_perceptionRangeM (perceptionRangeM),
      m_rearSensingRangeM (rearSensingRangeM),
      m_sensorFovDeg (sensorFovDeg),
      m_aoiHighThresholdSec (aoiHighThresholdSec),
      m_aoiCriticalThresholdSec (aoiCriticalThresholdSec),
      m_importanceThreshold (importanceThreshold),
      m_ipUdpHeaderBytes (ipUdpHeaderBytes),
      m_securityOverheadBytes (securityOverheadBytes),
      m_macOverheadBytes (macOverheadBytes),
      m_channelPacketSizeBits (
          m_packetSizeBits +
          8.0 * static_cast<double> (m_ipUdpHeaderBytes + m_securityOverheadBytes + m_macOverheadBytes)),
      m_baseStationSchedulingIntervalMs (baseStationSchedulingIntervalMs),
      m_backhaulDelayMs (backhaulDelayMs),
      m_coreNetworkDelayMs (coreNetworkDelayMs),
      m_channelSensingGain (channelSensingGain),
      m_v2vTxPowerDbm (v2vTxPowerDbm),
      m_distributedRuntime (std::move (distributedRuntime)),
      m_simTime (simTime),
      m_rsuLogPath (std::move (rsuLogPath)),
      m_vehicleLogPath (std::move (vehicleLogPath)),
      m_config (std::move (evaluationConfig)),
      m_metrics (m_config.objectsPerMessage),
      m_rng (m_config.randomSeed),
      m_uniform (0.0, 1.0),
      m_lastCbrEstByRsu (m_rsus.size (), std::numeric_limits<double>::quiet_NaN ()),
      m_preferredRouteByRsu (m_rsus.size (), RouteType::V2V)
  {
    m_metrics.SetE2eTargetLatencyMs (m_config.e2eTargetLatencyMs);
  }

  void
  Start ()
  {
    if (m_sumoClient == nullptr)
      {
        std::cout << "SUMOを起動していないため、CBRログは固定値デモのみになります。" << std::endl;
        return;
      }

    m_rsuLog.open (m_rsuLogPath);
    m_vehicleLog.open (m_vehicleLogPath);
    if (!m_config.communicationLogPath.empty ())
      {
        m_communicationLog.open (m_config.communicationLogPath);
      }
    if (!m_rsuLog.is_open () || !m_vehicleLog.is_open ())
      {
        NS_FATAL_ERROR ("CBR log file could not be opened.");
      }
    if (!m_config.communicationLogPath.empty () && !m_communicationLog.is_open ())
      {
        NS_FATAL_ERROR ("Communication log file could not be opened: "
                        << m_config.communicationLogPath);
      }

    m_rsuLog << "time_s,rsu_id,rsu_x_m,local_vehicle_count,local_avg_speed_mps,"
             << "upstream_vehicle_count,upstream_avg_speed_mps,cbr,cbr_est,cbr_trend_per_s,"
             << "phase,preferred_route,switched_to_v2n2v,switched_to_v2v,is_congested\n";
    m_vehicleLog << "time_s,vehicle_id,x_m,y_m,speed_mps,nearest_rsu_id,"
                 << "vehicle_cbr,nearest_rsu_cbr_est,cbr_trend_per_s,phase,preferred_route,"
                 << "is_congested\n";
    if (m_communicationLog.is_open ())
      {
        m_communicationLog << "time_s,packet_id,vehicle_id,priority,phase,cbr_est,"
                           << "cbr_trend_per_s,preferred_route,switched_to_v2n2v,"
                           << "low_priority_v2v_probability,low_priority_offload_probability,"
                           << "ground_truth_objects,visible_objects,recognized_objects,"
                           << "redundancy_ratio,aoi_s,blind_spot_miss_rate,importance_score,"
                           << "v2v_attempted,v2v_delivered,v2v_latency_ms,v2v_pdr,"
                           << "v2v_sinr_db,v2v_mac_collision_probability,v2v_resource_busy_ratio,"
                           << "v2v_queue_delay_ms,v2v_payload_bytes,"
                           << "v2n2v_attempted,v2n2v_delivered,v2n2v_latency_ms,v2n2v_pdr,"
                           << "v2n2v_sinr_db,v2n2v_queue_drop_probability,"
                           << "v2n2v_resource_busy_ratio,v2n2v_queue_delay_ms,"
                           << "v2n2v_scheduling_delay_ms,backhaul_delay_ms,"
                           << "core_network_delay_ms,v2n2v_payload_bytes,delivered,best_latency_ms,"
                           << "e2e_within_target,geocast_radius_m,dl_load_bps\n";
      }

    Simulator::Schedule (Seconds (m_deltaT), &CbrLogger::WriteOnce, this);
  }

  void
  PrintEvaluationSummary (std::ostream& os) const
  {
    m_metrics.PrintSummary (os, m_simTime, m_config.routeMode);
  }

  void
  WriteEvaluationSummary () const
  {
    if (!m_config.evaluationSummaryPath.empty ())
      {
        m_metrics.WriteCsv (m_config.evaluationSummaryPath, m_simTime, m_config.routeMode);
      }
  }

private:
  std::vector<VehicleSnapshot>
  ReadVehicles () const
  {
    std::vector<VehicleSnapshot> vehicles;
    const std::vector<std::string> ids = m_sumoClient->TraCIAPI::vehicle.getIDList ();
    vehicles.reserve (ids.size ());

    for (const auto& id : ids)
      {
        const libsumo::TraCIPosition position = m_sumoClient->TraCIAPI::vehicle.getPosition (id);
        VehicleSnapshot snapshot;
        snapshot.id = id;
        snapshot.x = position.x;
        snapshot.y = position.y;
        snapshot.speedMps = m_sumoClient->TraCIAPI::vehicle.getSpeed (id);
        vehicles.push_back (snapshot);
      }

    return vehicles;
  }

  std::vector<RsuMetric>
  BuildRsuMetrics (const std::vector<VehicleSnapshot>& vehicles) const
  {
    std::vector<RsuMetric> metrics;
    metrics.reserve (m_rsus.size ());

    for (const auto& rsu : m_rsus)
      {
        const double halfSegment = m_segmentLengthM / 2.0;
        const double localStart = std::max (0.0, rsu.x - halfSegment);
        const double localEnd = std::min (m_roadLengthM, rsu.x + halfSegment);
        const double upstreamStart = std::max (0.0, localStart - m_segmentLengthM);
        const double upstreamEnd = localStart;

        SegmentStats local = ComputeSegmentStats (vehicles,
                                                  localStart,
                                                  localEnd,
                                                  m_lambdaHz,
                                                  m_channelPacketSizeBits,
                                                  m_pc5RateBps);

        SegmentStats upstream;
        if (upstreamEnd > upstreamStart)
          {
            upstream = ComputeSegmentStats (vehicles,
                                            upstreamStart,
                                            upstreamEnd,
                                            m_lambdaHz,
                                            m_channelPacketSizeBits,
                                            m_pc5RateBps);
          }
        else
          {
            // 先頭 RSU には上流区間がないため、境界条件として自区間を流入側にも使う。
            upstream = local;
          }

        const double cbrEst = m_predictor.PredictCbr (local.cbr,
                                                      upstream.vehicleCount,
                                                      upstream.averageSpeedMps,
                                                      local.vehicleCount,
                                                      local.averageSpeedMps,
                                                      m_segmentLengthM,
                                                      m_deltaT,
                                                      m_lambdaHz,
                                                      m_channelPacketSizeBits,
                                                      m_pc5RateBps);

        RsuMetric metric;
        metric.index = rsu.index;
        metric.x = rsu.x;
        metric.upstream = upstream;
        metric.local = local;
        metric.cbrEst = cbrEst;
        metric.cbrTrendPerSec = 0.0;
        metric.phase = ClassifyCbrPhase (cbrEst, m_cbrMin, m_cbrMax);
        metric.preferredRoute = RouteType::V2V;
        metric.switchedToV2n2v = false;
        metric.switchedToV2v = false;
        metrics.push_back (metric);
      }

    return metrics;
  }

  void
  UpdateRouteControl (std::vector<RsuMetric>& metrics)
  {
    for (auto& metric : metrics)
      {
        if (metric.index >= m_lastCbrEstByRsu.size ())
          {
            m_lastCbrEstByRsu.resize (metric.index + 1,
                                      std::numeric_limits<double>::quiet_NaN ());
            m_preferredRouteByRsu.resize (metric.index + 1, RouteType::V2V);
          }

        const double previousCbr = m_lastCbrEstByRsu[metric.index];
        const bool hasPreviousCbr = std::isfinite (previousCbr);
        metric.cbrTrendPerSec =
            hasPreviousCbr ? (metric.cbrEst - previousCbr) / std::max (m_deltaT, 1e-9) : 0.0;

        const RouteType previousRoute = m_preferredRouteByRsu[metric.index];
        RouteType nextRoute = previousRoute;
        const bool predictedToCrossSoon =
            hasPreviousCbr &&
            metric.cbrEst + metric.cbrTrendPerSec * m_deltaT >= m_routeSwitchCbr;
        const bool risingFastEnough = metric.cbrTrendPerSec >= m_routeSwitchTrendPerSec;

        if (metric.cbrEst >= m_routeSwitchCbr || (predictedToCrossSoon && risingFastEnough))
          {
            nextRoute = RouteType::V2N2V;
          }
        else if (metric.cbrEst <= m_routeReleaseCbr && metric.cbrTrendPerSec <= 0.0)
          {
            nextRoute = RouteType::V2V;
          }

        metric.preferredRoute = nextRoute;
        metric.switchedToV2n2v = previousRoute == RouteType::V2V && nextRoute == RouteType::V2N2V;
        metric.switchedToV2v = previousRoute == RouteType::V2N2V && nextRoute == RouteType::V2V;

        if (previousRoute != nextRoute)
          {
            m_metrics.RecordRouteSwitch (previousRoute, nextRoute);
          }

        m_preferredRouteByRsu[metric.index] = nextRoute;
        m_lastCbrEstByRsu[metric.index] = metric.cbrEst;
      }
  }

  void
  InjectLocalChannelSensing (const std::vector<VehicleSnapshot>& vehicles)
  {
    if (m_distributedRuntime == nullptr)
      {
        return;
      }

    const uint32_t applicationPayloadBytes =
        static_cast<uint32_t> (std::ceil (m_packetSizeBits / 8.0));
    const uint32_t sensedPacketBytes =
        applicationPayloadBytes + m_ipUdpHeaderBytes + m_securityOverheadBytes + m_macOverheadBytes;
    const double packetAirtimeSec =
        static_cast<double> (sensedPacketBytes) * 8.0 / std::max (m_pc5RateBps, 1.0);

    for (const auto& observer : vehicles)
      {
        double busyTimeSec = 0.0;
        for (const auto& transmitter : vehicles)
          {
            if (observer.id == transmitter.id)
              {
                continue;
              }

            const double distanceM = Distance2d (observer, transmitter);
            if (distanceM > m_perceptionRangeM)
              {
                continue;
              }

            const double carrierSenseWeight =
                Clamp01 (1.0 - distanceM / std::max (m_perceptionRangeM, 1.0));
            busyTimeSec += m_channelSensingGain * carrierSenseWeight * m_lambdaHz * m_deltaT *
                           packetAirtimeSec;
          }

        m_distributedRuntime->AddSensedBusyTime (observer.id, busyTimeSec);
      }
  }

  RsuMetric
  BuildVehicleLocalMetric (const std::string& vehicleId,
                           const RsuMetric& nearestMetric,
                           bool updateRouteState)
  {
    if (m_distributedRuntime == nullptr)
      {
        return nearestMetric;
      }

    RsuMetric localMetric = nearestMetric;
    localMetric.cbrEst = m_distributedRuntime->GetLocalCbr (vehicleId, nearestMetric.cbrEst);
    localMetric.cbrTrendPerSec =
        m_distributedRuntime->GetLocalCbrTrend (vehicleId, nearestMetric.cbrTrendPerSec);
    localMetric.phase = ClassifyCbrPhase (localMetric.cbrEst, m_cbrMin, m_cbrMax);

    const auto lastCbrIt = m_vehicleLastCbr.find (vehicleId);
    const bool hasPreviousCbr = lastCbrIt != m_vehicleLastCbr.end ();

    RouteType previousRoute = RouteType::V2V;
    const auto routeIt = m_vehiclePreferredRoute.find (vehicleId);
    if (routeIt != m_vehiclePreferredRoute.end ())
      {
        previousRoute = routeIt->second;
      }

    if (!updateRouteState)
      {
        localMetric.preferredRoute = previousRoute;
        localMetric.switchedToV2n2v = false;
        localMetric.switchedToV2v = false;
        return localMetric;
      }

    RouteType nextRoute = previousRoute;
    const bool predictedToCrossSoon =
        hasPreviousCbr &&
        localMetric.cbrEst + localMetric.cbrTrendPerSec * m_deltaT >= m_routeSwitchCbr;
    const bool risingFastEnough = localMetric.cbrTrendPerSec >= m_routeSwitchTrendPerSec;

    if (localMetric.cbrEst >= m_routeSwitchCbr || (predictedToCrossSoon && risingFastEnough))
      {
        nextRoute = RouteType::V2N2V;
      }
    else if (localMetric.cbrEst <= m_routeReleaseCbr && localMetric.cbrTrendPerSec <= 0.0)
      {
        nextRoute = RouteType::V2V;
      }

    localMetric.preferredRoute = nextRoute;
    localMetric.switchedToV2n2v = previousRoute == RouteType::V2V && nextRoute == RouteType::V2N2V;
    localMetric.switchedToV2v = previousRoute == RouteType::V2N2V && nextRoute == RouteType::V2V;

    if (previousRoute != nextRoute)
      {
        m_metrics.RecordRouteSwitch (previousRoute, nextRoute);
      }

    m_vehiclePreferredRoute[vehicleId] = nextRoute;
    m_vehicleLastCbr[vehicleId] = localMetric.cbrEst;
    return localMetric;
  }

  CpmContentFeatures
  BuildCpmContentFeatures (double now,
                           const VehicleSnapshot& sender,
                           const std::vector<VehicleSnapshot>& vehicles) const
  {
    CpmContentFeatures features;
    features.groundTruthObjects = 0;
    features.visibleObjects = 0;
    features.averageObjectDistanceM = m_perceptionRangeM;
    features.nearestObjectDistanceM = m_perceptionRangeM;
    features.redundancyRatio = 0.0;
    features.averageAoiSec = m_aoiCriticalThresholdSec;
    features.maxAoiSec = 0.0;
    features.blindSpotMissRate = 0.0;
    features.occlusionRisk = 0.0;
    features.importanceScore = 0.0;

    double visibleDistanceSum = 0.0;
    double redundancySum = 0.0;
    double aoiSum = 0.0;
    double occlusionSum = 0.0;
    uint32_t hiddenObjects = 0;

    for (const auto& target : vehicles)
      {
        if (target.id == sender.id)
          {
            continue;
          }

        const double distanceM = Distance2d (sender, target);
        if (distanceM > m_perceptionRangeM)
          {
            continue;
          }

        ++features.groundTruthObjects;
        const bool inForwardFov = IsInForwardFov (sender, target, m_sensorFovDeg);
        const bool inRearNearField = IsInRearNearField (sender, target, m_rearSensingRangeM);
        if (!inForwardFov && !inRearNearField)
          {
            ++hiddenObjects;
            continue;
          }

        ++features.visibleObjects;
        features.visibleObjectIds.push_back (target.id);
        visibleDistanceSum += distanceM;
        features.nearestObjectDistanceM = std::min (features.nearestObjectDistanceM, distanceM);

        uint32_t observers = 0;
        for (const auto& observer : vehicles)
          {
            if (observer.id == sender.id || observer.id == target.id)
              {
                continue;
              }

            const double observerDistanceM = Distance2d (observer, target);
            if (observerDistanceM <= m_perceptionRangeM &&
                (IsInForwardFov (observer, target, m_sensorFovDeg) ||
                 IsInRearNearField (observer, target, m_rearSensingRangeM)))
              {
                ++observers;
              }
          }
        redundancySum += Clamp01 (static_cast<double> (observers) / 3.0);

        const auto lastUpdate = m_objectLastUpdateTime.find (target.id);
        const double aoiSec = lastUpdate == m_objectLastUpdateTime.end ()
                                  ? m_aoiCriticalThresholdSec
                                  : std::max (0.0, now - lastUpdate->second);
        aoiSum += aoiSec;
        features.maxAoiSec = std::max (features.maxAoiSec, aoiSec);

        if (IsOccludedOnLane (sender, target, vehicles))
          {
            occlusionSum += 1.0;
          }
      }

    if (features.groundTruthObjects == 0)
      {
        features.maxAoiSec = 0.0;
        features.averageAoiSec = 0.0;
        features.nearestObjectDistanceM = 0.0;
        return features;
      }

    features.blindSpotMissRate =
        static_cast<double> (hiddenObjects) / static_cast<double> (features.groundTruthObjects);

    if (features.visibleObjects > 0)
      {
        features.averageObjectDistanceM =
            visibleDistanceSum / static_cast<double> (features.visibleObjects);
        features.redundancyRatio = redundancySum / static_cast<double> (features.visibleObjects);
        features.averageAoiSec = aoiSum / static_cast<double> (features.visibleObjects);
        features.occlusionRisk = occlusionSum / static_cast<double> (features.visibleObjects);
      }

    const double noveltyScore = 1.0 - features.redundancyRatio;
    const double freshnessScore = Clamp01 (features.averageAoiSec / m_aoiHighThresholdSec);
    const double proximityScore =
        features.visibleObjects == 0
            ? 0.0
            : Clamp01 (1.0 - features.nearestObjectDistanceM / std::max (m_perceptionRangeM, 1.0));
    features.importanceScore = Clamp01 (0.35 * noveltyScore + 0.30 * freshnessScore +
                                        0.25 * proximityScore +
                                        0.10 * features.blindSpotMissRate);
    return features;
  }

  PriorityClass
  ClassifyPriority (const CpmContentFeatures& cpm) const
  {
    if (cpm.visibleObjects == 0)
      {
        return PriorityClass::Low;
      }

    if (cpm.maxAoiSec >= m_aoiCriticalThresholdSec ||
        cpm.nearestObjectDistanceM <= 5.0 ||
        cpm.importanceScore >= m_importanceThreshold)
      {
        return PriorityClass::High;
      }

    return PriorityClass::Low;
  }

  RecognitionEstimate
  BuildRecognitionEstimate (const CpmContentFeatures& cpm, bool delivered) const
  {
    RecognitionEstimate estimate;
    estimate.groundTruthObjects = static_cast<double> (cpm.groundTruthObjects);
    estimate.visibleObjects = static_cast<double> (cpm.visibleObjects);
    estimate.blindSpotMissedObjects =
        static_cast<double> (cpm.groundTruthObjects - cpm.visibleObjects);

    const double sensorDetectionProbability =
        Clamp01 (0.95 - 0.35 * cpm.occlusionRisk - 0.25 * cpm.blindSpotMissRate);
    const double noveltyWeight = Clamp01 (1.0 - 0.55 * cpm.redundancyRatio);
    estimate.duplicateSuppressedObjects =
        estimate.visibleObjects * std::max (0.0, 1.0 - noveltyWeight);
    estimate.recognizedObjects =
        delivered ? estimate.visibleObjects * sensorDetectionProbability * noveltyWeight : 0.0;
    return estimate;
  }

  void
  UpdateObjectFreshness (double now, const CpmContentFeatures& cpm)
  {
    for (const auto& objectId : cpm.visibleObjectIds)
      {
        m_objectLastUpdateTime[objectId] = now;
      }
  }

  std::vector<RouteType>
  SelectRoutes (PriorityClass priority, const TransmissionDecision& decision)
  {
    std::vector<RouteType> routes;
    auto addV2v = [&] ()
    {
      if (m_config.enableV2v)
        {
          routes.push_back (RouteType::V2V);
        }
    };
    auto addV2n2v = [&] ()
    {
      if (m_config.enableV2n2v)
        {
          routes.push_back (RouteType::V2N2V);
        }
    };

    switch (m_config.routeMode)
      {
      case RouteMode::V2vOnly:
        addV2v ();
        break;
      case RouteMode::V2n2vOnly:
        addV2n2v ();
        break;
      case RouteMode::Redundant:
        addV2v ();
        addV2n2v ();
        break;
      case RouteMode::HybridAdaptive:
        if (priority == PriorityClass::High)
          {
            if (decision.highPriorityV2v)
              {
                addV2v ();
              }
            if (decision.highPriorityV2n)
              {
                addV2n2v ();
              }
          }
        else if (decision.lowPriorityV2n && m_config.enableV2n2v)
          {
            if (m_config.enableV2v && m_uniform (m_rng) < decision.lowPriorityV2vProbability)
              {
                addV2v ();
              }
            else
              {
                addV2n2v ();
              }
          }
        else
          {
            addV2v ();
          }
        break;
      }

    if (routes.empty ())
      {
        addV2v ();
      }
    if (routes.empty ())
      {
        addV2n2v ();
      }

    return routes;
  }

  RouteAttempt
  SimulateRouteAttempt (RouteType route,
                        const VehicleSnapshot& vehicle,
                        const RsuMetric& metric,
                        const MecDecision& mec,
                        const CpmContentFeatures& cpm)
  {
    RouteAttempt attempt;
    attempt.attempted = true;
    attempt.delivered = false;
    attempt.latencyMs = 0.0;
    attempt.pdr = 0.0;
    attempt.sinrDb = 0.0;
    attempt.macCollisionProbability = 0.0;
    attempt.resourceBusyRatio = 0.0;
    const uint32_t applicationPayloadBytes =
        static_cast<uint32_t> (std::ceil (m_packetSizeBits / 8.0));
    attempt.payloadBytes =
        applicationPayloadBytes + m_ipUdpHeaderBytes + m_securityOverheadBytes + m_macOverheadBytes;
    const double txServiceTimeMs =
        (static_cast<double> (attempt.payloadBytes) * 8.0 /
         std::max (m_pc5RateBps, 1.0)) *
        1000.0;

    if (route == RouteType::V2V)
      {
        const double receiverDistanceM =
            cpm.visibleObjects > 0 ? cpm.averageObjectDistanceM : m_perceptionRangeM * 0.5;
        const double pathLossExponent = 2.15 + 0.35 * cpm.occlusionRisk + 0.20 * metric.cbrEst;
        const double fsplAtOneMeterDb = 47.86; // 5.9 GHz ITS band, 1 m reference.
        const double pathLossDb =
            fsplAtOneMeterDb + 10.0 * pathLossExponent * SafeLog10 (receiverDistanceM);
        const double shadowingDb = (m_uniform (m_rng) - 0.5) * 5.0;
        const double rxPowerDbm =
            m_v2vTxPowerDbm - pathLossDb - 6.0 * cpm.occlusionRisk - shadowingDb;
        const double thermalNoiseDbm = -99.0; // 10 MHz channel, receiver NF included.
        const double interferenceDbm =
            -103.0 + 36.0 * metric.cbrEst +
            10.0 * SafeLog10 (std::max (1.0, static_cast<double> (metric.local.vehicleCount)) / 12.0);
        const double totalInterferenceMw =
            DbmToMw (thermalNoiseDbm) + DbmToMw (interferenceDbm);
        attempt.sinrDb = rxPowerDbm - MwToDbm (totalInterferenceMw);

        const double transitionWidth = std::max (m_cbrMax - m_cbrMin, 1e-9);
        const double congestionRatio = Clamp01 ((metric.cbrEst - m_cbrMin) / transitionWidth);
        const double phySuccess = Logistic (attempt.sinrDb, 5.0, 2.8);
        const double densityPressure =
            Clamp01 (std::max (0.0, static_cast<double> (metric.local.vehicleCount) - 8.0) /
                     80.0);
        const double trendPressure =
            Clamp01 (std::max (0.0, metric.cbrTrendPerSec) / std::max (0.05, m_routeSwitchTrendPerSec));
        attempt.macCollisionProbability =
            Clamp01 (0.01 + 0.03 * metric.cbrEst +
                     0.22 * std::pow (metric.cbrEst, 2.4) +
                     0.08 * densityPressure +
                     0.06 * trendPressure);
        const double queueDropProbability = Clamp01 ((metric.cbrEst - 0.85) / 0.15);
        const double congestionCalibration =
            Clamp01 (m_config.v2vPdrNormal -
                     congestionRatio * (m_config.v2vPdrNormal - m_config.v2vPdrCongested));
        attempt.resourceBusyRatio = metric.cbrEst;
        attempt.pdr = Clamp01 (congestionCalibration * phySuccess *
                               (1.0 - attempt.macCollisionProbability) *
                               (1.0 - queueDropProbability));
        attempt.delivered = m_uniform (m_rng) <= attempt.pdr;
        attempt.queueDelayMs =
            txServiceTimeMs * metric.cbrEst / std::max (0.05, 1.0 - metric.cbrEst) +
            0.5 * m_uniform (m_rng);
        const double csmaBackoffMs =
            0.4 + 8.0 * metric.cbrEst / std::max (0.10, 1.0 - metric.cbrEst);
        attempt.latencyMs =
            m_config.v2vBaseLatencyMs + attempt.queueDelayMs + csmaBackoffMs +
            2.0 * cpm.occlusionRisk +
            1.5 * m_uniform (m_rng);
        return attempt;
      }

    const double dlLoadRatio = mec.estimatedDownlinkLoadBps / std::max (m_downlinkCapacityBps, 1.0);
    const double cellDistanceM = std::sqrt ((vehicle.x - metric.x) * (vehicle.x - metric.x) + 35.0 * 35.0);
    const double fsplAtOneMeterDb = 38.46; // 2 GHz cellular reference at 1 m.
    const double pathLossDb = fsplAtOneMeterDb + 10.0 * 2.05 * SafeLog10 (cellDistanceM);
    const double rxPowerDbm = 23.0 - pathLossDb - 2.0 * cpm.occlusionRisk;
    const double noiseDbm = -101.0;
    const double interCellInterferenceDbm =
        -108.0 + 28.0 * Clamp01 (dlLoadRatio) + 4.0 * SafeLog10 (std::max (1.0, cellDistanceM / 50.0));
    attempt.sinrDb =
        rxPowerDbm - MwToDbm (DbmToMw (noiseDbm) + DbmToMw (interCellInterferenceDbm));
    const double phySuccess = Logistic (attempt.sinrDb, 0.0, 3.2);
    attempt.resourceBusyRatio = Clamp01 (dlLoadRatio);
    const double schedulerGrantProbability =
        Clamp01 (1.0 - 0.70 * attempt.resourceBusyRatio);
    const double queueDropProbability = Clamp01 ((dlLoadRatio - 0.85) / 0.65);
    attempt.macCollisionProbability = queueDropProbability;
    attempt.pdr = Clamp01 (m_config.v2n2vPdr * phySuccess * schedulerGrantProbability *
                           (1.0 - queueDropProbability));
    attempt.delivered = m_uniform (m_rng) <= attempt.pdr;
    attempt.queueDelayMs =
        txServiceTimeMs * dlLoadRatio / std::max (0.05, 1.0 - dlLoadRatio) +
        0.5 * m_uniform (m_rng);
    attempt.schedulingDelayMs =
        m_baseStationSchedulingIntervalMs *
        (0.5 + attempt.resourceBusyRatio / std::max (0.05, 1.0 - attempt.resourceBusyRatio));
    attempt.backhaulDelayMs = m_backhaulDelayMs;
    attempt.coreNetworkDelayMs = m_coreNetworkDelayMs;
    attempt.latencyMs = m_config.v2n2vBaseLatencyMs + m_config.mecProcessingLatencyMs +
                        attempt.queueDelayMs + attempt.schedulingDelayMs +
                        attempt.backhaulDelayMs + attempt.coreNetworkDelayMs +
                        1.0 / std::max (0.05, schedulerGrantProbability) +
                        10.0 * dlLoadRatio + 3.0 * m_uniform (m_rng);
    return attempt;
  }

  void
  EvaluateVehicleMessages (double now,
                           const std::vector<VehicleSnapshot>& vehicles,
                           const std::vector<RsuMetric>& metrics)
  {
    if (metrics.empty () || vehicles.empty ())
      {
        return;
      }

    const uint32_t packetsPerVehicle =
        static_cast<uint32_t> (std::max (0.0, std::round (m_lambdaHz * m_deltaT)));
    if (packetsPerVehicle == 0)
      {
        return;
      }

    for (const auto& vehicle : vehicles)
      {
        const uint32_t nearestMetricIndex = FindNearestRsuIndex (metrics, vehicle.x);
        const RsuMetric localMetric =
            BuildVehicleLocalMetric (vehicle.id, metrics[nearestMetricIndex], true);
        const TransmissionDecision decision =
            BuildTransmissionDecision (localMetric, m_cbrMin, m_cbrMax, m_pMin);
        const MecDecision mec = BuildMecDecision (localMetric.phase,
                                                  decision,
                                                  localMetric.local.vehicleCount,
                                                  m_lambdaHz,
                                                  m_channelPacketSizeBits,
                                                  m_highPriorityTrafficRatio,
                                                  m_downlinkCapacityBps,
                                                  m_normalGeocastRadiusM,
                                                  m_congestedGeocastRadiusM);

        for (uint32_t i = 0; i < packetsPerVehicle; ++i)
          {
            const CpmContentFeatures cpm = BuildCpmContentFeatures (now, vehicle, vehicles);
            const uint64_t packetId = ++m_nextPacketId;
            const PriorityClass priority = ClassifyPriority (cpm);
            const std::vector<RouteType> routes = SelectRoutes (priority, decision);

            RouteAttempt v2v{false, false, 0.0, 0.0, 0.0, 0.0, 0.0};
            RouteAttempt v2n2v{false, false, 0.0, 0.0, 0.0, 0.0, 0.0};

            for (const auto& route : routes)
              {
                RouteAttempt attempt = SimulateRouteAttempt (route, vehicle, localMetric, mec, cpm);
                if (m_distributedRuntime != nullptr)
                  {
                    m_distributedRuntime->SendPacketProbe (vehicle.id,
                                                           route,
                                                           attempt.payloadBytes,
                                                           attempt.queueDelayMs);
                  }
                m_metrics.RecordRouteAttempt (route,
                                              priority,
                                              attempt.delivered,
                                              attempt.latencyMs,
                                              attempt.payloadBytes * 8);
                if (route == RouteType::V2V)
                  {
                    v2v = attempt;
                  }
                else
                  {
                    v2n2v = attempt;
                  }
              }

            const bool delivered = v2v.delivered || v2n2v.delivered;
            const bool duplicatedDelivery = v2v.delivered && v2n2v.delivered;
            double bestLatencyMs = 0.0;
            if (v2v.delivered && v2n2v.delivered)
              {
                bestLatencyMs = std::min (v2v.latencyMs, v2n2v.latencyMs);
              }
            else if (v2v.delivered)
              {
                bestLatencyMs = v2v.latencyMs;
              }
            else if (v2n2v.delivered)
              {
                bestLatencyMs = v2n2v.latencyMs;
              }

            const RecognitionEstimate recognition = BuildRecognitionEstimate (cpm, delivered);
            m_metrics.RecordMessage (priority,
                                     delivered,
                                     bestLatencyMs,
                                     duplicatedDelivery,
                                     cpm,
                                     recognition);
            if (delivered)
              {
                UpdateObjectFreshness (now, cpm);
              }

            if (m_communicationLog.is_open ())
              {
                m_communicationLog << now << "," << packetId << "," << vehicle.id << ","
                                   << ToString (priority) << "," << ToString (localMetric.phase) << ","
                                   << localMetric.cbrEst << "," << localMetric.cbrTrendPerSec << ","
                                   << ToString (localMetric.preferredRoute) << ","
                                   << localMetric.switchedToV2n2v << ","
                                   << decision.lowPriorityV2vProbability << ","
                                   << decision.lowPriorityOffloadProbability << ","
                                   << cpm.groundTruthObjects << "," << cpm.visibleObjects << ","
                                   << recognition.recognizedObjects << "," << cpm.redundancyRatio
                                   << "," << cpm.averageAoiSec << "," << cpm.blindSpotMissRate
                                   << "," << cpm.importanceScore
                                   << "," << v2v.attempted << "," << v2v.delivered << ","
                                   << v2v.latencyMs << "," << v2v.pdr << "," << v2v.sinrDb
                                   << "," << v2v.macCollisionProbability << ","
                                   << v2v.resourceBusyRatio << "," << v2v.queueDelayMs << ","
                                   << v2v.payloadBytes << "," << v2n2v.attempted << ","
                                   << v2n2v.delivered << "," << v2n2v.latencyMs << ","
                                   << v2n2v.pdr << "," << v2n2v.sinrDb << ","
                                   << v2n2v.macCollisionProbability << ","
                                   << v2n2v.resourceBusyRatio << "," << v2n2v.queueDelayMs
                                   << "," << v2n2v.schedulingDelayMs << ","
                                   << v2n2v.backhaulDelayMs << "," << v2n2v.coreNetworkDelayMs
                                   << "," << v2n2v.payloadBytes << ","
                                   << delivered << "," << bestLatencyMs << ","
                                   << (delivered &&
                                       bestLatencyMs <= m_config.e2eTargetLatencyMs)
                                   << "," << mec.geocastRadiusM << ","
                                   << mec.estimatedDownlinkLoadBps << "\n";
              }
          }
      }
  }

  void
  WriteOnce ()
  {
    const double now = Simulator::Now ().GetSeconds ();
    const std::vector<VehicleSnapshot> vehicles = ReadVehicles ();
    InjectLocalChannelSensing (vehicles);
    if (m_distributedRuntime != nullptr)
      {
        m_distributedRuntime->UpdateLocalCbr (now);
      }
    std::vector<RsuMetric> metrics = BuildRsuMetrics (vehicles);
    UpdateRouteControl (metrics);

    for (const auto& metric : metrics)
      {
        m_rsuLog << now << ",RSU_" << metric.index << "," << metric.x << ","
                 << metric.local.vehicleCount << "," << metric.local.averageSpeedMps << ","
                 << metric.upstream.vehicleCount << "," << metric.upstream.averageSpeedMps << ","
                 << metric.local.cbr << "," << metric.cbrEst << "," << metric.cbrTrendPerSec
                 << "," << ToString (metric.phase) << "," << ToString (metric.preferredRoute)
                 << "," << metric.switchedToV2n2v << "," << metric.switchedToV2v << ","
                 << (metric.phase == CbrPhase::Congested) << "\n";
      }

    if (!metrics.empty ())
      {
        for (const auto& vehicle : vehicles)
          {
            const uint32_t nearestMetricIndex = FindNearestRsuIndex (metrics, vehicle.x);
            const RsuMetric localMetric =
                BuildVehicleLocalMetric (vehicle.id, metrics[nearestMetricIndex], false);
            m_vehicleLog << now << "," << vehicle.id << "," << vehicle.x << "," << vehicle.y
                         << "," << vehicle.speedMps << ",RSU_" << localMetric.index << ","
                         << localMetric.local.cbr << "," << localMetric.cbrEst << ","
                         << localMetric.cbrTrendPerSec << "," << ToString (localMetric.phase) << ","
                         << ToString (localMetric.preferredRoute) << ","
                         << (localMetric.phase == CbrPhase::Congested) << "\n";
          }
      }

    EvaluateVehicleMessages (now, vehicles, metrics);

    m_rsuLog.flush ();
    m_vehicleLog.flush ();
    if (m_communicationLog.is_open ())
      {
        m_communicationLog.flush ();
      }

    std::cout << "[CBR log] t=" << now << "s, vehicles=" << vehicles.size ()
              << ", rsu_rows=" << metrics.size () << std::endl;

    if (now + m_deltaT <= m_simTime + 1e-9)
      {
        Simulator::Schedule (Seconds (m_deltaT), &CbrLogger::WriteOnce, this);
      }
  }

  Ptr<TraciClient> m_sumoClient;
  std::vector<RsuInfo> m_rsus;
  double m_roadLengthM;
  double m_segmentLengthM;
  double m_deltaT;
  double m_lambdaHz;
  double m_packetSizeBits;
  double m_pc5RateBps;
  double m_cbrMin;
  double m_cbrMax;
  double m_pMin;
  double m_routeSwitchCbr;
  double m_routeReleaseCbr;
  double m_routeSwitchTrendPerSec;
  double m_highPriorityTrafficRatio;
  double m_downlinkCapacityBps;
  double m_normalGeocastRadiusM;
  double m_congestedGeocastRadiusM;
  double m_perceptionRangeM;
  double m_rearSensingRangeM;
  double m_sensorFovDeg;
  double m_aoiHighThresholdSec;
  double m_aoiCriticalThresholdSec;
  double m_importanceThreshold;
  uint32_t m_ipUdpHeaderBytes;
  uint32_t m_securityOverheadBytes;
  uint32_t m_macOverheadBytes;
  double m_channelPacketSizeBits;
  double m_baseStationSchedulingIntervalMs;
  double m_backhaulDelayMs;
  double m_coreNetworkDelayMs;
  double m_channelSensingGain;
  double m_v2vTxPowerDbm;
  std::shared_ptr<DistributedV2xRuntime> m_distributedRuntime;
  double m_simTime;
  std::string m_rsuLogPath;
  std::string m_vehicleLogPath;
  std::ofstream m_rsuLog;
  std::ofstream m_vehicleLog;
  std::ofstream m_communicationLog;
  CbrPredictor m_predictor;
  EvaluationConfig m_config;
  EvaluationMetrics m_metrics;
  std::mt19937 m_rng;
  std::uniform_real_distribution<double> m_uniform;
  std::vector<double> m_lastCbrEstByRsu;
  std::vector<RouteType> m_preferredRouteByRsu;
  std::unordered_map<std::string, double> m_objectLastUpdateTime;
  std::unordered_map<std::string, double> m_vehicleLastCbr;
  std::unordered_map<std::string, RouteType> m_vehiclePreferredRoute;
  uint64_t m_nextPacketId = 0;
};

int
main (int argc, char* argv[])
{
  // SUMO GUI の delay を変更すると表示速度を直接調整できる。
  bool useSumo = true;
  bool sumoGui = true;
  bool realtime = true;
  double sumoUpdates = 0.1;
  double sumoDelayMs = 100.0;
  double simTime = 240.0;

  std::string sumoConfigPath = "src/traci/examples/straight_1km/straight_1km.sumocfg";
  std::string sumoBinaryPath = "";
  uint32_t traciNodePoolSize = 800;

  double roadLengthM = 2000.0;
  double rsuSpacingM = 200.0;
  double rsuHeightM = 1.5;
  double rsuMarkerSizeM = 10.0;

  // 固定値デモ用の初期パラメータ。実ログは SUMO 車両から再計算する。
  double currentCbr = 0.6;
  double upstreamVehicles = 50.0;
  double upstreamSpeedMps = 25.0;
  double localVehicles = 30.0;
  double localSpeedMps = 22.2;

  // 研究仕様書に基づく通信・制御パラメータ。
  double segmentLengthM = 500.0;
  double deltaT = 2.0;
  double lambdaHz = 10.0;
  double packetSizeBits = 4000.0;
  double pc5RateBps = 6000000.0;
  double cbrMin = 0.30;
  double cbrMax = 0.60;
  double pMin = 0.0;
  double routeSwitchCbr = cbrMin;
  double routeReleaseCbr = 0.25;
  double routeSwitchTrendPerSec = 0.0025;
  double highPriorityTrafficRatio = 0.1;
  double downlinkCapacityBps = 200000000.0;
  double normalGeocastRadiusM = 2000.0;
  double congestedGeocastRadiusM = 500.0;
  double perceptionRangeM = 150.0;
  double rearSensingRangeM = 25.0;
  double sensorFovDeg = 120.0;
  double aoiHighThresholdSec = 1.0;
  double aoiCriticalThresholdSec = 3.0;
  double importanceThreshold = 0.55;
  bool enableDistributedSockets = true;
  bool installWaveDevices = true;
  uint16_t v2xUdpPort = 49000;
  double waveTxPowerDbm = 23.0;
  std::string wavePhyMode = "OfdmRate6MbpsBW10MHz";
  uint32_t ipUdpHeaderBytes = 28;
  uint32_t securityOverheadBytes = 96;
  uint32_t macOverheadBytes = 36;
  double baseStationSchedulingIntervalMs = 1.0;
  double backhaulDelayMs = 4.0;
  double coreNetworkDelayMs = 6.0;
  double channelSensingGain = 3.0;
  std::string rsuCbrLogPath = "myfirst-rsu-cbr-log.csv";
  std::string vehicleCbrLogPath = "myfirst-vehicle-cbr-log.csv";
  std::string communicationLogPath = "myfirst-communication-log.csv";
  std::string evaluationSummaryPath = "myfirst-evaluation-summary.csv";
  std::string routeModeString = "hybrid";
  bool enableV2v = true;
  bool enableV2n2v = true;
  uint32_t randomSeed = 1;
  uint32_t objectsPerMessage = 1;
  double v2vPdrNormal = 0.98;
  double v2vPdrCongested = 0.70;
  double v2n2vPdr = 0.995;
  double v2vBaseLatencyMs = 3.0;
  double v2n2vBaseLatencyMs = 10.0;
  double mecProcessingLatencyMs = 2.0;
  double e2eTargetLatencyMs = 50.0;

  CommandLine cmd;
  cmd.AddValue ("use-sumo", "SUMO/TraCI連携を起動するかどうか。", useSumo);
  cmd.AddValue ("sumoGui", "SUMO GUIを表示するかどうか。", sumoGui);
  cmd.AddValue ("sumo-gui", "SUMO GUIを表示するかどうか。", sumoGui);
  cmd.AddValue ("realtime", "GUIで見やすいようns-3をリアルタイム実行するかどうか。", realtime);
  cmd.AddValue ("sumo-updates", "SUMO/ns-3の同期間隔[秒]。", sumoUpdates);
  cmd.AddValue ("sumo-delay-ms", "SUMO GUIの1ステップごとの表示遅延[ms]。大きいほど遅く見える。", sumoDelayMs);
  cmd.AddValue ("sim-time", "シミュレーション時間[秒]。", simTime);
  cmd.AddValue ("sumo-config", "SUMO設定ファイル(.sumocfg)のパス。", sumoConfigPath);
  cmd.AddValue ("sumo-binary-path", "SUMO実行ファイルのパス接頭辞。通常は空文字でよい。", sumoBinaryPath);
  cmd.AddValue ("traci-node-pool-size", "SUMO車両へ割り当てるns-3ノードの最大数。", traciNodePoolSize);

  cmd.AddValue ("road-length", "道路長[m]。", roadLengthM);
  cmd.AddValue ("rsu-spacing", "RSU間隔[m]。", rsuSpacingM);
  cmd.AddValue ("rsu-marker-size", "SUMO GUI上のRSUマーカーサイズ[m]。", rsuMarkerSizeM);
  cmd.AddValue ("segment-length", "CBR計算で使う各RSUの監視区間長[m]。", segmentLengthM);
  cmd.AddValue ("current-cbr", "固定値デモ用の現在CBR[0,1]。", currentCbr);
  cmd.AddValue ("upstream-vehicles", "固定値デモ用の上流車両数。", upstreamVehicles);
  cmd.AddValue ("upstream-speed", "固定値デモ用の上流平均速度[m/s]。", upstreamSpeedMps);
  cmd.AddValue ("local-vehicles", "固定値デモ用の自区間車両数。", localVehicles);
  cmd.AddValue ("local-speed", "固定値デモ用の自区間平均速度[m/s]。", localSpeedMps);
  cmd.AddValue ("delta-t", "CBR予測・ログ出力間隔[秒]。", deltaT);
  cmd.AddValue ("lambda", "CPM送信頻度[Hz]。", lambdaHz);
  cmd.AddValue ("packet-size-bits", "CPM平均パケットサイズ[bit]。", packetSizeBits);
  cmd.AddValue ("pc5-rate-bps", "V2V/PC5チャネルの想定データレート[bps]。", pc5RateBps);
  cmd.AddValue ("cbr-min", "正常/遷移のCBR閾値。", cbrMin);
  cmd.AddValue ("cbr-max", "遷移/逼迫のCBR閾値。", cbrMax);
  cmd.AddValue ("p-min", "V2N2V切替後も低重要度で残すV2V送信確率。0なら完全にV2N2Vへ切替。", pMin);
  cmd.AddValue ("route-switch-cbr", "V2VからV2N2Vへ切り替える予測CBR閾値。", routeSwitchCbr);
  cmd.AddValue ("route-release-cbr", "V2N2VからV2Vへ戻す予測CBR閾値。ヒステリシス用。", routeReleaseCbr);
  cmd.AddValue ("route-switch-trend", "次周期で閾値超過が予測される時に使うCBR上昇率閾値[1/s]。", routeSwitchTrendPerSec);
  cmd.AddValue ("high-priority-ratio", "高重要度情報として扱うトラフィック比率。", highPriorityTrafficRatio);
  cmd.AddValue ("dl-capacity-bps", "MBSダウンリンク実効容量[bps]。", downlinkCapacityBps);
  cmd.AddValue ("normal-geocast-radius", "通常時のN2Vジオキャスト半径[m]。", normalGeocastRadiusM);
  cmd.AddValue ("congested-geocast-radius", "逼迫時のN2Vジオキャスト半径[m]。", congestedGeocastRadiusM);
  cmd.AddValue ("perception-range", "CPM物標候補とする周辺認識距離[m]。", perceptionRangeM);
  cmd.AddValue ("rear-sensing-range", "後方近距離センサの認識距離[m]。", rearSensingRangeM);
  cmd.AddValue ("sensor-fov-deg", "前方センサ視野角[deg]。", sensorFovDeg);
  cmd.AddValue ("aoi-high-threshold", "重要度判定で高AoIと見なす閾値[秒]。", aoiHighThresholdSec);
  cmd.AddValue ("aoi-critical-threshold", "未更新物標へ与える最大AoI[秒]。", aoiCriticalThresholdSec);
  cmd.AddValue ("importance-threshold", "CPM重要度スコアのHigh判定閾値。", importanceThreshold);
  cmd.AddValue ("enable-distributed-sockets", "車両ノードにUDPソケットを載せ、受信履歴でローカルCBRを測る。", enableDistributedSockets);
  cmd.AddValue ("install-wave-devices", "車両/RSUノードへ802.11p Wave NetDeviceを事前インストールする。", installWaveDevices);
  cmd.AddValue ("v2x-udp-port", "分散V2X UDPプロトタイプの受信ポート。", v2xUdpPort);
  cmd.AddValue ("wave-tx-power-dbm", "802.11pプロトタイプの送信電力[dBm]。", waveTxPowerDbm);
  cmd.AddValue ("wave-phy-mode", "802.11pプロトタイプのPHYモード。", wavePhyMode);
  cmd.AddValue ("ip-udp-header-bytes", "IP/UDPヘッダのオーバーヘッド[byte]。", ipUdpHeaderBytes);
  cmd.AddValue ("security-overhead-bytes", "署名・証明書等のセキュリティオーバーヘッド[byte]。", securityOverheadBytes);
  cmd.AddValue ("mac-overhead-bytes", "MAC/LLC等のリンク層オーバーヘッド[byte]。", macOverheadBytes);
  cmd.AddValue ("bs-scheduling-interval-ms", "V2N2V基地局スケジューリング周期[ms]。", baseStationSchedulingIntervalMs);
  cmd.AddValue ("backhaul-delay-ms", "基地局-MEC間バックホール遅延[ms]。", backhaulDelayMs);
  cmd.AddValue ("core-network-delay-ms", "EPC/5GCコアルーティング遅延[ms]。", coreNetworkDelayMs);
  cmd.AddValue ("channel-sensing-gain", "ローカルCBR測定で、受信エネルギー/バックオフ分をairtimeに換算する係数。", channelSensingGain);
  cmd.AddValue ("rsu-cbr-log", "RSUごとのCBR/CBR_estログCSV。", rsuCbrLogPath);
  cmd.AddValue ("vehicle-cbr-log", "車両ごとのCBRログCSV。", vehicleCbrLogPath);
  cmd.AddValue ("comm-log", "V2V/V2N2V送信結果ログCSV。空文字なら出力しない。", communicationLogPath);
  cmd.AddValue ("evaluation-summary", "実験終了時の評価指標サマリCSV。空文字なら出力しない。", evaluationSummaryPath);
  cmd.AddValue ("route-mode", "送信経路: hybrid, v2v-only, v2n2v-only, redundant。", routeModeString);
  cmd.AddValue ("enable-v2v", "V2V送信経路を有効にする。", enableV2v);
  cmd.AddValue ("enable-v2n2v", "V2N2V送信経路を有効にする。", enableV2n2v);
  cmd.AddValue ("random-seed", "評価モデルの乱数シード。", randomSeed);
  cmd.AddValue ("objects-per-message", "1メッセージに含まれる認識対象物数。ORR計算に使う。", objectsPerMessage);
  cmd.AddValue ("v2v-pdr-normal", "正常時V2Vの到達確率。", v2vPdrNormal);
  cmd.AddValue ("v2v-pdr-congested", "逼迫時V2Vの到達確率。", v2vPdrCongested);
  cmd.AddValue ("v2n2v-pdr", "V2N2Vの基準到達確率。DL過負荷時は低下する。", v2n2vPdr);
  cmd.AddValue ("v2v-base-latency-ms", "V2V基準遅延[ms]。", v2vBaseLatencyMs);
  cmd.AddValue ("v2n2v-base-latency-ms", "V2N2V基準無線遅延[ms]。", v2n2vBaseLatencyMs);
  cmd.AddValue ("mec-processing-latency-ms", "MEC中継処理遅延[ms]。", mecProcessingLatencyMs);
  cmd.AddValue ("e2e-target-ms", "E2E遅延目標[ms]。", e2eTargetLatencyMs);
  cmd.Parse (argc, argv);

  sumoUpdates = std::max (sumoUpdates, 0.001);
  deltaT = std::max (deltaT, sumoUpdates);
  simTime = std::max (simTime, deltaT);
  currentCbr = Clamp01 (currentCbr);
  pMin = Clamp01 (pMin);
  routeSwitchCbr = Clamp01 (routeSwitchCbr);
  routeReleaseCbr = Clamp01 (routeReleaseCbr);
  routeSwitchTrendPerSec = std::max (0.0, routeSwitchTrendPerSec);
  if (routeReleaseCbr >= routeSwitchCbr)
    {
      routeReleaseCbr = std::max (0.0, routeSwitchCbr - 0.05);
    }
  highPriorityTrafficRatio = Clamp01 (highPriorityTrafficRatio);
  perceptionRangeM = std::max (1.0, perceptionRangeM);
  rearSensingRangeM = std::max (0.0, rearSensingRangeM);
  sensorFovDeg = std::max (1.0, std::min (180.0, sensorFovDeg));
  aoiHighThresholdSec = std::max (0.001, aoiHighThresholdSec);
  aoiCriticalThresholdSec = std::max (aoiHighThresholdSec, aoiCriticalThresholdSec);
  importanceThreshold = Clamp01 (importanceThreshold);
  waveTxPowerDbm = std::max (0.0, waveTxPowerDbm);
  ipUdpHeaderBytes = std::max (1u, ipUdpHeaderBytes);
  securityOverheadBytes = std::max (0u, securityOverheadBytes);
  macOverheadBytes = std::max (0u, macOverheadBytes);
  baseStationSchedulingIntervalMs = std::max (0.0, baseStationSchedulingIntervalMs);
  backhaulDelayMs = std::max (0.0, backhaulDelayMs);
  coreNetworkDelayMs = std::max (0.0, coreNetworkDelayMs);
  channelSensingGain = std::max (0.0, channelSensingGain);
  objectsPerMessage = std::max (1u, objectsPerMessage);
  v2vPdrNormal = Clamp01 (v2vPdrNormal);
  v2vPdrCongested = Clamp01 (v2vPdrCongested);
  v2n2vPdr = Clamp01 (v2n2vPdr);
  v2vBaseLatencyMs = std::max (0.0, v2vBaseLatencyMs);
  v2n2vBaseLatencyMs = std::max (0.0, v2n2vBaseLatencyMs);
  mecProcessingLatencyMs = std::max (0.0, mecProcessingLatencyMs);
  e2eTargetLatencyMs = std::max (0.0, e2eTargetLatencyMs);
  const double channelPacketSizeBits =
      packetSizeBits +
      8.0 * static_cast<double> (ipUdpHeaderBytes + securityOverheadBytes + macOverheadBytes);

  const RouteMode routeMode = ParseRouteMode (routeModeString);
  if (!enableV2v && !enableV2n2v)
    {
      NS_FATAL_ERROR ("At least one route must be enabled: enable-v2v or enable-v2n2v.");
    }
  if (routeMode == RouteMode::V2vOnly && !enableV2v)
    {
      NS_FATAL_ERROR ("route-mode=v2v-only requires enable-v2v=true.");
    }
  if (routeMode == RouteMode::V2n2vOnly && !enableV2n2v)
    {
      NS_FATAL_ERROR ("route-mode=v2n2v-only requires enable-v2n2v=true.");
    }

  Time::SetResolution (Time::NS);
  if (realtime)
    {
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
    }

  std::cout << std::boolalpha;
  std::cout << "高速道路V2X/MECシミュレーションを開始します。" << std::endl;
  std::cout << "SUMO表示速度: realtime=" << realtime << ", step=" << sumoUpdates
            << "s, gui-delay=" << sumoDelayMs << "ms" << std::endl;
  std::cout << "送信経路: mode=" << ToString (routeMode) << ", V2V=" << enableV2v
            << ", V2N2V=" << enableV2n2v << std::endl;
  std::cout << "経路切替: switch_cbr=" << routeSwitchCbr
            << ", release_cbr=" << routeReleaseCbr
            << ", trend_threshold=" << routeSwitchTrendPerSec << " /s" << std::endl;
  std::cout << "CPM重要度: range=" << perceptionRangeM << "m, fov=" << sensorFovDeg
            << "deg, AoI(high/critical)=" << aoiHighThresholdSec << "/"
            << aoiCriticalThresholdSec << "s" << std::endl;
  std::cout << "分散Socket: enabled=" << enableDistributedSockets
            << ", wave-devices=" << installWaveDevices
            << ", udp-port=" << v2xUdpPort << std::endl;

  const uint32_t rsuCount =
      static_cast<uint32_t> (std::max (1.0, std::floor (roadLengthM / std::max (rsuSpacingM, 1.0)) + 1.0));

  std::vector<RsuInfo> rsuInfos;
  rsuInfos.reserve (rsuCount);

  NodeContainer rsuNodes;
  rsuNodes.Create (rsuCount);

  MobilityHelper mobilityRsu;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  for (uint32_t i = 0; i < rsuNodes.GetN (); ++i)
    {
      RsuInfo rsu;
      rsu.index = i;
      rsu.x = std::min (i * rsuSpacingM, roadLengthM);
      rsu.y = 0.0;
      rsu.z = rsuHeightM;
      rsuInfos.push_back (rsu);
      positionAlloc->Add (Vector (rsu.x, rsu.y, rsu.z));
    }

  mobilityRsu.SetPositionAllocator (positionAlloc);
  mobilityRsu.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobilityRsu.Install (rsuNodes);

  std::cout << "RSU配置: 台数=" << rsuNodes.GetN () << ", 間隔=" << rsuSpacingM
            << "m, 道路長=" << roadLengthM << "m" << std::endl;

  NodeContainer traciNodePool;
  Ptr<TraciClient> sumoClient;
  auto vehicleNodeIndex = std::make_shared<std::unordered_map<std::string, uint32_t>> ();
  auto distributedRuntime = std::make_shared<DistributedV2xRuntime> ();

  if (useSumo)
    {
      traciNodePool.Create (traciNodePoolSize);
      MobilityHelper traciMobility;
      traciMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      traciMobility.Install (traciNodePool);

      DistributedV2xRuntime::Config distributedConfig;
      distributedConfig.enabled = enableDistributedSockets;
      distributedConfig.installWaveDevices = installWaveDevices;
      distributedConfig.udpPort = v2xUdpPort;
      distributedConfig.txPowerDbm = waveTxPowerDbm;
      distributedConfig.pc5RateBps = pc5RateBps;
      distributedConfig.cbrWindowSec = deltaT;
      distributedConfig.phyMode = wavePhyMode;
      distributedRuntime->Configure (traciNodePool, rsuNodes, vehicleNodeIndex, distributedConfig);

      std::ostringstream sumoAdditionalOptions;
      sumoAdditionalOptions << "--verbose true";
      if (sumoGui && sumoDelayMs > 0.0)
        {
          sumoAdditionalOptions << " --delay " << sumoDelayMs;
        }

      sumoClient = CreateObject<TraciClient> ();
      sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumoConfigPath));
      sumoClient->SetAttribute ("SumoBinaryPath", StringValue (sumoBinaryPath));
      sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumoGui));
      sumoClient->SetAttribute ("SumoLogFile", BooleanValue (true));
      sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
      sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumoUpdates)));
      sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
      sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
      sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (2.0)));
      sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
      sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumoAdditionalOptions.str ()));

      uint32_t currentVehicleNode = 0;
      std::function<Ptr<Node> (std::string, TraciClient::StationTypeTraCI_t)> includeNode =
          [&] (std::string id, TraciClient::StationTypeTraCI_t type) -> Ptr<Node>
      {
        if (currentVehicleNode >= traciNodePool.GetN ())
          {
            NS_FATAL_ERROR ("TraCI node pool is too small for the SUMO scenario.");
          }
        (*vehicleNodeIndex)[id] = currentVehicleNode;
        return traciNodePool.Get (currentVehicleNode++);
      };

      std::function<void (Ptr<Node>, std::string)> excludeNode =
          [] (Ptr<Node> node, std::string id)
      {
      };

      sumoClient->SumoSetup (includeNode, excludeNode);

      if (sumoGui)
        {
          AddVisibleRsusToSumo (sumoClient, rsuInfos, rsuMarkerSizeM);
          std::cout << "SUMO GUI上にRSUマーカーを追加しました。" << std::endl;
        }
    }

  CbrPredictor predictor;
  const double predictedCbr = predictor.PredictCbr (currentCbr,
                                                   upstreamVehicles,
                                                   upstreamSpeedMps,
                                                   localVehicles,
                                                   localSpeedMps,
                                                   segmentLengthM,
                                                   deltaT,
                                                   lambdaHz,
                                                   channelPacketSizeBits,
                                                   pc5RateBps);
  const CbrPhase phase = ClassifyCbrPhase (predictedCbr, cbrMin, cbrMax);
  const RouteType demoPreferredRoute = predictedCbr >= routeSwitchCbr ? RouteType::V2N2V
                                                                      : RouteType::V2V;
  RsuMetric demoMetric;
  demoMetric.cbrEst = predictedCbr;
  demoMetric.cbrTrendPerSec = 0.0;
  demoMetric.phase = phase;
  demoMetric.preferredRoute = demoPreferredRoute;
  const TransmissionDecision transmission =
      BuildTransmissionDecision (demoMetric, cbrMin, cbrMax, pMin);
  const MecDecision mec = BuildMecDecision (phase,
                                           transmission,
                                           localVehicles,
                                           lambdaHz,
                                           channelPacketSizeBits,
                                           highPriorityTrafficRatio,
                                           downlinkCapacityBps,
                                           normalGeocastRadiusM,
                                           congestedGeocastRadiusM);

  std::cout << "--- 固定値によるCBR制御デモ ---" << std::endl;
  std::cout << "現在CBR=" << currentCbr << ", deltaT=" << deltaT
            << "s後のCBR_est=" << predictedCbr << std::endl;
  std::cout << "CBR状態=" << ToString (phase) << " (min=" << cbrMin << ", max=" << cbrMax
            << ")" << std::endl;
  std::cout << "予測に基づく優先経路=" << ToString (demoPreferredRoute) << std::endl;
  std::cout << "低重要度: V2V送信確率=" << transmission.lowPriorityV2vProbability
            << ", V2N2Vオフロード=" << transmission.lowPriorityV2n << std::endl;
  std::cout << "高重要度: V2V=" << transmission.highPriorityV2v
            << ", V2N2V冗長送信=" << transmission.highPriorityV2n << std::endl;
  std::cout << "MEC推定: DL負荷=" << mec.estimatedDownlinkLoadBps
            << " bps, N2Vジオキャスト半径=" << mec.geocastRadiusM << "m" << std::endl;
  std::cout << "--------------------------------" << std::endl;

  EvaluationConfig evaluationConfig;
  evaluationConfig.enableV2v = enableV2v;
  evaluationConfig.enableV2n2v = enableV2n2v;
  evaluationConfig.routeMode = routeMode;
  evaluationConfig.randomSeed = randomSeed;
  evaluationConfig.objectsPerMessage = objectsPerMessage;
  evaluationConfig.v2vPdrNormal = v2vPdrNormal;
  evaluationConfig.v2vPdrCongested = v2vPdrCongested;
  evaluationConfig.v2n2vPdr = v2n2vPdr;
  evaluationConfig.v2vBaseLatencyMs = v2vBaseLatencyMs;
  evaluationConfig.v2n2vBaseLatencyMs = v2n2vBaseLatencyMs;
  evaluationConfig.mecProcessingLatencyMs = mecProcessingLatencyMs;
  evaluationConfig.e2eTargetLatencyMs = e2eTargetLatencyMs;
  evaluationConfig.communicationLogPath = communicationLogPath;
  evaluationConfig.evaluationSummaryPath = evaluationSummaryPath;

  CbrLogger cbrLogger (sumoClient,
                       rsuInfos,
                       roadLengthM,
                       segmentLengthM,
                       deltaT,
                       lambdaHz,
                       packetSizeBits,
                       pc5RateBps,
                       cbrMin,
                       cbrMax,
                       pMin,
                       routeSwitchCbr,
                       routeReleaseCbr,
                       routeSwitchTrendPerSec,
                       highPriorityTrafficRatio,
                       downlinkCapacityBps,
                       normalGeocastRadiusM,
                       congestedGeocastRadiusM,
                       perceptionRangeM,
                       rearSensingRangeM,
                       sensorFovDeg,
                       aoiHighThresholdSec,
                       aoiCriticalThresholdSec,
                       importanceThreshold,
                       ipUdpHeaderBytes,
                       securityOverheadBytes,
                       macOverheadBytes,
                       baseStationSchedulingIntervalMs,
                       backhaulDelayMs,
                       coreNetworkDelayMs,
                       channelSensingGain,
                       waveTxPowerDbm,
                       distributedRuntime,
                       simTime,
                       rsuCbrLogPath,
                       vehicleCbrLogPath,
                       evaluationConfig);
  cbrLogger.Start ();

  // simTime と同じ時刻に予約された deltaT ログが先に実行されるよう、停止時刻を少しだけ後ろへずらす。
  Simulator::Stop (Seconds (simTime + 1e-6));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << "シミュレーションが終了しました。" << std::endl;
  cbrLogger.PrintEvaluationSummary (std::cout);
  cbrLogger.WriteEvaluationSummary ();
  std::cout << "RSUログ: " << rsuCbrLogPath << std::endl;
  std::cout << "車両ログ: " << vehicleCbrLogPath << std::endl;
  if (!communicationLogPath.empty ())
    {
      std::cout << "送信ログ: " << communicationLogPath << std::endl;
    }
  if (!evaluationSummaryPath.empty ())
    {
      std::cout << "評価サマリ: " << evaluationSummaryPath << std::endl;
    }
  return 0;
}
