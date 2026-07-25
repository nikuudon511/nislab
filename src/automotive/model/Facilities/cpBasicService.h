#ifndef CPBASICSERVICE_H
#define CPBASICSERVICE_H

#include "ns3/socket.h"
#include "ns3/core-module.h"
#include "ns3/vdp.h"
#include "ns3/asn_utils.h"
#include "ns3/btp.h"
#include "ns3/btpHeader.h"
#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/vdp.h"
#include "ns3/vdpTraci.h"
#include "ns3/LDM.h"
#include "ns3/ldm-utils.h"
#include "signalInfoUtils.h"

#include <set>

extern "C" {
  #include "ns3/CollectivePerceptionMessage.h"
}

namespace ns3
{
/**
 * \ingroup automotive
 *
 * \brief This class implements the basic service for the Collective Perception Service.
 *
 * This class implements the basic functionalities of the Collective Perception Service defined in ETSI TS 103 324 V2.1.1 (2023-06).
 * Collective Perception Messages (CPMs) are transmitted by ITS-Ss in order to share information about perceived objects (non-connected vehicles in our framework).
 * This enhances the environmental perception of CPS-enabled ITS-Ss by providing information about non-V2X-equipped road users
 * and also increases the number of information sources for V2X-equipped road users.
 *
 */
class CPBasicService: public Object, public SignalInfoUtils
{ 
public:
  enum RmrForwardingMode
  {
    RMR_FORWARDING_PRIMARY = 0,
    RMR_FORWARDING_OFFLOAD = 1,
    RMR_FORWARDING_DUPLICATE = 2
  };

  /**
   * \brief Constructor
   *
   * This constructor initializes the CPBasicService object.
   */
  CPBasicService();
  ~CPBasicService();
  /**
   * @brief Set the station ID of the ITS-S.
   * @param fixed_stationid  The station ID to be set.
   */
  void setStationID(unsigned long fixed_stationid);
  /**
   * @brief Set the station type of the ITS-S.
   * @param fixed_stationtype   The station type to be set.
   */
  void setStationType(long fixed_stationtype);
  /**
   * @brief Set the station properties of the ITS-S.
   * @param fixed_stationid
   * @param fixed_stationtype
   */
  void setStationProperties(unsigned long fixed_stationid,long fixed_stationtype);
  /**
   * @brief Set the socket to be used by BTP/GeoNet for transmitting CPMs.
   * @param socket_tx The socket to be set.
   */
  void setSocketTx(Ptr<Socket> socket_tx) {m_btp->setSocketTx (socket_tx);}
  /**
   * @brief Set the socket to be used for receiving CPMs.
   * @param socket_rx The socket to be set.
   */
  void setSocketRx(Ptr<Socket> socket_rx);
  /**
   * @brief Set the LDM object to be used by the CPBasicService.
   * @param LDM The LDM object to be set.
   */
  void setLDM(Ptr<LDM> LDM){m_LDM = LDM;}
  /**
   * @brief Set the VDP object to be used by the CPBasicService.
   * @param vdp The VDP object to be set.
   */
  void setVDP(VDP* vdp) {m_vdp=vdp;}
  void setTraCIclient(Ptr<TraciClient> client){m_client=client;}
  /**
   * @brief Set the BTP object to be used by the CPBasicService.
   * @param btp The BTP object to be set.
   */
  void setBTP(Ptr<btp> btp){m_btp = btp;}

  /**
   * @brief Callback function used for the processing of received CPMs.
   * @param dataIndication The received data indication from BTP/GeoNet.
   * @param from The address of the sender of the CPM.
   */
  void receiveCpm(BTPDataIndication_t dataIndication, Address from);
  void changeNGenCpmMax(int16_t N_GenCpmMax) {m_N_GenCpmMax=N_GenCpmMax;}
  /**
   * @brief Set the callback function to be used for the processing of received CPMs.
   * @param rx_callback  The callback function to be set.
   */
  void addCPRxCallback(std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address)> rx_callback) {m_CPReceiveCallback=rx_callback;}
  void addCPRxCallbackExtended(std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, StationID_t, StationType_t, SignalInfo)> rx_callback) {m_CPReceiveCallbackExtended=rx_callback;}
  /**
   * @brief Specify if using the real time or simulation time for the CPM timestamps.
   * @param real_time
   */
  void setRealTime(bool real_time){m_real_time=real_time;}
  /**
   * @brief Start the dissemination of CPMs.
   */
  void startCpmDissemination();
  /**
   * @brief Stop the dissemination of CPMs.
   */
  uint64_t terminateDissemination();
  /**
   * @brief Specify if using redundancy mitigation defined for the CPM generation.
   *
   * This method is used to specify the value of the ObjectInclusionConfig value defined in ETSI TS 103 324 V2.1.1 (2023-06) Section 6.1.2.3
   * True (default) --> The inclusion rules defined in ETSI TS 103 324 V2.1.1 (2023-06) are applied
   * False --> The inclusion rules defined in ETSI TS 103 324 V2.1.1 (2023-06) are not applied
   * @param choice
   */
  void setRedundancyMitigation(bool choice){m_redundancy_mitigation = choice;}
  void disableRedundancyMitigation(){m_redundancy_mitigation = false;}

  void setCbrAdaptiveRmr(bool choice){m_cbr_adaptive_rmr = choice;}
  void setRmrCurrentCbr(double cbr){m_rmr_current_cbr = cbr;}
  void setRmrActionProbability(double probability){m_rmr_action_probability = probability;}
  void setRmrForwardingMode(RmrForwardingMode mode){m_rmr_forwarding_mode = mode;}
  void setExternalRmrDeletedIds(const std::set<uint64_t>& ids){m_external_rmr_deleted_ids = ids; m_use_external_rmr_deleted_ids = true;}
  void clearExternalRmrDeletedIds(){m_external_rmr_deleted_ids.clear(); m_use_external_rmr_deleted_ids = false;}
  const std::set<uint64_t>& getLastRmrDeletedIds() const {return m_last_rmr_deleted_ids;}
  void configureCbrAdaptiveRmr(double cbrLow,
                               double cbrHigh,
                               uint32_t deleteLow,
                               uint32_t deleteMiddle,
                               uint32_t deleteHigh);
  void configureRmrWeights(double frequencyWeight,
                           double dynamicsWeight,
                           double distanceWeight);
  void setRmrWindowMs(uint32_t windowMs){m_rmr_window_ms = windowMs;}
  void configureRmrImportantGuard(bool enabled,
                                  double protectedDistanceMeters,
                                  double protectedPositionChangeMeters,
                                  double protectedSpeedChangeMps);
  uint32_t getLastRmrCandidateCount() const {return m_last_rmr_candidate_count;}
  uint32_t getLastRmrIncludedObjectCount() const {return m_last_rmr_included_objects;}
  uint32_t getLastRmrDeletedObjectCount() const {return m_last_rmr_deleted_objects;}
  uint32_t getLastRmrDeletedFeatureCount() const {return m_last_rmr_deleted_feature_count;}
  double getLastRmrDeletedDistanceSum() const {return m_last_rmr_deleted_distance_sum;}
  double getLastRmrDeletedFrequencySum() const {return m_last_rmr_deleted_frequency_sum;}
  double getLastRmrDeletedPositionChangeSum() const {return m_last_rmr_deleted_position_change_sum;}
  double getLastRmrDeletedSpeedChangeSum() const {return m_last_rmr_deleted_speed_change_sum;}
  double getLastRmrDeletedScoreSum() const {return m_last_rmr_deleted_score_sum;}
  uint32_t getLastCpmSizeBytes() const {return m_last_cpm_size_bytes;}
  uint64_t getTotalRmrCandidateCount() const {return m_total_rmr_candidate_count;}
  uint64_t getTotalRmrIncludedObjectCount() const {return m_total_rmr_included_objects;}
  uint64_t getTotalRmrDeletedObjectCount() const {return m_total_rmr_deleted_objects;}
  uint64_t getTotalRmrDeletedFeatureCount() const {return m_total_rmr_deleted_feature_count;}
  double getTotalRmrDeletedDistanceSum() const {return m_total_rmr_deleted_distance_sum;}
  double getTotalRmrDeletedFrequencySum() const {return m_total_rmr_deleted_frequency_sum;}
  double getTotalRmrDeletedPositionChangeSum() const {return m_total_rmr_deleted_position_change_sum;}
  double getTotalRmrDeletedSpeedChangeSum() const {return m_total_rmr_deleted_speed_change_sum;}
  double getTotalRmrDeletedScoreSum() const {return m_total_rmr_deleted_score_sum;}
  uint64_t getTotalCpmSizeBytes() const {return m_total_cpm_size_bytes;}
  uint64_t getCpmSent() const {return m_cpm_sent;}

  uint64_t getWannabeSent() {return m_wannabe_sent;}

  const long T_GenCpmMin_ms = 100;
  const long T_GenCpm_ms = 100;
  const long T_GenCpmMax_ms = 1000;
  const long m_T_AddSensorInformation = 1000;


private:

  void initDissemination();
  void RSUDissemination();
  void checkCpmConditions();
  /**
   * @brief Generate and encode a CPM ASN.1 message.
   */
  void generateAndEncodeCPM();
  int64_t computeTimestampUInt64();
  /**
   * @brief Evaluate the conditions for the generation of a CPM as defined in ETSI TS 103 324 V2.1.1 (2023-06) Section 6.1.2.3
   * @param it  The iterator of the vehicle data to be checked.
   * @return
   */
  bool checkCPMconditions(std::vector<LDM::returnedVehicleData_t>::iterator it);
  std::set<uint64_t> selectCbrAdaptiveRmrDeletions(std::vector<LDM::returnedVehicleData_t>& LDM_POs);
  uint32_t getRmrDeletionBudget() const;
  double cartesian_dist(double lon1, double lat1, double lon2, double lat2);

  std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address)> m_CPReceiveCallback;  //! Callback function for received CPMs
  std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, Ptr<Packet>)> m_CPReceiveCallbackPkt;
  std::function<void(asn1cpp::Seq<CollectivePerceptionMessage>, Address, StationID_t, StationType_t, SignalInfo)> m_CPReceiveCallbackExtended;

  Ptr<btp> m_btp; //! BTP object

  long m_T_CheckCpmGen_ms; //! Time interval for checking the conditions for CPM generation
  long m_T_LastSensorInfoContainer; //! Time interval for adding sensor information to the CPM

  long m_T_GenCpm_ms;
  int16_t m_N_GenCpm;
  int16_t m_N_GenCpmMax;

  int64_t lastCpmGen; //! Timestamp of the last CPM generation
  int64_t lastCpmGenLowFrequency;
  int64_t lastCpmGenSpecialVehicle;

  bool m_real_time; //! Flag to specify if using real time or simulation time for the CPM timestamps
  bool m_vehicle;
  bool m_redundancy_mitigation; //! Flag to specify if using redundancy mitigation defined for the CPM generation
  bool m_cbr_adaptive_rmr;
  RmrForwardingMode m_rmr_forwarding_mode;
  double m_rmr_current_cbr;
  double m_rmr_action_probability;
  double m_rmr_cbr_low;
  double m_rmr_cbr_high;
  double m_rmr_frequency_weight;
  double m_rmr_dynamics_weight;
  double m_rmr_distance_weight;
  uint32_t m_rmr_delete_low;
  uint32_t m_rmr_delete_middle;
  uint32_t m_rmr_delete_high;
  uint32_t m_rmr_window_ms;
  bool m_rmr_guard_important;
  double m_rmr_guard_distance_m;
  double m_rmr_guard_position_change_m;
  double m_rmr_guard_speed_change_mps;
  uint32_t m_last_rmr_candidate_count;
  uint32_t m_last_rmr_included_objects;
  uint32_t m_last_rmr_deleted_objects;
  uint32_t m_last_rmr_deleted_feature_count;
  double m_last_rmr_deleted_distance_sum;
  double m_last_rmr_deleted_frequency_sum;
  double m_last_rmr_deleted_position_change_sum;
  double m_last_rmr_deleted_speed_change_sum;
  double m_last_rmr_deleted_score_sum;
  std::set<uint64_t> m_last_rmr_deleted_ids;
  std::set<uint64_t> m_external_rmr_deleted_ids;
  bool m_use_external_rmr_deleted_ids;
  uint32_t m_last_cpm_size_bytes;
  uint64_t m_total_rmr_candidate_count;
  uint64_t m_total_rmr_included_objects;
  uint64_t m_total_rmr_deleted_objects;
  uint64_t m_total_rmr_deleted_feature_count;
  double m_total_rmr_deleted_distance_sum;
  double m_total_rmr_deleted_frequency_sum;
  double m_total_rmr_deleted_position_change_sum;
  double m_total_rmr_deleted_speed_change_sum;
  double m_total_rmr_deleted_score_sum;
  uint64_t m_total_cpm_size_bytes;
  VDP* m_vdp; //! VDP object
  Ptr<TraciClient> m_client;

  Ptr<Socket> m_socket_tx; // Socket TX

  Ptr<LDM> m_LDM; //! LDM object

  StationID_t m_station_id; //! Station ID of the ITS-S
  StationType_t m_stationtype; //! Station type of the ITS-S

  // Previous Cpm relevant values
  double m_prev_heading;
  double m_prev_distance;
  double m_prev_speed;
  std::vector<long> m_lastCPM_POs; // Last Perceived Objects included in the previous CPM


  // The CP Basic Service can count up to 18446744073709551615 (UINT64_MAX) Cpms
  uint64_t m_cpm_sent; //! Statistic: number of CPMs successfully sent since the CP Basic Service has been started

  // ns-3 event IDs used to properly stop the simulation with terminateDissemination()
  EventId m_event_cpmDisseminationStart;
  EventId m_event_cpmSend;

  double m_last_transmission = 0;

  long m_T_next_dcc = -1;

  uint64_t m_wannabe_sent = 0;
};
}
#endif // CPBASICSERVICE_H
