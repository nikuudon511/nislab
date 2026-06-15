/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/areaSpeedAdvisorServer80211p-helper.h"
#include "ns3/areaSpeedAdvisorServer80211p.h"
#include "ns3/areaSpeedAdvisorClient80211p.h"
#include "ns3/areaSpeedAdvisorClient80211p-helper.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2i-80211p");

int
main (int argc, char *argv[])
{
  /*
   * In this example the generated vehicles will broadcast their CAMs, that will be also received by a RSU placed in the middle
   * of the simulated scenario. Both vehicles and RSU transmit V2X messages through a 802.11p interface.
   * The RSU broadcasts DENM messages with a frequency of 1 Hz, using the ETSI ITS-G5 stack, and the GeoNet
   * dissemination area is set as a circular area around the RSU (90 meters of diameter).
   * The dissemination of DENMs starts only after the RSU receives a CAM. If for 5 seconds
   * the RSU does not receive any CAM, the DENM dissemination stops.
   * Each DENM includes an optional container (à la carte), where there is a field named RoadWorks->SpeedLimit, in which
   * the maximum speed allowed in the dissemination area is specified (25km/h).
   * Whenever a vehicle receives a DENM at application layer (meaning that the information traverses GeoNet without being
   * filtered), it reads the information inside the à la carte container and corrects its speed accordingly.
   * If a vehicle doesn't receive any DENM for more than 1.5 seconds, it resumes its old speed.
   */

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2i_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string rsu_file = "stations.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2i_map/map.sumo.cfg";

  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  bool aggregate_out = false;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  bool print_summary = false;
  int txPower=23;
  float datarate=12;
  bool vehicle_vis = false;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = true;

  double simTime = 100;

  int numberOfNodes;
  uint32_t nodeCounter = 0;
  int numberOfRSUs;
  uint32_t rsuCounter = 0;

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for vehicular application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("server-aggregate-output", "Print an aggregate output for server", aggregate_out);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("summary", "Print a summary for each vehicle at the end of the simulation", print_summary);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);


  /* Cmd Line option for 802.11p */
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if(std::find(rate_admitted_values.begin(), rate_admitted_values.end(), datarate) == rate_admitted_values.end())
    {
      NS_FATAL_ERROR("Fatal error: invalid 802.11p data rate" << datarate << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if(datarate==4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string((int)datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2i-80211p", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("areaSpeedAdvisorServer80211p", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  std::string rsu_path = sumo_folder + rsu_file;
  std::ifstream rsu_file_stream (rsu_path.c_str());
  std::vector<std::tuple<std::string, float, float>> rsuData = XML_poli_count_stations(rsu_file_stream);
  numberOfRSUs = rsuData.size();

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for OBUs ***/
  NodeContainer obuNodes;
  obuNodes.Create(numberOfNodes);

  /*** 1.1 Create containers for RSUs ***/
  NodeContainer rsuNodes;
  rsuNodes.Create(numberOfRSUs);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  std::cout << "Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm." << std::endl;

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  
  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (datarate_config),
                                      "ControlMode", StringValue (datarate_config),
                                      "NonUnicastMode",StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes);
  NetDeviceContainer netRSUs = wifi80211p.Install (wifiPhy, wifi80211pMac, rsuNodes);
  //wifi80211p.EnableLogComponents ();

  wifiPhy.EnablePcap ("v2v-ASA",netDevices);
  wifiPhy.EnablePcap ("v2v-ASA-RSU",netRSUs);
  /* Give packet socket powers to nodes (otherwise, if the app tries to create a PacketSocket, CreateSocket will end up with a segmentation fault */
  PacketSocketHelper packetSocket;
  packetSocket.Install (obuNodes);
  packetSocket.Install (rsuNodes);

  /*** 6. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);
  MobilityHelper mobilityRSU;
  mobilityRSU.Install (rsuNodes);

  /*** 5. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumo_additional_options += " --collision.action warn --collision.check-junctions --error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  /*** 6. Create and Setup application for the server ***/
  areaSpeedAdvisorServer80211pHelper AreaSpeedAdvisorServer80211pHelper;
  AreaSpeedAdvisorServer80211pHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  AreaSpeedAdvisorServer80211pHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  AreaSpeedAdvisorServer80211pHelper.SetAttribute ("AggregateOutput", BooleanValue(aggregate_out));
  AreaSpeedAdvisorServer80211pHelper.SetAttribute ("CSV", StringValue(csv_name));
  AreaSpeedAdvisorServer80211pHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  int i = 0;
  for (auto rsu : rsuData)
    {
      std::string id = std::get<0>(rsu);
      float x = std::get<1>(rsu);
      float y = std::get<2>(rsu);
      Ptr<Node> rsuNode = rsuNodes.Get (i);
      sumoClient->AddStation(id, x, y, 0.0, rsuNode);
      ApplicationContainer AppServer = AreaSpeedAdvisorServer80211pHelper.Install (rsuNode);
      AppServer.Start (Seconds (0.0));
      AppServer.Stop (simulationTime - Seconds (0.1));
      ++rsuCounter;
      ++i;
    }

  /*** 7. Setup interface and application for dynamic nodes ***/
  areaSpeedAdvisorClient80211pHelper AreaSpeedAdvisorClient80211pHelper;
  Ipv4Address remoteHostAddr;

  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("ServerAddr", Ipv4AddressValue(remoteHostAddr));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("Client", (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("PrintSummary", BooleanValue(print_summary));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("CSV", StringValue(csv_name));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));
  AreaSpeedAdvisorClient80211pHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      /* Don't create and install the protocol stack of the node at simulation time -> take from "node pool" */
      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);
      ++nodeCounter; //increment counter for next node

      /* Install Application */
      //AreaSpeedAdvisorClient80211pHelper.SetAttribute ("PRRSupervisor", PointerValue (&prrSup));
      ApplicationContainer ClientApp = AreaSpeedAdvisorClient80211pHelper.Install (includedNode);
      ClientApp.Start (Seconds (0.0));
      ClientApp.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* Stop all applications */
      Ptr<areaSpeedAdvisorClient80211p> areaSpeedAdvisorClient80211p_ = exNode->GetApplication(0)->GetObject<areaSpeedAdvisorClient80211p>();
      if(areaSpeedAdvisorClient80211p_)
        areaSpeedAdvisorClient80211p_->StopApplicationNow ();

       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* Start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }

  return 0;
}


/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/
#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/emergencyVehicleWarningClient80211p-helper.h"
#include "ns3/emergencyVehicleWarningClient80211p.h"
#include "ns3/emergencyVehicleWarningServer80211p-helper.h"
#include "ns3/emergencyVehicleWarningServer80211p.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2v-80211p");

int
main (int argc, char *argv[])
{
  /*
   * In this example the generated vehicles will broadcast their CAMs. Every vehicle transmits its messages
   * through a 802.11p interface. There are two types of vehicles: normal "passenger" cars and emergency vehicles.
   * When a normal vehicle receives a CAM from an emergency vehicle it performs the following checks:
   * 1) It checks if the distance between itself and the emergency vehicle is under a certain threshold
   * 2) It checks if the difference in the heading angle between itself and the emergency vehicle is under a certain threshold
   * In case the two conditions are met, then the receiving vehicle will take some actions to facilitate the takover
   * maneuver (like slow down). When the emergency vehicle is far away, the normal vehicle will resume its old speed
   */

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2i_TM_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string rsu_file = "stations.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2i_TM_map/map.sumo.cfg";


  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  int txPower=26; //be aware of tx Power
  float datarate=12;  
  bool vehicle_vis = false;
  bool aggregate_out = false;
  bool send_cam = true;   // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  double m_baseline_prr = 150.0;
  bool m_metric_sup = true;


  double simTime = 100;

  int numberOfNodes;
  uint32_t nodeCounter = 0;
  int numberOfRSUs;
  uint32_t rsuCounter = 0;

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("server-aggregate-output", "Print an aggregate output for server", aggregate_out);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);

  /* Cmd Line option for 802.11p */
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if(std::find(rate_admitted_values.begin(), rate_admitted_values.end(), datarate) == rate_admitted_values.end())
    {
      NS_FATAL_ERROR("Fatal error: invalid 802.11p data rate" << datarate << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if(datarate==4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string((int)datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2v-80211p", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  std::string rsu_path = sumo_folder + rsu_file;
  std::ifstream rsu_file_stream (rsu_path.c_str());
  std::vector<std::tuple<std::string, float, float>> rsuData = XML_poli_count_stations(rsu_file_stream);
  numberOfRSUs = rsuData.size();

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for OBUs ***/
  NodeContainer obuNodes;
  obuNodes.Create(numberOfNodes);

  /*** 1.1 Create containers for RSUs ***/
  NodeContainer rsuNodes;
  rsuNodes.Create(numberOfRSUs);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  NS_LOG_INFO("Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm.");

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  /* To be removed when BPT is implemented */
  //Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue (Seconds (1)));

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (datarate_config), "ControlMode", StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes);
  NetDeviceContainer netRSUs = wifi80211p.Install (wifiPhy, wifi80211pMac, rsuNodes);

  //wifiPhy.EnablePcapAll ("ivimVehicleWarningExample");
  /*** 4. Give packet socket powers to nodes (otherwise, if the app tries to create a PacketSocket, CreateSocket will end up with a segmentation fault */
  PacketSocketHelper packetSocket;
  packetSocket.Install (obuNodes);
  packetSocket.Install (rsuNodes);

  /*** 5. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);
  MobilityHelper mobilityRSU;
  mobilityRSU.Install (rsuNodes);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumo_additional_options += " --collision.action warn --collision.check-junctions --error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  /*** 7. Create and Setup application for the server ***/
  emergencyVehicleWarningServer80211pHelper emergencyVehicleWarningServerHelper;
  emergencyVehicleWarningServerHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  emergencyVehicleWarningServerHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  emergencyVehicleWarningServerHelper.SetAttribute ("AggregateOutput", BooleanValue(aggregate_out));
  emergencyVehicleWarningServerHelper.SetAttribute ("CSV", StringValue(csv_name));
  emergencyVehicleWarningServerHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  int i = 0;
  for (auto rsu : rsuData)
    {
      std::string id = std::get<0>(rsu);
      float x = std::get<1>(rsu);
      float y = std::get<2>(rsu);
      Ptr<Node> rsuNode = rsuNodes.Get (i);
      sumoClient->AddStation(id, x, y, 0.0, rsuNode);
      ApplicationContainer AppServer = emergencyVehicleWarningServerHelper.Install (rsuNode);
      AppServer.Start (Seconds (0.0));
      AppServer.Stop (simulationTime - Seconds (0.1));
      ++rsuCounter;
      ++i;
    }

  /*** 8. Setup interface and application for dynamic nodes ***/
  emergencyVehicleWarningClient80211pHelper EmergencyVehicleWarningClientHelper;
  EmergencyVehicleWarningClientHelper.SetAttribute ("Client", PointerValue (sumoClient));
  EmergencyVehicleWarningClientHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  EmergencyVehicleWarningClientHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  EmergencyVehicleWarningClientHelper.SetAttribute ("Model", StringValue ("80211p"));
  EmergencyVehicleWarningClientHelper.SetAttribute ("CSV", StringValue(csv_name));
  EmergencyVehicleWarningClientHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));
  EmergencyVehicleWarningClientHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      ApplicationContainer AppSample = EmergencyVehicleWarningClientHelper.Install (includedNode);

      AppSample.Start (Seconds (0.0));
      AppSample.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* Callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* stop all applications */
      Ptr<emergencyVehicleWarningClient80211p> appSample_ = exNode->GetApplication(0)->GetObject<emergencyVehicleWarningClient80211p>();

      if(appSample_)
        appSample_->StopApplicationNow();

       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* Start TraCI client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 9. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," <<metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }

  return 0;
}

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

//#include "ns3/automotive-module.h"
#include "ns3/carla-module.h"
#include "ns3/trafficManagerClientLTE-helper.h"
#include "ns3/trafficManagerServerLTE-helper.h"
#include "ns3/trafficManagerServerLTE.h"
#include "ns3/trafficManagerClientLTE.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/lte-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2i-LTE-tm");

int
main (int argc, char *argv[])
{
  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2i_TM_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string eNodeB_file = "stations.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2i_TM_map/map.sumo.cfg";

  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  bool print_summary = false;
  bool vehicle_vis = false;

  double simTime = 200;

  bool useCBRSupervisor = true;

  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = true;

  //Traffic Lights phase duration
  int phase_time = 20;
  //Traffic Manager preemption ratio threshold
  double threshold = 70;

  /*** 0.b LENA Options ***/
  double interPacketInterval = 100;
  bool useCa = false;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  int numberOfENodeBs;
  uint32_t eNodeBCounter = 0;

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line option for vehicular application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("summary", "Print a summary for each vehicle at the end of the simulation", print_summary);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);

  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);

  cmd.AddValue ("phase-time", "Duration of traffic lights phases", phase_time);
  cmd.AddValue ("threshold","Threshold multiplier for traffic light phase preemption",threshold);

  /* Cmd Line option for Lena */
  cmd.AddValue("interPacketInterval", "Inter packet interval [ms]", interPacketInterval);
  cmd.AddValue("useCa", "Whether to use carrier aggregation", useCa);


  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("trafficManagerServerLTE", LOG_LEVEL_INFO);
      LogComponentEnable ("trafficManagerClientLTE", LOG_LEVEL_INFO);
    }

  /* Carrier aggregation for LTE */
  if (useCa)
   {
     Config::SetDefault ("ns3::LteHelper::UseCa", BooleanValue (useCa));
     Config::SetDefault ("ns3::LteHelper::NumberOfComponentCarriers", UintegerValue (2));
     Config::SetDefault ("ns3::LteHelper::EnbComponentCarrierManager", StringValue ("ns3::RrComponentCarrierManager"));
   }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  std::string eNodeB_path = sumo_folder + eNodeB_file;
  std::ifstream eNodeB_file_stream (eNodeB_path.c_str());
  std::vector<std::tuple<std::string, float, float>> eNodeBData = XML_poli_count_stations(eNodeB_file_stream);
  numberOfENodeBs = eNodeBData.size();

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create LTE objects
       the network topology created is the following:

       UEs->(LTE CHANNEL)->enB->(SGW->PGW)->RemoteHost
                                  ^EPC^
   ***/
  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  ConfigStore inputConfig;
  inputConfig.ConfigureDefaults();

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  /*** 2. Create (and configure) the remote host that will gather the CAM and send the DENM ***/
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  /* Connect the remote host to the packet gateway and create the Internet */
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("10Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.005)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("10.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  /* interface 0 is localhost, 1 is the p2p device */
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  // p2ph.EnablePcap ("trial", internetDevices.Get(0));

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  /*** 3. Create containers for UEs and eNB ***/
  NodeContainer ueNodes;
  ueNodes.Create(numberOfNodes);
  NodeContainer enbNodes;
  enbNodes.Create(numberOfENodeBs);

  /*** 4. Create and install mobility (SUMO will be attached later) ***/
  MobilityHelper mobility;
  mobility.Install(enbNodes);
  mobility.Install(ueNodes);

  /*** 5. Install LTE Devices to the nodes + assign IP to UE***/
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  /* Install the IP stack on the UEs */
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;

  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  /* Assign IP address to UEs */
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      /* Set the default gateway for the UE */
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  for (uint16_t i = 0; i < numberOfNodes; i++)
    {
      for (uint16_t j = 0; j < numberOfENodeBs; j++)
        {
          lteHelper->Attach(ueLteDevs.Get (i), enbLteDevs.Get (j));
        }
    }

  lteHelper->AddX2Interface (enbNodes);
  // lteHelper->EnableTraces ();

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", (BooleanValue) sumo_gui);
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumo_additional_options += " --collision.action warn --collision.check-junctions --error-log=sumo-errors-or-collisions.xml";

  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));
  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  /*** 6. Create and Setup application for the server ***/
  trafficManagerServerLTEHelper TrafficManagerServerLTEHelper;
  TrafficManagerServerLTEHelper.SetAttribute ("Client", (PointerValue) sumoClient);
  TrafficManagerServerLTEHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  TrafficManagerServerLTEHelper.SetAttribute ("PhaseTime", UintegerValue(phase_time));
  TrafficManagerServerLTEHelper.SetAttribute ("Threshold", DoubleValue(threshold));
  TrafficManagerServerLTEHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  int i = 0;
  for (auto e : eNodeBData)
    {
      std::string id = std::get<0> (e);
      float x = std::get<1> (e);
      float y = std::get<2> (e);
      Ptr<Node> eNodeB = enbNodes.Get (i);
      sumoClient->AddStation (id, x, y, 0.0, eNodeB);
      ++eNodeBCounter;
      ++i;
    }

  ApplicationContainer AppServer = TrafficManagerServerLTEHelper.Install (remoteHostContainer.Get (0));

  AppServer.Start (Seconds (0.0));
  AppServer.Stop (simulationTime - Seconds (0.1));

  /*** 7. Setup interface and application for dynamic nodes ***/
  trafficManagerClientLTEHelper TrafficManagerClientLTEHelper;
  TrafficManagerClientLTEHelper.SetAttribute ("ServerAddr", Ipv4AddressValue(remoteHostAddr));
  TrafficManagerClientLTEHelper.SetAttribute ("Client", (PointerValue) sumoClient); // pass TraciClient object for accessing sumo in application
  TrafficManagerClientLTEHelper.SetAttribute ("PrintSummary", BooleanValue(print_summary));
  TrafficManagerClientLTEHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  TrafficManagerClientLTEHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  TrafficManagerClientLTEHelper.SetAttribute ("SendCAM", BooleanValue (send_cam));


  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= ueNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      /* Don't create and install the protocol stack of the node at simulation time -> take from "node pool" */
      Ptr<Node> includedNode = ueNodes.Get(nodeCounter);
      ++nodeCounter; //increment counter for next node

      /* Install Application */
      ApplicationContainer ClientApp = TrafficManagerClientLTEHelper.Install (includedNode);
      ClientApp.Start (Seconds (0.0));
      ClientApp.Stop (simulationTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* Stop all applications */
      Ptr<trafficManagerClientLTE> trafficManagerClientLTE_ = exNode->GetApplication(0)->GetObject<trafficManagerClientLTE>();
      if(trafficManagerClientLTE_)
        trafficManagerClientLTE_->StopApplicationNow ();

       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* Start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }


  return 0;
}

#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/gn-utils.h"
#include "ns3/btp.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/caBasicService.h"
#include "ns3/denBasicService.h"
#include "ns3/VRUBasicService.h"
#include "ns3/BSContainer.h"
#include "ns3/BSMap.h"
#include <unistd.h>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2p-vam-80211p");

// Global variables, accessible from any function in this file
static int received_CAMs_veh = 0;
static int received_CAMs_ped = 0;
static int received_VAMs_veh = 0;
static int received_VAMs_ped = 0;

BSMap basicServices;
// ******************************************************************************************************************************************************************

void receiveCAM_veh(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType)
{
  received_CAMs_veh++;
}

void receiveVAM_veh(asn1cpp::Seq<VAM> vam, Address from)
{
  received_VAMs_veh++;
}

void receiveCAM_ped(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType)
{
  received_CAMs_ped++;
}

void receiveVAM_ped(asn1cpp::Seq<VAM> vam, Address from)
{
  received_VAMs_ped++;
}

int main (int argc, char *argv[])
{
  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string sumo_folder = "src/automotive/examples/v2p_map/";
  std::string mob_trace_veh = "vehicles.rou.xml";
  std::string mob_trace_ped = "pedestrians.rou.xml";
  std::string sumo_config = "src/automotive/examples/v2p_map/v2p_map.sumo.cfg";

  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  int txPower=23;

  std::string csv_name = "VAM_metrics";
  std::string csv_name_cumulative = "VAM_PRR_latency";

  float datarate = 12;
  bool vehicle_vis = false;

  long computationT_acc = 0;

  // Disabling this option turns off the whole V2X application (useful for comparing the situation when the application is enabled and the one in which it is disabled)
  bool send_cam = true;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = false;
  bool m_VAM_metrics = true;

  double simTime = 250;

  int numberOfNodes_veh;
  int numberOfNodes_ped;
    
  int up = 0; // User priority

  CommandLine cmd;

  xmlDocPtr rou_xml_file;

  /* Cmd Line options for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace-veh", "Name of the mobility trace file for vehicles", mob_trace_veh);
  cmd.AddValue ("mob-trace-ped", "Name of the mobility trace file for pedestrians", mob_trace_ped);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("send-cam", "Turn on or off the transmission of CAMs, thus turning on or off the whole V2X application",send_cam);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric Supervisor or not",m_metric_sup);

  /* Cmd Line option for 802.11p */
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if(std::find(rate_admitted_values.begin(), rate_admitted_values.end(), datarate) == rate_admitted_values.end())
    {
      NS_FATAL_ERROR("Fatal error: invalid 802.11p data rate" << datarate << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if(datarate==4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string((int)datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2p-vam-80211p", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the mob_trace_veh the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from vehicles.rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file containing vehicles' routes...");
  std::string path_veh = sumo_folder + mob_trace_veh;

  /* Load the rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path_veh.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path_veh);
    }
  numberOfNodes_veh = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes_veh==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path_veh<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes_veh << " vehicles will be present in the simulation.");

  /*** 0.c Read from the mob_trace_ped the number of pedestrians that will be created.
   *       The number of pedestrians is directly parsed from pedestrians.rou.xml file, looking at all
   *       the valid XML elements of type <person>
  ***/
  NS_LOG_INFO("Reading the .rou file containing pedestrians' routes...");
  std::string path_ped = sumo_folder + mob_trace_ped;

  /* Load the rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path_ped.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path_ped);
    }
  numberOfNodes_ped = XML_rou_count_pedestrians (rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes_ped==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of pedestrians from the specified XML file: "<<path_ped<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes_ped << " pedestrians will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for both vehicles and pedestrians ***/
  NodeContainer Nodes;
  Nodes.Create(numberOfNodes_veh+numberOfNodes_ped);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  NS_LOG_INFO("Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm.");

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  /* To be removed when BPT is implemented */
  //Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue (Seconds (1)));

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  std::cout << "Datarate: " << datarate_config << std::endl;
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode", StringValue (datarate_config),
                                      "ControlMode", StringValue (datarate_config),
                                      "NonUnicastMode",StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, Nodes);

  // Enable saving to Wireshark PCAP traces
  wifiPhy.EnablePcap ("v2p-vam-80211p", netDevices);

  /*** 4. Give packet socket powers to nodes (otherwise, if the app tries to create a PacketSocket, CreateSocket will end up with a segmentation fault) */
  PacketSocketHelper packetSocket;
  packetSocket.Install (Nodes);

  /*** 5. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (Nodes);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
    }

  // Creation of the file where VAM metrics will be stored
  std::ofstream csv_VAM_ofstream;
  std::string full_csv_VAM_name = csv_name + ".csv";
  if(m_VAM_metrics){
      if(full_csv_VAM_name != ""){
          csv_VAM_ofstream.open(full_csv_VAM_name, std::ofstream::out | std::ofstream::trunc);
          csv_VAM_ofstream << "station_ID,timestamp,time_elapsed_since_last_gen,triggering_condition,stationID_safe_dist,latitude,longitude,speed" << std::endl;
          csv_VAM_ofstream.close ();
        }
    }

  /*Ptr<WifiNetDevice> wifiAPdevice = DynamicCast<WifiNetDevice> (netDevices.Get(1)); // Getting the PHY of the first (and only, in this case, AP)
  Ptr<WifiPhy> actualPhy = wifiAPdevice->GetPhy ();
  std::cout<<actualPhy->GetTxPowerEnd()<<std::endl;*/

  /* callback function for node creation */
  STARTUP_FCN setupNewWifiNode = [&] (std::string stationID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(stationID.substr (3));
      std::string station_type_ID = stationID.substr(0,3);
      StationType_t station_type;

      // Identification of the station type and extraction of the node
      Ptr<Node> includedNode = nullptr;
      if(!station_type_ID.compare("veh")){
          station_type = StationType_passengerCar;
          includedNode = Nodes.Get(nodeID);
      } else{
          station_type = StationType_pedestrian;
          includedNode = Nodes.Get(nodeID-1000+numberOfNodes_veh);
      }

      Ptr<Socket> sock = GeoNet::createGNPacketSocket(includedNode);
      // Set the proper AC, through the specified UP
      sock->SetPriority (up);

      // Create a new Basic Service Container object, which includes, for the current vehicle, both the ETSI CA Basic Service, for the transmission/reception
      // of periodic CAMs, and the ETSI DEN Basic Service, for the transmission/reception of event-based DENMs
      // An ETSI Basic Services container is a wrapper class to enable easy handling of both CAMs and DENMs
      Ptr<BSContainer> bs_container = CreateObject<BSContainer>(nodeID,station_type,sumoClient,false,sock);

      // Setup the Metricsupervisor inside the BSContainer, to make each station collect latency and PRR metrics
      bs_container->linkMetricSupervisor(metSup);

      // Setup the file where VAM metrics will be stored
      bs_container->setVAMmetricsfile (full_csv_VAM_name, m_VAM_metrics);

      // This is needed just to simplify the whole application
      bs_container->disablePRRSupervisorForGNBeacons ();

      // Set the function which will be called every time a CAM is received and the function called every time a VAM is received
      if(!station_type_ID.compare("veh")){
        bs_container->addCAMRxCallback (std::bind(&receiveCAM_veh,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
        bs_container->addVAMRxCallback (std::bind(&receiveVAM_veh,std::placeholders::_1,std::placeholders::_2));
      } else{
        bs_container->addCAMRxCallback (std::bind(&receiveCAM_ped,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4));
        bs_container->addVAMRxCallback (std::bind(&receiveVAM_ped,std::placeholders::_1,std::placeholders::_2));
      }
      // Setup the new ETSI Basic Services container
      // The first parameter should be true is you want to setup a CA Basic Service (for sending/receiving CAMs)
      // The second parameter should be true if you want to setup a DEN Basic Service (for sending/receiving DENMs)
      // The third parameter should be true if you want to setup a VRU Basic Service (for sending/receiving VAMs)
      // The fourth parameter should be true if you want to setup a CP Service (for sending/receiving CPMs)
      // The fifth parameter should be true if you want to setup a MC Service (for sending/receiving MCMs)
      // The sixth parameter should be true if you want to enable V2X security
      bs_container->setupContainer(true, false, true, false, false, false);

      // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
      // retrieve the right BSContainer given a vehicle ID
      basicServices.add(bs_container);

      std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
      double desync = ((double)std::rand()/RAND_MAX);

      // Start transmitting CAMs (only vehicles) and VAMs (only pedestrians)
      if(!station_type_ID.compare("veh"))
        bs_container->getCABasicService ()->startCamDissemination ();
      else{
          bs_container->getVRUBasicService ()->startAccelerationComputation (computationT_acc);
          bs_container->getVRUBasicService ()->startVamDissemination ();
        }

      return includedNode;
    };

  /* Callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string stationID)
    {
       /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-100000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      // Terminate the transmission of CAMs and VAMs
      unsigned long nodeID = std::stol(stationID.substr (3));

      Ptr<BSContainer> bsc = basicServices.get(nodeID);
      bsc->cleanup();
    };

  /* Start TraCI client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative != "")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK) != -1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,datarate,num_vehicles,num_pedestrians,avg_PRR_vehicles,"
                              "avg_PRR_pedestrians,avg_PRR_individual_pedestrian,avg_latency_ms_vehicles,"
                              "avg_latency_ms_pedestrians,avg_latency_ms_individual_pedestrian,num_received_CAMs_veh,"
                              "num_received_CAMs_ped,num_received_VAMs_veh,num_received_VAMs_ped" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << datarate << "," << numberOfNodes_veh << "," << numberOfNodes_ped << "," <<
                            metSup->getAveragePRR_overall() << "," <<
                            metSup->getAveragePRR_pedestrian (1000) << "," << metSup->getAverageLatency_overall () <<
                            "," <<  metSup->getAverageLatency_pedestrian (1000) <<
                            "," << received_CAMs_veh << "," << received_CAMs_ped << "," << received_VAMs_veh << "," <<
                            received_VAMs_ped << std::endl;
      }
    }
    
  // Print the number of CAMs received by vehicles and pedestrians
  std::cout << "Number of received CAMs by vehicles =  " << received_CAMs_veh << std::endl;
  std::cout << "Number of received CAMs by pedestrians =  " << received_CAMs_ped << std::endl;

  // Print the number of VAMs received by vehicles and pedestrians
  std::cout << "Number of received VAMs by vehicles =  " << received_VAMs_veh << std::endl;
  std::cout << "Number of received VAMs by pedestrians =  " << received_VAMs_ped << std::endl;

  return 0;
}

/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
*/


//#include "ns3/automotive-module.h"
#include "ns3/carla-module.h"
#include "ns3/OpenCDAClient.h"
#include "ns3/simpleCAMSender-gps-tc.h"
#include "ns3/simpleVAMSender-gps-tc.h"
#include "ns3/simpleCAMSender-helper.h"
#include "ns3/simpleVAMSender-helper.h"
#include "ns3/gps-tc-module.h"
#include "ns3/internet-module.h"
#include "ns3/wave-module.h"
#include "ns3/mobility-module.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/DCC.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-address.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("v2v-80211p-gps-tc-example");

static void GenerateTraffic_interfering (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval)
{
  // Generate interfering traffic by sending pktCount packets (filled in with zeros), every pktInterval
  if (pktCount > 0)
    {
      // "Create<Packet> (pktSize)" creates a new packet of size pktSize bytes, composed by default by all zeros
      if (socket->Send (Create<Packet> (pktSize)) != -1 && pktCount%100==0)
        {
          // std::cout << "Interfering packet sent" << std::endl;
        }
      // Schedule again the same function (to send the next packet), and decrease by one the packet count
      Simulator::Schedule (pktInterval, &GenerateTraffic_interfering,
                           socket, pktSize, pktCount, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

int
main (int argc, char *argv[])
{
  /*
   * In this example instead of relying on SUMO, the vehicles' mobility will be managed by the "gps-tc" module.
   * "gps-tc" is a module that is able to load GPS traces in a specific CSV format, with few prerequisites, and use
   * them to simulate the presence of N vehicles from pre-recorded real GPS traces.
   * For the time being, the CSV format should be similar to the one available in "/src/gps-tc/examples/GPS-Traces-Sample" and
   * the positioning updates should occur, if possible, at least at around 5-10 Hz (1 Hz may impact the effectiveness of the dynamic
   * CAM generation). You can also select an example trace, however, taken from a low-end GNSS device, embedded inside a consumer
   * smartphone (in this case the trace is taken at 1 Hz).
   * This also showcases how the GPS Trace Client module is able to accept as input any kind of CSV trace, no matter if it is taken
   * with more professional devices (updating at least at 5-10 Hz - always the best option) or if it comes from the user's smartphone,
   * updating at 1 Hz (which can be used, for instance, to analyze the behaviour of a vehicular application when non-professional and
   * cheap GNSS devices are used, providing a low update rate).
   *
   * In this example the generated vehicles will simply broadcast their CAMs, relying on the application named simpleCAMSender-gps.
   * 802.11p is used as access technology.
   *
   * If the correct prerequisites for ns-3 PyViz are installed, the user can also see the moving nodes in a GUI by running this
   * example with the "--vis" option.
   */

  // Admitted data rates for 802.11p
  std::vector<float> rate_admitted_values{3,4.5,6,9,12,18,24,27};
  std::string datarate_config;

  /*** 0.a App Options ***/
  std::string trace_file_path = "src/gps-tc/examples/GPS-Traces-Sample/";
  // If you want to use a sample GPS trace obtained using a high-end expensive device (with
  // high update rate and inertial sensors - 3 vehicles), uncomment this line (and comment
  // the "BiellaTrace.csv" one)
  std::string gps_trace = "A4_andata_all_veh.csv";
  // If you want to use a sample GPS trace obtained using a normal smartphone, with the
  // Ultra GPS Logger app (update rate: 1 Hz, no acceleration, 2 vehicles), uncomment this
  // line (and comment the "sampletrace.csv" one)
  // std::string gps_trace = "BiellaTrace.csv";

  bool verbose = false;
  bool realtime = false;
  int txPower=30;
  float datarate=12;
  bool vehicle_vis = false;

  bool use_input_microseconds = true; // For all new traces, for old ones set false

  double simTime = 10000;

  uint32_t nodeCounter = 0;

  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("trace-folder","Position of GPS trace files",trace_file_path);
  cmd.AddValue ("gps-trace", "Name of the GPS trace file", gps_trace);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);

  /* Cmd Line option for 802.11p */
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("datarate", "802.11p channel data rate [Mbit/s]", datarate);

  cmd.AddValue("sim-time", "Total duration of the simulation [s]", simTime);

  cmd.Parse (argc, argv);

  if(std::find(rate_admitted_values.begin(), rate_admitted_values.end(), datarate) == rate_admitted_values.end())
    {
      NS_FATAL_ERROR("Fatal error: invalid 802.11p data rate" << datarate << "Mbit/s. Valid rates are: 3, 4.5, 6, 9, 12, 18, 24, 27 Mbit/s.");
    }
  else
    {
      if(datarate==4.5)
        {
          datarate_config = "OfdmRate4_5MbpsBW10MHz";
        }
      else
        {
          datarate_config = "OfdmRate" + std::to_string((int)datarate) + "MbpsBW10MHz";
        }
    }

  if (verbose)
    {
      LogComponentEnable ("v2v-80211p-gps-tc", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the GPS Trace and create a GPS Trace Client for each vehicle
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = trace_file_path + gps_trace;

  std::map<std::string,GPSTraceClient*> GPSTCMap;

  GPSTraceClientHelper GPSTCHelper;

  GPSTCHelper.setVerbose(verbose);

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();

      GPSTCHelper.setVehicleVisualizer(vehicleVis);
  }

  // If you want to use set up the GPS Trace Client Helper to read data from "BiellaTrace.csv", obtainer on a regular smartphone with
  // the Ultra GPS Logger app (update rate: 1 Hz, no acceleration), uncomment these lines
  // They will set the right columns names for the data inside the CSV file and turn on interpolation to get a more "smooth" trace
//  GPSTCHelper.setVehicleIDColumnName ("VehID");
//  GPSTCHelper.setLatitudeColumnName ("Lat");
//  GPSTCHelper.setLongitudeColumnName ("Lng");
//  // As not all the apps/low-end GNSS devices are able to output an acceleration value, you can also specify
//  // "unavailable": in this case, the acceleration will be estimated, between points, as contant, and calculated simply as delta(v)/delta(t)
//  // Of course this is an approximation of the acceleration and may not reflect the actual acceleration experienced by the vehicle
//  // in between two data points
//  GPSTCHelper.setAccelerationColumnName ("unavailable");
//  GPSTCHelper.setHeadingColumnName ("Bearing");
//  GPSTCHelper.setSpeedColumnName ("Speed");
//  GPSTCHelper.setTimestampColumnName ("TimeWithMS",true,"%H:%M:%S.%MSEC",{.tm_year=2020,.tm_mon=9,.tm_mday=28});


  // IMPORTANT to set the input time in microseconds (pass true to the function) if the trace has timestamps in microseconds (to avoid the conversion)
  GPSTCHelper.SetInputMicroseconds(use_input_microseconds);

  GPSTCMap=GPSTCHelper.createTraceClientsFromCSV(path);

  int numberOfNodes=GPSTCMap.size ();
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");

  /* Set the simulation time (in seconds) */
  NS_LOG_INFO("Simulation will last " << simTime << " seconds");
  ns3::Time simulationTime (ns3::Seconds(simTime));

  /*** 1. Create containers for OBUs ***/
  NodeContainer obuNodes;
  obuNodes.Create(numberOfNodes);

  /*** 2. Create and setup channel   ***/
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  NS_LOG_INFO("Setting up the 802.11p channel @ " << datarate << " Mbit/s, 10 MHz, and tx power " << (int)txPower << " dBm.");

  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  /* To be removed when BPT is implemented */
  //Config::SetDefault ("ns3::ArpCache::DeadTimeout", TimeValue (Seconds (1)));

  /*** 3. Create and setup MAC ***/
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue (datarate_config), "ControlMode", StringValue (datarate_config));
  NetDeviceContainer netDevices = wifi80211p.Install (wifiPhy, wifi80211pMac, obuNodes);

  wifiPhy.EnablePcap ("v2v-tracenx-dcc-A4", netDevices);
  /*** 4. Create Internet and ipv4 helpers ***/
  PacketSocketHelper packetSocket;
  packetSocket.Install (obuNodes);

  /*** 6. Setup Mobility and position node pool ***/
  MobilityHelper mobility;
  mobility.Install (obuNodes);

  /*** 7. Setup interface and application for dynamic nodes ***/
  simpleCAMSenderHelper SimpleCAMSenderHelper;
  SimpleCAMSenderHelper.SetAttribute ("RealTime", BooleanValue(realtime));

  simpleVAMSenderHelper SimpleVAMSenderHelper;
  SimpleVAMSenderHelper.SetAttribute ("RealTime", BooleanValue(realtime));

  // Create vector with the GPS Trace Client map values
  std::vector<GPSTraceClient*> v_gps_tc;
  GPS_TC_MAP_ITERATOR(GPSTCMap,GPSTCit) {
    v_gps_tc.push_back (GPS_TC_IT_OBJECT(GPSTCit));
  }

  Ptr<Socket> source_interfering;
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  Ptr<Socket> socket = Socket::CreateSocket (obuNodes.Get (0), tid);
  PacketSocketAddress local_source_interfering;
  local_source_interfering.SetSingleDevice (obuNodes.Get (0)->GetDevice(0)->GetIfIndex ());
  local_source_interfering.SetPhysicalAddress (obuNodes.Get (0)->GetDevice(0)->GetAddress());
  local_source_interfering.SetProtocol (0x88B5); // Setting the "Local Experimental Ethertype 1" to send interfering traffic
  if (socket->Bind (local_source_interfering) == -1)
    {
      NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
    }
  PacketSocketAddress remote_source_interfering;
  remote_source_interfering.SetSingleDevice (obuNodes.Get (0)->GetDevice(0)->GetIfIndex());
  // Set the broadcast MAC address as interfering traffic will be broadcasted by vehicle 3
  remote_source_interfering.SetPhysicalAddress (obuNodes.Get (0)->GetDevice(0)->GetBroadcast());
  remote_source_interfering.SetProtocol (0x88B5); // Setting the "Local Experimental Ethertype 1" to send interfering traffic
  socket->Connect (remote_source_interfering);
  // Important: this line lets you set the AC of the interfering traffic, through a User Priority (UP) value, like in real Linux kernels/OS
  socket->SetPriority (0); // Setting the priority of the interfering traffic from vehicle 0
  source_interfering = socket;

  Ptr<MetricSupervisor> metSup = NULL;
  // Set a baseline for the PRR computation when creating a new PRRsupervisor object
  MetricSupervisor metSupObj(150);
  metSup = &metSupObj;
  // This function enables printing the current and average latency and PRR for each received packet
  metSup->setChannelTechnology("80211p");
  // metSup->enableCBRVerboseOnStdout();
  // metSup->enableCBRWriteToFile();
  metSup->setCBRWindowValue(200);
  metSup->setCBRAlphaValue(0.0);
  metSup->setSimulationTimeValue(simTime);
  metSup->setNodeContainer(obuNodes);
  metSup->setGPSTCClient(v_gps_tc);

  /* callback function for node creation */
  STARTUP_GPS_FCN setupNewWifiNode = [&] (std::string ID) -> Ptr<Node>
    {

      if (nodeCounter >= obuNodes.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      if (nodeCounter == 0)
        {
          Simulator::ScheduleWithContext (nodeCounter,
                                          Seconds (1.0), &GenerateTraffic_interfering,
                                          source_interfering, 1500 /* Size */, simTime*2000, MilliSeconds (3) /* Frequency */);
        }

      Ptr<Node> includedNode = obuNodes.Get(nodeCounter);

      /* Install Application */
      ApplicationContainer setupAppSimpleSender;
      Ptr<GPSTraceClient> gps_tc = nullptr;

      for (auto it = v_gps_tc.begin(); it != v_gps_tc.end(); ++it)
        {
          if ((*it)->getID() == ID)
            {
              gps_tc = *it;
              break;
            }
        }

      NS_ASSERT_MSG (gps_tc != nullptr, "GPS Trace Client not found.");

      if (gps_tc->getType() == "car")
        {
          gps_tc->setNodeID(std::to_string (nodeCounter));
          SimpleCAMSenderHelper.SetAttribute ("GPSClient", PointerValue(v_gps_tc[nodeCounter]));
          SimpleCAMSenderHelper.SetAttribute ("DCC", BooleanValue(true));
          SimpleCAMSenderHelper.SetAttribute ("DCCWindow", IntegerValue(200));
          SimpleCAMSenderHelper.SetAttribute ("DCCModality", StringValue("reactive"));
          SimpleCAMSenderHelper.SetAttribute ("MetricSupervisor", PointerValue(metSup));
          setupAppSimpleSender = SimpleCAMSenderHelper.Install (includedNode);
        }
      else if (gps_tc->getType() == "vru")
        {
          SimpleVAMSenderHelper.SetAttribute ("GPSClient", PointerValue(v_gps_tc[nodeCounter]));
          setupAppSimpleSender = SimpleVAMSenderHelper.Install (includedNode);
        }
      setupAppSimpleSender.Start (Seconds (0.0));
      setupAppSimpleSender.Stop (simulationTime - Simulator::Now () - Seconds (0.1));
      ++nodeCounter; // increment counter for next node
      if (nodeCounter >= obuNodes.GetN())
        {
          metSup->startCheckCBR();
        }
      return includedNode;
    };

  // We can create a variable here and use it in the lambda functions below and main() will not terminate before the whole
  // simulation is terminated (thus the value of remainingNodes will always be accessible)
  int remainingNodes = obuNodes.GetN();

  /* callback function for node shutdown */
  SHUTDOWN_GPS_FCN shutdownWifiNode = [&] (Ptr<Node> exNode, std::string ID)
    {
      /* stop all applications */

      Ptr<simpleCAMSender> AppSimpleCAMSender_ = nullptr;
      Ptr<simpleVAMSender> AppSimpleVAMSender_ = nullptr;
      Ptr<GPSTraceClient> gps_tc = nullptr;

      for (auto it = v_gps_tc.begin(); it != v_gps_tc.end(); ++it)
        {
          if ((*it)->getID() == ID)
            {
              gps_tc = *it;
              break;
            }
        }

      NS_ASSERT_MSG (gps_tc != nullptr, "GPS Trace Client not found.");

      if (gps_tc->getType() == "car")
        {
          AppSimpleCAMSender_ = exNode->GetApplication(0)->GetObject<simpleCAMSender>();
        }
      else if (gps_tc->getType() == "vru")
        {
          AppSimpleVAMSender_ = exNode->GetApplication(0)->GetObject<simpleVAMSender>();
        }

      if(AppSimpleCAMSender_ != nullptr)
        {
          AppSimpleCAMSender_->StopApplicationNow();
          remainingNodes--;
        }

      if (AppSimpleVAMSender_ != nullptr)
        {
          AppSimpleVAMSender_->StopApplicationNow();
          remainingNodes--;
        }

      if(remainingNodes==0)
        {
          Simulator::Stop();
        }

       /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));// rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  // Start "playing" the GPS Trace for each vehicle (i.e. make the vehicle start their movements)
  // "Seconds(0)" is specified to "playTrace" to reproduce all the traces since the beginning
  // of the simulation. A different amount of time for all the vehicles or a different amount of time
  // for each vehicle can also be specified to delay the reproduction of the traces
  GPS_TC_MAP_ITERATOR(GPSTCMap,GPSTCit) {
      GPS_TC_IT_OBJECT(GPSTCit)->GPSTraceClientSetup(setupNewWifiNode,shutdownWifiNode);
      GPS_TC_IT_OBJECT(GPSTCit)->playTrace(Seconds(0));
  }

  /*** 8. Start Simulation ***/
  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/gn-utils.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/sionna-helper.h"
#include <unistd.h>
#include "ns3/core-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchangeNrv2x");

void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
      {
          { "0", 0 },
          { "1", 1 },
      };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}

static int packet_count=0;

BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles

void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  double snr = phy_info.snr;
  double sinr = phy_info.sinr;
  double rssi = phy_info.rssi;
  double rsrp = phy_info.rsrp;
  if (std::isnan(snr) && !std::isnan(sinr))
    {
      snr = sinr;
    }
  if (std::isnan(rssi) && !std::isnan(rsrp))
    {
      rssi = rsrp;
    }

  libsumo::TraCIPosition pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);

  // Get the position of the sender
  double lat_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/1e7;
  double lon_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/1e7;

  // Compute the distance between the sender and the receiver
  double distance = haversineDist (lat_sender, lon_sender, pos.y, pos.x);

  std::ofstream camFile;
  camFile.open("src/sionna/phy_with_sionna_nrv2x.csv", std::ios::out | std::ios::app);
  camFile.seekp (0, std::ios::end);
  if (camFile.tellp() == 0)
    {
      camFile << "tx_id,rx_id,distance,rssi,snr" << std::endl;
    }
  
  camFile << cam->header.stationId << "," << my_stationID << "," << distance << "," << rssi << "," << snr << std::endl;
  camFile.close();
}

void savePRRs(Ptr<MetricSupervisor> metSup, uint64_t numberOfNodes)
{
  std::ofstream file;
  file.open("src/sionna/prr_with_sionna_nrv2x.csv", std::ios::out | std::ios::app);
  file << "node_id,prr" << std::endl;
  for (int i = 1; i <= numberOfNodes; i++)
    {
      double prr = metSup->getAveragePRR_vehicle (i);
      file << i << "," << prr << std::endl;
    }
  file.close();
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  int up = 0;
  bool realtime = false;
  bool verbose = false; // Set to true to get a lot of verbose output from the PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 30.0; // Transmission power in dBm (default: 23 dBm)
  double sensitivity = -93.0;
  double snr_threshold = 10; // Default value
  double sinr_threshold = 10; // Default value
  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 200 seconds)

  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  // uint16_t bandwidthBandSl = 400;
  uint16_t bandwidthBandSl = 100; // 10 MHz
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  // uint16_t numerologyBwpSl = 0;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  // uint16_t t2 = 21;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

  // (T2-T1+1) x (1/(2^numerology)) < reservation period
  // (81-2+1) x (1/2^2) < 20

  bool sionna = false;
  std::string server_ip = "";
  bool local_machine = false;
  bool verb = false;

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  // Read the command line options
  CommandLine cmd (__FILE__);
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("sionna", "Enable SIONNA usage", sionna);
  cmd.AddValue ("sionna-server-ip", "SIONNA server IP address", server_ip);
  cmd.AddValue ("sionna-local-machine", "SIONNA will be executed on local machine", local_machine);
  cmd.AddValue ("sionna-verbose", "SIONNA server IP address", verb);
  cmd.Parse (argc, argv);

  std::cout << "Start running v2v-simple-cam-exchange-80211p-nrv2x simulation" << std::endl;

  SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();

  if (sionna)
    {
      sionnaHelper.SetSionna(sionna);
      sionnaHelper.SetServerIp(server_ip);
      sionnaHelper.SetLocalMachine(local_machine);
      sionnaHelper.SetVerbose(verb);
    }

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);
  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  // Check if there are enough nodes
  // This application requires at least three vehicles (as vehicle 3 is the one generating interfering traffic, it should exist)
  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();

  if (sionna)
    {
      sumoClient->SetSionnaUp();
    }

  uint64_t numberOfNodes_nr = numberOfNodes;

  Ptr<MetricSupervisor> metSup_nr = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_nr(m_baseline_prr);
  metSup_nr = &metSupObj_nr;
  metSup_nr->setTraCIClient(sumoClient);
  // metSup_nr->enablePRRVerboseOnStdout ();

  // Ptr<MultiModelSpectrumChannel> spectrumChannel = nullptr;

  MobilityHelper mobility;

  Time slBearersActivationTime = Seconds (2.0);

  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  NodeContainer nrNodes;
  nrNodes.Create(numberOfNodes_nr);

  mobility.Install (nrNodes);

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});

  // IMPORTANT: Get the SpectrumChannel for NR-V2X, to be used as SpectrumChannel for WiFi 80211.p
  // spectrumChannel = DynamicCast<MultiModelSpectrumChannel>(bandSl.GetBwpAt (0, 0)->m_channel);

  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold1", DoubleValue (sinr_threshold));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold2", DoubleValue (sinr_threshold));

  // nrHelper->SetUeAntennaAttribute ();

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (nrNodes, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);

  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();

  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (nrNodes);
  rxSlUes.Add (nrNodes);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  InternetStackHelper internet;
  internet.Install (nrNodes);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < nrNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = nrNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (true));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID, TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    unsigned long vehID = std::stol(vehicleID.substr (3));
    unsigned long nodeID = vehID - 1;

    Ptr<NetDevice> netDevice;
    Ptr<Socket> sock;
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Node> includedNode = nrNodes.Get(nodeID);
    sock = Socket::CreateSocket (includedNode, tid);
    if (sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
      {
        NS_FATAL_ERROR ("Failed to bind client socket for NR-V2X");
      }
    Ipv4Address groupAddress4 ("225.0.0.0");
    sock->Connect (InetSocketAddress (groupAddress4, 19));

    netDevice = nrNodes.Get(nodeID)->GetDevice(0);
    Ptr<NrUeNetDevice> nrDevice = DynamicCast<NrUeNetDevice>(netDevice);
    nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold1 (sinr_threshold);
    nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold2 (sinr_threshold);
    nrDevice->GetPhy(0)->GetSpectrumPhy ()->GetSpectrumChannel()->SetAttribute ("MaxLossDb", DoubleValue(128.0));

    Ptr<BSContainer> bs_container = CreateObject<BSContainer>(vehID,StationType_passengerCar,sumoClient,false,sock);
    bs_container->addCAMRxCallback (std::bind(&receiveCAM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
    bs_container->linkMetricSupervisor (metSup_nr);
    bs_container->disablePRRSupervisorForGNBeacons();
    bs_container->setupContainer(true,false,false,false,false,false);
    basicServices.add(bs_container);
    std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
    double desync = ((double)std::rand()/RAND_MAX);
    bs_container->getCABasicService ()->startCamDissemination (desync);

    return nrNodes.Get(nodeID);
  };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
  {
    /* Set position outside communication range */
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
    mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));
    unsigned long intVehicleID = std::stol(vehicleID.substr (3));

    Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
    bsc->cleanup();
  };

  // Link ns-3 and SUMO
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));

  auto start_time = std::chrono::high_resolution_clock::now();

  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
  std::cout << "Run terminated..." << std::endl;

  std::ofstream outputFile("src/output.txt");
  if (!outputFile.is_open())
    {
      std::cerr << "Unable to open file";
    }

  outputFile << "\nTotal number of CAMs received: " << packet_count << std::endl;

  outputFile << "\nMetric Supervisor statistics for NR-V2X" << std::endl;
  outputFile << "Average PRR: " << metSup_nr->getAveragePRR_overall () << std::endl;
  outputFile << "Average latency (ms): " << metSup_nr->getAverageLatency_overall () << std::endl;
  outputFile << "RX packet count (from PRR Supervisor): " << metSup_nr->getNumberRx_overall () << std::endl;
  outputFile << "TX packet count (from PRR Supervisor): " << metSup_nr->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_nr->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  savePRRs (metSup_nr, numberOfNodes);

  Simulator::Destroy ();

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  outputFile << "\nSimulation time: " << elapsed.count() << " seconds" << std::endl;

  return 0;
}

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/cooperativePerception-helper.h"
#include "ns3/cooperativePerception.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>


#include <unistd.h>
#include "ns3/core-module.h"
#include "ns3/sionna-helper.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("v2v-nrv2x");

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
  {
    { "0", 0 },
    { "1", 1 },
  };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}


int
main (int argc, char *argv[])
{

  /*** 0.a App Options ***/
  bool verbose = true;
  bool realtime = false;
  std::string csv_name_cumulative;
  std::string csv_name;

  /*** 0.a Mobility Options ***/
  std::string opencda_folder = "Opencda/";
  std::string opencda_config ="ms_van3t_example";
  bool opencda_ml = false;
  //Same mobility example but with active perception (using ML models for OpenCDA's perception module)
  //std::string opencda_config ="ms_van3t_example_ml";
  //bool opencda_ml = true;

  /*
   * Path to CARLA_OpenCDA.conf file created at installation with the following contents:
   *
   * OpenCDA_HOME=/path/to/OpenCDA/
   * CARLA_HOME=/path/to/CARLA_0.9.12
   * Python_Interpreter=/path/to/anaconda3/envs/opencda/bin/python3
   *
   * If installation was done without anaconda -> Python_Interpreter=python3.7
  */
  std::string carla_opencda_config="CARLA-OpenCDA.conf";

  std::string OpenCDA_HOME, CARLA_HOME, Python_Interpreter;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  double m_baseline_prr = 150.0;
  bool m_metric_sup = false;


  // Simulation parameters.
  Time simTime = Seconds (20);
  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (0.0);


  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  uint16_t bandwidthBandSl = 400;
  double txPower = 23; //dBm
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

  bool sionna = false;
  std::string server_ip = "";
  bool local_machine = false;
  bool verb = false;

  /*
   * From here, we instruct the ns3::CommandLine class of all the input parameters
   * that we may accept as input, as well as their description, and the storage
   * variable.
   */
  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("opencda-config", "Location and name of OpenCDA configuration file", opencda_config);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);

  cmd.AddValue ("simTime",
                "Simulation time in seconds",
                simTime);
  cmd.AddValue ("slBearerActivationTime",
                "Sidelik bearer activation time in seconds",
                slBearersActivationTime);
  cmd.AddValue ("centralFrequencyBandSl",
                "The central frequency to be used for Sidelink band/channel",
                centralFrequencyBandSl);
  cmd.AddValue ("bandwidthBandSl",
                "The system bandwidth to be used for Sidelink",
                bandwidthBandSl);
  cmd.AddValue ("txPower",
                "total tx power in dBm",
                txPower);
  cmd.AddValue ("tddPattern",
                "The TDD pattern string",
                tddPattern);
  cmd.AddValue ("slBitMap",
                "The Sidelink bitmap string",
                slBitMap);
  cmd.AddValue ("numerologyBwpSl",
                "The numerology to be used in Sidelink bandwidth part",
                numerologyBwpSl);
  cmd.AddValue ("slSensingWindow",
                "The Sidelink sensing window length in ms",
                slSensingWindow);
  cmd.AddValue ("slSelectionWindow",
                "The parameter which decides the minimum Sidelink selection "
                "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                slSelectionWindow);
  cmd.AddValue ("slSubchannelSize",
                "The Sidelink subchannel size in RBs",
                slSubchannelSize);
  cmd.AddValue ("slMaxNumPerReserve",
                "The parameter which indicates the maximum number of reserved "
                "PSCCH/PSSCH resources that can be indicated by an SCI.",
                slMaxNumPerReserve);
  cmd.AddValue ("slProbResourceKeep",
                "The parameter which indicates the probability with which the "
                "UE keeps the current resource when the resource reselection"
                "counter reaches zero.",
                slProbResourceKeep);
  cmd.AddValue ("slMaxTxTransNumPssch",
                "The parameter which indicates the maximum transmission number "
                "(including new transmission and retransmission) for PSSCH.",
                slMaxTxTransNumPssch);
  cmd.AddValue ("ReservationPeriod",
                "The resource reservation period in ms",
                reservationPeriod);
  cmd.AddValue ("enableSensing",
                "If true, it enables the sensing based resource selection for "
                "SL, otherwise, no sensing is applied",
                enableSensing);
  cmd.AddValue ("t1",
                "The start of the selection window in physical slots, "
                "accounting for physical layer processing delay",
                t1);
  cmd.AddValue ("t2",
                "The end of the selection window in physical slots",
                t2);
  cmd.AddValue ("slThresPsschRsrp",
                "A threshold in dBm used for sensing based UE autonomous resource selection",
                slThresPsschRsrp);
  cmd.AddValue ("enableChannelRandomness",
                "Enable shadowing and channel updates",
                enableChannelRandomness);
  cmd.AddValue ("channelUpdatePeriod",
                "The channel update period in ms",
                channelUpdatePeriod);
  cmd.AddValue ("mcs",
                "The MCS to used for sidelink",
                mcs);

  cmd.AddValue ("sionna", "Enable SIONNA usage", sionna);
  cmd.AddValue ("sionna-server-ip", "SIONNA server IP address", server_ip);
  cmd.AddValue ("sionna-local-machine", "SIONNA will be executed on local machine", local_machine);
  cmd.AddValue ("sionna-verbose", "SIONNA server IP address", verb);

  // Parse the command line
  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("v2v-nrv2x", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();

  if (sionna)
    {
      sionnaHelper.SetSionna(sionna);
      sionnaHelper.SetServerIp(server_ip);
      sionnaHelper.SetLocalMachine(local_machine);
      sionnaHelper.SetVerbose(verb);
    }

  /*
   * Check if the frequency is in the allowed range.
   * If you need to add other checks, here is the best position to put them.
   */
  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /*** 0.b Read from the CARLA_OpenCDA config file (default CARLA_OpenCDA.conf) to get the path for necessary executables
  ***/
  std::ifstream configFile(carla_opencda_config);
  if (!configFile) {
      NS_FATAL_ERROR("Error: File '"<< carla_opencda_config << "' does not exist. Please specify valid configuration file.");
  }

  std::string line;
  std::unordered_map<std::string, std::string*> configMap = {
      {"OpenCDA_HOME", &OpenCDA_HOME},
      {"CARLA_HOME", &CARLA_HOME},
      {"Python_Interpreter", &Python_Interpreter}
  };

  while (std::getline(configFile, line)) {
      std::istringstream lineStream(line);
      std::string key, value;
      if (std::getline(lineStream, key, '=') && std::getline(lineStream, value)) {
          auto it = configMap.find(key);
          if (it != configMap.end()) {
              *(it->second) = value;
          }
      }
  }

  for (const auto& pair : configMap) {
          if (pair.second->empty()) {
              NS_FATAL_ERROR( "Error: Value for '" << pair.first << "' in '"<< carla_opencda_config <<"' is empty.");
          }
      }

  /***
   * 0.c Read from OpenCDA config file the number of vehicles
   * */
  std::string config_yaml = OpenCDA_HOME+"/opencda/scenario_testing/config_yaml/" +opencda_config+ ".yaml";

  std::ifstream file(config_yaml);
      int spawnPositionCount = 0;
      int vehicleNum = 0;
      bool foundVehicleNum = false;

      if (file.is_open()) {
          while (getline(file, line)) {
              // Count occurrences of "spawn_position"
              if (line.find("spawn_position") != std::string::npos) {
                  spawnPositionCount++;
              }

              // Find and process "vehicle_num"
              if (!foundVehicleNum && line.find("vehicle_num") != std::string::npos) {
                  std::istringstream iss(line);
                  std::string key, value;
                  if (std::getline(iss, key, ':') && std::getline(iss, value)) {
                      foundVehicleNum = true;
                      vehicleNum = std::stoi(value);
                  }
              }
          }
          file.close();
      } else {
          NS_FATAL_ERROR("Error: cannot open file '"<< config_yaml << "'.");
      }

  numberOfNodes = spawnPositionCount + vehicleNum;
  std::cout<< "Number of vehicles: " << numberOfNodes << std::endl;


  /*
   * Create a NodeContainer for all the UEs
   */
  NodeContainer allSlUesContainer;
  allSlUesContainer.Create(numberOfNodes);

  /*
   * Assign mobility to the UEs.
   */
  MobilityHelper mobility;
  mobility.Install (allSlUesContainer);

  /*
   * Setup the NR module. We create the various helpers needed for the
   * NR simulation:
   * - EpcHelper, which will setup the core network
   * - NrHelper, which takes care of creating and connecting the various
   * part of the NR stack
   */
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  /*
   * Spectrum division. We create one operational band, containing
   * one component carrier, and a single bandwidth part
   * centered at the frequency specified by the input parameters.
   * We will use the StreetCanyon channel modeling.
   */
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
   * creates a single BWP per CC
   */
  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  //CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::CV2X_UrbanMicrocell);

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  /*
   * The configured spectrum division is:
   * ------------Band1--------------
   * ------------CC1----------------
   * ------------BwpSl--------------
   */
  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  /*
   * Initialize channel and pathloss, plus other things inside bandSl. If needed,
   * the band configuration can be done manually, but we leave it for more
   * sophisticated examples. For the moment, this method will take care
   * of all the spectrum initialization needs.
   */
  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});


  /*
   * Now, we can setup the attributes. We can have three kind of attributes:
   */

  /*
   * Antennas for all the UEs
   * We are not using beamforming in SL, rather we are using
   * quasi-omnidirectional transmission and reception, which is the default
   * configuration of the beams.
   *
   * Following attribute would be common for all the UEs
   */
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  /*
   * We have configured the attributes we needed. Now, install and get the pointers
   * to the NetDevices, which contains all the NR stack:
   */
  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (allSlUesContainer, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  /*
   * Configure Sidelink. We create the following helpers needed for the
   * NR Sidelink, i.e., V2X simulation:
   * - NrSlHelper, which will configure the UEs protocol stack to be ready to
   *   perform Sidelink related procedures.
   * - EpcHelper, which takes care of triggering the call to EpcUeNas class
   *   to establish the NR Sidelink bearer(s). We note that, at this stage
   *   just communicate the pointer of already instantiated EpcHelper object,
   *   which is the same pointer communicated to the NrHelper above.
   */
  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  /*
   * Set the SL error model and AMC
   * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
   *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
   * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
   */
  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  /*
   * Set the SL scheduler attributes
   * In this example we use NrSlUeMacSchedulerSimple scheduler, which uses
   * fix MCS value
   */
  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  /*
   * Very important method to configure UE protocol stack, i.e., it would
   * configure all the SAPs among the layers, setup callbacks, configure
   * error model, configure AMC, and configure ChunkProcessor in Interference
   * API.
   */
  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);


  /*
   * Start preparing for all the sub Structs/RRC Information Element (IEs)
   * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
   * hold all the pre-configuration related to Sidelink.
   */

  //SlResourcePoolNr IE
  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();
  /*
   * Above pool factory is created to help the users of the simulator to create
   * a pool with valid default configuration. Please have a look at the
   * constructor of NrSlCommPreconfigResourcePoolFactory class.
   *
   * In the following, we show how one could change those default pool parameter
   * values as per the need.
   */
  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  //Configure the BWP IE
  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  /****************************** End SL Configuration ***********************/

  /*
   * Fix the random streams
   */
  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  /*
   * if enableOneTxPerLane is true:
   *
   * Divide the UEs in transmitting UEs and receiving UEs. Each lane can
   * have only odd number of UEs, and on each lane middle UE would
   * be the transmitter.
   *
   * else:
   *
   * All the UEs can transmit and receive
   */
  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (allSlUesContainer);
  rxSlUes.Add (allSlUesContainer);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  /*
   * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
   * configured time.
   *
   * This example supports IPV4 and IPV6
   */

  InternetStackHelper internet;
  internet.Install (allSlUesContainer);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < allSlUesContainer.GetN (); ++u)
    {
      Ptr<Node> ueNode = allSlUesContainer.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  /*** 6. Setup OpenCDA client ***/
  Ptr<OpenCDAClient> opencda_client = CreateObject<OpenCDAClient> ();
  if (sionna)
    {
      opencda_client->SetSionnaUp();
    }
  opencda_client->SetAttribute ("CARLAHost", StringValue ("localhost"));
  opencda_client->SetAttribute ("UpdateInterval", DoubleValue (0.05));
  opencda_client->SetAttribute ("OpenCDACIPort", UintegerValue (1337));
  opencda_client->SetAttribute ("PenetrationRate",DoubleValue(0.5));
  opencda_client->SetAttribute ("OpenCDA_config", StringValue(opencda_config));
  opencda_client->SetAttribute ("OpenCDA_HOME", StringValue(OpenCDA_HOME));
  opencda_client->SetAttribute ("CARLA_HOME", StringValue(CARLA_HOME));
  opencda_client->SetAttribute ("PythonInterpreter", StringValue(Python_Interpreter));
  /* If active perception is specified in OpenCDA's config YAML (eg. ms_van3t_example_ml). Default -> false */
  opencda_client->SetAttribute ("ApplyML", BooleanValue(opencda_ml));

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setOpenCDACLient (opencda_client);
    }


  /*** 7. Setup interface and application for dynamic nodes ***/
  /*** 7. Setup interface and application for dynamic nodes ***/
  cooperativePerceptionHelper cooperativePerceptionHelper;
  cooperativePerceptionHelper.SetAttribute ("OpenCDAClient", PointerValue(opencda_client));
  cooperativePerceptionHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  cooperativePerceptionHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  cooperativePerceptionHelper.SetAttribute ("CSV", StringValue(csv_name));
  cooperativePerceptionHelper.SetAttribute ("Model", StringValue ("nrv2x"));

  /* callback function for node creation */
  int i=0;
  STARTUP_OPENCDA_FCN setupNewWifiNode = [&] (std::string vehicleID) -> Ptr<Node>
    {
      if (nodeCounter >= allSlUesContainer.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = allSlUesContainer.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      cooperativePerceptionHelper.SetAttribute ("IpAddr", Ipv4AddressValue(groupAddress4));
      i++;

      ApplicationContainer AppSample = cooperativePerceptionHelper.Install (includedNode);

      AppSample.Start (Seconds (0.0));
      AppSample.Stop (simTime - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_OPENCDA_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      Ptr<cooperativePerception> appSample_ = exNode->GetApplication(0)->GetObject<cooperativePerception>();

      /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* start traci client with given function pointers */
  opencda_client->startCarlaAdapter (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (simTime);

  Simulator::Run ();
  Simulator::Destroy ();
  /* stop OpenCDA and CARLA simulation */
  opencda_client->stopSimulation ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;

      for(int i=1;i<numberOfNodes+1;i++) {
          std::cout << "Average latency of vehicle " << i << " (ms): " << metSup->getAverageLatency_vehicle (i) << std::endl;
          std::cout << "Average PRR of vehicle " << i << " (%): " << metSup->getAveragePRR_vehicle (i) << std::endl;
      }
    }

  return 0;
}


/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

// 802.11p
#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include <iostream>
#include "ns3/MetricSupervisor.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include <fstream>

// NR-V2X
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/vehicle-visualizer-module.h"
#include <unistd.h>
#include "ns3/core-module.h"

#include "ns3/txTracker.h"

#include "ns3/sionna-helper.h"

#include <chrono>

// C-V2X
/*#include "../../dsr/model/dsr-fs-header.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/antenna-module.h"
#include "ns3/cv2x_lte-v2x-helper.h"
#include "ns3/internet-module.h"
#include "ns3/cv2x-module.h"*/

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchange80211pNrv2x");

enum class NodeType
{
  DSRC,
  NR,
  INTERFERING
};

std::vector<uint8_t> dsrcVehicles;
std::vector<uint8_t > nrv2xVehicles;
static int packet_count = 0;
static int counter = 0;
BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles
bool phy_collection = true;
bool use_sionna = false;

void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
      {
          { "0", 0 },
          { "1", 1 },
      };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}

void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationId_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  if (!phy_collection)
    {
      return;
    }

  double snr = phy_info.snr;
  double sinr = phy_info.sinr;
  double rssi = phy_info.rssi;
  double rsrp = phy_info.rsrp;
  if (std::isnan(sinr) && !std::isnan(snr))
    {
      sinr = snr;
    }

  libsumo::TraCIPosition pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  libsumo::TraCIPosition pos_lat_lon = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);

  // Get the position of the sender
  double lat_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/1e7;
  double lon_sender = asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/1e7;

  libsumo::TraCIPosition pos_sender = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(cam->header.stationId));

  // Compute the distance between the sender and the receiver
  double distance = haversineDist (lat_sender, lon_sender, pos_lat_lon.y, pos_lat_lon.x);

  ulong time = Simulator::Now().GetMicroSeconds();
  uint8_t los;
  Vector a_position = {pos.x, pos.y, pos.z};
  Vector b_position = {pos_sender.x, pos_sender.y, 0};
  if (use_sionna)
  {
    std::string los_str = getLOSStatusFromSionna(a_position, b_position);
    if (los_str == "[False]")
      {
        los = 0;
      }
    else
      {
        los = 1;
      }
  }
  else
  {
    los = 0; // Default, with ns-3 models we should check if the model used provides LOS/NLOS state and its current value
  }
  
  std::ifstream camFileHeader("src/sinr_ni.csv");
  std::ofstream camFile;
  camFile.open("src/sinr_ni.csv", std::ios::out | std::ios::app);
  if (!camFileHeader.is_open())
    {
      if (camFile.is_open())
      {
        camFile << "time,rx,tx,rx_lat,rx_lon,tx_lat,tx_lon,technology,distance,los,sinr" << std::endl;
      } 
      else
      {
        std::cerr << "Unable to create file .csv" << std::endl;
      }
    }

  std::string technology;
  if (std::find(dsrcVehicles.begin(), dsrcVehicles.end(), my_stationID) != dsrcVehicles.end())
    {
      technology = "DSRC";
    }
  else
    {
      technology = "NR-V2X";
    }
  
  camFile << time << "," << my_stationID << "," << std::to_string(cam->header.stationId) << "," << pos_lat_lon.y << "," << pos_lat_lon.x << "," << lat_sender << "," << lon_sender << "," << technology << "," << distance << "," << std::to_string(los) << "," << sinr << std::endl;
  camFile.close();
}

void receiveCPM(asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from, StationId_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  if (!phy_collection)
    {
      return;
    }

  double snr = phy_info.snr;
  double sinr = phy_info.sinr;
  double rssi = phy_info.rssi;
  double rsrp = phy_info.rsrp;
  if (std::isnan(sinr) && !std::isnan(snr))
    {
      sinr = snr;
    }
  libsumo::TraCIPosition pos = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  libsumo::TraCIPosition pos_lat_lon = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);

  // Get the position of the sender
  double lat_sender = asn1cpp::getField(cpm->payload.managementContainer.referencePosition.latitude,double)/1e7;
  double lon_sender = asn1cpp::getField(cpm->payload.managementContainer.referencePosition.longitude,double)/1e7;

  libsumo::TraCIPosition pos_sender = basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(cpm->header.stationId));

  // Compute the distance between the sender and the receiver
  double distance = haversineDist (lat_sender, lon_sender, pos_lat_lon.y, pos_lat_lon.x);

  ulong time = Simulator::Now().GetMicroSeconds();
  uint8_t los;
  Vector a_position = {pos.x, pos.y, pos.z};
  Vector b_position = {pos_sender.x, pos_sender.y, pos_sender.z};
  if (use_sionna)
  {
    std::string los_str = getLOSStatusFromSionna(a_position, b_position);
    if (los_str == "[False]")
      {
        los = 0;
      }
    else
      {
        los = 1;
      }
  }
  else
  {
    los = 0; // Default, with ns-3 models we should check if the model used provides LOS/NLOS state and its current value
  }
  std::ifstream cpmFileHeader("src/sinr_ni.csv");
  std::ofstream cpmFile;
  cpmFile.open("src/sinr_ni.csv", std::ios::out | std::ios::app);
  if (!cpmFileHeader.is_open())
    {
      if (cpmFile.is_open())
        {
          cpmFile << "time,rx,tx,rx_lat,rx_lon,tx_lat,tx_lon,technology,distance,los,sinr" << std::endl;
        }
      else
        {
          std::cerr << "Unable to create file .csv" << std::endl;
        }
    }

  std::string technology;
  if (std::find(dsrcVehicles.begin(), dsrcVehicles.end(), my_stationID) != dsrcVehicles.end())
    {
      technology = "DSRC";
    }
  else
    {
      technology = "NR-V2X";
    }

  cpmFile << time << "," << my_stationID << "," << std::to_string(cpm->header.stationId) << "," << pos_lat_lon.y << "," << pos_lat_lon.x << "," << lat_sender << "," << lon_sender << "," << technology << "," << distance << "," << std::to_string(los) << "," << sinr << std::endl;
  cpmFile.close();
}

void savePRRs(Ptr<MetricSupervisor> metSup, std::vector<std::string> nodes, std::string type)
{
  std::ofstream file;
  if (type == "nr")
    {
      file.open("src/prr_latency_ns3_coexistence_nrv2x.csv", std::ios::out | std::ios::app);
    }
  else if (type == "11p")
    {
      file.open("src/prr_latency_ns3_coexistence_11p.csv", std::ios::out | std::ios::app);
    }
  file << "node_id,prr,latency(ms)" << std::endl;
  for (int i = 0; i < nodes.size(); i++)
    {
      double prr = metSup->getAveragePRR_vehicle (std::stol(nodes[i].substr (3)));
      double latency = metSup->getAverageLatency_vehicle (std::stol(nodes[i].substr (3)));
      file << i << "," << prr << "," << latency << std::endl;
    }
  file.close();
}

void txTrackerSetup(std::vector<std::string> wifiVehicles, NodeContainer wifiNodes, std::vector<std::string> nrVehicles, NetDeviceContainer nrDevices, bool dsrc_interference, bool nr_interference)
{
  auto& tracker = TxTracker::GetInstance();

  std::vector<std::tuple<std::string, uint8_t, Ptr<WifiNetDevice>>> wifiVehiclesList;
  std::vector<std::tuple<std::string, uint8_t, Ptr<NrUeNetDevice>>> nrVehiclesList;

  uint8_t i = 1 ? dsrc_interference : 0; // Start from 1 because the first node is the interfering one
  for (auto v : wifiVehicles)
    {
      uint8_t id = wifiNodes.Get(i)->GetId();
      Ptr<WifiNetDevice> netDevice = DynamicCast<WifiNetDevice>(wifiNodes.Get(i)->GetDevice(0));
      wifiVehiclesList.push_back (std::make_tuple (v, id, netDevice));
      i++;
    }
  tracker.Insert11pNodes (wifiVehiclesList);

  i = 1 ? nr_interference : 0;
  for (auto v : nrVehicles)
    {
      Ptr<NrUeNetDevice> netDevice = DynamicCast<NrUeNetDevice>(nrDevices.Get(i));
      uint8_t id = nrDevices.Get (i)->GetNode()->GetId();
      nrVehiclesList.push_back (std::make_tuple (v, id, netDevice));
      i++;
    }
  tracker.InsertNrNodes (nrVehiclesList);
}

static void GenerateTraffic_interfering (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval, uint8_t vehicles, Ptr<TraciClient> traci)
{
  uint8_t present = traci->GetVehicleMapSize();
  if (present != vehicles)
    {
      Simulator::Schedule(pktInterval, &GenerateTraffic_interfering, socket, pktSize, pktCount, pktInterval, vehicles, traci);
      return;
    }
  // Generate interfering traffic by sending pktCount packets (filled in with zeros), every pktInterval
  if (pktCount > 0)
    {
      // "Create<Packet> (pktSize)" creates a new packet of size pktSize bytes, composed by default by all zero
      // Populate the packet with random data
      std::string data = "";
      uint16_t packetSize = 1000;
      for (int i = 0; i < packetSize; i++)
        {
          data += std::to_string (counter);
          data += ",";
          long long now = Simulator::Now().GetMicroSeconds();
          data += std::to_string (now);
          data += ",";
        }
      Ptr<Packet> packet = Create<Packet>((uint8_t*) data.c_str(), packetSize);
      if (socket->Send (packet) != -1)
        {
          // std::cout << "Interfering packet sent" << std::endl;
          counter ++;
        }
      // Schedule again the same function (to send the next packet), and decrease by one the packet count
      Simulator::Schedule (pktInterval, &GenerateTraffic_interfering, socket, pktSize, pktCount, pktInterval, vehicles, traci);
    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{
  phy_collection = true;

  // std::string phyMode ("OfdmRate6MbpsBW10MHz");
  std::string phyMode ("OfdmRate3MbpsBW10MHz");
  double bandwidth_11p = 10;
  int up = 0;
  bool realtime = false;
  bool verbose = false; // Set to true to get a lot of verbose output from the PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 30.0; // Transmission power in dBm (default: 23 dBm)
  double sensitivity = -93.0;
  double snr_threshold = 10; // Default value
  double sinr_threshold = 10; // Default value
  xmlDocPtr rou_xml_file;
  double simTime = 50.0; // Total simulation time (default: 200 seconds)

  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  // uint16_t bandwidthBandSl = 400;
  uint16_t bandwidthBandSl = 100; // 10 MHz
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  // uint16_t numerologyBwpSl = 0;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  // uint16_t t2 = 21;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

  bool m_metric_sup = true;

  bool sionna = false;
  std::string server_ip = "";
  bool local_machine = false;
  bool verb = false;

  bool interference = false;
  bool dsrc_interference = false;
  bool nr_interference = false;

  Time slBearersActivationTime = Seconds (2.0);

  if ((dsrc_interference && nr_interference) /*|| (interference && !(dsrc_interference || nr_interference))*/)
    {
      NS_FATAL_ERROR ("Check the interference setup.");
    }

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  // Read the command line options
  CommandLine cmd (__FILE__);
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("sionna", "Enable SIONNA usage", sionna);
  cmd.AddValue ("sionna-server-ip", "SIONNA server IP address", server_ip);
  cmd.AddValue ("sionna-local-machine", "SIONNA will be executed on local machine", local_machine);
  cmd.AddValue ("sionna-verbose", "SIONNA server IP address", verb);
  cmd.Parse (argc, argv);

  std::cout << "Start running v2v-simple-cam-exchange-80211p-nrv2x simulation" << std::endl;
  
  use_sionna = sionna;
  SionnaHelper& sionnaHelper = SionnaHelper::GetInstance();
  if (sionna)
    {
      sionnaHelper.SetSionna(sionna);
      sionnaHelper.SetServerIp(server_ip);
      sionnaHelper.SetLocalMachine(local_machine);
      sionnaHelper.SetVerbose(verb);
    }

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);
  /*if (interference)
    {
      numberOfNodes --;
    }*/
  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  // Check if there are enough nodes
  // This application requires at least three vehicles (as vehicle 3 is the one generating interfering traffic, it should exist)
  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();

  if (sionna)
    {
      sumoClient->SetSionnaUp();
    }

  bool odd = false;
  if (numberOfNodes%2 != 0)
    {
      odd = true;
    }

  std::vector<std::string> wifiVehicles;
  std::vector<std::string> nrVehicles;

  uint8_t i = 1;
  if (dsrc_interference || nr_interference)
    {
      i = 2;
    }

  for (; i <= numberOfNodes; i++)
    {
      if (i%2 == 0)
        {
          wifiVehicles.push_back ("veh" + std::to_string (i));
          dsrcVehicles.push_back (i);
        }
      else
        {
          nrVehicles.push_back("veh" + std::to_string (i));
          nrv2xVehicles.push_back (i);
        }
    }

  uint64_t numberOfNodes_11p = wifiVehicles.size();
  uint64_t numberOfNodes_nr = nrVehicles.size();

  if (dsrc_interference)
    {
      numberOfNodes_11p ++;
    }
  else if (nr_interference)
    {
      numberOfNodes_nr ++;
    }

  Ptr<MetricSupervisor> metSup_11p = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_11p(m_baseline_prr);
  metSup_11p = &metSupObj_11p;
  metSup_11p->setTraCIClient(sumoClient);
  // This function enables printing the current and average latency and PRR for each received packet
  // metSup_11p->enablePRRVerboseOnStdout ();
  metSup_11p->setChannelTechnology("80211p");
  metSup_11p->setCBRWindowValue(200);
  metSup_11p->setCBRAlphaValue(0.1);
  metSup_11p->setSimulationTimeValue(simTime);

  Ptr<MetricSupervisor> metSup_nr = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj_nr(m_baseline_prr);
  metSup_nr = &metSupObj_nr;
  metSup_nr->setTraCIClient(sumoClient);
  // metSup_nr->enablePRRVerboseOnStdout ();

  MobilityHelper mobility;

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
    GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  NodeContainer nrNodes;
  nrNodes.Create(numberOfNodes_nr);

  mobility.Install (nrNodes);

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});


  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold1", DoubleValue (sinr_threshold));
  nrHelper->SetUePhyAttribute ("RiSinrThreshold2", DoubleValue (sinr_threshold));

  // nrHelper->SetUeAntennaAttribute ();

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (nrNodes, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);

  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();

  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (nrNodes);
  rxSlUes.Add (nrNodes);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  InternetStackHelper internet;
  internet.Install (nrNodes);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < nrNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = nrNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  // Create numberOfNodes nodes
  NodeContainer wifiNodes;
  wifiNodes.Create (numberOfNodes_11p);

  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  wifiPhy.SetPreambleDetectionModel ("ns3::ThresholdPreambleDetectionModel",
                                     "MinimumRssi", DoubleValue (sensitivity),
                                      "Threshold", DoubleValue (snr_threshold));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  channel->SetAttribute ("PropagationLossModel", StringValue ("ns3::CniUrbanmicrocellPropagationLossModel"));
  wifiPhy.SetChannel (channel);

  // ns-3 supports generating a pcap trace, to be later analyzed in Wireshark
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // We need a QosWaveMac, as we need to enable QoS and EDCA
  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging, only if verbose is true
    }

  // Supported "phyMode"s:
  // OfdmRate3MbpsBW10MHz, OfdmRate6MbpsBW10MHz, OfdmRate9MbpsBW10MHz, OfdmRate12MbpsBW10MHz, OfdmRate18MbpsBW10MHz, OfdmRate24MbpsBW10MHz, OfdmRate27MbpsBW10MHz
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, wifiNodes);

  // metSup_11p->setNodeContainer(wifiNodes);
  // metSup_11p->startCheckCBR();

  // Enable saving to Wireshark PCAP traces
  // wifiPhy.EnablePcap ("v2v-80211p-student-application", devices);

  // Set up the link between SUMO and ns-3, to make each node "mobile" (i.e., linking each ns-3 node to each moving vehicle in ns-3,
  // which corresponds to installing the network stack to each SUMO vehicle)
  mobility.Install (wifiNodes);

  PacketSocketHelper packetSocket;
  packetSocket.Install(wifiNodes);

  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (false));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  if (interference)
  {
    std::cout << "Interference mode enabled" << std::endl;
    auto& tracker = TxTracker::GetInstance();
    tracker.SetCentralFrequencies(centralFrequencyBandSl, centralFrequencyBandSl, centralFrequencyBandSl);
    tracker.SetBandwidths(bandwidth_11p * 1e6, bandwidthBandSl/10 * 1e6, 0.0);
    txTrackerSetup(wifiVehicles, wifiNodes, nrVehicles, allSlUesNetDeviceContainer, dsrc_interference, nr_interference);
  }

  uint8_t node11pCounter = 0;
  if (dsrc_interference)
    {
      node11pCounter = 1;
    }

  uint8_t nodeNrCounter = 0;
  if (nr_interference)
    {
      nodeNrCounter = 1;
    }

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID, TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
  {
    NodeType type;
    unsigned long vehID = std::stol(vehicleID.substr (3));
    unsigned long nodeID;

    std::cout << "Vehicle entering in the simulation: " << vehicleID << " at time " << Simulator::Now().GetMicroSeconds() << std::endl;

    if (std::find(wifiVehicles.begin(), wifiVehicles.end(), vehicleID) != wifiVehicles.end())
      {
        type = NodeType::DSRC;
        nodeID = node11pCounter;
        node11pCounter++;
      }
    else if (std::find(nrVehicles.begin(), nrVehicles.end(), vehicleID) != nrVehicles.end())
      {
        type = NodeType::NR;
        nodeID = nodeNrCounter;
        nodeNrCounter++;
      }
    else
      {
        type = NodeType::INTERFERING;
      }

    Ptr<NetDevice> netDevice;
    Ptr<Socket> sock;
    Ptr<WifiNetDevice> wifiDevice;
    Ptr<Node> includedNode;
    Ipv4Address groupAddress4 ("225.0.0.0");
    Ptr<NrUeNetDevice> nrDevice;
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    std::ofstream file;
    switch (type)
      {
      case NodeType::DSRC:
        sock = GeoNet::createGNPacketSocket (wifiNodes.Get (nodeID));
        netDevice = wifiNodes.Get (nodeID)->GetDevice (0);
        wifiDevice = DynamicCast<WifiNetDevice> (netDevice);
        wifiDevice->GetPhy ()->SetRxSensitivity (sensitivity);
        metSup_nr->addExcludedID (vehID);
        break;
      case NodeType::NR:
        includedNode = nrNodes.Get (nodeID);
        sock = Socket::CreateSocket (includedNode, tid);
        if (sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
          {
            NS_FATAL_ERROR ("Failed to bind client socket for NR-V2X");
          }
        sock->Connect (InetSocketAddress (groupAddress4, 19));
        netDevice = nrNodes.Get (nodeID)->GetDevice (0);
        nrDevice = DynamicCast<NrUeNetDevice> (netDevice);
        nrDevice->GetPhy(0)->GetSpectrumPhy ()->GetSpectrumChannel()->SetAttribute ("MaxLossDb", DoubleValue(120.0));
        // nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold1 (snr_threshold);
        // nrHelper->GetUePhy (netDevice, 0)->SetRiSinrThreshold2 (snr_threshold);
        metSup_11p->addExcludedID (vehID);
        break;
      case NodeType::INTERFERING:
        metSup_11p->addExcludedID (vehID);
        metSup_nr->addExcludedID (vehID);
        if (dsrc_interference)
          {
            Ptr<Socket> socket = Socket::CreateSocket (wifiNodes.Get (0), TypeId::LookupByName ("ns3::PacketSocketFactory"));
            PacketSocketAddress local_source_interfering;
            local_source_interfering.SetSingleDevice (wifiNodes.Get (0)->GetDevice(0)->GetIfIndex ());
            local_source_interfering.SetPhysicalAddress (wifiNodes.Get (0)->GetDevice(0)->GetAddress());
            local_source_interfering.SetProtocol (0x88B5);
            if (socket->Bind (local_source_interfering) == -1)
              {
                NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
              }
            PacketSocketAddress remote_source_interfering;
            remote_source_interfering.SetSingleDevice (wifiNodes.Get (0)->GetDevice(0)->GetIfIndex());
            remote_source_interfering.SetPhysicalAddress (wifiNodes.Get (0)->GetDevice(0)->GetBroadcast());
            remote_source_interfering.SetProtocol (0x88B5);
            socket->Connect (remote_source_interfering);
            socket->SetPriority (0);
            Simulator::ScheduleWithContext (0,
                                            Seconds (1.0), &GenerateTraffic_interfering,
                                            socket, 1000, simTime*2000, MilliSeconds (5),
                                            numberOfNodes, sumoClient);
          }
        else if (nr_interference)
          {
            // Da testare
            Ptr<Socket> socket = Socket::CreateSocket (nrNodes.Get (0), tid);
            if (socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
              {
                NS_FATAL_ERROR ("Failed to bind client socket for NR-V2X");
              }
            socket->Connect (InetSocketAddress (groupAddress4, 19));
            socket->SetPriority (0);
            Simulator::ScheduleWithContext (0,
                                            Seconds (3.0), &GenerateTraffic_interfering,
                                            socket, 1000, simTime*2000, MilliSeconds (5),
                                            numberOfNodes, sumoClient);
          }
        break;
      default:
        NS_FATAL_ERROR ("No technology recognized.");
      }

    if (type != NodeType::INTERFERING)
      {
        Ptr<BSContainer> bs_container = CreateObject<BSContainer>(vehID,StationType_passengerCar,sumoClient,false,sock);
        bs_container->addCAMRxCallback (std::bind(&receiveCAM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
        bs_container->addCPMRxCallback (std::bind(&receiveCPM, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
        switch (type)
          {
          case NodeType::DSRC:
            bs_container->linkMetricSupervisor (metSup_11p);
            break;
          case NodeType::NR:
            bs_container->linkMetricSupervisor (metSup_nr);
            break;
          }
        bs_container->disablePRRSupervisorForGNBeacons();
        bs_container->setupContainer(true,false,false,true,false,false);
        basicServices.add(bs_container);
        std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
        bs_container->getCABasicService ()->startCamDissemination ();
        bs_container->getCPBasicService ()->startCpmDissemination ();
      }

    switch (type)
      {
      case NodeType::DSRC:
        return wifiNodes.Get(nodeID);
      case NodeType::NR:
        return nrNodes.Get(nodeID);
      case NodeType::INTERFERING:
        return wifiNodes.Get(0);
      }
  };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
  {
    /* Set position outside communication range */
    Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
    mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));
    unsigned long intVehicleID = std::stol(vehicleID.substr (3));

    Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
    bsc->cleanup();
  };

  // Link ns-3 and SUMO
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));

  auto start_time = std::chrono::high_resolution_clock::now();

  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
  std::cout << "Run terminated..." << std::endl;

  std::cout << "\nTotal number of packets received: " << packet_count << std::endl;

  std::cout << "\nMetric Supervisor statistics for 802.11p" << std::endl;
  std::cout << "Average PRR: " << metSup_11p->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup_11p->getAverageLatency_overall () << std::endl;
  std::cout << "RX packet count (from PRR Supervisor): " << metSup_11p->getNumberRx_overall () << std::endl;
  std::cout << "TX packet count (from PRR Supervisor): " << metSup_11p->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_11p->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  std::cout << "\nMetric Supervisor statistics for NR-V2X" << std::endl;
  std::cout << "Average PRR: " << metSup_nr->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup_nr->getAverageLatency_overall () << std::endl;
  std::cout << "RX packet count (from PRR Supervisor): " << metSup_nr->getNumberRx_overall () << std::endl;
  std::cout << "TX packet count (from PRR Supervisor): " << metSup_nr->getNumberTx_overall () << std::endl;
  // std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup_nr->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  Simulator::Destroy ();

  savePRRs(metSup_11p, wifiVehicles, "11p");
  savePRRs(metSup_nr, nrVehicles, "nr");

  auto end_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end_time - start_time;
  std::cout << "\nSimulation time: " << elapsed.count() << " seconds" << std::endl;

  return 0;
}

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/automotive-module.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"


#include <unistd.h>
#include "ns3/core-module.h"


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("v2v-nrv2x");

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
  {
    { "0", 0 },
    { "1", 1 },
  };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}

static void GenerateTraffic_interfering (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  // Generate interfering traffic by sending pktCount packets (filled in with zeros), every pktInterval
  if (pktCount > 0)
    {
      // "Create<Packet> (pktSize)" creates a new packet of size pktSize bytes, composed by default by all zeros
      if (socket->Send (Create<Packet> (pktSize)) != -1 && pktCount%100==0)
        {
          // std::cout << "Interfering packet sent" << std::endl;
        }
      // Schedule again the same function (to send the next packet), and decrease by one the packet count
      Simulator::Schedule (pktInterval, &GenerateTraffic_interfering,
                           socket, pktSize, pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}


int
main (int argc, char *argv[])
{

  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map_congestion/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map_congestion/map.sumo.cfg";

  /*** 0.a App Options ***/
  bool verbose = true;
  bool realtime = false;
  bool sumo_gui = true;
  double sumo_updates = 0.01;
  std::string csv_name;
  std::string csv_name_cumulative;
  std::string sumo_netstate_file_name;
  bool vehicle_vis = false;

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  double penetrationRate = 0.7;

  xmlDocPtr rou_xml_file;
  double m_baseline_prr = 150.0;
  bool m_metric_sup = true;


  // Simulation parameters.
  double simTime = 50.0;
  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (2.0);


  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  uint16_t bandwidthBandSl = 400;
  double txPower = 23; //dBm
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = 10;
  uint16_t slMaxNumPerReserve = 3;
  double slProbResourceKeep = 0.0;
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = 20; // in ms
  bool enableSensing = false;
  uint16_t t1 = 2;
  uint16_t t2 = 81;
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = 14;

  /*
   * From here, we instruct the ns3::CommandLine class of all the input parameters
   * that we may accept as input, as well as their description, and the storage
   * variable.
   */
  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("penetrationRate", "Rate of vehicles equipped with wireless communication devices", penetrationRate);

  cmd.AddValue ("simTime",
                "Simulation time in seconds",
                simTime);
  cmd.AddValue ("slBearerActivationTime",
                "Sidelik bearer activation time in seconds",
                slBearersActivationTime);
  cmd.AddValue ("centralFrequencyBandSl",
                "The central frequency to be used for Sidelink band/channel",
                centralFrequencyBandSl);
  cmd.AddValue ("bandwidthBandSl",
                "The system bandwidth to be used for Sidelink",
                bandwidthBandSl);
  cmd.AddValue ("txPower",
                "total tx power in dBm",
                txPower);
  cmd.AddValue ("tddPattern",
                "The TDD pattern string",
                tddPattern);
  cmd.AddValue ("slBitMap",
                "The Sidelink bitmap string",
                slBitMap);
  cmd.AddValue ("numerologyBwpSl",
                "The numerology to be used in Sidelink bandwidth part",
                numerologyBwpSl);
  cmd.AddValue ("slSensingWindow",
                "The Sidelink sensing window length in ms",
                slSensingWindow);
  cmd.AddValue ("slSelectionWindow",
                "The parameter which decides the minimum Sidelink selection "
                "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                slSelectionWindow);
  cmd.AddValue ("slSubchannelSize",
                "The Sidelink subchannel size in RBs",
                slSubchannelSize);
  cmd.AddValue ("slMaxNumPerReserve",
                "The parameter which indicates the maximum number of reserved "
                "PSCCH/PSSCH resources that can be indicated by an SCI.",
                slMaxNumPerReserve);
  cmd.AddValue ("slProbResourceKeep",
                "The parameter which indicates the probability with which the "
                "UE keeps the current resource when the resource reselection"
                "counter reaches zero.",
                slProbResourceKeep);
  cmd.AddValue ("slMaxTxTransNumPssch",
                "The parameter which indicates the maximum transmission number "
                "(including new transmission and retransmission) for PSSCH.",
                slMaxTxTransNumPssch);
  cmd.AddValue ("ReservationPeriod",
                "The resource reservation period in ms",
                reservationPeriod);
  cmd.AddValue ("enableSensing",
                "If true, it enables the sensing based resource selection for "
                "SL, otherwise, no sensing is applied",
                enableSensing);
  cmd.AddValue ("t1",
                "The start of the selection window in physical slots, "
                "accounting for physical layer processing delay",
                t1);
  cmd.AddValue ("t2",
                "The end of the selection window in physical slots",
                t2);
  cmd.AddValue ("slThresPsschRsrp",
                "A threshold in dBm used for sensing based UE autonomous resource selection",
                slThresPsschRsrp);
  cmd.AddValue ("enableChannelRandomness",
                "Enable shadowing and channel updates",
                enableChannelRandomness);
  cmd.AddValue ("channelUpdatePeriod",
                "The channel update period in ms",
                channelUpdatePeriod);
  cmd.AddValue ("mcs",
                "The MCS to used for sidelink",
                mcs);


  // Parse the command line
  cmd.Parse (argc, argv);

  if (verbose)
    {
      LogComponentEnable ("v2v-nrv2x", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
    }

  /*
   * Check if the frequency is in the allowed range.
   * If you need to add other checks, here is the best position to put them.
   */
  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /***  Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");
  /*
   * Create a NodeContainer for all the UEs
   */
  NodeContainer allSlUesContainer;
  allSlUesContainer.Create(numberOfNodes);

  /*
   * Assign mobility to the UEs.
   */
  MobilityHelper mobility;
  mobility.Install (allSlUesContainer);

  /*
   * Setup the NR module. We create the various helpers needed for the
   * NR simulation:
   * - EpcHelper, which will setup the core network
   * - NrHelper, which takes care of creating and connecting the various
   * part of the NR stack
   */
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  /*
   * Spectrum division. We create one operational band, containing
   * one component carrier, and a single bandwidth part
   * centered at the frequency specified by the input parameters.
   * We will use the StreetCanyon channel modeling.
   */
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
   * creates a single BWP per CC
   */
  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  //CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::CV2X_UrbanMicrocell);

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  /*
   * The configured spectrum division is:
   * ------------Band1--------------
   * ------------CC1----------------
   * ------------BwpSl--------------
   */
  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  /*
   * Initialize channel and pathloss, plus other things inside bandSl. If needed,
   * the band configuration can be done manually, but we leave it for more
   * sophisticated examples. For the moment, this method will take care
   * of all the spectrum initialization needs.
   */
  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});


  /*
   * Now, we can setup the attributes. We can have three kind of attributes:
   */

  /*
   * Antennas for all the UEs
   * We are not using beamforming in SL, rather we are using
   * quasi-omnidirectional transmission and reception, which is the default
   * configuration of the beams.
   *
   * Following attribute would be common for all the UEs
   */
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  /*
   * We have configured the attributes we needed. Now, install and get the pointers
   * to the NetDevices, which contains all the NR stack:
   */
  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (allSlUesContainer, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  /*
   * Configure Sidelink. We create the following helpers needed for the
   * NR Sidelink, i.e., V2X simulation:
   * - NrSlHelper, which will configure the UEs protocol stack to be ready to
   *   perform Sidelink related procedures.
   * - EpcHelper, which takes care of triggering the call to EpcUeNas class
   *   to establish the NR Sidelink bearer(s). We note that, at this stage
   *   just communicate the pointer of already instantiated EpcHelper object,
   *   which is the same pointer communicated to the NrHelper above.
   */
  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  /*
   * Set the SL error model and AMC
   * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
   *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
   * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
   */
  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  /*
   * Set the SL scheduler attributes
   * In this example we use NrSlUeMacSchedulerSimple scheduler, which uses
   * fix MCS value
   */
  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  /*
   * Very important method to configure UE protocol stack, i.e., it would
   * configure all the SAPs among the layers, setup callbacks, configure
   * error model, configure AMC, and configure ChunkProcessor in Interference
   * API.
   */
  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);


  /*
   * Start preparing for all the sub Structs/RRC Information Element (IEs)
   * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
   * hold all the pre-configuration related to Sidelink.
   */

  //SlResourcePoolNr IE
  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();
  /*
   * Above pool factory is created to help the users of the simulator to create
   * a pool with valid default configuration. Please have a look at the
   * constructor of NrSlCommPreconfigResourcePoolFactory class.
   *
   * In the following, we show how one could change those default pool parameter
   * values as per the need.
   */
  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  //Configure the BWP IE
  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  /****************************** End SL Configuration ***********************/

  /*
   * Fix the random streams
   */
  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  /*
   * if enableOneTxPerLane is true:
   *
   * Divide the UEs in transmitting UEs and receiving UEs. Each lane can
   * have only odd number of UEs, and on each lane middle UE would
   * be the transmitter.
   *
   * else:
   *
   * All the UEs can transmit and receive
   */
  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (allSlUesContainer);
  rxSlUes.Add (allSlUesContainer);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  /*
   * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
   * configured time.
   *
   * This example supports IPV4 and IPV6
   */

  InternetStackHelper internet;
  internet.Install (allSlUesContainer);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < allSlUesContainer.GetN (); ++u)
    {
      Ptr<Node> ueNode = allSlUesContainer.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  // enable log component
  //LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (penetrationRate));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor metSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &metSupObj;
      metSup->setTraCIClient(sumoClient);
      metSup->disablePRRVerboseOnStdout();
      metSup->setChannelTechnology("Nr");
      metSup->enableCBRVerboseOnStdout();
      metSup->enableCBRWriteToFile();
      metSup->setCBRWindowValue(100);
      metSup->setCBRAlphaValue(0.1);
      metSup->setSimulationTimeValue(simTime);
      metSup->startCheckCBR();
    }

  /*** 7. Setup interface and application for dynamic nodes ***/
  emergencyVehicleAlertHelper EmergencyVehicleAlertHelper;
  EmergencyVehicleAlertHelper.SetAttribute ("Client", PointerValue (sumoClient));
  EmergencyVehicleAlertHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  EmergencyVehicleAlertHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  EmergencyVehicleAlertHelper.SetAttribute ("CSV", StringValue(csv_name));
  EmergencyVehicleAlertHelper.SetAttribute ("Model", StringValue ("nrv2x"));
  EmergencyVehicleAlertHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));
  EmergencyVehicleAlertHelper.SetAttribute ("SendCPM", BooleanValue(false));

  /* callback function for node creation */
  int i=0;
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= allSlUesContainer.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = allSlUesContainer.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      EmergencyVehicleAlertHelper.SetAttribute ("IpAddr", Ipv4AddressValue(groupAddress4));
      i++;

      //ApplicationContainer CAMSenderApp = CamSenderHelper.Install (includedNode);
      ApplicationContainer AppSample = EmergencyVehicleAlertHelper.Install (includedNode);

      // TODO
      //dcc->AddCABasicService ();

      AppSample.Start (Seconds (0.0));
      AppSample.Stop (Seconds(simTime) - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* stop all applications */
      Ptr<emergencyVehicleAlert> appSample_ = exNode->GetApplication(0)->GetObject<emergencyVehicleAlert>();

      if(appSample_)
        appSample_->StopApplicationNow();

       /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /*** 8. Start Simulation ***/
  Simulator::Stop (Seconds(simTime));

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
    }


  return 0;
}


/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#define HORIZON_TIME 8000
#define NEGOTIATION_TIME 1000
#define DECELERATION_TIME 1000
#define STEP_TIME 100
#define LC_TIME_MSEC 1500


#include "ns3/carla-module.h"

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include <iostream>
#include "ns3/MetricSupervisor.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/gn-utils.h"
#include "ns3/csv-utils.h"
#include "ns3/foresee.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleMCMExchange80211p");

// ******* DEFINE HERE ANY LOCAL GLOBAL VARIABLE, ACCESSIBLE FROM ANY FUNCTION IN THIS FILE *******
// Variables defined here should always be "static"
static int packet_count=0;
BSMap basicServices;
void receiveMCM(asn1cpp::Seq<MCM> mcm, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{

}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz"); // Default IEEE 802.11p data rate
  int up=0;
  int interfering_up=0;
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 33.0; // IEEE 802.11p transmission power in dBm (default: 23 dBm)
  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 100 seconds)
  bool sumo_gui = true;

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_foresee/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_foresee/map.sumo.cfg";

  // Read the command line options
  CommandLine cmd (__FILE__);

  // Syntax to add new options: cmd.addValue (<option>,<brief description>,<destination variable>)
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("interfering-userpriority","User Priority for interfering traffic (default: 0, i.e., AC_BE)",interfering_up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue ("sumo-gui", "Activate SUMO GUI", sumo_gui);
  cmd.Parse (argc, argv);

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);
  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  // Check if there are enough nodes
  // This application requires at least three vehicles (as vehicle 3 is the one generating interfering traffic, it should exist)
  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  // Create numberOfNodes nodes
  NodeContainer c;
  c.Create (numberOfNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  // Set up the IEEE 802.11p model and PHY layer
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generating a pcap trace, to be later analyzed in Wireshark
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // We need a QosWaveMac, as we need to enable QoS and EDCA
  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging, only if verbose is true
    }

  // In order to properly set the IEEE 802.11p modulation for broadcast messages, you must always specify a "NonUnicastMode" too
  // This line sets the modulation and rata rate
  // Supported "phyMode"s:
  // OfdmRate3MbpsBW10MHz, OfdmRate6MbpsBW10MHz, OfdmRate9MbpsBW10MHz, OfdmRate12MbpsBW10MHz, OfdmRate18MbpsBW10MHz, OfdmRate24MbpsBW10MHz, OfdmRate27MbpsBW10MHz
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Enable saving to Wireshark PCAP traces
  wifiPhy.EnablePcap ("v2v-80211p-foresee-mcm", devices.Get (0));

  // Set up the link between SUMO and ns-3, to make each node "mobile" (i.e., linking each ns-3 node to each moving vehicle in ns-3,
  // which corresponds to installing the network stack to each SUMO vehicle)
  MobilityHelper mobility;
  mobility.Install (c);
  // Set up the TraCI interface and start SUMO with the default parameters
  // The simulation time step can be tuned by changing "SynchInterval"
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (10)));

  // Set up a Metricsupervisor
  // This module enables a trasparent and seamless collection of one-way latency (in ms) and PRR metrics
  Ptr<MetricSupervisor> metSup = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj(m_baseline_prr);
  metSup = &metSupObj;
  metSup->setTraCIClient(sumoClient);
  PacketSocketHelper packetSocket;
  packetSocket.Install(c);

  std::unordered_map<ulong, foresee> lc_model;
  // Create the Lane Change data structure shared with the environment
  // No need for a mutex since we are not in multi threading programming
  std::unordered_map<ulong, std::tuple<float, float, float>> lc_data_structure;
  // Set the coordination avoidance range to check ahead of ego vehicle (in meters)
  float ca_range = 200;

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  double avg_speed_cars = 33.3;  // m/s
  double avg_speed_trucks = 22.2;  // m/s
  double deviation = 0.2;   // 20%

  double min_speed_cars = avg_speed_cars * (1.0 - deviation);
  double min_speed_trucks = avg_speed_trucks * (1.0 - deviation);
  double max_speed_cars = avg_speed_cars * (1.0 + deviation);
  double max_speed_trucks = avg_speed_trucks * (1.0 + deviation);

  // Random number generator
  const unsigned int SEED = 42;
  std::mt19937 gen(SEED);
  std::uniform_real_distribution<double> dist1(min_speed_cars, max_speed_cars);
  std::uniform_real_distribution<double> dist2(min_speed_trucks, max_speed_trucks);

  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3))-1;

      std::string type = sumoClient->vehicle.getTypeID (vehicleID);

      double speed = type == "Car0" ? dist1(gen) : dist2(gen);

      // Set the desired speed
      sumoClient->vehicle.setMaxSpeed(vehicleID, speed);
      // Prevent uncontrolled lane change
      sumoClient->vehicle.setParameter(vehicleID, "laneChangeMode", "0");

      // Create a new ETSI GeoNetworking socket, thanks to the GeoNet::createGNPacketSocket() function, accepting as argument a pointer to the current node
      Ptr<Socket> sock;
      sock=GeoNet::createGNPacketSocket(c.Get(nodeID));
      // Set the proper AC, through the specified UP
      sock->SetPriority (up);
      StationType_t st_type = type == "Car0" ? StationType_passengerCar : StationType_lightTruck;
      Ptr<BSContainer> bs_container = CreateObject<BSContainer>(std::stol(vehicleID.substr(3)),st_type,sumoClient,false,sock);
      // Setup the PRRsupervisor inside the BSContainer, to make each vehicle collect latency and PRR metrics
      bs_container->linkMetricSupervisor(metSup);
      // This is needed just to simplify the whole application
      bs_container->disablePRRSupervisorForGNBeacons ();

      // Set the function which will be called every time a CAM is received, i.e., receiveCAM()
      // bs_container->addMCMRxCallback (std::bind(&receiveMCM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
      bs_container->setupContainer(true,false,false,false,true,false);

      // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
      // retrieve the right BSContainer given a vehicle ID
      basicServices.add(bs_container);

      lc_model[nodeID].setDesiredSpeed (speed);
      lc_model[nodeID].setNode(c.Get(nodeID));
      lc_model[nodeID].setStationType(st_type);
      lc_model[nodeID].setLDM (bs_container->getLDM());
      lc_model[nodeID].setVDP (bs_container->getVDP());
      lc_model[nodeID].setVehicleID (vehicleID);
      lc_model[nodeID].setTraciAPI(sumoClient);
      lc_model[nodeID].setNumberOfLanes();
      lc_model[nodeID].setCurrentLCData(&lc_data_structure);
      lc_model[nodeID].setCoordinationAvoidanceRange(ca_range);
      lc_model[nodeID].setMCBasicService(bs_container->getMCBasicService());
      lc_model[nodeID].addMCMRxCallback ();
      lc_model[nodeID].setStartTime(10);
      std::string my_type = sumoClient->vehicle.getTypeID (vehicleID);
      lc_model[nodeID].setTrajectoryPredictor(HORIZON_TIME, STEP_TIME, NEGOTIATION_TIME, DECELERATION_TIME, LC_TIME_MSEC, foresee::PredictionType::CONSTANT_SPEED);

      lc_model[nodeID].WrapperFORESEEMobilityModel();

      // Start transmitting CAMs
      // We randomize the instant in time in which the CAM dissemination is going to start
      // This simulates different startup times for the OBUs of the different vehicles, and
      // reduces the risk of multiple vehicles trying to send CAMs are the same time (causing more collisions);
      // "desync" is a value between 0 and 1 (seconds) after which the CAM dissemination should start
      std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
      double desync = ((double)std::rand()/RAND_MAX);
      bs_container->getCABasicService ()->startCamDissemination (desync);
      // bs_container->getMCBasicService()->startMCMDissemination(desync);

      return c.Get(nodeID);
    };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
    {
      /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));

      // Turn off the Basic Services and the ETSI ITS-G5 stack for the vehicle
      // which has exited from the simulated scenario, and should be thus no longer considered
      // We need to get the right Ptr<BSContainer> based on the station ID (not the nodeID used
      // as index for the nodeContainer), so we don't use "-1" to compute "intVehicleID" here
      unsigned long intVehicleID = std::stol(vehicleID.substr (3));

      Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
      bsc->cleanup();
    };

  // Link ns-3 and SUMO
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));
  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
  std::cout << "Run terminated..." << std::endl;

  std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;

  Simulator::Destroy ();

  return 0;
}

/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006,2007 INRIA
 * Copyright (c) 2013 Dalian University of Technology
 * Copyright (c) 2022 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr> (initial IEEE 802.11p example)
 * Author: Junling Bu <linlinjavaer@gmail.com> (initial IEEE 802.11p example)
 * Author: Francesco Raviglione <francescorav.es483@gmail.com> (IEEE 802.11p simple CAM exchange application)
 *
 * This is a simple example of a ms-van3t V2V communication scenario configured with a single .cc file,
 * where vehicles exchange Cooperative Awareness Messages (CAMs) using the IEEE 802.11p standard.
 * The user can specify several parameters, including the priority (Access Category) for the transmission
 * of CAMs. BSContainers are used to simplify the configuration of the ETSI C-ITS stack of each vehicle.
 * In this scenario, all vehicles transmit CAMs according to the ETSI standards, and one vehicle (vehicle 3)
 * sends a heavy interfering traffic, without useful informative content, to simulate a congested channel.
 * Through the --interfering-userpriority option, the user can specify the Access Category (AC) for the
 * interfering traffic, which is broadcasted by vehicle 3.
 * When a new CAM is received by one of the vehicles, the callback "receiveCAM()" is called, and the stationID of
 * the received is available in "my_stationID".
 * Currently, the function just counts the total number of CAMs, but it can be customized to impement more complex
 * approaches.
 * As output, the simulation provides, thanks to the PRRSupervisor module, the average latency and PRR over the
 * whole simulation, and the average one-way latency for each vehicle up to vehicle 4.
 * The reported latency of vehicle 3 is expected to be 0 as it transmits interfering traffic (so, no ETSI-compliant
 * message is transmitted), that is not considered by the PRRSupervisor.
 */
#include "ns3/carla-module.h"

#include "ns3/vector.h"
#include "ns3/string.h"
#include "ns3/socket.h"
#include "ns3/double.h"
#include "ns3/config.h"
#include "ns3/log.h"
#include "ns3/command-line.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/mobility-helper.h"
#include <iostream>
#include "ns3/MetricSupervisor.h"
#include "ns3/sumo_xml_parser.h"
#include "ns3/BSMap.h"
#include "ns3/caBasicService.h"
#include "ns3/btp.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/gn-utils.h"
#include "ns3/csv-utils.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("V2VSimpleCAMExchange80211p");

// ******* DEFINE HERE ANY LOCAL GLOBAL VARIABLE, ACCESSIBLE FROM ANY FUNCTION IN THIS FILE *******
// Variables defined here should always be "static"
static int packet_count=0;
BSMap basicServices; // Container for all ETSI Basic Services, installed on all vehicles
// ************************************************************************************************

// If you want to make a comparison with a received CAM MAC address, you can use:
// from == getGNAddress (0,Mac48Address(comparison_string)), where "comparison_string" is something like "00:00:00:00:00:09"

// Useful tip: you can get the latitude and longitude position of a given vehicle at any time with:
//   libsumo::TraCIPosition pos=<MobilityClient>->TraCIAPI::vehicle.getPosition(<string ID of the vehicle>);
//   pos=<MobilityClient>->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);
// <MobilityClient> should be the right Ptr<TraciClient> for the current vehicle, which can be retrieved from the
//   global Basic Services container with basicServices.get(my_stationID)->getTraCIclient ()
// The string ID of the vehicle should match the one in the XML file, i.e., for vehicle 7, the string id should be "veh7"
// After these two lines, pos.y will contain the latitude of the vehicle, while pos.x the longitude of the vehicle
void receiveCAM(asn1cpp::Seq<CAM> cam, Address from, StationID_t my_stationID, StationType_t my_StationType, SignalInfo phy_info)
{
  packet_count++;
  // Logging the distance with respect to the RSSI
  double lat_sender=asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.latitude,double)/1e7;
  double lon_sender=asn1cpp::getField(cam->cam.camParameters.basicContainer.referencePosition.longitude,double)/1e7;
  //
  libsumo::TraCIPosition pos=basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::vehicle.getPosition("veh" + std::to_string(my_stationID));
  pos=basicServices.get(my_stationID)->getTraCIclient ()->TraCIAPI::simulation.convertXYtoLonLat(pos.x,pos.y);
  //
  double distance=haversineDist (lat_sender, lon_sender, pos.y, pos.x);
  //
  writeDataToCSV("distance_rssi_int_extra.csv","distance_m,rssi",distance,phy_info.rssi);
}

static void GenerateTraffic_interfering (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval )
{
  // Generate interfering traffic by sending pktCount packets (filled in with zeros), every pktInterval
  if (pktCount > 0)
    {
      // "Create<Packet> (pktSize)" creates a new packet of size pktSize bytes, composed by default by all zeros
      socket->Send (Create<Packet> (pktSize));
      // Schedule again the same function (to send the next packet), and decrease by one the packet count
      Simulator::Schedule (pktInterval, &GenerateTraffic_interfering,
                           socket, pktSize,pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

int main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate3MbpsBW10MHz"); // Default IEEE 802.11p data rate
  int up=0;
  int interfering_up=0;
  bool verbose = false; // Set to true to get a lot of verbose output from the IEEE 802.11p PHY model (leave this to false)
  int numberOfNodes; // Total number of vehicles, automatically filled in by reading the XML file
  double m_baseline_prr = 150.0; // PRR baseline value (default: 150 m)
  int txPower = 23.0; // IEEE 802.11p transmission power in dBm (default: 23 dBm)
  xmlDocPtr rou_xml_file;
  double simTime = 100.0; // Total simulation time (default: 100 seconds)
  bool security_enabled = false; // V2X security is disabled by default

  // Set here the path to the SUMO XML files
  std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  std::string mob_trace = "cars.rou.xml";
  std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";

  // Read the command line options
  CommandLine cmd (__FILE__);

  // Syntax to add new options: cmd.addValue (<option>,<brief description>,<destination variable>)
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
  cmd.AddValue ("userpriority","EDCA User Priority for the ETSI messages",up);
  cmd.AddValue ("interfering-userpriority","User Priority for interfering traffic (default: 0, i.e., AC_BE)",interfering_up);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("tx-power", "OBUs transmission power [dBm]", txPower);
  cmd.AddValue ("sim-time", "Total duration of the simulation [s]", simTime);
  cmd.AddValue("enable-security","Enable the transmission of secured V2X packets",security_enabled);
  cmd.Parse (argc, argv);

  /* Load the .rou.xml file (SUMO map and scenario) */
  xmlInitParser();
  std::string path = sumo_folder + mob_trace;
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);
  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  // Check if there are enough nodes
  // This application requires at least three vehicles (as vehicle 3 is the one generating interfering traffic, it should exist)
  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }

  if(numberOfNodes<3)
    {
      NS_FATAL_ERROR("Fatal error: at least three vehicles are required.");
    }

  // Create numberOfNodes nodes
  NodeContainer c;
  c.Create (numberOfNodes);

  // The below set of helpers will help us to put together the wifi NICs we want
  // Set up the IEEE 802.11p model and PHY layer
  YansWifiPhyHelper wifiPhy;
  wifiPhy.Set ("TxPowerStart", DoubleValue (txPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (txPower));
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  Ptr<YansWifiChannel> channel = wifiChannel.Create ();
  wifiPhy.SetChannel (channel);
  // ns-3 supports generating a pcap trace, to be later analyzed in Wireshark
  wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);

  // We need a QosWaveMac, as we need to enable QoS and EDCA
  QosWaveMacHelper wifi80211pMac = QosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
  if (verbose)
    {
      wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging, only if verbose is true
    }

  // In order to properly set the IEEE 802.11p modulation for broadcast messages, you must always specify a "NonUnicastMode" too
  // This line sets the modulation and rata rate
  // Supported "phyMode"s:
  // OfdmRate3MbpsBW10MHz, OfdmRate6MbpsBW10MHz, OfdmRate9MbpsBW10MHz, OfdmRate12MbpsBW10MHz, OfdmRate18MbpsBW10MHz, OfdmRate24MbpsBW10MHz, OfdmRate27MbpsBW10MHz
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                      "DataMode",StringValue (phyMode),
                                      "ControlMode",StringValue (phyMode),
                                      "NonUnicastMode",StringValue (phyMode));
  NetDeviceContainer devices = wifi80211p.Install (wifiPhy, wifi80211pMac, c);

  // Enable saving to Wireshark PCAP traces
  wifiPhy.EnablePcap ("v2v-80211p-student-application", devices);

  // Set up the link between SUMO and ns-3, to make each node "mobile" (i.e., linking each ns-3 node to each moving vehicle in ns-3,
  // which corresponds to installing the network stack to each SUMO vehicle)
  MobilityHelper mobility;
  mobility.Install (c);
  // Set up the TraCI interface and start SUMO with the default parameters
  // The simulation time step can be tuned by changing "SynchInterval"
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (0.01)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (true));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (1.0));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (10));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  // Set up a Metricsupervisor
  // This module enables a trasparent and seamless collection of one-way latency (in ms) and PRR metrics
  Ptr<MetricSupervisor> metSup = NULL;
  // Set a baseline for the PRR computation when creating a new Metricsupervisor object
  MetricSupervisor metSupObj(m_baseline_prr);
  metSup = &metSupObj;
  metSup->setTraCIClient(sumoClient);
  // Vehicle 3 should *not* be considered in the computation of latency and PRR, as it generates only interfering traffic
  metSup->addExcludedID(3);
  // This function enables printing the current and average latency and PRR for each received packet
  // metSup->enablePRRVerboseOnStdout ();

  // Create a new socket for the generation of broadcast interfering traffic
  // With the aim of sending generic broadcast packets, we use a PacketSocket
  // This kind of facility enables the transmission of data with a customizable
  // Ethertype, and no specific stack for the higher layers
  // As we are just sending interfering traffic (without any useful informative content),
  // we can use the local experimental Ethertype (0x88B5)
  PacketSocketHelper packetSocket;
  packetSocket.Install(c);
  TypeId tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  Ptr<Socket> source_interfering = Socket::CreateSocket (c.Get (2), tid);
  PacketSocketAddress local_source_interfering;
  local_source_interfering.SetSingleDevice (c.Get(2)->GetDevice(0)->GetIfIndex ());
  local_source_interfering.SetPhysicalAddress (c.Get(2)->GetDevice(0)->GetAddress());
  local_source_interfering.SetProtocol (0x88B5); // Setting the "Local Experimental Ethertype 1" to send interfering traffic
  if (source_interfering->Bind (local_source_interfering) == -1)
  {
    NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
  }
  PacketSocketAddress remote_source_interfering;
  remote_source_interfering.SetSingleDevice (c.Get(2)->GetDevice(0)->GetIfIndex());
  // Set the broadcast MAC address as interfering traffic will be broadcasted by vehicle 3
  remote_source_interfering.SetPhysicalAddress (c.Get(2)->GetDevice(0)->GetBroadcast());
  remote_source_interfering.SetProtocol (0x88B5); // Setting the "Local Experimental Ethertype 1" to send interfering traffic
  source_interfering->Connect (remote_source_interfering);
   // Important: this line lets you set the AC of the interfering traffic, through a User Priority (UP) value, like in real Linux kernels/OS
  source_interfering->SetPriority (interfering_up); // Setting the priority of the interfering traffic from vehicle 3

  std::cout << "A transmission power of " << txPower << " dBm  will be used." << std::endl;

  std::cout << "Starting simulation... " << std::endl;

  // Important: what you write inside setupNewWifiNode() will be executed every time a new vehicle enters the simulation in SUMO
  // This kind of "std::function" is called lambda function, and it can access all variables outside its scope, thanks to the [&] capture
  // We setup here the ETSI stack for each vehicle (except the one generating interfering traffic), thanks to the BSContainer object
  // Furthermore, we schedule the transmission of interfering traffic for vehicle 3 only ("veh3")
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      unsigned long nodeID = std::stol(vehicleID.substr (3))-1;

      if(vehicleID=="veh3")
      {
        // The interfering traffic generation will start after 1 second ("Seconds (1.0)"), by calling the "GenerateTraffic_interfering" function
        // Then:
        // - the socket which will be used for the transmission of interfering traffic is pointed by "source_interfering"
        // - each interfering traffic packet will be quite large (2000 Bytes)
        // - interfering traffic is set to last for "simTime*1000" packets, as we send each interfering packet every 1 ms and simTime is specified in seconds
        // - interfering packets will be sent every 1 ms ("MilliSeconds (1)"), to analyse a scenario with a relatively congested channel
        Simulator::ScheduleWithContext (source_interfering->GetNode ()->GetId (),
                                        Seconds (1.0), &GenerateTraffic_interfering,
                                        source_interfering, 2000, simTime*1000.0, MilliSeconds (1));
      }
      else
      {
          // Create a new ETSI GeoNetworking socket, thanks to the GeoNet::createGNPacketSocket() function, accepting as argument a pointer to the current node
          Ptr<Socket> sock;
          sock=GeoNet::createGNPacketSocket(c.Get(nodeID));
          // Set the proper AC, through the specified UP
          sock->SetPriority (up);

          // Create a new Basic Service Container object, which includes, for the current vehicle, both the ETSI CA Basic Service, for the transmission/reception
          // of periodic CAMs, and the ETSI DEN Basic Service, for the transmission/reception of event-based DENMs
          // An ETSI Basic Services container is a wrapper class to enable easy handling of both CAMs and DENMs
          // The station ID is set to be equal to the SUMO ID without "veh" (i.e., the station ID of "veh1" will be "1")
          Ptr<BSContainer> bs_container = CreateObject<BSContainer>(std::stol(vehicleID.substr(3)),StationType_passengerCar,sumoClient,false,sock);
          // Setup the PRRsupervisor inside the BSContainer, to make each vehicle collect latency and PRR metrics
          bs_container->linkMetricSupervisor(metSup);
          // This is needed just to simplify the whole application
          bs_container->disablePRRSupervisorForGNBeacons ();

          // Set the function which will be called every time a CAM is received, i.e., receiveCAM()
          bs_container->addCAMRxCallback (std::bind(&receiveCAM,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5));
          // Setup the new ETSI Basic Services container
          // The first parameter is true is you want to setup a CA Basic Service (for sending/receiving CAMs)
          // The second parameter should be true if you want to setup a DEN Basic Service (for sending/receiving DENMs)
          // The third parameter should be true if you want to setup a VRU Basic Service (for sending/receiving VAMs)
          // The fourth parameter should be true if you want to setup a CP Service (for sending/receiving CPMs)
          // The fifth parameter should be true if you want to setup a MC Service (for sending/receiving MCMs)
          // The sixth parameter should be true if you want to enable V2X security
          bs_container->setupContainer(true,false,false,false,false,security_enabled);

          // Store the container for this vehicle inside a local global BSMap, i.e., a structure (similar to a hash table) which allows you to easily
          // retrieve the right BSContainer given a vehicle ID
          basicServices.add(bs_container);

          // Start transmitting CAMs
          // We randomize the instant in time in which the CAM dissemination is going to start
          // This simulates different startup times for the OBUs of the different vehicles, and
          // reduces the risk of multiple vehicles trying to send CAMs are the same time (causing more collisions);
          // "desync" is a value between 0 and 1 (seconds) after which the CAM dissemination should start
          std::srand(Simulator::Now().GetNanoSeconds ()*2); // Seed based on the simulation time to give each vehicle a different random seed
          double desync = ((double)std::rand()/RAND_MAX);
          bs_container->getCABasicService ()->startCamDissemination (desync);
      }

      return c.Get(nodeID);
    };

  // Important: what you write here is called every time a node exits the simulation in SUMO
  // You can safely keep this function as it is, and ignore it
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode, std::string vehicleID)
    {
      /* Set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0));

      // Turn off the Basic Services and the ETSI ITS-G5 stack for the vehicle
      // which has exited from the simulated scenario, and should be thus no longer considered
      // We need to get the right Ptr<BSContainer> based on the station ID (not the nodeID used
      // as index for the nodeContainer), so we don't use "-1" to compute "intVehicleID" here
      unsigned long intVehicleID = std::stol(vehicleID.substr (3));

      Ptr<BSContainer> bsc = basicServices.get(intVehicleID);
      bsc->cleanup();
    };

  // Link ns-3 and SUMO
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  // Start simulation, which will last for simTime seconds
  Simulator::Stop (Seconds(simTime));
  Simulator::Run ();

  // When the simulation is terminated, gather the most relevant metrics from the PRRsupervisor
  std::cout << "Run terminated..." << std::endl;

  std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
  std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;

  std::cout << "Average latency veh 1 (ms): " << metSup->getAverageLatency_vehicle (1) << std::endl;
  std::cout << "Average latency veh 2 (ms): " << metSup->getAverageLatency_vehicle (2) << std::endl;
  std::cout << "Average latency veh 3 (ms): " << metSup->getAverageLatency_vehicle (3) << std::endl; // Should return 0, as this vehicle generated only interfering traffic, and it is ignored by the PRRsupervisor
  std::cout << "Average latency veh 4 (ms): " << metSup->getAverageLatency_vehicle (4) << std::endl;

  std::cout << "RX packet count: " << packet_count << std::endl;
  std::cout << "RX packet count (from PRR Supervisor): " << metSup->getNumberRx_overall () << std::endl;
  std::cout << "TX packet count (from PRR Supervisor): " << metSup->getNumberTx_overall () << std::endl;
  std::cout << "Average number of vehicle within the " << m_baseline_prr << " m baseline: " << metSup->getAverageNumberOfVehiclesInBaseline_overall () << std::endl;

  Simulator::Destroy ();

  return 0;
}