#include "ns3/antenna-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/eps-bearer-tag.h"
#include "ns3/grid-scenario-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/nr-ue-rrc.h"

#include <random>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ChannelEstimation");

static bool g_rxPdcpCallbackCalled = false;
static bool g_rxRxRlcPDUCallbackCalled = false;

static uint64_t totalPdcpDelay = 0;
static uint32_t totalPdcpPackets = 0;
static double averagePdcpDelayNs = 0.0;

Time g_txPeriod = Seconds (0.1);
Time delay;
std::fstream m_ScenarioFile;

class MyModel : public Application
{
  public:
    MyModel ();
    virtual ~MyModel ();

    void Setup (Ptr<NetDevice> device, Address address, uint32_t packetSize, uint32_t nPackets,
              DataRate dataRate, uint8_t period, uint32_t deadline);
    
    // DL
    void SendPacketDl ();
    void ScheduleTxDl ();

    // UL
    void SendPacketUl ();
    void ScheduleTxUl (uint8_t period);
    void ScheduleTxUl_Configuration ();

  private:
    Ptr<NetDevice> m_device;
    Address m_addr;
    uint32_t m_packetSize;
    uint32_t m_nPackets;
    DataRate m_dataRate;
    EventId m_sendEvent;
    bool m_running;
    uint32_t m_packetsSent;
    uint8_t m_periodicity;
    uint32_t m_deadline;
};

MyModel::MyModel ()
    : m_device (),
      m_addr (),
      m_packetSize (0),
      m_nPackets (0),
      m_dataRate (0),
      m_sendEvent (),
      m_running (false),
      m_packetsSent (0),
      m_periodicity (0),
      m_deadline (0)
{
}

MyModel::~MyModel ()
{
}

void
MyModel::Setup (Ptr<NetDevice> device, Address address, uint32_t packetSize, uint32_t nPackets,
                DataRate dataRate, uint8_t period, uint32_t deadline)
{
  m_device = device;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
  m_running = true;
  m_packetsSent = 0;
  m_periodicity = period;
  m_deadline = deadline;
}

/*
   * This is the first event that is executed  for DL traffic.
   */
void
StartApplicationDl (Ptr<MyModel> model)
{
  model->SendPacketDl ();
}
/*
   * Function creates a single packet and directly calls the function send
   * of a device to send the packet to the destination address.
   * (DL TRAFFIC)
   */
void
MyModel::SendPacketDl ()
{
  Ptr<Packet> pkt = Create<Packet> (m_packetSize, m_periodicity, m_deadline);
  Ipv4Header ipv4Header;
  ipv4Header.SetProtocol (Ipv4L3Protocol::PROT_NUMBER);
  pkt->AddHeader (ipv4Header);

  EpsBearerTag tag (1, 1);
  pkt->AddPacketTag (tag);

  m_device->Send (pkt, m_addr, Ipv4L3Protocol::PROT_NUMBER);
  NS_LOG_INFO ("Sending DL");

  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTxDl ();
    }
}
/*
   * SendPacket creates the packet at tNext time instant.
   */

void
MyModel::ScheduleTxDl ()
{
    if (m_running)
    {
        Time tNext = MilliSeconds (2);
        m_sendEvent = Simulator::Schedule (tNext, &MyModel::SendPacketDl, this);
    }
}

/*
   * This is the first event that is executed  for UL traffic.
   */
void
StartApplicationUl (Ptr<MyModel> model)
{
    model->SendPacketUl ();
}

/*
   * Function creates a single packet and directly calls the function send
   * of a device to send the packet to the destination address.
   * (UL TRAFFIC)
   */
void
MyModel::SendPacketUl ()
{
    // 랜덤 패킷 크기 생성
    Ptr<UniformRandomVariable> packetSizeRand = CreateObject<UniformRandomVariable> ();
    uint32_t randomPacketSize = packetSizeRand->GetInteger (150, 150); //just
    Ptr<UniformRandomVariable> periodRand = CreateObject<UniformRandomVariable> ();
    uint8_t randomPeriod = periodRand->GetInteger (10, 10);

    Ptr<Packet> pkt = Create<Packet> (randomPacketSize, randomPeriod, m_deadline);

    Ipv4Header ipv4Header;
    ipv4Header.SetProtocol (Ipv4L3Protocol::PROT_NUMBER);
    pkt->AddHeader (ipv4Header);

    m_device->Send (pkt, m_addr, Ipv4L3Protocol::PROT_NUMBER);
    NS_LOG_INFO ("Sending UL");

    if (m_packetsSent == 0)
    {
        ScheduleTxUl_Configuration ();
        m_packetsSent = 1;
    }
    else if (++m_packetsSent < m_nPackets)
    {
        ScheduleTxUl (randomPeriod);
    }
}

/*
   * SendPacket creates the packet at tNext time instant.
   */
void
MyModel::ScheduleTxUl (uint8_t period)
{
  if (m_running)
  {
       Time tNext = MilliSeconds (period);
       m_sendEvent = Simulator::Schedule (tNext, &MyModel::SendPacketUl, this);
  }
}

void
MyModel::ScheduleTxUl_Configuration (void)
{
    uint8_t configurationTime = 60;
    Time tNext = MilliSeconds (configurationTime);
    m_sendEvent = Simulator::Schedule (tNext, &MyModel::SendPacketUl, this);
}

/*
   * TraceSink, RxRlcPDU connects the trace sink with the trace source (RxPDU). It connects the UE with gNB and vice versa.
   */
void
RxRlcPDU (std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t rlcDelay)
{
    g_rxRxRlcPDUCallbackCalled = true;
    delay = Time::FromInteger (rlcDelay, Time::NS);
    // NS_LOG_UNCOND("\n rlcDelay in NS (Time):", delay);

    // std::cout << "\n\n Data received at RLC layer at:" << Simulator::Now () << std::endl;

    m_ScenarioFile << "\n\n Data received at RLC layer at:" << Simulator::Now () << std::endl;
    m_ScenarioFile << "\n rnti:" << rnti << std::endl;
    m_ScenarioFile << "\n delay :" << rlcDelay << std::endl;
}

void
RxPdcpPDU (std::string path, uint16_t rnti, uint8_t lcid, uint32_t bytes, uint64_t pdcpDelay)
{
    //   std::cout << "\n Packet PDCP delay:" << pdcpDelay << "\n";
    g_rxPdcpCallbackCalled = true;

    totalPdcpDelay += pdcpDelay;
    totalPdcpPackets += 1;

    // 평균 PDCP 딜레이 계산 (나노초 기준)
    averagePdcpDelayNs = (double) totalPdcpDelay / totalPdcpPackets;

    //   std::cout << "\nPacket PDCP Delay (ns): " << pdcpDelay;
    //   std::cout << "\nAverage PDCP Delay (ns): " << averagePdcpDelayNs << "\n";
}

void
ConnectUlPdcpRlcTraces ()
{
    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/DataRadioBearerMap/*/LtePdcp/RxPDU",
                    MakeCallback (&RxPdcpPDU));

    Config::Connect ("/NodeList/*/DeviceList/*/LteEnbRrc/UeMap/*/DataRadioBearerMap/*/LteRlc/RxPDU",
                    MakeCallback (&RxRlcPDU));
    NS_LOG_INFO ("Received PDCP RLC UL");
}


int
main (int argc, char *argv[])
{
    // Simulation Parameter
    uint16_t numerologyBwp1 = 0; 
    uint32_t packetSize = 512;
    double centralFrequencyBand1 = 700e6;
    double bandwidthBand1 = 20e6; //
    uint8_t period = uint8_t (10);
    uint16_t gNbNum = 1;
    uint16_t ueNum = 50;
    bool enableUl = true;
    uint32_t nPackets = 1000000;


    // Setting Random Seed
    RngSeedManager::SetSeed (12345);
    RngSeedManager::SetRun (1);


    // Pasing CommandLine
    CommandLine cmd;
    cmd.AddValue ("numerologyBwp1", "The numerology to be used in bandwidth part 1", numerologyBwp1);
    cmd.AddValue ("centralFrequencyBand1", "The system frequency to be used in band 1",
                    centralFrequencyBand1);
    cmd.AddValue ("bandwidthBand1", "The system bandwidth to be used in band 1", bandwidthBand1);
    cmd.AddValue ("packetSize", "packet size in bytes", packetSize);
    cmd.AddValue ("enableUl", "Enable Uplink", enableUl);
    cmd.Parse (argc, argv);


    // 각 UE 별 트래픽 설정 (패킷 시작 시간, 주기(의미x), 데드라인, 총 패킷 수)
    std::vector<uint32_t> v_init (ueNum);
    std::vector<uint32_t> v_period (ueNum);
    std::vector<uint32_t> v_deadline (ueNum);
    std::vector<uint32_t> v_packet (ueNum);
    std::cout << "\n Init values: " << '\n';
    v_init = std::vector<uint32_t> (ueNum, {300000});
    for (int val : v_init)
        std::cout << val << std::endl;
    std::cout << "Deadline values: " << '\n';
    v_deadline = std::vector<uint32_t> (ueNum, {10000000});
    for (int val : v_deadline)
        std::cout << val << std::endl;
    std::cout << "Packet values: " << '\n';
    v_packet = std::vector<uint32_t> (ueNum, {packetSize});
    for (int val : v_packet)
        std::cout << val << std::endl;
    std::cout << "Period values: " << '\n';
    v_period = std::vector<uint32_t> (ueNum, {10});
    for (int val : v_period)
        std::cout << val << "\t";

    
    // 파일 저장 시작
    m_ScenarioFile.open ("Scenario.txt", std::ofstream::out | std::ofstream::trunc);
    std::ostream_iterator<std::uint32_t> output_iterator (m_ScenarioFile, "\n");
    m_ScenarioFile << "Nº UE" << "\t" << "Init" << "\t" << "Latency" << "\t" << "Periodicity"
                    << std::endl;
    m_ScenarioFile << ueNum << std::endl;
    std::copy (v_init.begin (), v_init.end (), output_iterator);
    m_ScenarioFile << std::endl;
    std::copy (v_deadline.begin (), v_deadline.end (), output_iterator);
    m_ScenarioFile << std::endl;
    std::copy (v_period.begin (), v_period.end (), output_iterator);
    m_ScenarioFile << std::endl;


    int64_t randomStream = 1;


    // gNB 및 UE 위치와 이동성 설정
    NodeContainer gnbNodes;
    gnbNodes.Create (gNbNum);
    NodeContainer ueNodes;
    ueNodes.Create (ueNum);
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    gnbMobility.Install (gnbNodes);
    // gNB 위치 설정 (육각형 레이아웃, ISD 500m)
    double isd = 500.0;
    std::vector<Vector> gnbPositions = {
        Vector (750.0, 750.0, 3.0), // 중앙 gNB
    };
    for (uint32_t i = 0; i < gnbNodes.GetN (); ++i)
    {
        Ptr<MobilityModel> mob = gnbNodes.Get (i)->GetObject<MobilityModel> ();
        mob->SetPosition (gnbPositions[i]);
    }
    // === UE Mobility 설정 ===
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install (ueNodes);
    // 3GPP TR 38.913 6.1.7 시나리오 기준 위치 분포
    Ptr<UniformDiscPositionAllocator> uePosAlloc = CreateObject<UniformDiscPositionAllocator> ();
    uePosAlloc->SetX (750.0); // gNB 중심 위치
    uePosAlloc->SetY (750.0);
    uePosAlloc->SetRho (500.0); // Urban macro에 맞게 500m 반지름 내 분포
    Ptr<UniformRandomVariable> speedSelector = CreateObject<UniformRandomVariable> ();
    speedSelector->SetAttribute ("Min", DoubleValue (0.0));
    speedSelector->SetAttribute ("Max", DoubleValue (1.0));
    for (uint32_t i = 0; i < ueNodes.GetN (); ++i)
    {
        Vector pos = uePosAlloc->GetNext ();
        Ptr<MobilityModel> mob = ueNodes.Get (i)->GetObject<MobilityModel> ();
        mob->SetPosition (pos);

        Ptr<ConstantVelocityMobilityModel> cvm = DynamicCast<ConstantVelocityMobilityModel> (mob);
        double angle = (i * 37) % 360; // 이동 방향 분산
        double rad = angle * M_PI / 180.0;
        double speedRatio = speedSelector->GetValue ();

        // 20%: 차량 (100 km/h), 80%: 보행자 (3 km/h)
        double speed = (speedRatio < 0.2) ? 27.78 : 0.833;
        cvm->SetVelocity (Vector (speed * cos (rad), speed * sin (rad), 0.0));
    }


    // Setting nrHelper (epc, beamforming)
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper> ();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();
    nrHelper->SetBeamformingHelper (idealBeamformingHelper);
    nrHelper->SetEpcHelper (epcHelper);


    // Setting SRS (채널 상태 보고 UE=>gNB)
    nrHelper->SetSchedulerAttribute ("SrsSymbols", UintegerValue (0));


    // Setting HARQ
    nrHelper->SetSchedulerAttribute ("EnableHarqReTx", BooleanValue (true)); // just
    Config::SetDefault ("ns3::NrHelper::HarqEnabled", BooleanValue (true));

    
    // Setting Scheduler Type
    // 1 : OfdmaRR
    // 2 : ~~
    uint8_t sch = 1;
    if (sch != 0)
    {
        nrHelper->SetSchedulerTypeId (
            NrMacSchedulerOfdmaRR::GetTypeId ()); // Setting Scheduling Algorithm
        nrHelper->SetSchedulerAttribute ("schOFDMA", UintegerValue (sch)); // sch = 0 for TDMA
            // 1 for 5GL-OFDMA
            // 2 for Sym-OFDMA
            // 3 for RB-OFDMA
    }

    
    // Setting Single CC, Bandwidth, Channel Model
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    CcBwpCreator::SimpleOperationBandConf bandConf1 (centralFrequencyBand1, bandwidthBand1,
                                                    numCcPerBand, BandwidthPartInfo::UMa_nLoS);
    OperationBandInfo band1 = ccBwpCreator.CreateOperationBandContiguousCc (bandConf1);
    Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
    nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));


    // Setting MCS (MCS를 고정해야 학습 시 Replay가 쉽다고 가정)
    nrHelper->SetSchedulerAttribute ("FixedMcsDl", BooleanValue (true));
    nrHelper->SetSchedulerAttribute ("StartingMcsDl", UintegerValue (4));
    nrHelper->SetSchedulerAttribute ("FixedMcsUl", BooleanValue (true)); 
    nrHelper->SetSchedulerAttribute ("StartingMcsUl", UintegerValue (8));
    

    // Setting Error Model
    // NrEesmIrT1 = 64(low drop) QAM, NrEesmIrT2 = 256(high drop) QAM 
    nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true)); 
    std::string errorModel = "ns3::NrEesmIrT1";
    nrHelper->SetUlErrorModel (errorModel);
    nrHelper->SetDlErrorModel (errorModel);
    nrHelper->SetGnbDlAmcAttribute (
        "AmcModel", EnumValue (NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
    nrHelper->SetGnbUlAmcAttribute (
        "AmcModel", EnumValue (NrAmc::ErrorModel)); // NrAmc::ShannonModel or NrAmc::ErrorModel
    bool fadingEnabled = true;
    auto bandMask = NrHelper::INIT_PROPAGATION | NrHelper::INIT_CHANNEL;
    if (fadingEnabled)
    {
        bandMask |= NrHelper::INIT_FADING;
    }
    nrHelper->InitializeOperationBand (&band1, bandMask);
    allBwps = CcBwpCreator::GetAllBwps ({band1});


    // Setting Beamforming Method
    idealBeamformingHelper->SetAttribute ("BeamformingMethod",
                                            TypeIdValue (QuasiOmniDirectPathBeamforming::GetTypeId ()));


    // Setting gNB, UE Antenna Elements
    nrHelper->SetGnbAntennaAttribute ("NumRows", UintegerValue (2));
    nrHelper->SetGnbAntennaAttribute ("NumColumns", UintegerValue (4));
    nrHelper->SetGnbAntennaAttribute ("AntennaElement",
                                        PointerValue (CreateObject<IsotropicAntennaModel> ()));

    nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));
    nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
    nrHelper->SetUeAntennaAttribute ("AntennaElement",
                                    PointerValue (CreateObject<IsotropicAntennaModel> ()));


    // Install NetDevice
    NetDeviceContainer enbNetDev = nrHelper->InstallGnbDevice (gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice (ueNodes, allBwps);


    // 랜덤 스트림 적용
    // randomStream += ueMobility.AssignStreams(ueNodes, randomStream);
    // randomStream += gnbMobility.AssignStreams(gnbNodes, randomStream);
    randomStream += nrHelper->AssignStreams (enbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams (ueNetDev, randomStream);


    // gNB, UE Tx Power
    nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (26.0));
    nrHelper->SetGnbPhyAttribute ("TxPower", DoubleValue (46.0));


    // Set the attribute of the netdevice (enbNetDev.Get (0)) and bandwidth part (0)
    nrHelper->GetGnbPhy(enbNetDev.Get(0), 0)
            ->SetAttribute("Numerology", UintegerValue(numerologyBwp1));
    for (auto it = enbNetDev.Begin(); it != enbNetDev.End(); ++it)
    {
        DynamicCast<NrGnbNetDevice>(*it)->UpdateConfig();
    }
    for (auto it = ueNetDev.Begin(); it != ueNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }


    // Setting Ip
    InternetStackHelper internet;
    internet.Install (ueNodes); // gridScenario.GetUserTerminals() 대신 ueNodes 사용
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueNetDev));


    // Uplink Traffic Schedule
    std::vector<Ptr<MyModel>> v_modelUl;
    v_modelUl = std::vector<Ptr<MyModel>> (ueNum, {0});
    for (uint8_t ii = 0; ii < ueNum; ++ii)
    {
        Ptr<MyModel> modelUl = CreateObject<MyModel> ();
        modelUl->Setup (ueNetDev.Get (ii), enbNetDev.Get (0)->GetAddress (), v_packet[ii], nPackets,
                        DataRate ("1Mbps"), v_period[ii], v_deadline[ii]);
        v_modelUl[ii] = modelUl;
        Simulator::Schedule (MicroSeconds (v_init[ii]), &StartApplicationUl, v_modelUl[ii]);
    }


    // Attach UEs to the closest eNB
    nrHelper->AttachToClosestEnb (ueNetDev, enbNetDev);
    for (uint32_t i = 0; i < ueNetDev.GetN (); ++i)
    {
        auto dev = DynamicCast<NrUeNetDevice> (ueNetDev.Get (i));
        if (dev->GetTargetEnb () == nullptr)
        {
            std::cout << "UE[" << i << "] attach failed!" << std::endl;
        }
        else
        {
            std::cout << "UE[" << i << "] attached to gNB" << std::endl;
        }
    }


    // Enable Tracing
    nrHelper->EnableTraces ();
    Simulator::Schedule (Seconds (0.16), &ConnectUlPdcpRlcTraces);


    // Pring Output (Throughput, Delay, PDR, System Weighted Average AoI, )
    Ptr<NrGnbMac> gnbMac = DynamicCast<NrGnbMac> (
        enbNetDev.Get (0)->GetObject<NrGnbNetDevice> ()->GetMac (0)); // gNB 객체 포인터
    Simulator::Schedule (Seconds (10) - NanoSeconds (1), &NrGnbMac::PrintThroughput, gnbMac);

    Simulator::Stop (Seconds (10));
    Simulator::Run ();

    std::cout << "\nAverage PDCP Delay (ns): " << averagePdcpDelayNs << "\n";
    std::cout << "\n FIN. " << std::endl;

    if (g_rxPdcpCallbackCalled && g_rxRxRlcPDUCallbackCalled)
    {
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }

    Simulator::Destroy ();
}