/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2019 Universita' degli Studi di Napoli Federico II
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
 * Author: Stefano Avallone <stavallo@unina.it>
 */

#include "ns3/log.h"
#include "rr-ofdma-manager.h"
#include "wifi-ack-policy-selector.h"
#include "wifi-phy.h"
#include <utility>
#include <algorithm>


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RrOfdmaManager");

NS_OBJECT_ENSURE_REGISTERED (RrOfdmaManager);

TypeId
RrOfdmaManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrOfdmaManager")
    .SetParent<OfdmaManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RrOfdmaManager> ()
    .AddAttribute ("NStations",
                   "The maximum number of stations that can be granted an RU in the MU DL OFDMA transmission",
                   UintegerValue (4),
                   MakeUintegerAccessor (&RrOfdmaManager::m_nStations),
                   MakeUintegerChecker<uint8_t> (1, 74))
    .AddAttribute ("ForceDlOfdma",
                   "If enabled, return DL_OFDMA even if no DL MU PPDU could be built.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&RrOfdmaManager::m_forceDlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("EnableUlOfdma",
                   "If enabled, return UL_OFDMA if DL_OFDMA was returned the previous time.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrOfdmaManager::m_enableUlOfdma),
                   MakeBooleanChecker ())
    .AddAttribute ("UlPsduSize",
                   "The size in bytes of the solicited PSDU (to be sent in an HE TB PPDU)",
                   UintegerValue (500),
                   MakeUintegerAccessor (&RrOfdmaManager::m_ulPsduSize),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ChannelBw",
                   "For TESTING only",
                   UintegerValue (20),
                   MakeUintegerAccessor (&RrOfdmaManager::m_bw),
                   MakeUintegerChecker<uint16_t> (5, 160))
  ;
  return tid;
}

RrOfdmaManager::RrOfdmaManager ()
  : m_startStation (0)
{
  NS_LOG_FUNCTION (this);
}

RrOfdmaManager::~RrOfdmaManager ()
{
  NS_LOG_FUNCTION_NOARGS ();
}

void
RrOfdmaManager::InitTxVectorAndParams (std::map<Mac48Address, DlPerStaInfo> staList,
                                        std::vector<std::pair<HeRu::RuType,size_t>> ruAssigned, DlMuAckSequenceType dlMuAckSequence)
{
  NS_LOG_FUNCTION (this);
  m_txVector = WifiTxVector ();
  m_txVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
  m_txVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
  m_txVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
  m_txVector.SetTxPowerLevel (GetWifiRemoteStationManager ()->GetDefaultTxPowerLevel ());
  m_txParams = MacLowTransmissionParameters ();
  m_txParams.SetDlMuAckSequenceType (dlMuAckSequence);

  Ptr<WifiMacQueueItem> mpdu = Copy (m_mpdu);
  auto ruIt=ruAssigned.begin();
  for (auto& sta : staList)
    {if(ruIt==ruAssigned.end()){break;}
      mpdu->GetHeader ().SetAddr1 (sta.first);
      // Get the TX vector used to transmit single user frames to the receiver
      // station (the RU index will be assigned by ComputeDlOfdmaInfo)
      WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu);
      NS_LOG_DEBUG ("Adding STA with AID=" << sta.second.aid << " and TX mode="
                    << suTxVector.GetMode () << " to the TX vector" << staList.size() << ruAssigned.size());

      m_txVector.SetHeMuUserInfo (sta.second.aid, {{false, ruIt->first, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});

      // Add the receiver station to the appropriate list of the TX params
      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (sta.second.tid)];
      BlockAckReqType barType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                                ? txop->GetBlockAckReqType (sta.first, sta.second.tid)
                                : BlockAckReqType::COMPRESSED;
      BlockAckType baType = txop->GetBaAgreementEstablished (sta.first, sta.second.tid)
                            ? txop->GetBlockAckType (sta.first, sta.second.tid)
                            : BlockAckType::COMPRESSED;

      if (dlMuAckSequence == DlMuAckSequenceType::DL_SU_FORMAT)
        {
          // Enable BAR/BA exchange for all the receiver stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR)
        {
          // Send a MU-BAR to all the stations
          m_txParams.EnableBlockAckRequest (sta.first, barType, baType);
        }
      else if (dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Expect to receive a Block Ack from all the stations
          m_txParams.EnableBlockAck (sta.first, baType);
        }
        ruIt++;
    }
}

OfdmaTxFormat
RrOfdmaManager::SelectTxFormat (Ptr<const WifiMacQueueItem> mpdu)
{
  // --- for TESTING only ---
//   for (uint8_t i = 1; i <= m_nStations; i++)
//     {
//       DlPerStaInfo info {i, 0};
//       m_staInfo.push_back (std::make_pair (Mac48Address::Allocate (), info));
//     }
//   return OfdmaTxFormat::DL_OFDMA;
  // --- --- ---
  NS_LOG_FUNCTION (this << *mpdu);
  NS_ASSERT (mpdu->GetHeader ().IsQosData ());

  if (m_enableUlOfdma && GetTxFormat () == DL_OFDMA)
    {
      // check if an UL OFDMA transmission is possible after a DL OFDMA transmission
      NS_ABORT_MSG_IF (m_ulPsduSize == 0, "The UlPsduSize attribute must be set to a non-null value");

      Ptr<QosTxop> txop = m_qosTxop[QosUtilsMapTidToAc (mpdu->GetHeader ().GetQosTid ())];
      m_ulMuAckSequence = txop->GetAckPolicySelector ()->GetAckSequenceForUlMu ();
      MacLowTransmissionParameters params;
      params.SetUlMuAckSequenceType (m_ulMuAckSequence);
      BlockAckType baType;

      if (m_ulMuAckSequence == UL_MULTI_STA_BLOCK_ACK)
        {
          baType = BlockAckType::MULTI_STA;
          for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
            {
              auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
              if (addressIt != m_apMac->GetStaList ().end ())
                {
                  baType.m_bitmapLen.push_back (32);
                  params.EnableBlockAck (addressIt->second, baType);
                }
              else
                {
                  NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
                }
            }
        }
      else
        {
          NS_FATAL_ERROR ("Sending Block Acks in an MU DL PPDU is not supported yet");
        }

      CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);

      // compute the maximum amount of time that can be granted to stations.
      // This value is limited by the max PPDU duration
      Time maxDuration = GetPpduMaxTime (m_txVector.GetPreambleType ());

      // compute the time required by stations based on the buffer status reports, if any
      uint32_t maxBufferSize = 0;

      for (auto& userInfo : m_txVector.GetHeMuUserInfoMap ())
        {
          auto addressIt = m_apMac->GetStaList ().find (userInfo.first);
          if (addressIt != m_apMac->GetStaList ().end ())
            {
              uint8_t queueSize = m_apMac->GetMaxBufferStatus (addressIt->second);
              if (queueSize == 255)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is unknown");
                  maxBufferSize = std::max (maxBufferSize, m_ulPsduSize);
                }
              else if (queueSize == 254)
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is not limited");
                  maxBufferSize = 0xffffffff;
                  break;
                }
              else
                {
                  NS_LOG_DEBUG ("Buffer status of station " << addressIt->second << " is " << +queueSize);
                  maxBufferSize = std::max (maxBufferSize, static_cast<uint32_t> (queueSize * 256));
                }
            }
          else
            {
              NS_LOG_WARN ("Maybe station with AID=" << userInfo.first << " left the BSS since the last MU DL transmission?");
            }
        }

      // if the maximum buffer size is 0, skip UL OFDMA and proceed with trying DL OFDMA
      if (maxBufferSize > 0)
        {
          // if we are within a TXOP, we have to consider the response time and the
          // remaining TXOP duration
          if (txop->GetTxopLimit ().IsStrictlyPositive ())
            {
              // we need to define the HE TB PPDU duration in order to compute the response to
              // the Trigger Frame. Let's use 1 ms for this purpose. We'll subtract it later.
              uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (MilliSeconds (1),
                                                                              m_low->GetPhy ()->GetFrequency ());
              trigger.SetUlLength (length);

              Ptr<Packet> packet = Create<Packet> ();
              packet->AddHeader (trigger);
              WifiMacHeader hdr;
              hdr.SetType (WIFI_MAC_CTL_TRIGGER);
              hdr.SetAddr1 (Mac48Address::GetBroadcast ());
              Ptr<WifiMacQueueItem> item = Create<WifiMacQueueItem> (packet, hdr);

              Time response = m_low->GetResponseDuration (params, m_txVector, item);

              // Add the time to transmit the Trigger Frame itself
              WifiTxVector txVector = GetWifiRemoteStationManager ()->GetRtsTxVector (hdr.GetAddr1 (), &hdr, packet);

              response += m_low->GetPhy ()->CalculateTxDuration (item->GetSize (), txVector,
                                                                 m_low->GetPhy ()->GetFrequency ());

              // Subtract the duration of the HE TB PPDU
              response -= WifiPhy::ConvertLSigLengthToHeTbPpduDuration (length, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency ());

              if (response > txop->GetTxopRemaining ())
                {
                  // an UL OFDMA transmission is not possible. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Remaining TXOP duration is not enough for UL MU exchange");
                  m_dataInfo.clear();
                  m_staInfo.clear ();
                  // ruAssigned.clear();
                  // ruIndexValues.clear();
                  return DL_OFDMA;
                }

              maxDuration = Min (maxDuration, txop->GetTxopRemaining () - response);
            }

          Time bufferTxTime = m_low->GetPhy ()->CalculateTxDuration (maxBufferSize, m_txVector,
                                                                     m_low->GetPhy ()->GetFrequency (),
                                                                     trigger.begin ()->GetAid12 ());
          if (bufferTxTime < maxDuration)
            {
              // the maximum buffer size can be transmitted within the allowed time
              maxDuration = bufferTxTime;
            }
          else
            {
              // maxDuration may be a too short time. If it does not allow to transmit
              // at least m_ulPsduSize bytes, give up the UL MU transmission for now
              Time minDuration = m_low->GetPhy ()->CalculateTxDuration (m_ulPsduSize, m_txVector,
                                                                        m_low->GetPhy ()->GetFrequency (),
                                                                        trigger.begin ()->GetAid12 ());
              if (maxDuration < minDuration)
                {
                  // maxDuration is a too short time. Reset m_staInfo and return DL_OFDMA.
                  // In this way, no transmission will occur now and the next time we will try again
                  // performing an UL OFDMA transmission.
                  NS_LOG_DEBUG ("Available time " << maxDuration << " is too short");
                  m_dataInfo.clear();
                  m_staInfo.clear ();
                  // ruAssigned.clear();
                  // ruIndexValues.clear();
                  return DL_OFDMA;
                }
            }

          // maxDuration is the time to grant to the stations. Store it in the TX vector
          NS_LOG_DEBUG ("HE TB PPDU duration: " << maxDuration.ToDouble (Time::MS));
          uint16_t length = WifiPhy::ConvertHeTbPpduDurationToLSigLength (maxDuration,
                                                                          m_low->GetPhy ()->GetFrequency ());
          m_txVector.SetLength (length);
          m_txParams = params;
          return UL_OFDMA;
        }
    }

  // get the list of associated stations ((AID, MAC address) pairs)
  const std::map<uint16_t, Mac48Address>& staList = m_apMac->GetStaList ();
  auto startIt = staList.find (m_startStation);

  // This may be the first invocation or the starting station left
  if (startIt == staList.end ())
    {
      startIt = staList.begin ();
      m_startStation = startIt->first;
    }

  uint8_t currTid = mpdu->GetHeader ().GetQosTid ();
  AcIndex primaryAc = QosUtilsMapTidToAc (currTid);
  m_dataInfo.clear();
  // ruIndexValues.clear();
  // ruAssigned.clear();
  m_staInfo.clear ();

  // If the primary AC holds a TXOP, we can select a station as a receiver of
  // the MU PPDU only if the AP has frames to send to such station that fit into
  // the remaining TXOP time. To this end, we need to determine the type of ack
  // sequence and the time it takes. To compute the latter, we can call the
  // MacLow::GetResponseDuration () method, which requires TX vector and TX params.
  // Our best guess at this stage is that the AP has frames to send to all the
  // associated stations and hence we initialize the TX vector and the TX params
  // by considering the starting station and those that immediately follow it in
  // the list of associated stations.
  std::size_t count = m_nStations;
 std::vector<std::pair<HeRu::RuType,size_t>> ruType = GetNumberAndTypeOfRus (m_low->GetPhy ()->GetChannelWidth (), count,m_staInfo);
  NS_ASSERT (count >= 1);

  std::map<Mac48Address, DlPerStaInfo> guess;
  auto staIt = startIt;
  do
    {
      guess[staIt->second] = {staIt->first, currTid};
      if (++staIt == staList.end ())
        {
          staIt = staList.begin ();
        }
    } while (guess.size () < count && staIt != startIt);

  Ptr<WifiAckPolicySelector> ackSelector = m_qosTxop[primaryAc]->GetAckPolicySelector ();
  NS_ASSERT (ackSelector != 0);
  m_dlMuAckSequence = ackSelector->GetAckSequenceForDlMu ();
  InitTxVectorAndParams (guess, ruType, m_dlMuAckSequence);

  // if the AC owns a TXOP, compute the time available for the transmission of data frames
  Time txopLimit = Seconds (0);
  if (m_qosTxop[primaryAc]->GetTxopLimit ().IsStrictlyPositive ())
    {
      // TODO Account for MU-RTS/CTS when implemented
      CtrlTriggerHeader trigger;

      if (m_dlMuAckSequence == DlMuAckSequenceType::DL_MU_BAR
          || m_dlMuAckSequence == DlMuAckSequenceType::DL_AGGREGATE_TF)
        {
          // Need to prepare the MU-BAR to correctly get the response time
          trigger = GetTriggerFrameHeader (m_txVector, 5);
          trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (trigger, m_txParams));
        }
      txopLimit = m_qosTxop[primaryAc]->GetTxopRemaining () - GetResponseDuration (m_txParams, m_txVector, trigger);

      if (txopLimit.IsNegative ())
        {
          if (m_forceDlOfdma)
            {
              NS_LOG_DEBUG ("Not enough TXOP remaining time: return DL_OFDMA with empty set of receiver stations");
              return OfdmaTxFormat::DL_OFDMA;
            }
          NS_LOG_DEBUG ("Not enough TXOP remaining time: return NON_OFDMA");
          return OfdmaTxFormat::NON_OFDMA;
        }
    }

  // iterate over the associated stations until an enough number of stations is identified
  do
    {
      NS_LOG_DEBUG ("Next candidate STA (MAC=" << startIt->second << ", AID=" << startIt->first << ")");
      // check if the AP has at least one frame to be sent to the current station
      auto ruIt=ruType.begin();
      for (uint8_t tid : std::initializer_list<uint8_t> {currTid, 1, 2, 0, 3, 4, 5, 6, 7})
        {
          AcIndex ac = QosUtilsMapTidToAc (tid);
          // check that a BA agreement is established with the receiver for the
          // considered TID, since ack sequences for DL MU PPDUs require block ack
          if (ac >= primaryAc && m_qosTxop[ac]->GetBaAgreementEstablished (startIt->second, tid))
            {
              Ptr<const WifiMacQueueItem> mpdu;
              mpdu = m_qosTxop[ac]->PeekNextFrame (tid, startIt->second);

              // we only check if the first frame of the current TID meets the size
              // and duration constraints. We do not explore the queues further.
              if (mpdu != 0)
                {
                  // Use a temporary TX vector including only the STA-ID of the
                  // candidate station to check if the MPDU meets the size and time limits.
                  // An RU of the computed size is tentatively assigned to the candidate
                  // station, so that the TX duration can be correctly computed.
                  WifiTxVector suTxVector = m_low->GetDataTxVector (mpdu),
                               muTxVector;

                  muTxVector.SetPreambleType (WIFI_PREAMBLE_HE_MU);
                  muTxVector.SetChannelWidth (m_low->GetPhy ()->GetChannelWidth ());
                  muTxVector.SetGuardInterval (m_low->GetPhy ()->GetGuardInterval ().GetNanoSeconds ());
                  muTxVector.SetHeMuUserInfo (startIt->first,
                                              {{false, ruIt->first, 1}, suTxVector.GetMode (), suTxVector.GetNss ()});

                  if (m_low->IsWithinSizeAndTimeLimits (mpdu, muTxVector, 0, txopLimit))
                    {
                      // the frame meets the constraints, add the station to the list
                      NS_LOG_DEBUG ("Adding candidate STA (MAC=" << startIt->second << ", AID="
                                    << startIt->first << ") TID=" << +tid);
                      
                      DlPerStaInfo info {startIt->first, tid};
   
                      m_dataInfo.push_back(std::make_tuple (startIt->second,mpdu->GetPacket()->GetSize(),info));
                      m_staInfo.push_back (std::make_pair (startIt->second, info));
                      
                      break;    // terminate the for loop
                    }
                }
              else
                {
                  NS_LOG_DEBUG ("No frames to send to " << startIt->second << " with TID=" << +tid);
                }
            }
            ruIt++;
        }

      // move to the next station in the map
      startIt++;
      if (startIt == staList.end ())
        {
          startIt = staList.begin ();
        }
    } while (m_staInfo.size () < m_nStations && startIt->first != m_startStation);

  if (m_staInfo.empty ())
    {
      if (m_forceDlOfdma)
        {
          NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return DL_OFDMA with empty set of receiver stations");
          return OfdmaTxFormat::DL_OFDMA;
        }
      NS_LOG_DEBUG ("The AP does not have suitable frames to transmit: return NON_OFDMA");
      return OfdmaTxFormat::NON_OFDMA;
    }

  m_startStation = startIt->first;
  return OfdmaTxFormat::DL_OFDMA;
}
void 
RrOfdmaManager::merge(std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>>& v, int p, int q, int r) 
{
    int size1 = q-p+1;
    int size2 = r-q;
    std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>> L(size1);
    std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>> R(size2);

    for(int i = 0; i < size1; i++)
    {
        L[i] = v[p+i];
    }
    for(int j = 0; j < size2; j++)
    {
        R[j]=v[q+j+1];
    }

    int i=0,j=0;
    int k;
    for(k = p; k <= r && i < size1 && j < size2; k++)
    {
        if(std::get<1>(L[i]) >= std::get<1>(R[j]))
        {
            v[k] = L[i];
            i++;
        }
        else
        {
            v[k] = R[j];
            j++;
        }
    }
    for(i = i; i < size1; ++i)
    {
        v[k] = L[i];
        k++;
    }

    for(j = j; j < size2; j++)
    {
        v[k] = R[j];
        k++;
    }
}
void 
RrOfdmaManager::merge_sort(std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>>& v, int p, int r) 
{
    if(p < r)
    {
        int q = (p+r)/2;
        merge_sort(v, p, q);
        merge_sort(v, q+1, r);
        merge(v, p, q, r);
    }
}
char* 
RrOfdmaManager::decToHexa(int n) 
{    
    // char array to store hexadecimal number 
    char hexaDeciNum[100]; 
      
    // counter for hexadecimal number array 
    int i = 0; 
    while(n!=0) 
    {    
        // temporary variable to store remainder 
        int temp  = 0; 
          
        // storing remainder in temp variable. 
        temp = n % 16; 
          
        // check if temp < 10 
        if(temp < 10) 
        { 
            hexaDeciNum[i] = temp + 48; 
            i++; 
        } 
        else
        { 
            hexaDeciNum[i] = temp + 55; 
            i++; 
        } 
          
        n = n/16; 
    } 
char array[]="0:0:0:0:0:";
   char *s=new char[10+i];
      strcpy(s, array);    
    // printing hexadecimal number array in reverse order 
      int u=10;
    for(int j=i-1; j>=0; j--)
    { 
      s[u++]=hexaDeciNum[j];
    }
    return s;
} 
std::vector<std::pair<HeRu::RuType,size_t>>
RrOfdmaManager::GetNumberAndTypeOfRus (uint16_t bandwidth, std::size_t& nStations,std::list<std::pair<Mac48Address, DlPerStaInfo>> m_staInfo)
{
    //onoff 2-16
 //   //bulksend 17-21
 //   //http 22-32
  // NS_LOG_FUNCTION (this);

  HeRu::RuType ruType;
  uint8_t nRusAssigned = 0;
  std::vector<std::pair<HeRu::RuType,size_t>> ruAssigned;
  std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>> onoff;
  std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>> bulksend;
  std::vector<std::tuple<Mac48Address,uint32_t ,DlPerStaInfo>> http;

  auto staInfoIt = m_dataInfo.begin ();
 
  int size1=(int)m_dataInfo.size();
  for (int i = 0; i < size1; i++)
  {
    // NS_ASSERT (staInfoIt != m_staInfo.end ());
    // printf("here" );
    // char add[6];
    // 
    int flag=0;
   
 
    // Mac48Address *mac=new Mac48Address("0:0:0:0:0:2");

  // NS_LOG_DEBUG ("MAC ADDRESS are " << std::get<0>(*staInfoIt) << std::get<1>(*staInfoIt)<<" here") ;
     auto b1 = bulksend1.begin ();
 
      for(int i=0;i<(int)bulksend1.size();i++)
      {
        if(*b1==std::get<0>(*staInfoIt)){bulksend.push_back(*staInfoIt);flag=1;break;}
        b1++;
      }
      if(flag==0)
      {
        b1 = onoff1.begin ();
        for(auto i=0;i<(int)onoff1.size();i++)
      {
        if(*b1==std::get<0>(*staInfoIt)){onoff.push_back(*staInfoIt);flag=1;break;}
        b1++;
      }
    }
      if(flag==0)
      {
        b1 = http1.begin ();
        for(auto i=0;i<(int)http1.size();i++)
      {
        if(*b1==std::get<0>(*staInfoIt)){http.push_back(*staInfoIt);flag=1;break;}
        b1++;
      }
      }
      NS_LOG_DEBUG("flag value "<<flag <<std::get<0>(*staInfoIt));
    staInfoIt++;
  }
  int size2=onoff.size();
  merge_sort(onoff,0,size2-1);
  int size3=bulksend.size();
  merge_sort(bulksend,0,size3-1);
  int size4=http.size();
  merge_sort(http,0,size4-1);
  if(size3 ==0 ||(size2+size3+size4<=1))
  {
    // iterate over all the available RU types
    NS_LOG_DEBUG(size1 <<" "<<size2<<" "<< size3<<" "<< size4);
    for (auto& ru : HeRu::m_heRuSubcarrierGroups)
      {
        if (ru.first.first == bandwidth && ru.second.size () <=nStations)
          {
            ruType = ru.first.second;
            nRusAssigned = ru.second.size ();
            break;
          }
        else if (bandwidth == 160 && ru.first.first == 80 && (2 * ru.second.size ()) <= nStations)
          {
            ruType = ru.first.second;
            nRusAssigned = 2 * ru.second.size ();
            break;
          }
      }
    if (nRusAssigned == 0)
      {
        NS_ASSERT (bandwidth == 160 && nStations == 1);
        nRusAssigned = 1;
        ruType = HeRu::RU_2x996_TONE;
      }
    m_dataInfo.clear();
    staInfoIt = onoff.begin ();
    for(int i=0;i<size2;i++){m_dataInfo.push_back(*staInfoIt);staInfoIt++;}
    staInfoIt = bulksend.begin ();
    for(int i=0;i<size3;i++){m_dataInfo.push_back(*staInfoIt);staInfoIt++;}
    staInfoIt = http.begin ();
    for(int i=0;i<size4;i++){m_dataInfo.push_back(*staInfoIt);staInfoIt++;}
    for (std::size_t ruIndex = 1; ruIndex <= HeRu::m_heRuSubcarrierGroups.at ({bandwidth, ruType}).size (); ruIndex++)
    {
      ruAssigned.push_back(std::make_pair(ruType,ruIndex));
    }
    // nStations = nRusAssigned;
    // NS_LOG_DEBUG("here before");
    return ruAssigned;
  }

  else 
  {
 
  
      if(size2>7)
      {
        m_dataInfo.clear();
        staInfoIt = onoff.begin ();

        for(int i=0;i<size2;i++){m_dataInfo.push_back(*staInfoIt);staInfoIt++;}

        staInfoIt = bulksend.begin ();

        for(int i=0;i<size2;i++){m_dataInfo.push_back(*staInfoIt);staInfoIt++;}
        
        staInfoIt = http.begin ();
        
        for(int i=0;i<size4;i++){m_dataInfo.push_back(*staInfoIt);staInfoIt++;}
        
        for (auto& ru : HeRu::m_heRuSubcarrierGroups)
        {


          ruType = ru.first.second;
          nRusAssigned = ru.second.size ();
          break;

        }
        for (std::size_t ruIndex = 1; ruIndex <= HeRu::m_heRuSubcarrierGroups.at ({bandwidth, ruType}).size (); ruIndex++)
        {
          ruAssigned.push_back(std::make_pair(ruType,ruIndex));
        }
      // nRusAssigned=size2;

      return ruAssigned;
    }
    else
    {
        int allocated106=0;
        int allocated52=0;
        int allocated26=size2;
        int a=242-26*size2;
        if(a>=106 && size3>0){allocated106+=1;a=a-106;size3-=1;}

        while(a>=52 && size3>0)
        {
          a-=52;
          allocated52+=1;
          size3-=1;
          
        }
        while(a>=26 && size4>0)
        {
          a-=26;
          allocated26+=1;
          size4-=1;
        }
   
        
        
        nRusAssigned=allocated106+allocated26+allocated52;
        NS_LOG_DEBUG("HERE "<< allocated52 <<" "<<allocated26 <<" "<<allocated106<< "before "<<m_dataInfo.size());
        size2=(int)onoff.size();
        size3=(int)bulksend.size();
        size4=(int)http.size();
        m_dataInfo.clear();
        auto staInfoIt1=onoff.begin();
        auto staInfoIt2=bulksend.begin();
        auto staInfoIt3=http.begin();
        HeRu::RuType type1=table.at("RU_106_TONE");
        HeRu::RuType type2=table.at("RU_52_TONE");
        HeRu::RuType type3=table.at("RU_26_TONE");

        if(allocated106>0){m_dataInfo.push_back(*staInfoIt2);staInfoIt2++;ruAssigned.push_back(std::make_pair(type1,1));size3-=1;}
        else 
        {
          m_dataInfo.push_back(*staInfoIt2);
          
          size3-=1;
          allocated52-=1;
          staInfoIt2++;
          ruAssigned.push_back(std::make_pair(type2,1)); 
          int u=2;
          size_t in=3;
          while(size2>0 &&u>0)
            {
              m_dataInfo.push_back(*staInfoIt1);
              staInfoIt1++;size2-=1;u-=1;
              ruAssigned.push_back(std::make_pair(type3,in++)); 
              // ruAssigned.push_back(type3);
              // ruIndexValues.push_back(in++);
            }
           while(size4>0 &&u>0)
            {
              m_dataInfo.push_back(*staInfoIt3);
              staInfoIt3++;u-=1;
              size4-=1;
              ruAssigned.push_back(std::make_pair(type3,in++)); 
              // ruAssigned.push_back(type3);
              // ruIndexValues.push_back(in++);
            }
        }
        if(allocated26%2!=0)
        {
          if(size2>0)
          {
            m_dataInfo.push_back(*staInfoIt1);staInfoIt1++;

            size2-=1;
          }
          else
          {
            m_dataInfo.push_back(*staInfoIt3);staInfoIt3++;
            size4-=1;
          }
          allocated26-=1;
          ruAssigned.push_back(std::make_pair(type3,5)); 
          // ruAssigned.push_back(type3);
          // ruIndexValues.push_back(5);
        }
        size_t in1=3;
        size_t in2=6;
        if(allocated52>0){in2=8;}
        while(allocated52>0)
        {
          m_dataInfo.push_back(*staInfoIt2);
          staInfoIt2++;
          allocated52-=1;
          ruAssigned.push_back(std::make_pair(type2,in1++)); 
          // ruAssigned.push_back(type2);
          // ruIndexValues.push_back(in1++);
          size3-=1;
        }
        while(allocated26>0 && size2>0)
        {
          m_dataInfo.push_back(*staInfoIt1);
          staInfoIt1++;
          allocated26-=1;
          size2-=1;
          ruAssigned.push_back(std::make_pair(type3,in2++));
          // ruAssigned.push_back(type3);
          // ruIndexValues.push_back(in2++);
        }
        while(allocated26>0 && size4>0)
        {
          m_dataInfo.push_back(*staInfoIt3);
          staInfoIt3++;
          allocated26-=1;
          size4-=1;
          ruAssigned.push_back(std::make_pair(type3,in2++)); 
          // ruAssigned.push_back(type3);
          // ruIndexValues.push_back(in2++);
        }
        for(int i=0;i<size2;i++){m_dataInfo.push_back(*staInfoIt1);staInfoIt1++;}
        for(int i=0;i<size3;i++){m_dataInfo.push_back(*staInfoIt2);staInfoIt2++;}
        for(int i=0;i<size4;i++){m_dataInfo.push_back(*staInfoIt3);staInfoIt3++;}

        NS_LOG_DEBUG("after "<<m_dataInfo.size());

         // nStations = nRusAssigned;
        return ruAssigned;
    }
  }
     
}
    




OfdmaManager::DlOfdmaInfo
RrOfdmaManager::ComputeDlOfdmaInfo (void)
{
  NS_LOG_FUNCTION (this);

  if (m_staInfo.empty ())
    {
      return DlOfdmaInfo ();
    }

  uint16_t bw = m_low->GetPhy ()->GetChannelWidth ();
//   uint16_t bw = m_bw;   // for TESTING only

  // compute how many stations can be granted an RU and the RU size
  std::size_t nRusAssigned = m_dataInfo.size ();
  std::vector<std::pair<HeRu::RuType,size_t>> ruAssigned = GetNumberAndTypeOfRus (bw, nRusAssigned,m_staInfo);
// int size1=m_dataInfo.size()-1;
  nRusAssigned=ruAssigned.size();
NS_LOG_DEBUG (nRusAssigned);

  DlOfdmaInfo dlOfdmaInfo;
  auto staInfoIt = m_dataInfo.begin (); // iterator over the list of candidate receivers

  for (std::size_t i = 0; i < nRusAssigned; i++)
    {
   NS_ASSERT (staInfoIt != m_dataInfo.end ());
      std::pair <Mac48Address,DlPerStaInfo>p(std::get<0>(*staInfoIt),std::get<2>(*staInfoIt));
      dlOfdmaInfo.staInfo.insert (p);
      NS_LOG_DEBUG("sizeonly "<<dlOfdmaInfo.staInfo.size());
      staInfoIt++;
    }
    

  // if not all the stations are assigned an RU, the first station to serve next
  // time is the first one that was not served this time
  if (nRusAssigned < m_dataInfo.size ())
    {
      NS_ASSERT (staInfoIt != m_dataInfo.end ());

      m_startStation = std::get<2>(*staInfoIt).aid;
      NS_LOG_DEBUG ("Next station to serve has AID=" << m_startStation<<" "<<m_dataInfo.size());
    }
 

  // set TX vector and TX params
  InitTxVectorAndParams (dlOfdmaInfo.staInfo, ruAssigned, m_dlMuAckSequence);
  dlOfdmaInfo.params = m_txParams;

  // assign RUs to stations

  // if (ruType == HeRu::RU_2x996_TONE)
  //   {
  //     HeRu::RuSpec ru = {true, ruType, 1};
  //     NS_LOG_DEBUG ("STA " << m_staInfo.front ().first << " assigned " << ru);
  //     m_txVector.SetRu (ru, m_staInfo.front ().second.aid);
  //   }
  // else
  //   {
      std::vector<bool> primary80MHzSet {true};

      if (bw == 160)
        {
          primary80MHzSet.push_back (false);
          bw = 80;
        }

      auto mapIt = dlOfdmaInfo.staInfo.begin ();
      NS_LOG_DEBUG("sizes "<< dlOfdmaInfo.staInfo.size() << " "<< nRusAssigned << " "<<ruAssigned.size());
      auto ruIt=ruAssigned.begin();
      // auto ruin1=ruIndexValues.begin();
      for (auto primary80MHz : primary80MHzSet)
        {
          for (std::size_t ruIndex = 1; ruIndex <= nRusAssigned; ruIndex++)
            {
              NS_ASSERT (mapIt != dlOfdmaInfo.staInfo.end ());
              HeRu::RuSpec ru = {primary80MHz, ruIt->first, ruIt->second};
              NS_LOG_DEBUG ("STA " << mapIt->first << " assigned " << ru);
              m_txVector.SetRu (ru, mapIt->second.aid);
              mapIt++;
              // ruin1++;
              ruIt++;
            }
        }
    // }
  dlOfdmaInfo.txVector = m_txVector;

  if (m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_MU_BAR
      || m_txParams.GetDlMuAckSequenceType () == DlMuAckSequenceType::DL_AGGREGATE_TF)
    {
      Ptr<WifiRemoteStationManager> stationManager = GetWifiRemoteStationManager ();
      // The Trigger Frame to be returned is built from the TX vector used for the DL MU PPDU
      // (i.e., responses will use the same set of RUs) and modified to ensure that responses
      // are sent at a rate not higher than MCS 5.
      dlOfdmaInfo.trigger = GetTriggerFrameHeader (dlOfdmaInfo.txVector, 5);
      dlOfdmaInfo.trigger.SetUlLength (m_low->CalculateUlLengthForBlockAcks (dlOfdmaInfo.trigger, m_txParams));
      SetTargetRssi (dlOfdmaInfo.trigger);
    }
    // ruIndexValues.clear();
    // ruAssigned.clear();
  return dlOfdmaInfo;
}

CtrlTriggerHeader
RrOfdmaManager::GetTriggerFrameHeader (WifiTxVector dlMuTxVector, uint8_t maxMcs)
{
  auto userInfoMap = dlMuTxVector.GetHeMuUserInfoMap ();

  for (auto& userInfo : userInfoMap)
    {
      uint8_t mcs = std::min (userInfo.second.mcs.GetMcsValue (), maxMcs);
      dlMuTxVector.SetHeMuUserInfo (userInfo.first, {userInfo.second.ru,
                                                     WifiPhy::GetHeMcs (mcs),
                                                     userInfo.second.nss});
    }

  return CtrlTriggerHeader (TriggerFrameType::MU_BAR_TRIGGER, dlMuTxVector);
}

OfdmaManager::UlOfdmaInfo
RrOfdmaManager::ComputeUlOfdmaInfo (void)
{
  CtrlTriggerHeader trigger (TriggerFrameType::BASIC_TRIGGER, m_txVector);
  trigger.SetUlLength (m_txVector.GetLength ());
  SetTargetRssi (trigger);

  UlOfdmaInfo ulOfdmaInfo;
  ulOfdmaInfo.params = m_txParams;
  ulOfdmaInfo.trigger = trigger;

  return ulOfdmaInfo;
}

} //namespace ns3
