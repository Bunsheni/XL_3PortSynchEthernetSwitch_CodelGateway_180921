/*
 * ethernet_phy.xc
 *
 *  Created on: 2018. 2. 2.
 *      Author: 22wow
 */

// Here are the port definitions required by ethernet. This port assignment
// is for the KONG Borad of YoonLAB

#include "ethernet_phy.h"

port p_eth0_rxclk  = on tile[1]: XS1_PORT_1C;
port p_eth0_rxd    = on tile[1]: XS1_PORT_4F;
port p_eth0_txd    = on tile[1]: XS1_PORT_4D;
port p_eth0_rxdv   = on tile[1]: XS1_PORT_1D;
port p_eth0_txen   = on tile[1]: XS1_PORT_1O;
port p_eth0_txclk  = on tile[1]: XS1_PORT_1P;
port p_eth0_rxerr  = on tile[1]: XS1_PORT_1L;
port p_eth0_dummy  = on tile[1]: XS1_PORT_1M;
clock eth0_rxclk   = on tile[1]: XS1_CLKBLK_1;
clock eth0_txclk   = on tile[1]: XS1_CLKBLK_2;

port p_eth1_rxclk  = on tile[0]: XS1_PORT_1K;
port p_eth1_rxd    = on tile[0]: XS1_PORT_4E;
port p_eth1_txd    = on tile[0]: XS1_PORT_4F;
port p_eth1_rxdv   = on tile[0]: XS1_PORT_1L;
port p_eth1_txen   = on tile[0]: XS1_PORT_1I;
port p_eth1_txclk  = on tile[0]: XS1_PORT_1J;
port p_eth1_rxerr  = on tile[0]: XS1_PORT_1A;
port p_eth1_dummy  = on tile[0]: XS1_PORT_1B;
clock eth1_rxclk   = on tile[0]: XS1_CLKBLK_3;
clock eth1_txclk   = on tile[0]: XS1_CLKBLK_4;

port p_eth2_rxclk  = on tile[0]: XS1_PORT_1H;
port p_eth2_rxd    = on tile[0]: XS1_PORT_4A;
port p_eth2_txd    = on tile[0]: XS1_PORT_4C;
port p_eth2_rxdv   = on tile[0]: XS1_PORT_1G;
port p_eth2_txen   = on tile[0]: XS1_PORT_1F;
port p_eth2_txclk  = on tile[0]: XS1_PORT_1E;
port p_eth2_rxerr  = on tile[0]: XS1_PORT_1C;
port p_eth2_dummy  = on tile[0]: XS1_PORT_1D;
clock eth2_rxclk   = on tile[0]: XS1_CLKBLK_1;
clock eth2_txclk   = on tile[0]: XS1_CLKBLK_2;


port p_eth_reset  = on tile[0]: XS1_PORT_1M;
port p_smi_mdio   = on tile[0]: XS1_PORT_1P;
port p_smi_mdc    = on tile[0]: XS1_PORT_1O;

static unsigned char phy_address[3] = {0x2, 0x2, 0x9};
static unsigned char phy_embeded[3] = {1, 1, 0};

enum status_update_state_t {
  STATUS_UPDATE_IGNORING,
  STATUS_UPDATE_WAITING,
  STATUS_UPDATE_PENDING,
};

// data structure to keep track of link layer status.
typedef struct
{
  int status_update_state;
  int incoming_packet;
  int outgoing_packet;
  size_t num_etype_filters;
  uint16_t etype_filters[ETHERNET_MAX_ETHERTYPE_FILTERS];
} client_state_t;

#pragma unsafe arrays
unsigned do_forwarding(eth_global_forward_info_t table,
        char buf[packet_size], size_t packet_size, char nport)
{
    unsigned char res = 3;
    unsigned found = 0;
    char nPort = nport;
    int blankTableIndex = -1;
    unsigned *addr;

  unsigned *words = (unsigned *)buf;
  // Do all entries without an early exit so that it is always worst-case timing
  for (size_t i = 0;i < ETHERNET_MACADDR_FOWARDING_TABLE_SIZE; i++)
  {
      addr = (unsigned *)table[i].addr;
    if(table[i].result != 0)
    {
        //source address check
        if(((words[1]>>16&0xffff)|((words[2]<<16)&(0xffff0000))) == addr[0] && (words[2]>>16 & 0xffff) == (addr[1] & 0xffff))
        {
            table[i].result = FILTER_TIMER;
            found = 1;
        }
        //destination address check
        if(((words[0] == addr[0]) && ((words[1] & 0xffff) == (addr[1] & 0xffff))))
        {
            res = nPort != table[i].nPort ? table[i].nPort : -1;
        }
    }
    else if(blankTableIndex == -1)
    {
        blankTableIndex = i;
    }
  }
  //못찾았고 빈곳이 있으면 그곳에 저장
  if(!found && blankTableIndex != -1)
  {
      int j;
      // Found an empty entry, use it
      memcpy(table[blankTableIndex].addr, ((char*) words) + 6, 6);
      table[blankTableIndex].result = FILTER_TIMER;
      table[blankTableIndex].nPort = nPort;
#if(DEBUG)
      for(j=0;j<6;j++)
      {
          printf("%x",((char*)words)[6+j]);
      }
      printf(" is Registered with port%d at table %d\n", nPort, blankTableIndex);
#endif
  }
  return res;
}

void process_time_forward_table(eth_global_forward_info_t table)
{
  for (size_t i = 0; i < ETHERNET_MACADDR_FOWARDING_TABLE_SIZE; i++) {
      if(table[i].result > 0)
      {
          table[i].result--;
#if(DEBUG)
          if(table[i].result == 0)
              printf("Filter %d is deleted\n", i);
#endif
      }
  }
}

void smi_kong(server interface smi_if i_smi)
{
    p_eth_reset <: 1;
    smi(i_smi, p_smi_mdio, p_smi_mdc);
}

void mii0_kong(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1, char gate)
{
    printf("MII 0 Run\n");
    mii_ethernet_switch_mac(i_cfg, i_filter,
         i_rx0, i_rx1, i_tx0, i_tx1,
         p_eth0_rxclk, p_eth0_rxerr,
         p_eth0_rxd, p_eth0_rxdv,
         p_eth0_txclk, p_eth0_txen, p_eth0_txd,
         p_eth0_dummy,
         eth0_rxclk, eth0_txclk,
         ETH_RX_BUFFER_SIZE_WORDS, 0, gate);
}

void mii1_kong(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1, char gate)
{
    printf("MII 1 Run\n");
    mii_ethernet_switch_mac(i_cfg, i_filter,
        i_rx0, i_rx1, i_tx0, i_tx1,
        p_eth1_rxclk, p_eth1_rxerr,
        p_eth1_rxd, p_eth1_rxdv,
        p_eth1_txclk, p_eth1_txen, p_eth1_txd,
        p_eth1_dummy,
        eth1_rxclk, eth1_txclk,
        ETH_RX_BUFFER_SIZE_WORDS, 1, gate);
}

void mii2_kong(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1, char gate)
{
    printf("MII 2 Run\n");
    mii_ethernet_switch_mac(i_cfg, i_filter,
        i_rx0, i_rx1, i_tx0, i_tx1,
        p_eth2_rxclk, p_eth2_rxerr,
        p_eth2_rxd, p_eth2_rxdv,
        p_eth2_txclk, p_eth2_txen, p_eth2_txd,
        p_eth2_dummy,
        eth2_rxclk, eth2_txclk,
        ETH_RX_BUFFER_SIZE_WORDS, 2, gate);
}

unsigned smi_phy_is_powered_down_n(client smi_if smi, int i)
{
  return ((smi.read_reg(phy_address[i], BASIC_CONTROL_REG) >> BASIC_CONTROL_POWER_DOWN_BIT) & 1);
}

ethernet_link_state_t smi_get_link_state_n(client interface smi_if smi, int i)
{
    unsigned link_up = ((smi.read_reg(phy_address[i], BASIC_STATUS_REG) >> BASIC_STATUS_LINK_BIT) & 1);
    return link_up ? ETHERNET_LINK_UP : ETHERNET_LINK_DOWN;;
}

ethernet_speed_t smi_get_link_speed_n(client smi_if smi, int i)
{
    if ((smi.read_reg(phy_address[i], 0x1F) >> 2) & 1) {
      return LINK_10_MBPS_FULL_DUPLEX;
    }
    else {
      return LINK_100_MBPS_FULL_DUPLEX;
    }
}

void smi_configure_n(client smi_if smi, int i, ethernet_speed_t speed_mbps, smi_autoneg_t auto_neg)
{
  if (speed_mbps != LINK_10_MBPS_FULL_DUPLEX &&
      speed_mbps != LINK_100_MBPS_FULL_DUPLEX &&
      speed_mbps != LINK_1000_MBPS_FULL_DUPLEX) {
    printf("Invalid Ethernet speed provided, must be 10, 100 or 1000");
  }

  if (auto_neg == SMI_ENABLE_AUTONEG) {
    uint16_t auto_neg_advert_100_reg = smi.read_reg(phy_address[i], AUTONEG_ADVERT_REG);
    uint16_t gige_control_reg = smi.read_reg(phy_address[i], GIGE_CONTROL_REG);

    // Clear bits [9:5]
    auto_neg_advert_100_reg &= 0xfc1f;
    // Clear bits [9:8]
    gige_control_reg &= 0xfcff;

    switch (speed_mbps) {
    #pragma fallthrough
      case LINK_1000_MBPS_FULL_DUPLEX: gige_control_reg |= 1 << AUTONEG_ADVERT_1000BASE_T_FULL_DUPLEX;
    #pragma fallthrough
      case LINK_100_MBPS_FULL_DUPLEX: auto_neg_advert_100_reg |= 1 << AUTONEG_ADVERT_100BASE_TX_FULL_DUPLEX;
      case LINK_10_MBPS_FULL_DUPLEX: auto_neg_advert_100_reg |= 1 << AUTONEG_ADVERT_10BASE_TX_FULL_DUPLEX; break;
      default: __builtin_unreachable(); break;
    }

    // Write back
    smi.write_reg(phy_address[i], AUTONEG_ADVERT_REG, auto_neg_advert_100_reg);
    smi.write_reg(phy_address[i], GIGE_CONTROL_REG, gige_control_reg);
  }

  uint16_t basic_control = smi.read_reg(phy_address[i], BASIC_CONTROL_REG);
  if (auto_neg == SMI_ENABLE_AUTONEG) {
    // set autoneg bit
    basic_control |= 1 << BASIC_CONTROL_AUTONEG_EN_BIT;
    smi.write_reg(phy_address[i], BASIC_CONTROL_REG, basic_control);
    // restart autoneg
    basic_control |= 1 << BASIC_CONTROL_RESTART_AUTONEG_BIT;
  }
  else {
    // set duplex mode, clear autoneg and speed
    basic_control |= 1 << BASIC_CONTROL_FULL_DUPLEX_BIT;
    basic_control &= ~( (1 << BASIC_CONTROL_AUTONEG_EN_BIT)|
                          (1 << BASIC_CONTROL_100_MBPS_BIT)|
                         (1 << BASIC_CONTROL_1000_MBPS_BIT));

    if (speed_mbps == LINK_100_MBPS_FULL_DUPLEX) {
      basic_control |= 1 << BASIC_CONTROL_100_MBPS_BIT;
    } else if (speed_mbps == LINK_1000_MBPS_FULL_DUPLEX) {
      printf("Autonegotiation cannot be disabled in 1000 Mbps mode");
    }
  }
  smi.write_reg(phy_address[i], BASIC_CONTROL_REG, basic_control);
}

void ethernet_init_forward_table(eth_global_forward_info_t table)
{
  for (size_t i = 0; i < ETHERNET_MACADDR_FOWARDING_TABLE_SIZE; i++) {
    memset(table[i].addr, 0, sizeof table[i].addr);
    table[i].result = 0;
    table[i].appdata = 0;
  }
}

void filter_kong(client interface smi_if i_smi, client ethernet_cfg_if i_cfg[3], client switch_filter_if i_filter[3])
{

#if(DEBUG)
//    int i, j, k;
//    for(k=0;k<3;k++)
//    {
//        if(phy_embeded[k])
//        {
//            printf("Port%d PHY Address: 0x%.4x\n",k, phy_address[k]);
//            for(i=0;i<32;i++)
//                {
//                j = i_smi.read_reg(phy_address[k], i);
//                printf("reg%d: 0x%.4x\n",i,j);
//                delay_millisecond(100);
//                }
//        }
//    }
#endif

    const int link_poll_period_ms = 1000;
    eth_global_forward_info_t forward_info;
    ethernet_packet_info_t packet_info;
    ethernet_link_state_t link_state[ETHERNET_PORTS];
    ethernet_speed_t link_speed[ETHERNET_PORTS];
    unsigned char phy_init_state[ETHERNET_PORTS];
    timer tmr;
    int t, index;
    char res;
    unsigned char buf[12];

    ethernet_init_forward_table(forward_info);

    for(index = 0; index < ETHERNET_PORTS; index++)
    {
        phy_init_state[index] = 0;
        link_state[index]  = ETHERNET_LINK_DOWN;
        link_speed[index]  = ETH_SPEED_INIT;
    }

    tmr :> t;
    while(1)
    {
      select
      {
      case tmr when timerafter(t) :> t:
        for(index = 0; index<3; index++)
        {
          if (phy_init_state[index] == 0 && phy_embeded[index])
              if(!smi_phy_is_powered_down_n(i_smi, index))
          {
              smi_configure_n(i_smi, index, LINK_100_MBPS_FULL_DUPLEX, SMI_ENABLE_AUTONEG);
              phy_init_state[index] = 1;
              printf("PHY%d on\n", index);
          }

          if(phy_init_state[index] == 1)
          {
              ethernet_link_state_t new_state = smi_get_link_state_n(i_smi, index);
              if (new_state != link_state[index]){
                  if (new_state == ETHERNET_LINK_UP) {
                      printf("Port%d link_state: UP\n", index);
                      link_speed[index] = 1;//smi_get_link_speed_n(i_smi, index)
                      if(link_speed[index] == 1)
                          printf("Port%d link_speed: 100Mb\n", index);
                      else
                          printf("Port%d link_speed: 10Mb\n", index);
                  }else{
                      printf("Port%d link_state: DOWN\n", index);
                  }
                  link_state[index] = new_state;
                  i_cfg[index].set_link_state(0, new_state, link_speed[index]);
              }
          }
        }
        process_time_forward_table(forward_info);
        t += link_poll_period_ms * XS1_TIMER_KHZ;
        break;

      case i_filter[int i].filtering_reqeust():
        i_filter[i].get_packet(packet_info, buf, 12);
        //printf("Get Packet %d at %d\n", packet_info.len, i);
        res = do_forwarding(forward_info, buf, 12, i);
        //printf("%x\n",res);
        if(res == 3)
        {
            if(i == 0)
            {
                if(link_state[1] && link_state[2])
                    res = 2;
                else if(link_state[1])
                    res = 0;
                else if(link_state[2])
                    res = 1;
                else
                    res = -1;
            }
            else if(i == 1)
            {
                if(link_state[0] && link_state[2])
                    res = 2;
                else if(link_state[2])
                    res = 0;
                else if(link_state[0])
                    res = 1;
                else
                    res = -1;
            }
            else if(i == 2)
            {
                if(link_state[0] && link_state[1])
                    res = 2;
                else if(link_state[0])
                    res = 0;
                else if(link_state[1])
                    res = 1;
                else
                    res = -1;
            }
        }
        else if(res == 2 && link_state[2])
        {
            if(i == 0)
                res = 1;
            else if(i == 1)
                res = 0;
            else
                res = -1;
        }
        else if(res == 1 && link_state[1])
        {
            if(i == 2)
                res = 1;
            else if(i == 0)
                res = 0;
            else
                res = -1;
        }
        else if(res == 0 && link_state[0])
        {
            if(i == 2)
                res = 0;
            else if(i == 1)
                res = 1;
            else
                res = -1;
        }
        else res = -1;
        //printf("%x\n",res);
        i_filter[i].filter_response(res);
        break;
      }
    }
}


unsigned control_law(unsigned t, unsigned count, unsigned interval)
{
    return t + interval/sqrt(count);
}

static void mii_ethernet_switch_aux(client mii_if i_mii,
                             server ethernet_cfg_if i_cfg,
                             server switch_filter_if i_filter,
                             server switch_data_if i_rx0,
                             server switch_data_if i_rx1,
                             client switch_data_if i_tx0,
                             client switch_data_if i_tx1,
                             char nPort, char gate)
{
  unsafe {
    uint8_t mac_address[MACADDR_NUM_BYTES] = {0};
    ethernet_link_state_t link_status = ETHERNET_LINK_DOWN;
    ethernet_speed_t link_speed = LINK_100_MBPS_FULL_DUPLEX;
    client_state_t client_state;
    int txbuf[(ETHERNET_MAX_PACKET_SIZE+3)/4];
    mii_info_t mii_info;
    int incoming_nbytes;
    int incoming_timestamp;
    int incoming_tcount;
    int outgoing_nbytes;
    int outgoing_timestamp;
    int outgoing_tcount;
    int outgoing_sourceport;
    unsigned incoming_appdata;
    int * unsafe incoming_data = null;
    eth_global_forward_info_t forward_info;

    mii_info = i_mii.init();

    client_state.status_update_state = STATUS_UPDATE_IGNORING;
    client_state.incoming_packet = 0;
    client_state.outgoing_packet = 0;
    client_state.num_etype_filters = 0;

    ethernet_init_forward_table(forward_info);

    //for queueing init
    timer queueing_timer;
    queue_t *data_queue_p;
    packet_t *packet_p;
    unsigned time_gate_open = GATEOPENUS;
    unsigned time_gate_close = GATECLOSEUS;
    unsigned time;
    unsigned period;
    packet_t *packet;
    char *data;
    unsigned bytelen;
    char gate_open = 0;
    data_queue_p = (queue_t *)(init_queue(1000));
    queue_t a = *data_queue_p;
    queueing_timer:>time;
    char nofiltering = 1;

    //
    unsigned target = USTIMERCOUNT * TARGET;
    unsigned interval = USTIMERCOUNT * INTERVAL;
    unsigned MTU = 10;
    char drop_state = 0;
    unsigned numQueueDropped = 0;
    unsigned count = 0;
    unsigned last_count = 0;
    unsigned next_drop_time = 0;
    unsigned delta = 0;

    while (1) {

        if(gate)
        {
            if(gate_open && data_queue_p->length > 0 && client_state.outgoing_packet == 0)
            {
                unsigned length = data_queue_p->head->next->length;
                double length4 = ((length*8+3)/100);
//                printf("%8.fus",length4);
                unsigned duration = length4 * USTIMERCOUNT;
                unsigned ctime;
                queueing_timer :> ctime;
//                if(ctime + duration < time)
                {
                    packet_p = (packet_t *)(dequeue(data_queue_p));

                    unsigned deqtime;
                    unsigned enqtime;
                    unsigned sojourn_time;
                    unsigned queue_length = data_queue_p->length;
                    queueing_timer:> deqtime;
                    enqtime = packet_p->timestamp;
                    sojourn_time = deqtime - enqtime;

                    if(drop_state)
                    {
                        if(sojourn_time < target || queue_length < MTU)
                        {
                            drop_state = 0;
                        }
                        else
                        {
                            while(deqtime >= next_drop_time && drop_state)
                            {
                                numQueueDropped++;

                                free(packet_p->data);
                                free(packet_p);//drop
//                                printf("packet was dropped. total:%d\n", numQueueDropped);

                                packet_p = (packet_t *)(dequeue(data_queue_p));

                                enqtime = packet_p->timestamp;
                                sojourn_time = deqtime - enqtime;

                                count++;

                                if(sojourn_time < target || data_queue_p->length < MTU)
                                {
                                    drop_state = 0;
                                }
                                else
                                {
                                    next_drop_time = control_law(next_drop_time, count, interval);
                                }
                            }
                        }
                    }
                    else if(sojourn_time >= target && data_queue_p->length >= MTU)
                    {
                        //drop
                        numQueueDropped++;

                        free(packet_p->data);
                        free(packet_p);//drop

//                        printf("packet was dropped. total:%d\n", numQueueDropped);
                        packet_p = 0;

                        drop_state = 1;
                        delta  = count - last_count;
                        count = 1;

                        if((delta > 1) && (deqtime - next_drop_time < 16 * interval))
                            count = delta;
                        next_drop_time = control_law(deqtime, count, interval);
                        last_count = count;
                    }

                    if(packet_p && packet_p->data && packet_p->length)
                    {
                        data = (char *)(packet_p->data);
                        bytelen = (unsigned)(packet_p->length);
                        i_mii.send_packet(data, bytelen);
    //                    printf("dequeue length: %d\n", data_queue_p->length);
                        client_state.outgoing_packet = 1;
                    }
                }
            }
            if(client_state.outgoing_packet == 1)
            {
                select {
                case mii_packet_sent(mii_info):
                    client_state.outgoing_packet = 0;
                    free(data);
                  break;
                }
            }
        }
        else
        {
            if(client_state.outgoing_packet == 1)
            {
                i_mii.send_packet(txbuf, outgoing_nbytes);
                client_state.outgoing_packet = 2;
            }

            if(client_state.outgoing_packet == 2)
            {
                select {
                case mii_packet_sent(mii_info):
                    client_state.outgoing_packet = 0;
                  break;
                }
            }

        }


      select {


      case mii_incoming_packet(mii_info):
        int * unsafe data;
        int nbytes;
        unsigned timestamp;
        {data, nbytes, timestamp} = i_mii.get_incoming_packet();

        if(nofiltering)
        {
            i_tx0.forward_packet((char *)data, nbytes, timestamp);
            i_tx1.forward_packet((char *)data, nbytes, timestamp);
            if (data != null) {
              i_mii.release_packet(data);
              data = null;
            }
        }
        else
        {

        if ((incoming_data && data)){
          // Can only handle one packet at a time at this level
          i_mii.release_packet(data);
          printf(".");
        }
        else if (data) {
          unsigned appdata;
          incoming_timestamp = timestamp;
          incoming_nbytes = nbytes;
          incoming_data = data;
          incoming_tcount = 0;

          int *unsafe p_len_type = (int *unsafe) &data[3];
          uint16_t len_type = (uint16_t) NTOH_U16_ALIGNED(p_len_type);
          unsigned header_len = 14;
          if (len_type == 0x8100) {
            header_len += 4;
            p_len_type = (int *unsafe) &data[4];
            len_type = (uint16_t) NTOH_U16_ALIGNED(p_len_type);
          }
          const unsigned rx_data_len = nbytes - header_len;

          if ((len_type < 1536) && (len_type > rx_data_len) || link_status == ETHERNET_LINK_DOWN) {
            // Invalid len_type field, will fall out and free the buffer below
              i_mii.release_packet(incoming_data);
              incoming_data = null;
          }
          else {
              client_state.incoming_packet = 1;
              i_filter.filtering_reqeust();
          }
        }
        }
        break;
      case i_filter.get_packet(ethernet_packet_info_t &desc, char data[n], unsigned n):
        if (client_state.status_update_state == STATUS_UPDATE_PENDING) {
          data[0] = link_status;
          data[1] = link_speed;
          desc.type = ETH_IF_STATUS;
          desc.src_ifnum = 0;
          desc.timestamp = 0;
          desc.len = 2;
          desc.filter_data = 0;
          client_state.status_update_state = STATUS_UPDATE_WAITING;
        } else if (client_state.incoming_packet) {
          ethernet_packet_info_t info;
          info.type = ETH_DATA;
          info.timestamp = incoming_timestamp;
          info.src_ifnum = 0;
          info.filter_data = incoming_appdata;
          info.len = incoming_nbytes;
          memcpy(&desc, &info, sizeof(info));
          memcpy(data, incoming_data, n);
        } else {
          desc.type = ETH_NO_DATA;
        }
        break;
      case i_filter.filter_response(char ports):
          if(ports == 2)
          {
              i_tx0.forward_packet((char *)incoming_data, incoming_nbytes, incoming_timestamp);
              i_tx1.forward_packet((char *)incoming_data, incoming_nbytes, incoming_timestamp);
          }
          else if(ports == 1)
          {
              i_tx1.forward_packet((char *)incoming_data, incoming_nbytes, incoming_timestamp);
          }
          else if(ports == 0)
          {
              i_tx0.forward_packet((char *)incoming_data, incoming_nbytes, incoming_timestamp);
          }
          if (incoming_data != null) {
            i_mii.release_packet(incoming_data);
            incoming_data = null;
            client_state.incoming_packet = 0;
          }
          break;
        break;

      case i_rx0.forward_packet(char data[n], unsigned n, int request_timestamp):
        if(link_status)
        {
            if(gate)
            {
                unsigned timestamp;
                queueing_timer :> timestamp;
                packet = (packet_t *)(malloc(sizeof(packet_t)));
//                    char *buf = (char *)malloc(sizeof(char)*n);
                char *buf = (char *)malloc(ETHERNET_MAX_PACKET_SIZE);
                memcpy(buf, data, n);
                packet->data = (char *)buf;
                packet->length = n;
                packet->timestamp = timestamp;
                enqueue(data_queue_p, packet);
//                printf("in data length: %d\n", packet->length);
//                if(data_queue_p->length > 1)
//                printf("queue length: %d\n", data_queue_p->length);

            }
            else
            {
                memcpy(txbuf, data, n);
                client_state.outgoing_packet = 1;
                outgoing_nbytes = n;
                outgoing_timestamp = request_timestamp;
                outgoing_sourceport = nPort - 1;
            }
        }
        break;
      case i_rx1.forward_packet(char data[n], unsigned n, int request_timestamp):

              if(link_status)
              {
                  if(gate)
                  {
                      unsigned timestamp;
                      queueing_timer :> timestamp;
                      packet = (packet_t *)(malloc(sizeof(packet_t)));
          //                    char *buf = (char *)malloc(sizeof(char)*n);
                      char *buf = (char *)malloc(ETHERNET_MAX_PACKET_SIZE);
                      memcpy(buf, data, n);
                      packet->data = (char *)buf;
                      packet->length = n;
                      packet->timestamp = timestamp;
                      enqueue(data_queue_p, packet);
//                      printf("in data length: %d\n", packet->length);
//                      printf("queue length: %d\n", data_queue_p->length);

                  }
                  else
                  {
                      memcpy(txbuf, data, n);
                      client_state.outgoing_packet = 1;
                      outgoing_nbytes = n;
                      outgoing_timestamp = request_timestamp;
                      outgoing_sourceport = nPort - 1;
                  }
              }
        break;
      case i_cfg.get_macaddr(size_t ifnum, uint8_t r_mac_address[MACADDR_NUM_BYTES]):
        memcpy(r_mac_address, mac_address, sizeof mac_address);
        break;
      case i_cfg.set_macaddr(size_t ifnum, uint8_t r_mac_address[MACADDR_NUM_BYTES]):
        memcpy(mac_address, r_mac_address, sizeof r_mac_address);
        break;
      case i_cfg.add_macaddr_filter(size_t client_num, int is_hp,
                                           ethernet_macaddr_filter_t entry) ->
                                             ethernet_macaddr_filter_result_t result:
        break;

      case i_cfg.del_macaddr_filter(size_t client_num, int is_hp,
                                           ethernet_macaddr_filter_t entry):
        break;

      case i_cfg.del_all_macaddr_filters(size_t client_num, int is_hp):
        break;

      case i_cfg.add_ethertype_filter(size_t client_num, uint16_t ethertype):
        break;

      case i_cfg.del_ethertype_filter(size_t client_num, uint16_t ethertype):
        break;

      case i_cfg.get_tile_id_and_timer_value(unsigned &tile_id, unsigned &time_on_tile): {
        fail("Outgoing timestamps are not supported in standard MII Ethernet MAC");
        break;
      }
      case i_cfg.set_egress_qav_idle_slope(size_t ifnum, unsigned slope):
        fail("Shaper not supported in standard MII Ethernet MAC");
        break;

      case i_cfg.set_ingress_timestamp_latency(size_t ifnum, ethernet_speed_t speed, unsigned value): {
        fail("Timestamp correction not supported in standard MII Ethernet MAC");
        break;
      }
      case i_cfg.set_egress_timestamp_latency(size_t ifnum, ethernet_speed_t speed, unsigned value): {
        fail("Timestamp correction not supported in standard MII Ethernet MAC");
        break;
      }
      case i_cfg.enable_strip_vlan_tag(size_t client_num):
        fail("VLAN tag stripping not supported in standard MII Ethernet MAC");
        break;
      case i_cfg.disable_strip_vlan_tag(size_t client_num):
        fail("VLAN tag stripping not supported in standard MII Ethernet MAC");
        break;
      case i_cfg.enable_link_status_notification(size_t client_num):
        client_state.status_update_state = STATUS_UPDATE_WAITING;
        break;

      case i_cfg.disable_link_status_notification(size_t client_num):
        client_state.status_update_state = STATUS_UPDATE_IGNORING;
        break;

      case i_cfg.set_link_state(int ifnum, ethernet_link_state_t status, ethernet_speed_t speed):
        if (link_status != status) {
          link_status = status;
          link_speed = speed;
          if (client_state.status_update_state == STATUS_UPDATE_WAITING) {
            client_state.status_update_state = STATUS_UPDATE_PENDING;
            i_filter.filtering_reqeust();
          }
        }
        break;
      case queueing_timer when timerafter(time) :> void:
        if(gate_open)
        {
          period = time_gate_close;
          gate_open = 0;
//          printf("Gate Close\n");
        }
        else
        {
          period = time_gate_open;
          gate_open = 1;
//          printf("Gate Open\n");
        }
        time += USTIMERCOUNT * period;
        break;

      }
    }
  }
}

void mii_ethernet_switch_mac(server ethernet_cfg_if i_cfg,
                    server switch_filter_if i_filter,
                      server switch_data_if i_rx0,
                      server switch_data_if i_rx1,
                      client switch_data_if i_tx0,
                      client switch_data_if i_tx1,
                      in port p_rxclk, in port p_rxer, in port p_rxd,
                      in port p_rxdv,
                      in port p_txclk, out port p_txen, out port p_txd,
                      port p_timing,
                      clock rxclk,
                      clock txclk,
                      static const unsigned double_rx_bufsize_words, char nPort, char gate)

{
  interface mii_if i_mii;
  par {
      mii(i_mii, p_rxclk, p_rxer, p_rxd, p_rxdv, p_txclk,
          p_txen, p_txd, p_timing,
          rxclk, txclk, double_rx_bufsize_words);
      mii_ethernet_switch_aux(i_mii, i_cfg, i_filter,
                   i_rx0, i_rx1, i_tx0, i_tx1, nPort, gate);

  }
}
