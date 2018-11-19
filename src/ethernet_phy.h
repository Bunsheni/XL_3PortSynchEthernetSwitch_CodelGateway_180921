/*
 * ethernet.h
 *
 *  Created on: 2018. 2. 12.
 *      Author: 22wow
 */

#include <xs1.h>
#include <platform.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "ethernet.h"
#include "smi.h"
#include "mii.h"
#include "print.h"
#include "xassert.h"
#include "print.h"
#include "ntoh.h"
#include "stddef.h"
#include "macaddr_filter.h"
#include "queue.h"
#include "delay.h"

#ifndef ETHERNET_H_
#define ETHERNET_H_

#define DEBUG 1
#define IP101_ALF_ADDRESS 0x2
#define IP101_GR_ADDRESS 0x8  //0x8 or 0x9

#define ETH_RX_BUFFER_SIZE_WORDS 5000
#define ETH_SPEED_INIT LINK_100_MBPS_FULL_DUPLEX
#define FILTER_TIMER 30

#define GATEOPENUS 1000
#define GATECLOSEUS 9000
#define TARGET 5000
#define INTERVAL 10000

// This is the worst-case that can be supported for line-rate processing
#define ETHERNET_MACADDR_FOWARDING_TABLE_SIZE 100

typedef struct eth_global_forward_entry_t {
  char addr[MACADDR_NUM_BYTES];
  unsigned result;
  unsigned appdata;
  unsigned char nPort;   //added by son
} eth_global_forward_entry_t;

typedef eth_global_forward_entry_t eth_global_forward_info_t[ETHERNET_MACADDR_FOWARDING_TABLE_SIZE];

// An enum to manage the array of connections from the ethernet component
// to its clients.
enum eth_clients {
  ETH_TO_SWITCH,
  NUM_ETH_CLIENTS
};
enum cfg_clients {
  CFG_TO_SWITCH,
  NUM_CFG_CLIENTS
};

enum eth_ports {
  ETHERNET_PORT_0,
  ETHERNET_PORT_1,
  ETHERNET_PORT_2,
  ETHERNET_PORTS
};

typedef interface filter_reqest_if {
    [[notification]] slave void filtering_reqeust();

    [[clears_notification]] void get_packet(ethernet_packet_info_t &desc,
                                            char packet[n],
                                            unsigned n);
   void filter_response(char ports);

} switch_filter_if;

typedef interface switch_data_if {
    void forward_packet(char packet[n], unsigned n,
                               int request_timestamp);
} switch_data_if;


void process_time_filter_table(eth_global_filter_info_t table);

unsigned do_filtering(eth_global_filter_info_t table,
                               char buf[packet_size_],
                               size_t packet_size_, char nport);

void kong_filter(client interface smi_if i_smi, client ethernet_cfg_if i_cfg[3], client switch_filter_if i_filter[3]);


void smi_kong(server interface smi_if i_smi);
void mii0_kong(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1, char gate);
void mii1_kong(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1, char gate);
void mii2_kong(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1, char gate);
void filter_kong(client interface smi_if i_smi,
        client ethernet_cfg_if i_cfg[3],
        client switch_filter_if i_filter[3]);
void mii_ethernet_switch_mac(server ethernet_cfg_if i_cfg,
        server switch_filter_if i_filter,
        server switch_data_if i_rx0,
        server switch_data_if i_rx1,
        client switch_data_if i_tx0,
        client switch_data_if i_tx1,
        in port p_rxclk, in port p_rxer, in port p_rxd, in port p_rxdv,
        in port p_txclk, out port p_txen, out port p_txd,
        port p_timing,
        clock rxclk,
        clock txclk,
        static const unsigned rx_bufsize_words, char nPort, char gate);
#endif /* ETHERNET_H_ */
