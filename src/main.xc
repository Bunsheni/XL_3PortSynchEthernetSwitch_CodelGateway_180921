/*
 * XL_3PortSynchEthernetSwitch_EthernetDemo_180222.xc
 *
 *  Created on: 2018. 2. 22.
 *      Author: 22wow
 */


#include <xs1.h>
#include <platform.h>
#include <stdio.h>
#include <delay.h>
#include "ethernet.h"
#include "smi.h"
#include "ethernet_phy.h"

int main(void)
{
    ethernet_cfg_if i_cfg[ETHERNET_PORTS];
    switch_data_if i_tx[ETHERNET_PORTS][2];
    smi_if i_smi;
    switch_filter_if i_filter[3];

  par
  {
    on tile[0]: smi_kong(i_smi);
    on tile[0]: filter_kong(i_smi, i_cfg, i_filter);
    on tile[1]: mii0_kong(i_cfg[0], i_filter[0], i_tx[0][1], i_tx[2][0], i_tx[0][0], i_tx[2][1], 0);
    on tile[0]: mii1_kong(i_cfg[1], i_filter[1], i_tx[1][1], i_tx[0][0], i_tx[1][0], i_tx[0][1], 1);
//    on tile[0]: mii2_kong(i_cfg[2], i_filter[2], i_tx[2][1], i_tx[1][0], i_tx[2][0], i_tx[1][1], 0);
  }

  return 0;
}
