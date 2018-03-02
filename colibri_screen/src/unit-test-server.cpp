/*
 * Copyright © 2008-2010 Stéphane Raimbault <stephane.raimbault@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

 
#include "ros/ros.h"
#include "colibri_battery/Battery.h"
#include "colibri_screen/Ipc.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>
#include <memory>
#include "unit-test.h"

enum {
    TCP,
    TCP_PI,
    RTU
};

int rate_battery = 50;
int ctrl_mode = 0;

void battery_info_Callback(const colibri_battery::Battery::ConstPtr& msg)
{
//  ROS_INFO("I heard: [%d]", msg->Chg_state);
  rate_battery = msg->SOC;
}

void ipc_info_Callback(const colibri_screen::Ipc::ConstPtr& msg)
{
  ctrl_mode = msg->ctrl_mode;
}


int main(int argc, char*argv[])
{
    int socket;
    modbus_t *ctx;
    modbus_mapping_t *mb_mapping;
    int rc;
    int i;
	uint8_t j = 1;
	uint8_t k = 1;
	int use_backend;
    uint8_t *query;
    int header_length;	
	
    use_backend = RTU;
      
    ctx = modbus_new_rtu("/dev/ttyS2", 9600, 'E', 8, 1);
        modbus_set_slave(ctx, SERVER_ID);
        query = (uint8_t *)malloc(MODBUS_RTU_MAX_ADU_LENGTH);
    
    header_length = modbus_get_header_length(ctx);

    modbus_set_debug(ctx, TRUE);

    mb_mapping = modbus_mapping_new(
        UT_BITS_ADDRESS + UT_BITS_NB,
        UT_INPUT_BITS_ADDRESS + UT_INPUT_BITS_NB,
        UT_REGISTERS_ADDRESS + UT_REGISTERS_NB,
        UT_INPUT_REGISTERS_ADDRESS + UT_INPUT_REGISTERS_NB);
    if (mb_mapping == NULL) {
        fprintf(stderr, "Failed to allocate the mapping: %s\n",
                modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }

    /* Examples from PI_MODBUS_300.pdf.
       Only the read-only input values are assigned. */

    /** INPUT STATUS **/
    modbus_set_bits_from_bytes(mb_mapping->tab_input_bits,
                               UT_INPUT_BITS_ADDRESS, UT_INPUT_BITS_NB,
                               UT_INPUT_BITS_TAB);

    /** INPUT REGISTERS **/
    for (i=0; i < UT_INPUT_REGISTERS_NB; i++) {
        mb_mapping->tab_input_registers[UT_INPUT_REGISTERS_ADDRESS+i] =
            UT_INPUT_REGISTERS_TAB[i];;
    }

    if (use_backend == TCP) {
        socket = modbus_tcp_listen(ctx, 1);
        modbus_tcp_accept(ctx, &socket);
    } else if (use_backend == TCP_PI) {
        socket = modbus_tcp_pi_listen(ctx, 1);
        modbus_tcp_pi_accept(ctx, &socket);
    } else {
        rc = modbus_connect(ctx);
        if (rc == -1) {
            fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
            modbus_free(ctx);
            return -1;
        }
    }

	ros::init(argc, argv, "touch_screen");
	ros::NodeHandle n;
	ros::Subscriber sub4bat_= n.subscribe("battery_info", 30, battery_info_Callback);
	ros::Subscriber sub4ipc = n.subscribe("ipc_info", 30, ipc_info_Callback);
	 
	ros::Rate loop_rate(5);

	
	
	while (ros::ok())
    {

		mb_mapping->tab_registers[0] = ctrl_mode;
		mb_mapping->tab_registers[1] = rate_battery;
		mb_mapping->tab_registers[2] = 0x00+j;
		
		j++;
		if (j>100){
            j=0;
		}
		
		
		k++;
		if (k>10){
            k=0;
		}

        rc = modbus_receive(ctx, query);
        if (rc == -1) {
            /* Connection closed by the client or error */
            break;
        }

        /* Read holding registers */
        if (query[header_length] == 0x03) {
            if (MODBUS_GET_INT16_FROM_INT8(query, header_length + 3)
                == UT_REGISTERS_NB_SPECIAL) {
                printf("Set an incorrect number of values\n");
                MODBUS_SET_INT16_TO_INT8(query, header_length + 3,
                                         UT_REGISTERS_NB_SPECIAL - 1);
            } else if (MODBUS_GET_INT16_FROM_INT8(query, header_length + 1)
                == UT_REGISTERS_ADDRESS_SPECIAL) {
                printf("Reply to this special register address by an exception\n");
                modbus_reply_exception(ctx, query,
                                       MODBUS_EXCEPTION_SLAVE_OR_SERVER_BUSY);
                continue;
            }
        }

        rc = modbus_reply(ctx, query, rc, mb_mapping);
        if (rc == -1) {
            break;
        }

		ros::spinOnce();
        loop_rate.sleep();
    }

    printf("Quit the loop: %s\n", modbus_strerror(errno));

    if (use_backend == TCP) {
        close(socket);
    }
    modbus_mapping_free(mb_mapping);
    free(query);
    modbus_free(ctx);

    return 0;
}
