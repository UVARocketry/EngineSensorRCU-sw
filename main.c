/*
 * File:   main.c
 * Author: henry
 *
 * Created on July 28, 2021, 6:10 PM
 */

#include <xc.h> //compiler

//standard C libraries
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

//code common to all RCUs
#include "libpicutil/config_mem.h" //configuration fuses for chip options
#include "libpicutil/time.h"
#include "libpicutil/uart_debug.h"
#include "libpicutil/leds.h"
#include "libcan/can_messages.h"
#include "libcan/can.h"
#include "libpicutil/adc.h"

//code specific to this RCU

#define _XTAL_FREQ 64000000UL //needed for delays to work, but not much else

const uint8_t RCU_ID_LOCAL = RCU_ID_ENGINE_SENSOR_RCU;

/*
 * 
 */

uint16_t last_2Hz_time, last_10Hz_time, last_200Hz_time, last_hb_rx_time;

uint8_t connected;

struct Heartbeat_t hb;
struct EngineSensorsA_t sensorsA;
struct EngineSensorsB_t sensorsB;

char msg[64];

void on_can_rx(const struct can_msg_t *msg);

uint8_t shutdown_req_flag;

void transmit_sensors() {
    //read all ADC channels
    sensorsA.fuel_tank_press_raw = adc_read(0); //RA0
    sensorsA.chamber_press_raw = adc_read(1); //RA1
    sensorsA.fuel_inj_press_raw = adc_read(2); //RA2
    sensorsA.ox_inj_press_raw = adc_read(3); //RA3
    sensorsB.thrust_raw = adc_read(4); //RA4
    sensorsB.aux_1_raw = adc_read(5); //RA5
    sensorsB.aux_2_raw = adc_read(6); //RA6
    sensorsB.aux_3_raw = adc_read(7); //RA7
    //possible to use any pin for ADC channel! expand as required.

    //send as two packets on the bus
    can_txq_push(ID_ENGINE_SENSORS_A, CAN_CONVERT(sensorsA));
    can_txq_push(ID_ENGINE_SENSORS_B, CAN_CONVERT(sensorsB));
}

int main() {

    INTCON0bits.GIE = 1; //enable global interrupts

    time_init();
    leds_init();
    uart_init();
    adc_init();
    can_rx_callback = &on_can_rx;
    can_init();

    while (1) {
        uint16_t ms = time_millis(); //minimize calls to time_millis() 
        //since interrupts get disabled briefly
        if (one_kHz_flag) { //1kHz
            one_kHz_flag = 0;
            if (connected) { //only do high-rate ADC conversion if the main RCU is online
                transmit_sensors();
            }
        }
        if (ms - last_200Hz_time > 5) {//200Hz
            last_200Hz_time = ms;
            leds_connected(connected); //blink LED to show connection status
        }
        if (ms - last_10Hz_time > 10) {//10Hz
            last_10Hz_time = ms;
            connected = can_hb_check_connected(ms);
            if (!connected) { //when main RCU offline, do ADC conversion at much lower rate
                transmit_sensors();
            }
        }
        if (ms - last_2Hz_time > 500) { //2Hz
            last_2Hz_time = ms;
            //send a heartbeat msg
            hb.health = HEALTH_NOMINAL;
            hb.uptime_s = time_secs();
            can_txq_push(ID_HEARTBEAT, CAN_CONVERT(hb));
        }
    }
}
