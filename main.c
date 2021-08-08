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
#include "libcan/can_messages.h"
#include "libcan/can.h"
#include "libpicutil/adc.h"

//code specific to this RCU

#define _XTAL_FREQ 64000000UL //needed for delays to work, but not much else

const uint8_t RCU_ID_LOCAL = RCU_ID_ENGINE_SENSOR_RCU;

/*
 * 
 */

uint16_t last_2Hz_time, last_hb_rx_time;

uint8_t hb_rx_flag;

struct Heartbeat_t hb;
struct EngineSensorsA_t sensorsA;
struct EngineSensorsB_t sensorsB;

char msg[64];

void on_can_rx(const struct can_msg_t *msg);

int main() {
    //RC3 is a digital output
    ANSELCbits.ANSELC3 = 0;
    TRISCbits.TRISC3 = 0;

    INTCON0bits.GIE = 1; //enable global interrupts

    time_init();
    uart_init();
    adc_init();
    can_rx_callback = &on_can_rx;
    can_init();

    while (1) {
        if (one_kHz_flag) {
            one_kHz_flag = 0;
            //read ADC channels and transmit to bus as two packets
            //ADC reads are 12-bit raw values
            //PIC is too slow to do floating point conversion, so we do it on the Pi
            //plus this allows for conversion
            sensorsA.thrust_raw = adc_read(0);
            sensorsA.chamber_press_raw = adc_read(1);
            sensorsA.fuel_inj_press_raw = adc_read(2);
            sensorsA.ox_inj_press_raw = adc_read(3);
            can_txq_push(ID_ENGINE_SENSORS_A, sizeof(struct EngineSensorsA_t), (uint8_t*)&sensorsA);
            
            sensorsB.fuel_tank_press_raw = adc_read(4);
            sensorsB.aux_1_raw= adc_read(5);
            sensorsB.aux_2_raw= adc_read(6);
            sensorsB.aux_3_raw= adc_read(7);
            can_txq_push(ID_ENGINE_SENSORS_B, sizeof(struct EngineSensorsB_t), (uint8_t*)&sensorsB);
        }
        if (time_millis() - last_2Hz_time > 500) { //2Hz
            last_2Hz_time = time_millis();
            LATCbits.LC3 = ~LATCbits.LC3; //blink LED at 1Hz
            //send a heartbeat msg
            hb.health = HEALTH_NOMINAL;
            hb.uptime_s = time_secs();
            can_txq_push(ID_HEARTBEAT, sizeof (struct Heartbeat_t), (uint8_t *) & hb);
        }
    }
}

void on_can_rx(const struct can_msg_t *msg) {
    switch (msg->id) {
        case (ID_HEARTBEAT | RCU_ID_MAIN_RCU): //heartbeat from main RCU
            //set flag to indicate heartbeat received. main loop can note the time
            if (msg->len == sizeof (struct Heartbeat_t)) {
                hb_rx_flag = 1;
            }
            break;
    }
}
