#include "supercap.h"
#include "bsp_uart.h"
#include "bsp_daemon.h"
#include "bsp_serial.h"
// #include "c_board_comm.h"

#include <string.h>
#include <stdlib.h>

#include "referee_system.h"

Supercap_t g_supercap;
CAN_Instance_t *supercap_can_instance;
Daemon_Instance_t *g_supercap_daemon_ptr;
UART_Instance_t *supercap_uart_instance_ptr;
extern Jetson_Orin_Data_t g_orin_data;
// extern Board_Comm_Package_t g_board_comm_package; 

struct rx_data
{
    uint16_t max_discharge;
    uint16_t base_power;
    int16_t cap_energy_percent;
    uint16_t cap_state;
} g_supercap_data;

char* middle_cpy;
char uart_buffer[SUPERCAP_RX_BUFFER_SIZE];
uint8_t uart_byte;
uint8_t uart_index = 0;
uint8_t supercap_uart_instance_initialized;
// Parsed values
//float Vi = 0, Vo = 0, Pi = 0, Io = 0, Ps = 0, Ii = 0;


void Supercap_Timeout_Callback(void){
    UART_Service_Init(supercap_uart_instance_ptr);
}

// void Supercap_Decode(CAN_Instance_t *can_instance)
// {
//     // Recieve supercap data
//     uint16_t *supercap_rx = (uint16_t *) can_instance->rx_buffer;

//     g_supercap_rx_data.cap_percentage = supercap_rx[0];
//     g_supercap_rx_data.reserved1 = 0;
//     g_supercap_rx_data.reserved2 = 0;
//     g_supercap_rx_data.reserved3 = 0;
//     // ! do not read more than 8 bytes from the buffer
// }

void Supercap_Decode_Callback(UART_Instance_t *uart_instance) {
    // static float Vi, Vo, Pi, Io, Ps, Ii;
    
    // uart_instance->rx_buffer[0];
    if (uart_instance->rx_buffer[0] != '\n') {
        
        g_supercap.buffer_for_construction[g_supercap.receive_counter] = uart_instance->rx_buffer[0]; // store the byte in the buffer
        g_supercap.receive_counter++; // R
    }
    else // if the end of frame is reached
    {
        g_supercap.buffer_for_construction[g_supercap.receive_counter] = '\0'; 
        g_supercap.receive_counter = 0; // reset counter if \n is received
        Daemon_Reload(g_supercap_daemon_ptr);

        // Supercap decode logic
        // format of rx:
        // printf("Vi:%2.2f Vo:%2.2f Pi:%3.2f Ii:%2.2f Io:%2.2f Pa:%3.2f\r\n", VIPMR, VOFWR,POW_IN, IIPWR, IOPWR, pref);
       sscanf((char *)g_supercap.buffer_for_construction, "Vi:%f Vo:%f Pi:%f Ii:%f Io:%f Ps:%f",
              &g_supercap.Vi, &g_supercap.Vo, &g_supercap.Pi, &g_supercap.Ii, &g_supercap.Io, &g_supercap.Ps);
    }
}

// void Supercap_Init(Supercap_t *g_supercap)
// {
//     // Initialize supercap
//     g_supercap->can_bus = 1;
//     g_supercap->tx_id = 0x2C8;
//     g_supercap->rx_id = 0x2C7;
//     supercap_can_instance =
//         CAN_Device_Register(g_supercap->can_bus, g_supercap->tx_id,
//                             g_supercap->rx_id, Supercap_Decode);
// }

void Supercap_Init(UART_HandleTypeDef *huartx)
{
    //supercap_uart_instance_ptr = UART_Register(huartx, g_supercap.rx_buffer, SUPERCAP_RX_BUFFER_SIZE, Supercap_Decode);
    supercap_uart_instance_ptr = UART_Register(huartx, g_supercap.rx_buffer, SUPERCAP_RX_BUFFER_SIZE, Supercap_Decode_Callback);    // matches expected signature
    uint16_t reload_value = SUPERCAP_TIMEOUT_MS / DAEMON_PERIOD;
    uint16_t intial_counter = reload_value;
    g_supercap_daemon_ptr = Daemon_Register(reload_value, intial_counter, Supercap_Timeout_Callback);
    supercap_uart_instance_initialized = 1; //turn on supercap uart 
}

// void Supercap_Send(void)
// {
//     // Send supercap data
//     uint16_t *supercap_tx = (uint16_t *) supercap_can_instance->tx_buffer;
//     supercap_tx[0] = Referee_System.Robot_State.Chassis_Power_Max;    // The power limit from the referee system
//     supercap_tx[1] = (uint16_t) Referee_System.Power_Heat.Chassis_Power;        // the total power requested from the chassis
//     supercap_tx[2] = 0;
//     supercap_tx[3] = 0;
//     // ! do not write more than 8 bytes to the buffer
    
//     CAN_Transmit(supercap_can_instance);
// }

// fomrate PXXXP -  sending XXX as power
// PVONP - Turn on supercap
// PVOFFP - Turn off supercap

void Supercap_Send(void)
{
    if (!supercap_uart_instance_initialized) {
        return;
    }
    
    // Send supercap data
    // uint16_t *supercap_tx = (uint16_t *) supercap_can_instance->tx_buffer;
    // supercap_tx[0] = 0x003C;
    // supercap_tx[1] = 0x003C;
    // supercap_tx[2] = 0x2012;
    // supercap_tx[3] = 0x0112;
    // ! do not write more than 8 bytes to the buffer

	// UART_Transmit(supercap_uart_instance_ptr, g_supercap.tx_buffer, sizeof(g_orin_data.tx_buffer), UART_DMA);
    // uint8_t max_power = 60;
    // UART_Transmit(supercap_uart_instance_ptr->uart_handle, "P%03dP\r\n", g_board_comm_package.Chassis_Power_Max);
    if(0)
    {
        HAL_Delay(100); // wait for the uart to be ready
        DEBUG_PRINTF(supercap_uart_instance_ptr->uart_handle, "PVONP\r\n");
        HAL_Delay(100);
        // DEBUG_PRINTF(supercap_uart_instance_ptr->uart_handle, "P%03dP\r\n", g_board_comm_package.power_limit);
    }
    else
    {
        HAL_Delay(100); // wait for the uart to be ready
        DEBUG_PRINTF(supercap_uart_instance_ptr->uart_handle, "PVOFP\r\n");
    }

}
