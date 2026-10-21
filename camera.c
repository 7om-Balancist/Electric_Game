// /**
//  * camera.c — 相机 UART 接收 + 颜色识别
//  * UART CAM (UART1, PB7=RX), 115200
//  */
// #include "camera.h"
// #include "ili9341.h"
// #include <string.h>

// #define BUF_SIZE 32

// static char    rx_buf[BUF_SIZE];
// static volatile uint8_t rx_idx;
// static volatile bool    ready;
// static char    last_color[BUF_SIZE];

// void Camera_Init(void)
// {
//     rx_idx = 0;
//     ready  = false;
//     last_color[0] = '\0';
//     DL_UART_Main_enableInterrupt(CAM_INST, DL_UART_MAIN_INTERRUPT_RX);
//     NVIC_EnableIRQ(CAM_INST_INT_IRQN);
// }

// void CAM_INST_IRQHandler(void)
// {
//     switch (DL_UART_Main_getPendingInterrupt(CAM_INST)) {
//     case DL_UART_MAIN_IIDX_RX: {
//         uint8_t ch = DL_UART_Main_receiveData(CAM_INST);
//         if (ch == '\n' || ch == '\r') {
//             if (rx_idx > 0) {
//                 rx_buf[rx_idx] = '\0';
//                 strncpy(last_color, rx_buf, BUF_SIZE - 1);
//                 ready = true;
//             }
//             rx_idx = 0;
//         } else if (rx_idx < BUF_SIZE - 1) {
//             rx_buf[rx_idx++] = (char)ch;
//         }
//         break;
//     }
//     default: break;
//     }
// }

// bool Camera_Ready(void)     { return ready; }
// void Camera_GetStr(char *b, int n) { ready = false; strncpy(b, rx_buf, n-1); b[n-1]='\0'; }

// /* 颜色名 → RGB565 */
// uint16_t Camera_Color(void)
// {
//     if      (strstr(last_color, "red")   || strstr(last_color, "RED"))   return COLOR_RED;
//     else if (strstr(last_color, "green") || strstr(last_color, "GREEN")) return COLOR_GREEN;
//     else if (strstr(last_color, "blue")  || strstr(last_color, "BLUE"))  return COLOR_BLUE;
//     else if (strstr(last_color, "yellow")|| strstr(last_color, "YELLOW"))return COLOR_YELLOW;
//     else if (strstr(last_color, "white") || strstr(last_color, "WHITE")) return COLOR_WHITE;
//     else if (strstr(last_color, "black") || strstr(last_color, "BLACK")) return COLOR_BLACK;
//     return 0x0000;   /* 未识别 */
// }
