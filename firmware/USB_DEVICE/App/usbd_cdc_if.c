#include "usbd_cdc_if.h"

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);


USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};


static int8_t CDC_Init_FS(void)
{

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);

}

static int8_t CDC_DeInit_FS(void)
{

  return (USBD_OK);

}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{

  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);

}
#define CMD_BUF_SIZE 128
static char cmd_buf[CMD_BUF_SIZE];
static size_t cmd_len = 0;

// Буфер для команды, готовой к обработке
char pending_cmd[CMD_BUF_SIZE];
volatile uint8_t cmd_ready = 0;

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
    for (uint32_t i = 0; i < *Len; i++) {
        char c = Buf[i];

        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                // Копируем команду в pending_cmd, чтобы не затиралось при следующем приёме
                strncpy(pending_cmd, cmd_buf, CMD_BUF_SIZE);
                pending_cmd[CMD_BUF_SIZE - 1] = '\0';
                cmd_ready = 1; // флаг для main()
                cmd_len = 0;  // очищаем буфер
            }
        } else {
            if (cmd_len < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_len++] = c;
            } else {
                // переполнение буфера, сброс
                cmd_len = 0;
            }
        }
    }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;

  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  return result;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;

  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);

  return result;
}
