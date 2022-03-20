#ifndef _MMODBUS_CONFIG_H_
#define _MMODBUS_CONFIG_H_

#define RS485_CTRL_GPIO_Port      GPIOA
#define RS485_CTRL_Pin            (RS485_RE_GPIO_Pin | RS485_DE_GPIO_Pin)

#define _MMODBUS_FREERTOS         0
#define _MMODBUS_RTU              1
#define _MMODBUS_ASCII            0 //  not implemented yet
#define _MMODBUS_USART            USART2
#define _MMODBUS_RXSIZE           64  
#define _MMODBUS_TXDMA            0
#if     _MMODBUS_TXDMA == 1
#define _MMODBUS_DMA              DMA2
#define _MMODBUS_DMASTREAM        LL_DMA_STREAM_7   
#endif
#define _MMODBUS_CTRL_GPIO        RS485_CTRL_GPIO_Port
#define _MMODBUS_CTRL_PIN         RS485_CTRL_Pin

#define _MMODBUS_USE_LL_DRIVERS   0

#if (_MMODBUS_RTU == 1) && (_MMODBUS_ASCII == 1)
#error please select _MMODBUS_RTU or _MMODBUS_ASCII
#endif
#endif
