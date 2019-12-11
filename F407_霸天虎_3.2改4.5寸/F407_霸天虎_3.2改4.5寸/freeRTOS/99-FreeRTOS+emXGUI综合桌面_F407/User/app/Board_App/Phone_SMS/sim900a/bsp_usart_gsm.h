#ifndef __GSM_USART_H
#define	__GSM_USART_H

#include "stm32f4xx.h"

#define GSM_USART                             USART2
#define GSM_USART_CLK                         RCC_APB1Periph_USART2
#define GSM_USART_CLKCMD                      RCC_APB1PeriphClockCmd
#define GSM_USART_BAUDRATE                    115200
#define GSM_USART_IRQ                         USART2_IRQn
#define GSM_USART_IRQHandler                  USART2_IRQHandler

#define GSM_USART_RX_GPIO_PORT                GPIOA
#define GSM_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define GSM_USART_RX_PIN                      GPIO_Pin_2
#define GSM_USART_RX_AF                       GPIO_AF_USART2
#define GSM_USART_RX_SOURCE                   GPIO_PinSource2

#define GSM_USART_TX_GPIO_PORT                GPIOA
#define GSM_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define GSM_USART_TX_PIN                      GPIO_Pin_3
#define GSM_USART_TX_AF                       GPIO_AF_USART2
#define GSM_USART_TX_SOURCE                   GPIO_PinSource3

void GSM_USART_Config(void);
void GSM_USART_printf(USART_TypeDef* USARTx, char *Data,...);
char *gsm_get_rebuff(uint16_t *len);
void gsm_clean_rebuff(void);

#endif /* __GSM_USART_H */
