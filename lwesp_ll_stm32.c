#include "lwesp/lwesp.h"
#include "lwesp/lwesp_mem.h"
#include "lwesp/lwesp_input.h"
#include "system/lwesp_ll.h"
#include "lwesp_ll_stm32f103c8.h"
#include "task.h"
#include "queue.h"

#if !LWESP_CFG_INPUT_USE_PROCESS
#error "LWESP_CFG_INPUT_USE_PROCESS must be enabled in `lwesp_config.h` to use this driver."
#endif /* LWESP_CFG_INPUT_USE_PROCESS */

#if !defined(LWESP_USART_DMA_RX_BUFF_SIZE)
#define LWESP_USART_DMA_RX_BUFF_SIZE      0x1000
#endif /* !defined(LWESP_USART_DMA_RX_BUFF_SIZE) */

#if !defined(LWESP_MEM_SIZE)
#define LWESP_MEM_SIZE                    0x1000
#endif /* !defined(LWESP_MEM_SIZE) */

#if !defined(LWESP_USART_RDR_NAME)
#define LWESP_USART_RDR_NAME              DR
#endif /* !defined(LWESP_USART_RDR_NAME) */

/* USART memory */
static uint8_t      usart_mem[LWESP_USART_DMA_RX_BUFF_SIZE];
static uint8_t      is_running, initialized;
static size_t       old_pos;

/* USART thread */
static void usart_ll_thread(void* arg);
static xTaskHandle usart_ll_thread_id;

/* Message queue */
static xQueueHandle usart_ll_mbox_id;

/**
 * \brief           USART data processing
 */
static void
usart_ll_thread(void* arg) {
    size_t pos;

    LWESP_UNUSED(arg);

    while (1) {
        void* d;
        /* Wait for the event message from DMA or USART */
        xQueueReceive(usart_ll_mbox_id, &d, portMAX_DELAY);

        /* Read data */
        pos = sizeof(usart_mem) - LL_DMA_GetDataLength(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);

        if (pos != old_pos && is_running) {
            if (pos > old_pos) {
                lwesp_input_process(&usart_mem[old_pos], pos - old_pos);
            } else {
                lwesp_input_process(&usart_mem[old_pos], sizeof(usart_mem) - old_pos);
                if (pos > 0) {
                    lwesp_input_process(&usart_mem[0], pos);
                }
            }
            old_pos = pos;
            if (old_pos == sizeof(usart_mem)) {
                old_pos = 0;
            }
        }
    }
}

/**
 * \brief           Configure UART using DMA for receive in double buffer mode and IDLE line detection
 */
static void
configure_uart(uint32_t baudrate) {
    static LL_USART_InitTypeDef usart_init;
    static LL_DMA_InitTypeDef dma_init;
    LL_GPIO_InitTypeDef gpio_init;

    if (!initialized) {
        /* Enable peripheral clocks */
        LWESP_USART_CLK;
        LWESP_USART_DMA_CLK;
        LWESP_USART_TX_PORT_CLK;
        LWESP_USART_RX_PORT_CLK;

#if defined(LWESP_RESET_PIN)
        LWESP_RESET_PORT_CLK;
#endif /* defined(LWESP_RESET_PIN) */

#if defined(LWESP_GPIO0_PIN)
        LWESP_GPIO0_PORT_CLK;
#endif /* defined(LWESP_GPIO0_PIN) */

#if defined(LWESP_GPIO2_PIN)
        LWESP_GPIO2_PORT_CLK;
#endif /* defined(LWESP_GPIO2_PIN) */

#if defined(LWESP_CH_PD_PIN)
        LWESP_CH_PD_PORT_CLK;
#endif /* defined(LWESP_CH_PD_PIN) */

        /* Global pin configuration */
        LL_GPIO_StructInit(&gpio_init);
        gpio_init.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
        gpio_init.Pull = LL_GPIO_PULL_UP;
        gpio_init.Speed = LL_GPIO_SPEED_FREQ_HIGH;
        gpio_init.Mode = LL_GPIO_MODE_OUTPUT;

#if defined(LWESP_RESET_PIN)
        /* Configure RESET pin */
        gpio_init.Pin = LWESP_RESET_PIN;
        LL_GPIO_Init(LWESP_RESET_PORT, &gpio_init);
#endif /* defined(LWESP_RESET_PIN) */

#if defined(LWESP_GPIO0_PIN)
        /* Configure GPIO0 pin */
        gpio_init.Pin = LWESP_GPIO0_PIN;
        LL_GPIO_Init(LWESP_GPIO0_PORT, &gpio_init);
        LL_GPIO_SetOutputPin(LWESP_GPIO0_PORT, LWESP_GPIO0_PIN);
#endif /* defined(LWESP_GPIO0_PIN) */

#if defined(LWESP_GPIO2_PIN)
        /* Configure GPIO2 pin */
        gpio_init.Pin = LWESP_GPIO2_PIN;
        LL_GPIO_Init(LWESP_GPIO2_PORT, &gpio_init);
        LL_GPIO_SetOutputPin(LWESP_GPIO2_PORT, LWESP_GPIO2_PIN);
#endif /* defined(LWESP_GPIO2_PIN) */

#if defined(LWESP_CH_PD_PIN)
        /* Configure CH_PD pin */
        gpio_init.Pin = LWESP_CH_PD_PIN;
        LL_GPIO_Init(LWESP_CH_PD_PORT, &gpio_init);
        LL_GPIO_SetOutputPin(LWESP_CH_PD_PORT, LWESP_CH_PD_PIN);
#endif /* defined(LWESP_CH_PD_PIN) */

        /* Configure USART pins */
        gpio_init.Mode = LL_GPIO_MODE_ALTERNATE;

        /* TX PIN */
        gpio_init.Pin = LWESP_USART_TX_PIN;
        LL_GPIO_Init(LWESP_USART_TX_PORT, &gpio_init);

        /* RX PIN */
        gpio_init.Mode = LL_GPIO_MODE_INPUT;
        gpio_init.Pin = LWESP_USART_RX_PIN;
        LL_GPIO_Init(LWESP_USART_RX_PORT, &gpio_init);

        /* Configure UART */
        LL_USART_DeInit(LWESP_USART);
        LL_USART_StructInit(&usart_init);
        usart_init.BaudRate = baudrate;
        usart_init.DataWidth = LL_USART_DATAWIDTH_8B;
        usart_init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
        usart_init.OverSampling = LL_USART_OVERSAMPLING_16;
        usart_init.Parity = LL_USART_PARITY_NONE;
        usart_init.StopBits = LL_USART_STOPBITS_1;
        usart_init.TransferDirection = LL_USART_DIRECTION_TX_RX;
        LL_USART_Init(LWESP_USART, &usart_init);
        
        /* Enable USART interrupts and DMA request */
        LL_USART_EnableIT_IDLE(LWESP_USART);
        LL_USART_EnableIT_PE(LWESP_USART);
        LL_USART_EnableIT_ERROR(LWESP_USART);
        LL_USART_EnableDMAReq_RX(LWESP_USART);

        /* Enable USART interrupts */
        NVIC_SetPriority(LWESP_USART_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x07, 0x00));
        NVIC_EnableIRQ(LWESP_USART_IRQ);

        /* Configure DMA */
        is_running = 0;

        LL_DMA_DeInit(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);

        dma_init.PeriphOrM2MSrcAddress = (uint32_t)&LWESP_USART->LWESP_USART_RDR_NAME;
        dma_init.MemoryOrM2MDstAddress = (uint32_t)usart_mem;
        dma_init.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
        dma_init.Mode = LL_DMA_MODE_CIRCULAR;
        dma_init.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
        dma_init.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
        dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
        dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
        dma_init.NbData = sizeof(usart_mem);
        dma_init.Priority = LL_DMA_PRIORITY_MEDIUM;

        LL_DMA_Init(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH, &dma_init);

        /* Enable DMA interrupts */
        LL_DMA_EnableIT_HT(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);
        LL_DMA_EnableIT_TC(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);
        LL_DMA_EnableIT_TE(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);

        /* Enable DMA interrupts */
        NVIC_SetPriority(LWESP_USART_DMA_RX_IRQ, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0x07, 0x00));
        NVIC_EnableIRQ(LWESP_USART_DMA_RX_IRQ);

        old_pos = 0;
        is_running = 1;

        /* Start DMA and USART */
        LL_DMA_EnableChannel(LWESP_USART_DMA, LWESP_USART_DMA_RX_CH);

        LL_USART_Enable(LWESP_USART);
    } else {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        LL_USART_Disable(LWESP_USART);
        usart_init.BaudRate = baudrate;
        LL_USART_Init(LWESP_USART, &usart_init);
        LL_USART_Enable(LWESP_USART);
    }

    /* Create mbox and start thread */
    if (usart_ll_mbox_id == NULL) {
        usart_ll_mbox_id = xQueueCreate(10, sizeof(void*));
    }
    if (usart_ll_thread_id == NULL) {
        xTaskCreate(usart_ll_thread, "usart_ll_thread", 256, usart_ll_mbox_id, 24, &usart_ll_thread_id);
    }
}

#if defined(LWESP_RESET_PIN)
/**
 * \brief           Hardware reset callback
 */
static uint8_t
reset_device(uint8_t state) {
    if (state) {                                /* Activate reset line */
        LL_GPIO_ResetOutputPin(LWESP_RESET_PORT, LWESP_RESET_PIN);
    } else {
        LL_GPIO_SetOutputPin(LWESP_RESET_PORT, LWESP_RESET_PIN);
    }
    return 1;
}
#endif /* defined(LWESP_RESET_PIN) */

/**
 * \brief           Send data to ESP device
 * \param[in]       data: Pointer to data to send
 * \param[in]       len: Number of bytes to send
 * \return          Number of bytes sent
 */
static size_t
send_data(const void* data, size_t len) {
    const uint8_t* d = data;

    for (size_t i = 0; i < len; ++i, ++d) {
        LL_USART_TransmitData8(LWESP_USART, *d);
        while (!LL_USART_IsActiveFlag_TXE(LWESP_USART)) {}
    }
    return len;
}

/**
 * \brief           Callback function called from initialization process
 */
lwespr_t
lwesp_ll_init(lwesp_ll_t* ll) {
#if !LWESP_CFG_MEM_CUSTOM
    static uint8_t memory[LWESP_MEM_SIZE];
    lwesp_mem_region_t mem_regions[] = {
        { memory, sizeof(memory) }
    };

    if (!initialized) {
        lwesp_mem_assignmemory(mem_regions, LWESP_ARRAYSIZE(mem_regions));  /* Assign memory for allocations */
    }
#endif /* !LWESP_CFG_MEM_CUSTOM */

    if (!initialized) {
        ll->send_fn = send_data;                /* Set callback function to send data */
#if defined(LWESP_RESET_PIN)
        ll->reset_fn = reset_device;            /* Set callback for hardware reset */
#endif /* defined(LWESP_RESET_PIN) */
    }

    configure_uart(ll->uart.baudrate);          /* Initialize UART for communication */
    initialized = 1;
    return lwespOK;
}

/**
 * \brief           Callback function to de-init low-level communication part
 */
lwespr_t
lwesp_ll_deinit(lwesp_ll_t* ll) {
    if (usart_ll_mbox_id != NULL) {
        xQueueHandle tmp = usart_ll_mbox_id;
        usart_ll_mbox_id = NULL;
        vQueueDelete(tmp);
    }
    if (usart_ll_thread_id != NULL) {
        xTaskHandle tmp = usart_ll_thread_id;
        usart_ll_thread_id = NULL;
        vTaskDelete(tmp); 
    }
    initialized = 0;
    LWESP_UNUSED(ll);
    return lwespOK;
}

/**
 * \brief           UART global interrupt handler
 */
void
LWESP_USART_IRQHANDLER(void) {
    LL_USART_ClearFlag_IDLE(LWESP_USART);
    LL_USART_ClearFlag_PE(LWESP_USART);
    LL_USART_ClearFlag_FE(LWESP_USART);
    LL_USART_ClearFlag_ORE(LWESP_USART);
    LL_USART_ClearFlag_NE(LWESP_USART);

    if (usart_ll_mbox_id != NULL) {
        void* d = (void*)1;
        xQueueSendToBackFromISR(usart_ll_mbox_id, &d, NULL);
    }
}

/**
 * \brief           UART DMA stream/channel handler
 */
void
LWESP_USART_DMA_RX_IRQHANDLER(void) {
    LWESP_USART_DMA_RX_CLEAR_TC;
    LWESP_USART_DMA_RX_CLEAR_HT;

    if (usart_ll_mbox_id != NULL) {
        void* d = (void*)1;
        xQueueSendToBackFromISR(usart_ll_mbox_id, &d, NULL);
    }
}
