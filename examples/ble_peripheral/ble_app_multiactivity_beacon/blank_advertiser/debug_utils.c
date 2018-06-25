#include "debug_utils.h"

void HardFault_Handler(void)
{
    uint32_t *sp = (uint32_t *) __get_MSP();
    uint32_t ia = sp[24/4];
#ifndef DEBUG
    NVIC_SystemReset();
#endif
    DEBUG_PRINT(0,"Hard Fault at address: 0x%08x\r\n", (unsigned int)ia);
    while(1)
        ;
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
#ifndef DEBUG
    NVIC_SystemReset();
#endif

    __disable_irq();
    
    DEBUG_PRINT(0,"ASSERT\r\nCode: 0x%08x\r\nLine: %d\r\nFile: %s\r\n", (unsigned int)error_code, (unsigned int)line_num, p_file_name);
    
    while (true)
    {
        __NOP();
    }
}

