#include <stdio.h>
#include "flash_opt.h"
#include "app_error.h"
#include "pstorage_platform.h"
#include "nrf_soc.h"
#include "pstorage.h"
#include "debug_utils.h"

pstorage_handle_t base_flash_handle;

static nvm_complete_cb_t nvm_callbacks[NVM_MAX_CALLBACKS];
static uint8_t nvm_num_callbacks;

static void nvm_pstorage_cb_handler(pstorage_handle_t * handle,
									uint8_t op_code,
									uint32_t result,
									uint8_t * p_data,
									uint32_t data_len)
{	
	switch(op_code)
	{
	case PSTORAGE_LOAD_OP_CODE:
		if (result == NRF_SUCCESS)
		{
			DEBUG_PRINT(0,"pstorage LOAD callback received \r\n");
		}
		else
		{
			DEBUG_PRINT(0,"pstorage LOAD ERROR callback received \r\n");

		}
		break;
	case PSTORAGE_STORE_OP_CODE:	
		if (result == NRF_SUCCESS)
		{
			DEBUG_PRINT(0,"pstorage STORE callback received \r\n");

		}
		else
		{
			DEBUG_PRINT(0,"pstorage STORE ERROR callback received \r\n");
		}
		break;
	case PSTORAGE_UPDATE_OP_CODE:
        for (uint8_t i = 0; i < NVM_MAX_CALLBACKS; i++)
		{
			if ((handle->block_id == nvm_callbacks[i].handle.block_id) && (nvm_callbacks[i].complete_cb != NULL))
			{
                nvm_callbacks[i].complete_cb(result);
				break;
			}
		}
        break;
		
	case PSTORAGE_CLEAR_OP_CODE:
		if (result == NRF_SUCCESS)
		{
			DEBUG_PRINT(0,"pstorage CLEAR callback received \r\n");

		}
		else
		{
			DEBUG_PRINT(0,"pstorage CLEAR ERROR callback received \r\n");

		}
		break;

	}
}

void nvm_storage_init(void)
{
	uint32_t err_code;
	nvm_num_callbacks = 0;
	 
	pstorage_module_param_t PST_param;
	PST_param.block_size  = NVM_BLOCK_SIZE;
	PST_param.block_count = NVM_NUM_BLOCKS;
	PST_param.cb = nvm_pstorage_cb_handler;
	err_code = pstorage_register(&PST_param, &base_flash_handle);
	APP_ERROR_CHECK(err_code);
}

void nvm_clear(NVM_BLOCK_ID_E block_num)
{
	uint32_t err_code;
	pstorage_handle_t m_flash_handle;
	err_code= pstorage_block_identifier_get(&base_flash_handle, block_num, &m_flash_handle);
	APP_ERROR_CHECK(err_code);
	err_code = pstorage_clear(&m_flash_handle,NVM_BLOCK_SIZE);
	APP_ERROR_CHECK(err_code);
}

void nvm_load(NVM_BLOCK_ID_E block_id, 
			  uint8_t * target, 
			  uint16_t length, 
			  uint16_t offset)
{
	uint32_t err_code;
	pstorage_handle_t m_flash_handle;
	err_code  =  pstorage_block_identifier_get(&base_flash_handle,block_id,&m_flash_handle);
	APP_ERROR_CHECK(err_code);
	err_code = pstorage_load(target, &m_flash_handle,length,offset);
    APP_ERROR_CHECK(err_code);
}



void nvm_write(pstorage_size_t block_num, 
				uint8_t * src, 
				uint16_t length, 
				uint16_t offset, 
				write_complete_cb_t complete_cb)
{
	uint32_t err_code;
    err_code = pstorage_block_identifier_get(&base_flash_handle, block_num, &nvm_callbacks[nvm_num_callbacks].handle);
	APP_ERROR_CHECK(err_code);
	err_code =pstorage_update(&nvm_callbacks[nvm_num_callbacks].handle, src, length, offset);
	APP_ERROR_CHECK(err_code);
	
	nvm_callbacks[nvm_num_callbacks].complete_cb = complete_cb;
    nvm_num_callbacks += 1;
}
