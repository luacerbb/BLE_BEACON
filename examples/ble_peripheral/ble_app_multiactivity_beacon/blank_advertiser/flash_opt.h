#ifndef __FLAH_OPT_H__
#define __FLAH_OPT_H__
#include "pstorage.h"

typedef void (* write_complete_cb_t)( uint32_t  result );

typedef struct
{
    pstorage_handle_t handle;
    write_complete_cb_t complete_cb;
} nvm_complete_cb_t;

#define NVM_BLOCK_SIZE      128     

typedef enum
{
   ADV_DATA_BLOCK_ID = 0x00,
   RADIO_PARM_BLOCK_ID,
	 BEACON_SCAN_STA,			//ÊÇ·ñÆô¶¯É¨Ãè×´Ì¬
	// BEACON_SAVE_DATA,
	 
   NVM_NUM_BLOCKS,
} NVM_BLOCK_ID_E;

#define NVM_MAX_CALLBACKS  NVM_NUM_BLOCKS


void nvm_storage_init(void);
void nvm_clear(NVM_BLOCK_ID_E block_num);
void nvm_load(NVM_BLOCK_ID_E block_id,uint8_t * target,uint16_t length,  uint16_t offset);
void nvm_write(pstorage_size_t block_num, uint8_t * src, uint16_t length, uint16_t offset, write_complete_cb_t complete_cb);

#endif  //__FLAH_OPT_H__

