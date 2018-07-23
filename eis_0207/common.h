#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdint.h>

#define SUCCESS				0
#define FAILED				-1

//------------------------------------------------------------------------------
//CS in:
#define GDU_PROTOCOL_CS_CMD_KET_EIS             0x3F  //eis


//------------------------------------------------------------------------------
//CS out:
#define GDU_EIS_DATA_TO_CE_LEN                          8     //eis

//------------------------------------------------------------------------------
#define GDU_PROTOCOL_CMD_ADDR_POS                             2     //地址是第二位
#define GDU_PROTOCOL_CMD_KEY_POS                                 3     //命令字是第三位

#define GDU_PROTOCOL_APP_TO_CS_ADDR  			0x74  //APP -> CS  地址
#define GDU_PROTOCOL_CS_TO_APP_ADDR  			0x47  //CS  -> APP 地址



//------------------------------------------------------------------------------

//CS receive the data from APP
//algorithm_type:APP commod:0x00:open,0x01:close
struct impr_app_data_eis{
	int8_t algorithm_type;

};

//callback to APP
struct impr_cb_comp_data_eis {
	int8_t	         status;
};
//------------------------------------------------------------------------------

uint8_t xorCheck(uint8_t *pData, int offset, int len);

//input:  size data_in 
//output: analysis_data
int8_t inputData_analysis(uint8_t size, uint8_t *data_in);

//input:  cb_data
//output: size data_out
int8_t outputData_pack(uint8_t *size, uint8_t *data_out);

#endif //_COMMON_H_


