#include "common.h"
#include "stdio.h"

extern uint8_t g_eis_switch;
extern uint8_t g_quence;

//#include "Tracker_macro.h"

uint8_t xorCheck(uint8_t *pData, int offset, int len)
{
	int index=0;
	uint8_t xorCheckout = 0x00;

	for( index=0; index<len; index++)
	{
		xorCheckout = xorCheckout ^ *(pData+index+offset);
	}
	return xorCheckout;
}

int8_t inputData_analysis(uint8_t size, uint8_t *data_in)
{
	int8_t rt_ = SUCCESS;
	uint8_t xorCheckValue = 0;
	
	if(NULL == data_in || size < 7)
	{
		rt_ = FAILED;
		printf("FAILED!! data_in is NULL! data_in's size is %d. Returned %d.\n",size, rt_);
		return rt_;
	}

	uint8_t normal_size_ = 0;
	uint8_t normal_addr_ = 0;
	uint8_t cmd_addr_value = data_in[GDU_PROTOCOL_CMD_ADDR_POS];
	uint8_t cmd_key_value = data_in[GDU_PROTOCOL_CMD_KEY_POS];
	switch(cmd_key_value)
	{
		case GDU_PROTOCOL_CS_CMD_KET_EIS:
			normal_size_  = 8;
			normal_addr_ = 0x74;
			
		default:
			break;
	}

	//check with size and addr
	if(normal_size_ != size || normal_addr_ != cmd_addr_value)
	{
		rt_ = FAILED;
		printf("ERROR!! normal size is %d, but data_in size is %d! normal_addr_ is %x, but cmd_addr_value is %x. Returned %d.\n",normal_size_, size, normal_addr_, cmd_addr_value, rt_);
		return rt_;
	}
	
	xorCheckValue = xorCheck( data_in, 1, normal_size_-3 );
	if( xorCheckValue == data_in[normal_size_-2] )
	{
		g_eis_switch = data_in[5];
		g_quence = data_in[4];
	}
	else
	{
		rt_ = FAILED;
		printf("FAILED!! xorCheck failed! Returned %d.\n",rt_);
	}

	return rt_;
}

int8_t outputData_pack(uint8_t *size, uint8_t *data_out)
{
	int8_t rt_ = SUCCESS;
	uint8_t xorCheckValue = 0;

	if(NULL == data_out)
	{
		rt_ = FAILED;
		printf("FAILED!! data_in is NULL! Returned %d.\n", rt_);
		return rt_;
	}

	*size = 8;
	data_out[0] = 0x55;
	data_out[1]    = 4;
	data_out[2] = 0x47;
	data_out[3]    = 0x3f;
	data_out[4] = g_quence;   //°üÐòºÅ
	data_out[5]    = 0x00;
	xorCheckValue  = xorCheck(data_out, 1, *size-3);
	data_out[*size-2] = xorCheckValue;
	data_out[*size-1] = 0xf0;
	return rt_;
}
