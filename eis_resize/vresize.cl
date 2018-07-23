
#define storepix(val,addr) *(__global uchar4*)(addr)=val

__kernel void resize(__global const uchar* srcptr, __global uchar4* dstptr)
{
	int x = get_global_id(0);
	int y = get_global_id(1);
	
	int tmpx = x << 2;
	float4 vfx = (float4)(tmpx, tmpx + 1, tmpx + 2, tmpx + 3) * HSCALE;
	float4 vfy = (float4)(y * VSCALE);
	
    int4 vox = convert_int4(vfx);
    int4 voy = convert_int4(vfy);
	
    float4 vdx = vfx - convert_float4(vox);
    float4 vdy = vfy - convert_float4(voy);
		
	float4 color[4];// = {(float4)(50), (float4)(100), (float4)(150), (float4)(200)};
    float4 w[4] = {(1.0f - vdx) * (1.0f - vdy),  vdx * (1.0f - vdy),  (1.0f - vdx) * vdy,  vdx * vdy};

	__global const uchar* sample_up_ptr = srcptr + mad24(voy.s0, IN_STRIDE, vox.s0);
	
	uchar8 up;
	uchar8 down;
	uchar pixup[8];
	uchar pixdown[8];
	
	up   = vload8(0, sample_up_ptr);
	down = vload8(0, sample_up_ptr + IN_STRIDE);
	
	pixup[0] = up.s0;
	pixup[1] = up.s1;
	pixup[2] = up.s2;
	pixup[3] = up.s3;
	pixup[4] = up.s4;
	pixup[5] = up.s5;
	pixup[6] = up.s6;
	pixup[7] = up.s7;
	
	pixdown[0] = down.s0;
	pixdown[1] = down.s1;
	pixdown[2] = down.s2;
	pixdown[3] = down.s3;
	pixdown[4] = down.s4;
	pixdown[5] = down.s5;
	pixdown[6] = down.s6;
	pixdown[7] = down.s7;
	
	int  indexs[4] = {0, (int)(vox.s1-vox.s0), (int)(vox.s2-vox.s0), (int)(vox.s3-vox.s0)};
//	#pragma unroll 4
//	for (int i = 0; i < 4; ++i) {
//		indexs[i] = (int)(i * HSCALE);
//	}
	
    color[0] = (float4)(pixup[indexs[0]], pixup[indexs[1]], pixup[indexs[2]], pixup[indexs[3]]);
    color[1] = (float4)(pixup[indexs[0]+1], pixup[indexs[1]+1], pixup[indexs[2]+1], pixup[indexs[3]+1]);
	color[2] = (float4)(pixdown[indexs[0]], pixdown[indexs[1]], pixdown[indexs[2]], pixdown[indexs[3]]);
	color[3] = (float4)(pixdown[indexs[0]+1], pixdown[indexs[1]+1], pixdown[indexs[2]+1], pixdown[indexs[3]+1]);

    float4 value=(float4)(0.0f);

	#pragma unroll 4
	for (int i=0; i < 4; i++){
 		value = value + w[i] * color[i];
	}

	storepix(convert_uchar4_sat(value),dstptr + mad24(y, OUT_STRIDE, x));
	
	if (y % 2 == 0) 
	{
		int srcOffsetY = voy.s0 / 2 * IN_STRIDE;
		int dstOffsetY = y / 2 * OUT_STRIDE;
		vox = vox / 2 * 2;
		__global const uchar2* src_uv_ptr0 = (__global const uchar2*)(srcptr + IN_UV_OFFSET  + srcOffsetY + vox.s0);
		__global const uchar2* src_uv_ptr1 = (__global const uchar2*)(srcptr + IN_UV_OFFSET  + srcOffsetY + vox.s2);
		__global uchar4* dst_uv_ptr = dstptr + OUT_UV_OFFSET / 4 + dstOffsetY + x;
		
		uchar4 value = (uchar4)(*src_uv_ptr0, *src_uv_ptr1);
		
		storepix(value, dst_uv_ptr);
	}
}
