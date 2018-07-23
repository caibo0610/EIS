#define storepix(val,addr) *(__global uchar4*)(addr)=val

__kernel void warpAffine(__global const uchar* srcptr, __global uchar4* dstptr, __constant float * M)
{
	int orx = get_global_id(0);
	int ory = get_global_id(1);
	
	int x = orx + XOFFSET;
	int y = ory + YOFFSET;

	int tmpx = x << 2;
	float4 vx = (float4)(tmpx, tmpx + 1, tmpx + 2, tmpx + 3);
	float4 fx = M[0] * vx + mad(M[1], y, M[2]);
	float4 fy = M[3] * vx + mad(M[4], y, M[5]);
	
    int4 ix = convert_int4(fx);
    int4 iy = convert_int4(fy);
	
	int4 srcidx = mad24(iy, SRC_STEP, ix);
	
	if (ix.s0 < 0 || ix.s0 >= SRC_STEP || ix.s3 < 0 || ix.s3 >= SRC_STEP || 
		iy.s0 < 0 || iy.s0 >= SRC_SCANLINE || iy.s3 < 0 || iy.s3 >= SRC_SCANLINE ) {
		
		storepix((uchar4)(0),dstptr + mad24(ory, DST_STEP, orx));
		return ;
	}
	
    float4 dx = fx - convert_float4(ix);
    float4 dy = fy - convert_float4(iy);
		
	float4 color[4];
    float4 w[4] = {(1.0f - dx) * (1.0f - dy),  dx * (1.0f - dy),  (1.0f - dx) * dy,  dx * dy};

	uchar2 a0 = vload2(0, srcptr + srcidx.s0);
	uchar2 a1 = vload2(0, srcptr + srcidx.s1);
	uchar2 a2 = vload2(0, srcptr + srcidx.s2);
	uchar2 a3 = vload2(0, srcptr + srcidx.s3);
	
	uchar2 b0 = vload2(0, srcptr + srcidx.s0 + SRC_STEP);
	uchar2 b1 = vload2(0, srcptr + srcidx.s1 + SRC_STEP);
	uchar2 b2 = vload2(0, srcptr + srcidx.s2 + SRC_STEP);
	uchar2 b3 = vload2(0, srcptr + srcidx.s3 + SRC_STEP);
	
    color[0] = (float4)(a0.s0, a1.s0, a2.s0, a3.s0);
    color[1] = (float4)(a0.s1, a1.s1, a2.s1, a3.s1);
	color[2] = (float4)(b0.s0, b1.s0, b2.s0, b3.s0);
	color[3] = (float4)(b0.s1, b1.s1, b2.s1, b3.s1);

    float4 value=(float4)(0.0f);

	#pragma unroll 4
	for (int i = 0; i < 4; i++){
 		value = value + w[i] * color[i];
	}

	storepix(convert_uchar4_sat(value), dstptr + mad24(ory, DST_STEP, orx));
	
	if (ory % 2 == 0) 
	{
		int2 uv_x = convert_int2(fx.s01) / 2 * 2;
		int2 uv_y = convert_int2(fy.s01) / 2;

		int2 idx = mad24(SRC_SCANLINE + uv_y, SRC_STEP, uv_x);
		uchar2 ua0 = vload2(0, srcptr + idx.s0);
		uchar2 ub0 = vload2(0, srcptr + idx.s1);
     
        uchar4 value=(uchar4)(ua0, ub0);
		
        storepix(convert_uchar4_sat(value), dstptr + mad24(ory / 2 + DST_SCANLINE, DST_STEP, orx));
	}
	
/*
	if (ory % 2 == 0) 
	{	
		int uv_x = x << 1;
		int uv_y = y >> 1;
		float2 uvx = (float2)(uv_x, uv_x + 1);
		float2 uv_fx = M[0] * uvx + mad(M[1], uv_y, M[2]);
		float2 uv_fy = M[3] * uvx + mad(M[4], uv_y, M[5]);
	
		int2 uvox = convert_int2(uv_fx);
		int2 uvoy = convert_int2(uv_fy);

		int delt = ( SRC_SCANLINE + ory >> 1 )* DST_STEP;
		
		if (uvox.s0 < 0 || uvox.s0 * 2 >= SRC_STEP || uvox.s0 * 2 + 1>= SRC_STEP ||
			uvox.s1 < 0 || uvox.s1 * 2 >= SRC_STEP || uvox.s1 * 2 + 1>= SRC_STEP  ||
			uvoy.s0 < 0 || uvoy.s0 >= SRC_SCANLINE || uvoy.s1 < 0 || uvoy.s1 >= SRC_SCANLINE) {

			storepix((uchar4)(0),dstptr + delt+ orx);
			return;
		} 

		int2 srcidx_v = mad24(SRC_SCANLINE + uvoy, SRC_STEP, uvox << 1);
		int2 srcidx_u = mad24(SRC_SCANLINE + uvoy, SRC_STEP, (uvox << 1) + 1);
             
        float4 value=(float4)(srcptr[srcidx_v.s0],srcptr[srcidx_u.s0],srcptr[srcidx_v.s1],srcptr[srcidx_u.s1]);
		
        storepix(convert_uchar4_sat(value),dstptr + delt + orx);
	}
	*/
}
