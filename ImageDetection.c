/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * 
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xiic.h"
#include "xscugic.h"
#include "mt9p031.h"
#include "camera.h"
#include "xparameters.h"
#include "xil_mmu.h"
#include "demosaic.h"
//#include "imgfilter.h"

XScuGic InterruptController;
XScuGic_Config *gicConfig;

#define CAMBASE_ADDR	XPAR_CAMERA_0_S00_AXI_BASEADDR

void img_detectLine2(){
	int width = 640;
	int height = 480;
	int row,col;
	int offset = 5;
	uint16_t* src;
	uint16_t* dst;
	uint16_t dot;
	uint32_t r;
	uint32_t max,mo,mr;
	int i,j,x;
	volatile int sin10[21] = {9848,9877,9903,9925,9945,9962,9976,9986,9994,9998,10000,9998,9994,9986,9976,9962,9945,9925,9903,9877,9848};
	volatile int cos10[21] = {1736,1564,1392,1219,1045,872,698,523,349,175,0,-175,-349,-523,-698,-872,-1045,-1219,-1392,-1564,-1736};
	//cos90 = -sin10
	//sin90 = cos10

	src = (uint16_t*)0x1F500000;
	dst = (uint16_t*)0x1F400000;

	memset(src,0,21*800*4);

	xil_printf("prepare ok\r\n");
	for(row=offset;row<height-offset;row++){
		for(col=offset;col<width-offset;col++){
			dot = (dst[row*width+col] & 0x000F);
			if(dot==0)
				continue;
			for(i=0;i<21;i++){
				r = col;
				r = r*cos10[i];
				r += row*sin10[i] + 5000;
				r = r/10000;
				if(r>=0 || r<800){
					src[i*800+r]++;
				}
			}
		}
	}
	xil_printf("created po plane\r\n");
	max = 0;
	for(i=0;i<21;i++){
		for(j=0;j<800;j++){
			if(max<src[i*800+j]){
				max = src[i*800+j];
				mo = i;
				mr = j;
			}
		}
	}

	for(i=0;i<width;i++){
		x = mr-(i*cos10[mo]/10000);
		x = x*10000/sin10[mo];
		if(x>0 && x<height)
			dst[x*width+i] = 0x0F00;
	}

	i = (mo-8<0)?0:(mo-8);
	for(;i<(mo+8);i++){
		if(i>=21)
			break;
		j = (mr-50<0)?0:(mr-50);
		for(;j<(mr+50);j++){
			if(j>=800)
				break;
			src[i*800+j] = 0;
		}
	}

	max = 0;
	for(i=0;i<21;i++){
		for(j=0;j<800;j++){
			if(max<src[i*800+j]){
				max = src[i*800+j];
				mo = i;
				mr = j;
			}
		}
	}

	for(i=0;i<width;i++){
		x = mr-(i*cos10[mo]/10000);
		x = x*10000/sin10[mo];
		if(x>0 && x<height)
			dst[x*width+i] = 0x00F0;
	}
	Xil_DCacheFlush();
}

void img_detectLine(){
	int width = 640;
	int height = 480;
	int row,col;
	int offset = 5;
	uint16_t* src;
	uint16_t* dst;
	uint16_t dot;
	uint32_t r;
	uint32_t max,mo,mr;
	int i,j,x;
	volatile int cos10[21] = {9848,9877,9903,9925,9945,9962,9976,9986,9994,9998,10000,9998,9994,9986,9976,9962,9945,9925,9903,9877,9848};
	volatile int sin10[21] = {-1736,-1564,-1392,-1219,-1045,-872,-698,-523,-349,-175,0,175,349,523,698,872,1045,1219,1392,1564,1736};
	//cos90 = -sin10
	//sin90 = cos10

	src = (uint16_t*)0x1F500000;
	dst = (uint16_t*)0x1F400000;

	memset(src,0,21*800*4);

	xil_printf("prepare ok\r\n");
	for(row=offset;row<height-offset;row++){
		for(col=offset;col<width-offset;col++){
			dot = (dst[row*width+col] & 0x000F);
			if(dot==0)
				continue;
			for(i=0;i<21;i++){
				r = col;
				r = r*cos10[i];
				r += row*sin10[i] + 5000;
				r = r/10000;
				if(r>=0 || r<800){
					src[i*800+r]++;
				}
			}
		}
	}
	xil_printf("created po plane\r\n");
	max = 0;
	for(i=0;i<21;i++){
		for(j=0;j<800;j++){
			if(max<src[i*800+j]){
				max = src[i*800+j];
				mo = i;
				mr = j;
			}
		}
	}

	for(i=0;i<height;i++){
		x = mr-(i*sin10[mo]/10000);
		x = x*10000/cos10[mo];
		if(x>0 && x<width)
			dst[i*width+x] = 0x0F00;
	}

	i = (mo-8<0)?0:(mo-8);
	for(;i<(mo+8);i++){
		if(i>=21)
			break;
		j = (mr-50<0)?0:(mr-50);
		for(;j<(mr+50);j++){
			if(j>=800)
				break;
			src[i*800+j] = 0;
		}
	}

	max = 0;
	for(i=0;i<21;i++){
		for(j=0;j<800;j++){
			if(max<src[i*800+j]){
				max = src[i*800+j];
				mo = i;
				mr = j;
			}
		}
	}

	for(i=0;i<height;i++){
		x = mr-(i*sin10[mo]/10000);
		x = x*10000/cos10[mo];
		if(x>0 && x<width)
			dst[i*width+x] = 0x00F0;
	}
	Xil_DCacheFlush();
}

void img_edge(uint8_t threshold){
	int width = 640;
	int height = 480;
	int16_t tmp1,tmp2;
	int row,col;
	int r,c;
	uint16_t* src;
	uint16_t* dst;
	int8_t coefs1[9] = {-1,0,1,-1,0,1,-1,0,1};
	int8_t coefs2[9] = {-1,-1,-1,0,0,0,1,1,1};


	src = (uint16_t*)0x1F500000;
	dst = (uint16_t*)0x1F400000;

	memcpy(src,dst,640*480*2);

	for(row=1;row<height-1;row++){
		for(col=1;col<width-1;col++){
			tmp1 = 0;
			tmp2 = 0;
			for(r=-1;r<=1;r++){
				for(c=-1;c<=1;c++){
					tmp1 += coefs1[4+r*3+c]*(src[(row+r)*width+(col+c)]&0x000F);
					tmp2 += coefs2[4+r*3+c]*(src[(row+r)*width+(col+c)]&0x000F);
				}
			}
			tmp1 = tmp1*tmp1;
			tmp2 = tmp2*tmp2;
			tmp1 = tmp1+tmp2;
			if(tmp1>=threshold)
				tmp1 = 0x0FFF;
			else
				tmp1 = 0;
			dst[row*width+col] = tmp1;
		}
	}
	Xil_DCacheFlush();
}

void img_removenoise(){
	int width = 640;
	int height = 480;
	int32_t tmp1,tmp2;
	int row,col;
	int r,c;
	int8_t coefs[9] = {1,1,1,1,1,1,1,1,1};
	uint16_t* src;
	uint16_t* dst;

	src = (uint16_t*)0x1F500000;
	dst = (uint16_t*)0x1F400000;

	memcpy(src,dst,640*480*2);

	for(row=1;row<height-1;row++){
		for(col=1;col<width-1;col++){
			tmp1 = 0;
			tmp2 = 0;
			for(r=-1;r<=1;r++){
				for(c=-1;c<=1;c++){
					tmp1 += coefs[4+r*3+c]*(src[(row+r)*width+(col+c)]&0x000F);
				}
			}
			tmp1 = (tmp1/9) &0x000F;
			dst[row*width+col] = (tmp1<<8)+(tmp1<<4)+tmp1;
		}
	}
	Xil_DCacheFlush();
}

void img_grayscale(){
	int width = 640;
	int height = 480;
	uint16_t tmp;
	uint16_t r,g,b;
	int row,col;
	uint16_t* src;

	src = (uint16_t*)0x1F400000;

	for(row=0;row<height;row++){
		for(col=0;col<width;col++){
			r = (src[row*width+col]&0x0F00)>>8;
			g = (src[row*width+col]&0x00F0)>>4;
			b = (src[row*width+col]&0x000F);
			tmp = (r*243+g*410+b*347+500)/1000;
			src[row*width+col] = (tmp<<8)+(tmp<<4)+tmp;
		}
	}
	Xil_DCacheFlush();
}

/**
 * output format: XXXXRRRRGGGGBBBB
 */
void convert_image(){
	int src_width = 1280;
	int src_height = 1024;
	int dst_width = 640;
	int dst_height = 480;

	uint16_t* src;
	uint16_t* dst;
	int row,col;

	src = (uint16_t*)0x1F100000;
	dst = (uint16_t*)0x1F400000;

	for(row=0;row<dst_height;row++){
		for(col=0;col<dst_width;col++){
			uint32_t r;
			r = (src[row*src_width*2+col*2]>>12);
			r += (src[row*src_width*2+(col+1)*2]>>12);
			r += (src[(row+1)*src_width*2+2*col]>>12);
			r += (src[(row+1)*src_width*2+2*(col+1)]>>12);
			r = r/4;
			dst[row*dst_width+col] = (r<<8) + (r<<4) +r;
		}
	}
	Xil_DCacheFlush();
	xil_printf("convert done\r\n");
}

int image_main()
{
	int status;
		uint16_t val;
		uint8_t reg;
		uint32_t pixnum;
		uint32_t tmp;
		int detectEdge = 0;
		int edgeThreshold = 20;
		int detectLine =0;
		int filter0 = 0;
		uint32_t f1coef1 = 0x00010101;
		uint32_t f1coef2 = 0x00010101;
		uint32_t f1coef3 = 0x00010101;
		uint32_t f1coefdiv = 0x00000009;
		uint32_t f2coef1 = 0x00010101;
		uint32_t f2coef2 = 0x00010101;
		uint32_t f2coef3 = 0x00010101;
		uint32_t f2coefdiv = 0x00000009;
		uint32_t f3coef1 = 0x00010101;
		uint32_t f3coef2 = 0x00010101;
		uint32_t f3coef3 = 0x00010101;
		uint32_t f3coefdiv = 0x00000009;

	    xil_printf("Hello World\n\r");
	    gicConfig = XScuGic_LookupConfig(XPAR_PS7_SCUGIC_0_DEVICE_ID);
	    if(gicConfig == NULL){
	    	xil_printf("failed init xscugicr\n");
	    }
	    status = XScuGic_CfgInitialize(&InterruptController,gicConfig,gicConfig->CpuBaseAddress);
	    if(status != XST_SUCCESS){
	    	xil_printf("failed init xscugicr\n");
	    }

	    mt9p031_init();

	    Xil_ExceptionInit();
	    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler)XScuGic_InterruptHandler,&InterruptController);
	    Xil_ExceptionEnable();

	    mt9p031_reset(1);
	    mt9p031_trigger(1);
	    xil_printf("reset on MT9P031\r\n");
	    xil_printf("reset on MT9P031\r\n");
	    xil_printf("reset on MT9P031\r\n");
	    xil_printf("reset on MT9P031\r\n");
	    xil_printf("reset on MT9P031\r\n");
	    xil_printf("reset on MT9P031\r\n");
	    xil_printf("reset on MT9P031\r\n");
	    mt9p031_write(MT9P_ROW_SIZE,0x03FF);
	    mt9p031_write(MT9P_COLUMN_SIZE,0x04FF);
	    mt9p031_write(MT9P_ROW_START,0x0300);
	    mt9p031_write(MT9P_COLUMN_START,0x02DC);
	    mt9p031_write(MT9P_SHUTTER_WIDTH_LOWER,0x0500);

	    for(reg=0;reg<=0x23;reg++){
	    	val = mt9p031_read(reg);
	    	xil_printf("reg%x = %x\r\n",reg,val);
	    }

	    reg = MT9P_READ_MODE1;
	    mt9p031_write(reg,0x4106);
	    val = mt9p031_read(reg);
	    xil_printf("reg%x = %x\r\n",reg,val);

//	    mt9p031_write(75,0x0000);
//	    mt9p031_write(MT9P_READ_MODE2,0x0000);
//	    mt9p031_write(MT9P_TEST_PATTERN_RED,0x0444);
//	    mt9p031_write(MT9P_TEST_PATTERN_GREEN,0x0111);
//	    mt9p031_write(MT9P_TEST_PATTERN_BLUE,0x0888);
//	    mt9p031_write(MT9P_TEST_PATTERN_CTRL,0x0011);

	    pixnum =CAMERA_mReadReg(CAMBASE_ADDR,CAMERA_S00_AXI_SLV_REG0_OFFSET);
	    DEMOSAIC_mWriteReg(XPAR_DEMOSAIC_0_S00_AXI_BASEADDR,DEMOSAIC_S00_AXI_SLV_REG0_OFFSET,1279);

	    xil_printf("demosaic=%d\r\n",DEMOSAIC_mReadReg(XPAR_DEMOSAIC_0_S00_AXI_BASEADDR,DEMOSAIC_S00_AXI_SLV_REG0_OFFSET));
	    xil_printf("pxlnum = %x\r\n",pixnum);
	    CAMERA_mWriteReg(CAMBASE_ADDR,CAMERA_S00_AXI_SLV_REG0_OFFSET,0x140000);
	    pixnum =CAMERA_mReadReg(CAMBASE_ADDR,CAMERA_S00_AXI_SLV_REG0_OFFSET);
	    xil_printf("pxlnum = %x\r\n",pixnum);
	    Xil_SetTlbAttributes(0x1F100000, 0x14de2);
	    Xil_SetTlbAttributes(0x1F200000, 0x14de2);
	    Xil_SetTlbAttributes(0x1F300000, 0x14de2);

	    memset((void*)0x1F100000,0,2*1024*1280);
	    memset((void*)0x1F400000,0,2*640*480);
	    Xil_DCacheFlush();
		while(1){
			vTaskDelay(500);
			mt9p031_resetDMA();
			if(mt9p031_setDMA(0x1F100000,2*1024*1280)!=0)
				continue;
			xil_printf("set up dma complete\r\n");

//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_0_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG0_OFFSET,f1coef1);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_0_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG1_OFFSET,f1coef2);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_0_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG2_OFFSET,f1coef3);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_0_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG3_OFFSET,f1coefdiv);
//
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_1_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG0_OFFSET,f2coef1);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_1_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG1_OFFSET,f2coef2);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_1_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG2_OFFSET,f2coef3);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_1_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG3_OFFSET,f2coefdiv);
//
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_2_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG0_OFFSET,f3coef1);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_2_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG1_OFFSET,f3coef2);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_2_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG2_OFFSET,f3coef3);
//			IMGFILTER_mWriteReg(XPAR_IMGFILTER_2_S00_AXI_BASEADDR,IMGFILTER_S00_AXI_SLV_REG3_OFFSET,f3coefdiv);

			mt9p031_trigger(0);
			mt9p031_trigger(1);
			while(1){
				pixnum = CAMERA_mReadReg(CAMBASE_ADDR,CAMERA_S00_AXI_SLV_REG1_OFFSET);
				if(tmp!=pixnum){
					xil_printf("recvd = %x\r\n",pixnum);
					tmp = pixnum;
					if(tmp==0x140000-1 && pixnum==0x140000-1)
						break;
				}
			}
			mt9p031_wait_dma();
			convert_image();
			if(detectEdge){
				img_edge(edgeThreshold);
				if(detectLine){
					img_detectLine();
					img_detectLine2();
				}
			}
		}

	    cleanup_platform();
	    return 0;
}
