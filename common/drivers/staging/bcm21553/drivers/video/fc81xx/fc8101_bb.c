/*****************************************************************************
 Copyright(c) 2009 FCI Inc. All Rights Reserved
 
 File name : fc8101_bb.c
 
 Description : API of dmb baseband module
 
 History : 
 ----------------------------------------------------------------------
 2009/08/29 	jason		initial
                            BB Config 1p0
*******************************************************************************/

#include <asm/io.h>
#include "fci_types.h"
#include "fci_oal.h"
#include "fci_hal.h"
#include "fci_tun.h"
#include "fc8101_regs.h"

#define BB_SW_RESET_USE 1  //  0: NOT USE    1. USE 

// select BB Xtal frequency
//  16Mhz:16000  16.384MHz:16384   18Mhz:18000   19.2Mhz:19200   24Mhz:24000 
//  26Mhz:26000  32Mhz:32000   	   40Mhz:40000
#define FC8100_FREQ_XTAL_BB    26000	

static int fc8100_bb_pll_set(HANDLE hDevice, u32 xtal)
{
	int res = BBM_NOK;

	if( BB_SW_RESET_USE) {
		res = bbm_write(hDevice, BBM_SYSRST, 0x01);                  // System Reset
		if(res)
			return res;
		msWait(10);
	}

	switch(xtal) {
		case 16000:													
			res = bbm_write(hDevice, BBM_PLL0 , 0x0F);              // PLL 16MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x50);              
			res |= bbm_write(hDevice, BBM_PLL2, 0xBF);              
			break;
		case 16384:													
			res = bbm_write(hDevice, BBM_PLL0 , 0x3F);              // PLL 16.384MHz Setting 
			res |= bbm_write(hDevice, BBM_PLL1, 0x31);              
			res |= bbm_write(hDevice, BBM_PLL2, 0xF3);              
			break;
		case 18000:
			res = bbm_write(hDevice, BBM_PLL0, 0x11);              	// PLL 18MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x50);              
			res |= bbm_write(hDevice, BBM_PLL2, 0xBF);              
			break;
		case 19200:
			res = bbm_write(hDevice, BBM_PLL0, 0x09);              	// PLL 19.2MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x50);              
			res |= bbm_write(hDevice, BBM_PLL2, 0x63);              
			break;
		case 24000:
			res = bbm_write(hDevice, BBM_PLL0, 0x17);              	// PLL 24MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x50);              
			res |= bbm_write(hDevice, BBM_PLL2, 0xBF);              
			break;
		case 26000:
			res = bbm_write(hDevice, BBM_PLL0, 0x19);              	// PLL 26MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x50);              
			res |= bbm_write(hDevice, BBM_PLL2, 0xBF);              
			break;
		case 32000:
			res = bbm_write(hDevice, BBM_PLL0, 0x1F);              	// PLL 32MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x50);              
			res |= bbm_write(hDevice, BBM_PLL2, 0xBF);              
			break;
		case 40000:
			res = bbm_write(hDevice, BBM_PLL0, 0x27);              	// PLL 40MHz Setting
			res |= bbm_write(hDevice, BBM_PLL1, 0x30);              
			res |= bbm_write(hDevice, BBM_PLL2, 0x9F);              
			break;
		default:
			return BBM_NOK;
			break;
	}

	return res;
}

static int fc8100_ofdm_Init(HANDLE hDevice)
{
	int res = BBM_NOK;

	// General Setting
	res = bbm_write(hDevice,BBM_INCTRL, 0x03);              	// IF Low injenction
	//res |= bbm_write(hDevice,BBM_OUTCTRL0 , 0x28);            // TS Setting  0x28
	res |= bbm_write(hDevice,BBM_OUTCTRL0 , 0x2D);              // 
	res |= bbm_write(hDevice,BBM_OUTCTRL1, 0x10);               // TS Serial Out, DTEN High active
	res |= bbm_write(hDevice,BBM_GPIOSEL1, 0xA0);               // PCKETERR, LOCK out
	res |= bbm_write(hDevice,BBM_RSEPER, BER_PACKET_SIZE);      // BER counter update period @ 256 TSP
	res |= bbm_write(hDevice,BBM_RSERST, 0x01);                 // reset for BER counter change 
	res |= bbm_write(hDevice,BBM_THRURS, 0x00);              	// Pst-Rs

	// AGC
	res |= bbm_write(hDevice,BBM_AGTIME, 0x0E);              	// Sync. Speed          
	res |= bbm_write(hDevice,BBM_AGREF1, 0x40);              	// Sensitivity
	
	// Digital AGC Setting
	res |= bbm_write(hDevice,BBM_DAM3REF0, 0x00);              	// multipath measure          
	res |= bbm_write(hDevice,BBM_DAM3REF1, 0x80);              	// multipath measure
	res |= bbm_write(hDevice,BBM_DALPG, 0x01);              	// Shadowing

	// Sync.
	res |= bbm_write(hDevice,BBM_SYATTH, 0x58);              	// Sync. Speed 
	res |= bbm_write(hDevice,BBM_SYSW2, 0x4E);              	// Move Ini. Window Pos. (64point)

	// TMCC
	res |= bbm_write(hDevice,BBM_TMCTRL1, 0xEB);              	// Sync. Speed ([7:5] default)                   
	res |= bbm_write(hDevice,BBM_TMCTRL2, 0x60);              	// Avoid Count Down Error

	// CF Setting
	res |= bbm_write(hDevice,BBM_FTCTRL, 0x33);              	// TU6 & Multipath
	res |= bbm_write(hDevice,BBM_EQCONF, 0x08);              	// Sensitivity

	// De-Noise Filter
	res |= bbm_write(hDevice,BBM_EQCTRL, 0x14);              	// DNF Ini Setting       
	res |= bbm_write(hDevice,BBM_GNFCNT1, 0xE4);              	// DNF On/Off Setting
	res |= bbm_write(hDevice,BBM_WINPTMN, 0x09);              	// 12point from Symbol Head
	res |= bbm_write(hDevice,BBM_IFCTRL, 0x11);              	// for Clip

	// Window Position for 2path Raylength
	res |= bbm_write(hDevice,BBM_WINCTRL1, 0x24);              	// Window Position
	res |= bbm_write(hDevice,BBM_WINCTRL2_1, 0x42);             // move limit & timing

	//GI
	res |= bbm_write(hDevice,BBM_GICCNT1, 0x77);              	// m Up/Dn count slow       
	res |= bbm_write(hDevice,BBM_WINTHTR, 0x0B);              	// Pst-Ghost DU=5dB

	// Ghost
	res |= bbm_write(hDevice,BBM_GNFSNTH, 0x04);              	//Pst-Ghost 11dB       
	res |= bbm_write(hDevice,BBM_GNFCNT2, 0x0A);              	// Skip for 6path Multipath
	res |= bbm_write(hDevice,BBM_WINCTRL, 0x35);              	// Pre-Ghost 13dB
	res |= bbm_write(hDevice,BBM_IFUN, 0x02);              		// Clp Off

	// Switch Interpolation
	res |= bbm_write(hDevice,BBM_EQSI1, 0x3E);              	// Fav with 512symbol       
	res |= bbm_write(hDevice,BBM_EQSI2, 0x10);              	// Threshold

	// External LNA Control
	res |= bbm_write(hDevice,BBM_GPIOSEL0, 0x01);
	res |= bbm_write(hDevice,BBM_AWCTRL, 0x2D);
	res |= bbm_write(hDevice,BBM_AWONTH, 0x20);              	// LNA On CNR = 9sB       
	res |= bbm_write(hDevice,BBM_AWOFFTH, 0x10);              	// LNA Off CNR+13dB
	res |= bbm_write(hDevice,BBM_AWAGCTH, 0x01);

	res |= bbm_write(hDevice,BBM_SYSRST, 0x02);              	// SW Reset

	return res;
}

int fc8101_reset(HANDLE hDevice)
{
	bbm_ext_write(hDevice, BBM_SW_RESET, 0x01); 	//Ext buffer reset
	msWait(1);
	bbm_ext_write(hDevice, BBM_SW_RESET, 0x03);

	return BBM_OK;
}

int fc8101_probe(HANDLE hDevice)
{
	u16 ver;
	bbm_ext_word_read(hDevice, BBM_CHIP_IDL, &ver);
	PRINTF(NULL, "FC8101 PROBE : 0x%x\n", ver);
	return (ver == 0x8101)? BBM_OK : BBM_NOK;
}

int fc8101_ext_init(HANDLE hDevice)
{
	u8 intMask;
	u8 intStatusEn;
		
	bbm_ext_word_write(hDevice, BBM_BUF_THR, (188 * 32)-1);
	bbm_ext_word_write(hDevice, BBM_BUF_END, (188 * 64)-1);
	
	intMask = BBM_MF_INT;
	intStatusEn = BBM_MF_INT;
	bbm_ext_write(hDevice, BBM_INT_MASK, intMask);
	bbm_ext_write(hDevice, BBM_STATUS_ENABLE, intStatusEn);

	fc8101_reset(hDevice);

	return BBM_OK;
}

int fc8101_init(HANDLE hDevice)
{
	int res = BBM_NOK;

	res = fc8101_ext_init(hDevice);
	res |= fc8100_bb_pll_set( hDevice, FC8100_FREQ_XTAL_BB);
	res |= fc8100_ofdm_Init( hDevice);

	return res;
}

int fc8101_deinit(HANDLE hDevice)
{
	int res = BBM_NOK;
	
	res = bbm_ext_write(hDevice, BBM_SW_RESET, 0x00);
	
	return res;
}

int fc8101_scan_status(HANDLE hDevice)
{
	int res = BBM_NOK;
	int i;
	unsigned char lock_data;
	unsigned char first_timeout = 6;
	unsigned char second_timeout = 70;

	for(i=0; i<first_timeout; i++)
	{
		res = bbm_read(hDevice, BBM_STATEF, &lock_data);

		if(res)
			return res;

		if(lock_data >= 0x4)
		{
			res = BBM_OK;
			break;
		}

		msWait(10);
	}

	if(i == first_timeout)
		return BBM_NOK;

	for(i=0; i<second_timeout; i++)
	{
		res = bbm_read(hDevice, BBM_STATEF, &lock_data);
		
		if(res)
			return res;

		if(lock_data == 0xA)
		{
			res = BBM_OK;
			break;
		}
		msWait(10);
	}

	if(i == second_timeout)
		return BBM_NOK;

	return res;
}



