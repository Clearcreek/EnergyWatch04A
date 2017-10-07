/*******************************************************************************
* File Name: stpm33.h
*
* Version 1.0
*
* Description:
*  HRS service related code header.
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(STPM33_H)
#define STPM33_H

#include "PM_EN.h"
#include "PM_SYN.h"
#include "SPI_AFE_SPI_UART.h"
#include "SysTickAppTimer.h"

/***************************************
*        Constant definitions
***************************************/
// Register Addresses
#define REG_DSP_CR1         (0x00)
#define REG_DSP_CR2         (0x02)
#define REG_DSP_CR3         (0x04)
#define REG_DSP_CR4         (0x06)
#define REG_DSP_CR5         (0x08)
#define REG_DSP_CR6         (0x0A)
#define REG_DSP_CR7         (0x0C)
#define REG_DSP_CR8         (0x0E)
#define REG_DSP_CR9         (0x10)
#define REG_DSP_CR10        (0x12)
#define REG_DSP_CR11        (0x14)
#define REG_DSP_CR12        (0x16)
#define REG_DFE_CR1         (0x18)
#define REG_DFE_CR2         (0x1A)
#define REG_DSP_IRQ1        (0x1C)
#define REG_DSP_IRQ2        (0x1E)
#define REG_DSP_SR1         (0x20)
#define REG_DSP_SR2         (0x22)
#define REG_US_REG1         (0x24)
#define REG_US_REG2         (0x26)
#define REG_US_REG3         (0x28)
#define REG_DSP_EV1         (0x2A)
#define REG_DSP_EV2         (0x2C)
#define REG_PH_PERIOD       (0x2E)  // dsp_reg1
#define REG_V1_DATA         (0x30)  // dsp_reg2
#define REG_C1_DATA         (0x32)  // dsp_reg3
#define REG_V2_DATA         (0x34)  // dsp_reg4
#define REG_C2_DATA         (0x36)  // dsp_reg5
#define REG_V1_FUND         (0x38)  // dsp_reg6
#define REG_C1_FUND         (0x3A)  // dsp_reg7
#define REG_V2_FUND         (0x3C)  // dsp_reg8
#define REG_C2_FUND         (0x3E)  // dsp_reg9
#define REG_DSP_REG10       (0x40)  // Reserved
#define REG_DSP_REG11       (0x42)  // Reserved
#define REG_DSP_REG12       (0x44)  // Reserved
#define REG_DSP_REG13       (0x46)  // Reserved
#define REG_C1V1_RMS        (0x48)  // dsp_reg14
#define REG_C2V2_RMS        (0x4A)  // dsp_reg15
#define REG_SAG1SWV1_T      (0x4C)  // dsp_reg16
#define REG_C1PHA_SWC1T     (0x4E)  // dsp_reg17
#define REG_SAG2SWV2_T      (0x50)  // dsp_reg18
#define REG_C2PHA_SWC2T     (0x52)  // dsp_reg19
#define REG_PH1_ACTIVE_E    (0x54)  // ph1_reg1
#define REG_PH1_FUNDAMNT_E  (0x56)  // ph1_reg2
#define REG_PH1_REACTIVE_E  (0x58)  // ph1_reg3
#define REG_PH1_APPARENT_E  (0x5A)  // ph1_reg4
#define REG_PH1_ACTIVE_P    (0x5C)  // ph1_reg5
#define REG_PH1_FUNDAMNT_P  (0x5E)  // ph1_reg6
#define REG_PH1_REACTIVE_P  (0x60)  // ph1_reg7
#define REG_PH1_APPRNT_RMSP (0x62)  // ph1_reg8  PH1 apparent RMS power 
#define REG_PH1_APPRNT_VECP (0x64)  // ph1_reg9  PH1 apparent ventorial power
#define REG_PH1_MOMNT_ACTP  (0x66)  // ph1_reg10 PH1 momentary avtive power
#define REG_PH1_MOMNT_FUNDP (0x68)  // ph1_reg11 PH1 momentary fundamental power
#define REG_PH1_AH_ACC      (0x6A)  // ph1_reg12
#define REG_PH2_ACTIVE_E    (0x6C)  // ph2_reg1
#define REG_PH2_FUNDAMNT_E  (0x6E)  // ph2_reg2
#define REG_PH2_REACTIVE_E  (0x70)  // ph2_reg3
#define REG_PH2_APPRNT_RMSE (0x72)  // ph2_reg4
#define REG_PH2_ACTIVE_P    (0x74)  // ph2_reg5
#define REG_PH2_FUNDAMNT_P  (0x76)  // ph2_reg6
#define REG_PH2_REACTIVE_P  (0x78)  // ph2_reg7
#define REG_PH2_APPRNT_RMSP (0x7A)  // ph2_reg8  PH2 apparent RMS power
#define REG_PH2_APPRNT_VECP (0x7C)  // ph2_reg9  PH2 apparent vectorial power
#define REG_PH2_MOMNT_ACTP  (0x7E)  // ph2_reg10 PH2 momentary active power
#define REG_PH2_MOMNT_FUNDP (0x80)  // ph2_reg11 PH2 momentary fundamental power
#define REG_PH2_AH_ACC      (0x82)  // ph2_reg12
#define REG_TOT_ACTIVE_E    (0x84)  // tot_reg1
#define REG_TOT_FUNDAMNT_E  (0x86)  // tot_reg2
#define REG_TOT_REACTIVE_E  (0x88)  // tot_reg3
#define REG_TOT_APPRNT_E    (0x8A)  // tot_reg4
#define REG_READ_ADD_INC    (0xFF)
#define REG_WRITE_DUMMY     (0xFF)
#define REGISTER_READ       (0x00)
#define REGISTER_WRTL       (0x00)
#define REGISTER_WRTH       (0x01)

#define STPM3x_FRAME_LEN    (4u)    // without CRC byte
#define CRC8_POLY           (0x07)  // Defualt polynomial is 0x07 (x8+x2+x1+1 = 100000111)
    
/***************************************
*            Data Types
***************************************/

#define Vrms_LSB        (2411u)   // 0.02411 V/LSB
#define Vrms_DIVIDER    (1000u)   // Resulting calculated Vrms with 0.1V resolution
#define Irms_LSB        (547u)    // 0.000547 A/LSB)
#define Irms_DIVIDER    (1000u)   // Resulting calculated Irms with 0.01A resolution
#define POWER_LSB       (162u)    // 0.00016165 W/LSB
#define POWER_DIVIDER   (10000u)  // Resulting the calculated power with 0.1W resolution
#define ENERGY_LSB      (154u)    // 0.0000000154 Wh/LSB = 0.0389 mWs/LSB
#define ENERGY_DIVIDER  (100000000u)  // Resulting the calculated energy with 0.1Wh resolution
#define THREASHOULD     (2147483647u)
#define MAX_UINT32      (4294967296u)
    
    
/***************************************
*        Function Prototypes
***************************************/

/*******************************************************************************
* Function Name: STPM3x_InterfaceSel
********************************************************************************
*
* Summary:
*  Reset STPM34x device by
*  1. Set EN, SCS low (slect SPI interface), SYN high for 10ms.
*  2. Set EN high and wait 5ms.
*
* Parameters:
*  Nothing
*
* Return:
*  None
*
*******************************************************************************/
void STPM3x_InterfaceSel(void);

/*******************************************************************************
* Function Name: STPM3x_HardReset
********************************************************************************
*
* Summary:
*  Reset STPM34x device by applying SYN and SCS sequences
*  1. Set SCS high and wait 35ms.
*  2. Generate 3 nagtive pulses on SYN line with 4ms period and 2ms width.
*  3. Then wait 2ms to generate a 2ms width nagtive pulse on SCS line. 
*
* Parameters:
*  Nothing
*
* Return:
*  None
*
*******************************************************************************/
void STPM3x_HardReset(void);

/*******************************************************************************
* Function Name: STPM3x_SoftReset
********************************************************************************
*
* Summary:
*  Reset Data register by setting the bit
*
* Parameters:
*  Nothing
*
* Return:
*  None
*
*******************************************************************************/
void STPM3x_SoftReset(void);

/*******************************************************************************
* Function Name: STPM3x_Init
********************************************************************************
*
* Summary:
*  STPM3x initialization function.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_Init(void);

/*******************************************************************************
* Function Name: STPM3x_Start
********************************************************************************
*
* Summary:
*  Enable STPM3x device start to operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_Start(void);

/*******************************************************************************
* Function Name: STPM3x_HardDataLatch
********************************************************************************
*
* Summary:
*  Lathc the data register contents by applying SYN and SCS sequence
*  1. Set SYN low for 2ms while SCS high, and wait 2mS.
*  2. Set SCS low for 2mS.
*
* Parameters:
*  Nothing
*
* Return:
*  None
*
*******************************************************************************/
void STPM3x_HardDataLatch(void);

/*******************************************************************************
* Function Name: STPM3x_SoftDataLatch
********************************************************************************
*
* Summary:
*  Lathc the data register contents by setting the bit
*
* Parameters:
*  Nothing
*
* Return:
*  None
*
*******************************************************************************/
void STPM3x_SoftDataLatch(void);

void STPM3x_EnableCRC(void);

void STPM3x_DisableCRC(void);

/*******************************************************************************
* Function Name: STPM3x_DataExchange
********************************************************************************
*
* Summary:
*  Send read/write address and the data to write to the device, and store the
*  replied 4 bytes data into the buffer.
*
* Parameters:
*  *pCmd - Pointer to the buffer contain read/write address and the data to 
*           write, must be at least contaoin 4 bytes space
*
* Return:
*  CRC check status, 0 = Ok, 1 = CRC check fail.
*
*******************************************************************************/
uint8 STPM3x_DataExchange(uint8 *pCmd);

/*******************************************************************************
* Function Name: STPM3x_GetRegData
********************************************************************************
*
* Summary:
*  .
*
* Parameters:
*  *p_Buffer.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_GetRegData(uint8 *p_Buffer);

/*******************************************************************************
* Function Name: STPM3x_PreRead
********************************************************************************
*
* Summary:
*  Move the internal pointer to access the register dsp_reg1.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_PreRead(void);

/*******************************************************************************
* Function Name: STPM3x_ReadPeriods
********************************************************************************
*
* Summary:
*  Read Phase 1 and Phase 2 period, then move the internal pointer to access 
*  the register dsp_reg14 to read current and voltage of the phase 1.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadPeriods(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_ReadCVRMSPha1
********************************************************************************
*
* Summary:
*  Read current and voltage on the Phase 1, then move the internal pointer to 
*  access the register dsp_reg15 to read current and voltage of the phase 2.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadCVRMSPha1(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_ReadCVRMSPha2
********************************************************************************
*
* Summary:
*  Read current and voltage on the Phase 2, then move the internal pointer to 
*  access the register ph1_reg1 to read active energy of the phase 1.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadCVRMSPha2(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_ReadActiveEPha1
********************************************************************************
*
* Summary:
*  Read active energy of the Phase 1, then move the internal pointer to 
*  access the register ph1_reg5 to read active power of the phase 1.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadActiveEPha1(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_ReadActivePwrPha1
********************************************************************************
*
* Summary:
*  Read active power of the Phase 1, then move the internal pointer to 
*  access the register ph2_reg1 to read active energy of the phase 2.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadActivePwrPha1(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_ReadActiveEPha2
********************************************************************************
*
* Summary:
*  Read active energy of the Phase 2, then move the internal pointer to 
*  access the register ph2_reg5 to read active power of the phase 2.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadActiveEPha2(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_ReadActivePwrPha2
********************************************************************************
*
* Summary:
*  Read active power of the Phase 2, then move the internal pointer to 
*  access the register dsp_reg1 to read periods next time.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadActivePwrPha2(uint8 *pStore);

/*******************************************************************************
* Function Name: STPM3x_HardDataReset
********************************************************************************
*
* Summary:
*  .
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_HardDataReset(void);

void STPM3x_SoftDataReset(void);

void STPM3x_HardDspReser(void);

void STPM3x_SoftDspReset(void);

void STPM3x_CalcCRC8(uint8 *pBuffer);

int32 STPM3x_GetTotalEnergy1(void);

int32 STPM3x_GetTotalEnergy2(void);
    
/***************************************
*      External data references
***************************************/

#endif /* STPM33_H */

/* [] END OF FILE */
