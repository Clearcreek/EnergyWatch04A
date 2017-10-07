/*******************************************************************************
* File Name: STPM3x_Drv.c
*
* Version 1.0
*
* Description:
*  STPM3x driver.
*
* Hardware Dependency:
* 
*
********************************************************************************
* Copyright 2017, Intellisys Controls Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "stpm33.h"

/*******************************************************************************
* Local Variables
********************************************************************************/
static uint8 Stpm3x_Enabled = 0u;
static uint8 Stpm3x_CRCEnabled = 1u;  // CRC is enabled by STPM3x default
//static uint8 Stpm3x_ValidData = 0u;
static uint8 Stpm3x_CRC8Checksum;
static uint8 Stpm3x_CharRead[STPM3x_FRAME_LEN];
static int32 Total_Energy1;
static int32 Total_Energy2;

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
void STPM3x_InterfaceSel(void)
{
    PM_EN_Write(0);
    SPI_AFE_ss0_m_Write(0);
    PM_SYN_Write(1);
    SysTickApp_SetSTPM33Timer(10);
    while(SysTickApp_GetSTPM33Timer() != 0);
    // Select the communication interface
    PM_EN_Write(1);
    SysTickApp_SetSTPM33Timer(5);
    while(SysTickApp_GetSTPM33Timer() != 0);
    SPI_AFE_ss0_m_Write(1);
}

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
void STPM3x_HardReset(void)
{
    // Reset STPM3x
    SysTickApp_SetSTPM33Timer(35);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(0);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(1);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(0);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(1);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(0);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(1);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    // Reset communication interface
    SPI_AFE_ss0_m_Write(0);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    SPI_AFE_ss0_m_Write(1);
    
    Stpm3x_Enabled = 0;
}

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
void STPM3x_SoftReset(void)
{
}

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
void STPM3x_Init(void)
{
    uint8 static ChanGainC[STPM3x_FRAME_LEN];
    // Set current channle gains to 2:
    // Use read-modify-write procedure to set the GAIN1 and GAIN2 of 
    // register dfe_cr1 and dfe_cr2, then move the internal pinter to
    // access the register dsp_cr1 at the address 0x00
    
    ChanGainC[0] = (REG_DFE_CR1 | REGISTER_READ);  // Move the internal pointer to access dfe_cr1
    ChanGainC[1] = REG_WRITE_DUMMY;  // Write nothing
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dsp_cr1 should be returned by default
    
    // Read the content of dfe_cr1 and move the internal pointer to
    // access dfe_cr2
    ChanGainC[0] = (REG_DFE_CR2 | REGISTER_READ);
    ChanGainC[1] = REG_WRITE_DUMMY;  // Write nothing
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dfe_cr1 returned
    // Modify the current gain for channel 1
    ChanGainC[3] &= 0xF3;
    // Read the content of dfe_cr2, write the current gain of channel 1,
    // and move the internal pointer to access dsp_cr1
    ChanGainC[0] = (REG_DSP_CR1 | REGISTER_READ);
    ChanGainC[1] = (REG_DFE_CR1 | REGISTER_WRTH);
    STPM3x_DataExchange(ChanGainC);
    // Modify the current gain for channel 2
    STPM3x_GetRegData(ChanGainC); // dfe_cr2
    ChanGainC[3] &= 0xF3;
    // Write the current gain of channel 2, read dsp_cr1
    // and move the internal pointer to access dsp_cr2
    ChanGainC[0] = (REG_DSP_CR2 | REGISTER_READ);
    ChanGainC[1] = (REG_DFE_CR2 | REGISTER_WRTH);
    STPM3x_DataExchange(ChanGainC);
    // Modify the LPW1 for channel 1
    STPM3x_GetRegData(ChanGainC);  // dsp_cr1
    
    ChanGainC[3] &= 0x00;
    ChanGainC[3] |= 0x0A;
    // Write the new LPW1 to the register, and read dfe_cr1
    ChanGainC[0] = (REG_DFE_CR1 | REGISTER_READ);
    ChanGainC[1] = (REG_DSP_CR1 | REGISTER_WRTH);
    STPM3x_DataExchange(ChanGainC);
    // Modify the LPW2 for channel 2
    STPM3x_GetRegData(ChanGainC);  // dsp_cr2
    
    ChanGainC[3] &= 0x00;  // Set LED2 to show up active energy on the channel 2
    ChanGainC[3] |= 0x0A;
    // Write the new LPW2 to the register, and move the internal
    // pointer to access dsp_cr1
    ChanGainC[0] = (REG_DFE_CR2 | REGISTER_READ);
    ChanGainC[1] = (REG_DSP_CR2 | REGISTER_WRTH);
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dfe_cr1
    
    ChanGainC[0] = (REG_DSP_CR1 | REGISTER_READ);  // Move the internal pointer to access dsp_cr1
    ChanGainC[1] = REG_WRITE_DUMMY;  // Write nothing
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dfe_cr2 should be returned
    
    ChanGainC[0] = (REG_DSP_CR2 | REGISTER_READ);  // Move the internal pointer to access dsp_cr2
    ChanGainC[1] = REG_WRITE_DUMMY;  // Write nothing
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dsp_cr1 should be returned
    
    // Move the internal pointer to access dsp_cr1
    ChanGainC[0] = (REG_DSP_CR1 | REGISTER_READ);  // Move the internal pointer to access dsp_cr1
    ChanGainC[1] = REG_WRITE_DUMMY;  // Write nothing
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dsp_cr2
    
    // Following code is for test
    /*ChanGainC[0] = (REG_DSP_CR2 | REGISTER_READ);  // Move the internal pointer to access dsp_cr2
    ChanGainC[1] = REG_WRITE_DUMMY;  // Write nothing
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dsp_cr1
    
    ChanGainC[0] = (REG_DFE_CR1 | REGISTER_READ);
    ChanGainC[1] = REG_WRITE_DUMMY;
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // dsp_cr2
    
    ChanGainC[0] = (REG_DFE_CR2 | REGISTER_READ);
    ChanGainC[1] = REG_WRITE_DUMMY;
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);  // def_cr1
    
    ChanGainC[0] = (REG_DSP_CR1 | REGISTER_READ);
    ChanGainC[1] = REG_WRITE_DUMMY;
    ChanGainC[2] = 0xFF;
    ChanGainC[3] = 0xFF;
    STPM3x_DataExchange(ChanGainC);
    STPM3x_GetRegData(ChanGainC);*/  // def_cr2
}

/*******************************************************************************
* Function Name: STPM3x_Start
********************************************************************************
*
* Summary:
*  Rest and enable STPM3x device starts to operation.
*
* Parameters:
*  None.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_Start(void)
{
    STPM3x_InterfaceSel();
    STPM3x_HardReset();
    
    if(Stpm3x_Enabled == 0u)
    {
        STPM3x_Init();
        Stpm3x_Enabled = 1u;
    }
}

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
void STPM3x_HardDataLatch(void)
{
    SPI_AFE_ss0_m_Write(1);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(0);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    PM_SYN_Write(1);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    SPI_AFE_ss0_m_Write(0);
    SysTickApp_SetSTPM33Timer(2);
    while(SysTickApp_GetSTPM33Timer() != 0);
    SPI_AFE_ss0_m_Write(1); 
}

/*******************************************************************************
* Function Name: STPM3x_SoftLatch
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
void STPM3x_SoftLatch(void)
{
}

void STPM3x_EnableCRC(void)
{
    uint8 EnableCRC[STPM3x_FRAME_LEN];
    
    // Use read-modify-write procedure to set the bit14 of register US_REG1
    // Move the register address to point to US_REG1
    EnableCRC[0] = (REG_US_REG1 | REGISTER_READ);
    EnableCRC[1] = REG_WRITE_DUMMY;
    EnableCRC[2] = 0xFF;
    EnableCRC[3] = 0xFF;
    STPM3x_DataExchange(EnableCRC);
    // Read the content of US_REG1
    STPM3x_DataExchange(EnableCRC);
    STPM3x_GetRegData(EnableCRC);
    // to enable CRC (CRC is enabled by default)
    EnableCRC[2] = EnableCRC[0];
    EnableCRC[3] = (EnableCRC[1] | 0x40);
    EnableCRC[0] = REG_READ_ADD_INC;
    EnableCRC[1] = (REG_US_REG1 | REGISTER_WRTL);
    STPM3x_DataExchange(EnableCRC);
}

void STPM3x_DisableCRC(void)
{
    uint8 DisableCRC[STPM3x_FRAME_LEN];
    
    // Use read-modify-write procedure to set the bit14 of register US_REG1
    // Move the register address to point to US_REG1
    DisableCRC[0] = (REG_US_REG1 | REGISTER_READ);
    DisableCRC[1] = REG_WRITE_DUMMY;
    DisableCRC[2] = 0xFF;
    DisableCRC[3] = 0xFF;
    STPM3x_DataExchange(DisableCRC);
    // Read the content of US_REG1
    STPM3x_DataExchange(DisableCRC);
    STPM3x_GetRegData(DisableCRC);
    // to enable CRC (CRC is enabled by default)
    DisableCRC[2] = DisableCRC[0];
    DisableCRC[3] = (DisableCRC[1] & 0xBF);
    DisableCRC[0] = REG_READ_ADD_INC;
    DisableCRC[1] = (REG_US_REG1 | REGISTER_WRTL);
    STPM3x_DataExchange(DisableCRC);
}

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
uint8 STPM3x_DataExchange(uint8 *pCmd) // teste ok
{
    uint8 TempFrame[STPM3x_FRAME_LEN+1];
    uint8 FrameIndex;
    uint8 FrameSize;
    uint8 crc_status; // 0 = Ok, 1 = Error
    
    for(FrameIndex = 0; FrameIndex < STPM3x_FRAME_LEN; FrameIndex++)
    {
        TempFrame[FrameIndex] = pCmd[FrameIndex];
    }
    FrameSize = STPM3x_FRAME_LEN;
    if(Stpm3x_CRCEnabled != 0)
    {
        STPM3x_CalcCRC8(pCmd);
        TempFrame[STPM3x_FRAME_LEN] = Stpm3x_CRC8Checksum;
        FrameSize = (STPM3x_FRAME_LEN+1);
    }
    // Send data here
    SPI_AFE_ss0_m_Write(0);
    SPI_AFE_SpiUartPutArray((const uint8 *)TempFrame, (uint32)FrameSize);
    // Get return data here
    while(SPI_AFE_SpiUartGetRxBufferSize() < FrameSize);
    SPI_AFE_ss0_m_Write(1);
    for(FrameIndex = 0; FrameIndex < FrameSize; FrameIndex++)
    {
        TempFrame[FrameIndex] = SPI_AFE_SpiUartReadRxData();
    }
    SPI_AFE_SpiUartClearRxBuffer();
    
    for(FrameIndex = 0; FrameIndex < STPM3x_FRAME_LEN; FrameIndex++)
    {
        Stpm3x_CharRead[FrameIndex] = TempFrame[FrameIndex];
    }
    
    crc_status = 0;
    if(Stpm3x_CRCEnabled != 0)
    {
        for(FrameIndex = 0; FrameIndex < STPM3x_FRAME_LEN; FrameIndex++)
        {
            pCmd[FrameIndex] = TempFrame[FrameIndex];
        }
        STPM3x_CalcCRC8(pCmd);
        if((TempFrame[4] ^ Stpm3x_CRC8Checksum) != 0)
        {
            // CRC error
            crc_status = 1;
        }
    }
    
    return crc_status;
}

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
void STPM3x_GetRegData(uint8 *p_Buffer)
{
    p_Buffer[0] = Stpm3x_CharRead[0];
    p_Buffer[1] = Stpm3x_CharRead[1];
    p_Buffer[2] = Stpm3x_CharRead[2];
    p_Buffer[3] = Stpm3x_CharRead[3];
}

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
void STPM3x_PreRead(void)
{
    uint8 data_exchgd[STPM3x_FRAME_LEN];
    
    data_exchgd[0] = (REG_PH_PERIOD | REGISTER_READ);
    data_exchgd[1] = REG_WRITE_DUMMY;
    data_exchgd[2] = REG_WRITE_DUMMY;
    data_exchgd[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_exchgd);
}

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
void STPM3x_ReadPeriods(uint8 *pStore)
{
    uint8 data_period[STPM3x_FRAME_LEN];
    uint32 period;
    uint16 frequency;
    
    data_period[0] = (REG_C1V1_RMS | REGISTER_READ);
    data_period[1] = REG_WRITE_DUMMY;
    data_period[2] = REG_WRITE_DUMMY;
    data_period[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_period);
    STPM3x_GetRegData(data_period);
    period = (((uint16)data_period[1] << 8) | (uint16)data_period[0]);
    period &= 0x00FFFFFF;
    frequency = 1250000/period;
    frequency = (frequency + 5)/10;
    pStore[0] = (uint8)frequency; // channel 1 frequency
    period = (((uint16)data_period[3] << 8) | (uint16)data_period[2]);
    period &= 0x00FFFFFF;
    frequency = 1250000/period;
    frequency = (frequency + 5)/10;
    pStore[1] = frequency; // channel 2 frequency
}

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
void STPM3x_ReadCVRMSPha1(uint8 *pStore)
{
    uint8 data_cvph1[STPM3x_FRAME_LEN];
    uint32 VIrms1_read;
    uint32 VIrms1_calc;
    
    data_cvph1[0] = (REG_C2V2_RMS | REGISTER_READ);
    data_cvph1[1] = REG_WRITE_DUMMY;
    data_cvph1[2] = REG_WRITE_DUMMY;
    data_cvph1[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_cvph1);
    STPM3x_GetRegData(data_cvph1);
    
    // Calculat Vrms
    VIrms1_read = (uint32)data_cvph1[1];
    VIrms1_read = ((VIrms1_read << 8) | (uint32)data_cvph1[0]);
    VIrms1_read &= 0x00007FFF;  // keep lower 15 bits
    VIrms1_calc = (VIrms1_read * Vrms_LSB)/Vrms_DIVIDER;
    VIrms1_calc = (VIrms1_calc + 5)/10;  // Round up on 0.5
    pStore[1] = (uint8)(VIrms1_calc & 0x000000FF);
    pStore[2] = (uint8)((VIrms1_calc >> 8) & 0x0000001F);  // Resulting 13 bits wide calculated Vrms
    // Calcilate Irms
    VIrms1_read = (uint32)data_cvph1[3];
    VIrms1_read = ((VIrms1_read << 8) | (uint32)data_cvph1[2]);
    VIrms1_read = ((VIrms1_read << 1) | (uint32)(data_cvph1[1] >> 7));
    VIrms1_calc = (VIrms1_read * Irms_LSB)/Irms_DIVIDER;
    VIrms1_calc = (VIrms1_calc + 5)/10;  // Round up on 0.5
    pStore[2] |= (uint8)((VIrms1_calc << 5) & 0x000000E0);
    pStore[3] = (uint8)((VIrms1_calc >> 3) & 0x000000FF);  // Resulting 11 bits wide calculated Irms
    pStore[0] = 1;  // Vrms and Irms of Phase 1 
}

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
void STPM3x_ReadCVRMSPha2(uint8 *pStore)
{
    uint8 data_cvph2[STPM3x_FRAME_LEN];
    uint32 VIrms2_read;
    uint32 VIrms2_calc;
    
    data_cvph2[0] = (REG_PH1_ACTIVE_E | REGISTER_READ);
    data_cvph2[1] = REG_WRITE_DUMMY;
    data_cvph2[2] = REG_WRITE_DUMMY;
    data_cvph2[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_cvph2);
    STPM3x_GetRegData(data_cvph2);
    
    // Calculat Vrms
    VIrms2_read = (uint32)data_cvph2[1];
    VIrms2_read = ((VIrms2_read << 8) | (uint32)data_cvph2[0]);
    VIrms2_read &= 0x00007FFF;  // keep lower 15 bits
    VIrms2_calc = (VIrms2_read * Vrms_LSB)/Vrms_DIVIDER;
    VIrms2_calc = (VIrms2_calc + 5)/10;  // ROund up on 0.5
    pStore[1] = (uint8)(VIrms2_calc & 0x000000FF);
    pStore[2] = (uint8)((VIrms2_calc >> 8) & 0x0000001F);  // Resulting 13 bits wide calculated Vrms
    // Calculate Irms
    VIrms2_read = (uint32)data_cvph2[3];
    VIrms2_read = ((VIrms2_read << 8) | (uint32)data_cvph2[2]);
    VIrms2_read = ((VIrms2_read << 1) | (uint32)(data_cvph2[1] >> 7));
    VIrms2_calc = (VIrms2_read * Irms_LSB)/Irms_DIVIDER;
    VIrms2_calc = (VIrms2_calc + 5)/10;  // ROund up on 0.5
    pStore[2] |= (uint8)((VIrms2_calc << 5) & 0x000000E0);
    pStore[3] = (uint8)((VIrms2_calc >> 3) & 0x000000FF);  // Resulting 11 bits wide calculated Irms
    pStore[0] = 2;  // Vrms and Irms of Phase 2
}

/*******************************************************************************
* Function Name: STPM3x_ReadActiveEPha1
********************************************************************************
*
* Summary:
*  Read active energy of the Phase 1, then move the internal pointer to 
*  access the register ph1_reg5 to read active power of the phase 1.
*  The function shall be called every 1 second.
*
* Parameters:
*  pStore - the pointer to the place holder storing the periods, the holder 
*           length must be STPM3x_FRAME_LEN long.
*
* Return:
*  None.
*
*******************************************************************************/
void STPM3x_ReadActiveEPha1(uint8 *pStore)
{
    static uint32 lastE1_read;
    static uint8 firstE1_read = 1;
    static uint8 energy1_timer = 1;
    uint8 data_activE1[STPM3x_FRAME_LEN];
    uint32 crntE1_read;
    int32 diffE1_read;
    uint8 diffE1_calc;
    
    data_activE1[0] = (REG_PH1_ACTIVE_P | REGISTER_READ);
    data_activE1[1] = REG_WRITE_DUMMY;
    data_activE1[2] = REG_WRITE_DUMMY;
    data_activE1[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_activE1);
    STPM3x_GetRegData(data_activE1);
    
    crntE1_read = (uint32)data_activE1[3];
    crntE1_read = ((crntE1_read << 8) | (uint32)data_activE1[2]);
    crntE1_read = ((crntE1_read << 8) | (uint32)data_activE1[1]);
    crntE1_read = ((crntE1_read << 8) | (uint32)data_activE1[0]);
    
    Total_Energy1 = ((int64)crntE1_read * ENERGY_LSB)/(1000*ENERGY_DIVIDER);
    if(Total_Energy1 >= 0)
    {
        Total_Energy1 += 5;
    }
    else
    {
        Total_Energy1 -= 5;
    }
    Total_Energy1 = Total_Energy1/10;
        
    pStore[0] = 3;  // Data type: Power and energy of the phase 1
    pStore[1] = 0;
    pStore[2] = 0;
    pStore[3] = 0;
    
    if(++energy1_timer == 2)
    {
        energy1_timer = 0;
        // Calculate energy increment in 10 seconds
        if(firstE1_read != 0)
        {
            firstE1_read = 0;
            lastE1_read = crntE1_read;
        }
        else
        {
            diffE1_read = (int32)(crntE1_read - lastE1_read);
            lastE1_read = crntE1_read;
            diffE1_read = ((int64)diffE1_read * ENERGY_LSB);
            diffE1_read = ((int64)diffE1_read/ENERGY_DIVIDER);
            if(diffE1_read < 0)
            {
                diffE1_read -= 5;
            }
            else if(diffE1_read > 5)
            {
                diffE1_read += 5;
            }
            diffE1_calc = (int8)(diffE1_read/10);
            pStore[3] = (uint8)diffE1_calc;
        }
    }
}

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
void STPM3x_ReadActivePwrPha1(uint8 *pStore)
{
    uint8 data_activP1[STPM3x_FRAME_LEN];
    int32 pwr1_read;
    int64 pwr1_calc;
    uint8 sign1;
    
    data_activP1[0] = (REG_PH2_ACTIVE_E | REGISTER_READ);
    data_activP1[1] = REG_WRITE_DUMMY;
    data_activP1[2] = REG_WRITE_DUMMY;
    data_activP1[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_activP1);
    STPM3x_GetRegData(data_activP1);
    
    sign1 = 0;
    // Calculate power of the phase 1
    pwr1_read = data_activP1[3];
    pwr1_read = ((pwr1_read << 8) | data_activP1[2]);
    pwr1_read = ((pwr1_read << 8) | data_activP1[1]);
    pwr1_read = ((pwr1_read << 8) | data_activP1[0]);
    pwr1_calc = (((int64)pwr1_read * POWER_LSB)/POWER_DIVIDER);
    // Round up on 0.5
    if(pwr1_calc < 0)
    {
        sign1 = 0x80;
        pwr1_calc = -pwr1_calc;
    }
    pwr1_calc += 5;
    pwr1_calc = (pwr1_calc/10);
    pStore[0] = 3;  // Data type: Power and energy of the phase 1
    pStore[1] = ((uint8)(pwr1_calc & 0x00000000000000FF));
    pStore[2] = (((uint8)((pwr1_calc >> 8) & 0x00000000000000FF)) | sign1);
}

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
void STPM3x_ReadActiveEPha2(uint8 *pStore)
{
    static uint32 lastE2_read;
    static uint8 firstE2_read = 1;
    static uint8 energy2_timer = 1;
    uint8 data_activE2[STPM3x_FRAME_LEN];
    uint32 crntE2_read;
    int32 diffE2_read;
    uint8 diffE2_calc;
    
    data_activE2[0] = (REG_PH2_ACTIVE_P | REGISTER_READ);
    data_activE2[1] = REG_WRITE_DUMMY;
    data_activE2[2] = REG_WRITE_DUMMY;
    data_activE2[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_activE2);
    STPM3x_GetRegData(data_activE2);
    
    crntE2_read = (uint32)data_activE2[3];
    crntE2_read = ((crntE2_read << 8) | (uint32)data_activE2[2]);
    crntE2_read = ((crntE2_read << 8) | (uint32)data_activE2[1]);
    crntE2_read = ((crntE2_read << 8) | (uint32)data_activE2[0]);
    
    Total_Energy2 = ((int64)crntE2_read * ENERGY_LSB)/(1000*ENERGY_DIVIDER);
    if(Total_Energy2 >= 0)
    {
        Total_Energy2 += 5;
    }
    else
    {
        Total_Energy2 -= 5;
    }
    Total_Energy2 = Total_Energy2/10;
    
    pStore[0] = 4;  // Data type: Power and energy of the phase 2
    pStore[1] = 0;
    pStore[2] = 0;
    pStore[3] = 0;
    
    if(++energy2_timer == 2)
    {
        energy2_timer = 0;
        // Calculate energy increment in 5 seconds
        if(firstE2_read != 0)
        {
            firstE2_read = 0;
            lastE2_read = crntE2_read;
        }
        else
        {
            diffE2_read = ((int32)crntE2_read - (int32)lastE2_read);
            lastE2_read = crntE2_read;
            diffE2_read = ((int64)diffE2_read * ENERGY_LSB);
            diffE2_read = ((int64)diffE2_read/ENERGY_DIVIDER);
            if(diffE2_read < 0)
            {
                diffE2_read -= 5;
            }
            else if(diffE2_read > 0)
            {
                diffE2_read += 5;
            }
            diffE2_calc = (int8)(diffE2_read/10);
            pStore[3] = (uint8)diffE2_calc;
        }
    }
}

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
void STPM3x_ReadActivePwrPha2(uint8 *pStore)
{
    uint8 data_activP2[STPM3x_FRAME_LEN];
    int32 pwr2_read;
    int64 pwr2_calc;
    uint8 sign2;
    
    data_activP2[0] = (REG_PH_PERIOD | REGISTER_READ);
    data_activP2[1] = REG_WRITE_DUMMY;
    data_activP2[2] = REG_WRITE_DUMMY;
    data_activP2[3] = REG_WRITE_DUMMY;
    STPM3x_DataExchange(data_activP2);
    STPM3x_GetRegData(data_activP2);
    
    sign2 = 0;
    // Calculate power of the phase 1
    pwr2_read =data_activP2[3];
    pwr2_read = ((pwr2_read << 8) | data_activP2[2]);
    pwr2_read = ((pwr2_read << 8) | data_activP2[1]);
    pwr2_read = ((pwr2_read << 8) | data_activP2[0]);
    pwr2_calc = (((int64)pwr2_read * POWER_LSB)/POWER_DIVIDER);
    if(pwr2_calc < 0)
    {
        sign2 = 0x80;
        pwr2_calc = -pwr2_calc;
    }
    // Round up on 0.5
    pwr2_calc += 5;
    pwr2_calc = (pwr2_calc/10);
    pStore[0] = 4;  // Data type: Power and energy of the phase 2
    pStore[1] = ((uint8)(pwr2_calc & 0x00000000000000FF));
    pStore[2] = (((uint8)((pwr2_calc >> 8) & 0x00000000000000FF)) | sign2);
}

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
void STPM3x_HardDataReset(void)
{
}

void STPM3x_SoftDataReset(void)
{
}

void STPM3x_HardDspReser(void)
{
}

void STPM3x_SoftDspReset(void)
{
}

static uint8 STPM3x_ByteReverse(uint8 u8Data)
{
    // Swap the bit order to bit6:bit7:bit4:bit5:bit2:bit3:bit0:bit1
    u8Data = (((u8Data >> 1) & 0x55) | ((u8Data << 1) & 0xAA));
    // Swap the bit order to bit4:bit5:bit6:bit7:bit0:bit1:bit2:bit3
    u8Data = (((u8Data >> 2) & 0x33) | ((u8Data << 2) & 0xCC));
    // Swap the bit order to bit0:bit1:bit2:bit3:bit4:bit5:bit6:bit7
    u8Data = (((u8Data >> 4) & 0x0F) | ((u8Data << 4) & 0xF0));
    
    return u8Data;
}

static void STPM3x_InsideCRC8Calc(uint8 InData)
{
    uint8 loc_Index;
    uint8 loc_Temp;
    
    loc_Index = 0u;
    while(loc_Index < 8)
    {
        loc_Temp = InData^Stpm3x_CRC8Checksum;
        Stpm3x_CRC8Checksum <<= 1;
        if((loc_Temp & 0x80) != 0)
        {
           Stpm3x_CRC8Checksum ^= CRC8_POLY; 
        }
        InData <<= 1;
        loc_Index++;
    }
}

void STPM3x_CalcCRC8(uint8 *pBuffer)
{
    uint8 i;
    
    Stpm3x_CRC8Checksum = 0;
    #if defined (INTERFACE_UART)
    for(i = 0; i < STPM3x_FRAME_LEN; i++)
    {
        pBuffer[i] = STPM3x_ByteReverse(pBuffer[i]);
    }
    #endif
    for(i = 0; i < STPM3x_FRAME_LEN; i++)
    {
        STPM3x_InsideCRC8Calc(pBuffer[i]);
    }
    #if defined (INTERFACE_UART)
    Stpm3x_CRC8Checksum = STPM3x_ByteReverse(Stpm3x_CRC8Checksum);    
    #endif
}

int32 STPM3x_GetTotalEnergy1(void)
{
    return Total_Energy1;
}

int32 STPM3x_GetTotalEnergy2(void)
{
    return Total_Energy2;
}

/* [] END OF FILE */
