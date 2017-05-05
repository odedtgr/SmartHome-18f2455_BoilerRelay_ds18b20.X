   /*** FILEHEADER ****************************************************************
 *
 *    FILENAME:    ds1820.h
 *    DATE:        25.02.2005
 *    AUTHOR:      Christian Stadler
 *
 *    DESCRIPTION: Driver for DS1820 1-Wire Temperature sensor (Dallas)
 *
 ******************************************************************************/

/*** HISTORY OF CHANGE *********************************************************
 *
 ******************************************************************************/
#include <delays.h>
#include "types.h"
#ifndef FALSE  
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef _DS1820_H
#define _DS1820_H

#define TEMP_RES              0x100 /* temperature resolution => 1/256�C = 0.0039�C */


/* -------------------------------------------------------------------------- */
/*                         DS1820 Timing Parameters                           */
/* -------------------------------------------------------------------------- */

void rst_pulse_delay(void){
    Delay10TCYx(96);
}

void mstr_bitstart_delay(void){
    Delay1TCY();
}

void presence_wait_delay(void){
    Delay10TCYx(12);
}

void presence_fin_delay(void){
    Delay10TCYx(96);
}

void bit_read_delay(void){
   Delay1TCY();
   Delay1TCY();
   Delay1TCY();
   Delay1TCY();
   Delay1TCY();
}

void bit_write_delay(void){
    Delay10TCYx(12);
}


/* -------------------------------------------------------------------------- */
/*                            DS1820 Registers                                */
/* -------------------------------------------------------------------------- */

#define DS1820_REG_TEMPLSB    0
#define DS1820_REG_TEMPMSB    1
#define DS1820_REG_CNTREMAIN  6
#define DS1820_REG_CNTPERSEC  7
#define DS1820_SCRPADMEM_LEN  9     /* length of scratchpad memory */

#define DS1820_ADDR_LEN       8


/* -------------------------------------------------------------------------- */
/*                            DS1820 Commands                                 */
/* -------------------------------------------------------------------------- */

#define DS1820_CMD_SEARCHROM     0xF0
#define DS1820_CMD_READROM       0x33
#define DS1820_CMD_MATCHROM      0x55
#define DS1820_CMD_SKIPROM       0xCC
#define DS1820_CMD_ALARMSEARCH   0xEC
#define DS1820_CMD_CONVERTTEMP   0x44
#define DS1820_CMD_WRITESCRPAD   0x4E
#define DS1820_CMD_READSCRPAD    0xBE
#define DS1820_CMD_COPYSCRPAD    0x48
#define DS1820_CMD_RECALLEE      0xB8


#define DS1820_FAMILY_CODE_DS18B20      0x28
#define DS1820_FAMILY_CODE_DS18S20      0x10


/* -------------------------------------------------------------------------- */
/*                            static variables                                */
/* -------------------------------------------------------------------------- */

static bool bDoneFlag;
static uint8 nLastDiscrepancy_u8;
static uint8 nRomAddr_au8[DS1820_ADDR_LEN];



/* -------------------------------------------------------------------------- */
/*                           Low-Level Functions                              */
/* -------------------------------------------------------------------------- */


#define DS1820_DelayMs(dly_ms)   delay_ms(dly_ms)

/*******************************************************************************
 * FUNCTION:   void delay_ms(uint16 msec)
 * PURPOSE:    delay for the given number of miliseconds
 ******************************************************************************/
void delay_ms(uint16 msec)
{
    while(msec!=0)
    {
        msec--;
        Delay100TCYx(20);
    }
}

/*******************************************************************************
 * FUNCTION:   DS1820_DisableInterrupts
 * PURPOSE:    Disable interrupts
 *
 * INPUT:      -
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
#ifdef DS1820_INTERRUPT_LOCK
#define DS1820_DisableInterrupts()  disable_interrupts(GLOBAL)
#else
#define DS1820_DisableInterrupts()
#endif



/*******************************************************************************
 * FUNCTION:   DS1820_EnableInterrupts
 * PURPOSE:    Enable interrupts
 *
 * INPUT:      -
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
#ifdef DS1820_INTERRUPT_LOCK
#define DS1820_EnableInterrupts()   enable_interrupts(GLOBAL)
#else
#define DS1820_EnableInterrupts()
#endif


/*******************************************************************************
 * FUNCTION:   DS1820_Reset
 * PURPOSE:    Initializes the DS1820 device.
 *
 * INPUT:      -
 * OUTPUT:     -
 * RETURN:     FALSE if at least one device is on the 1-wire bus, TRUE otherwise
 ******************************************************************************/
bool DS1820_Reset(void)
{
   bool bPresPulse;

   DS1820_DisableInterrupts();

   /* reset pulse */
   output_low(DS1820_DATAPIN);
   rst_pulse_delay();
   output_high(DS1820_DATAPIN);
     
   /* wait until pullup pull 1-wire bus to high */
   presence_wait_delay();

   /* get presence pulse */
   bPresPulse = input(DS1820_DATAPIN);

   //DS1820_DelayUs(424);
   Delay10TCYx(85);
   
   DS1820_EnableInterrupts();

   return bPresPulse;
   
}


/*******************************************************************************
 * FUNCTION:   DS1820_ReadBit
 * PURPOSE:    Reads a single bit from the DS1820 device.
 *
 * INPUT:      -
 * OUTPUT:     -
 * RETURN:     bool        value of the bit which as been read form the DS1820
 ******************************************************************************/
bool DS1820_ReadBit(void)
{
   bool bBit;

   DS1820_DisableInterrupts();
   
   output_low(DS1820_DATAPIN);
   mstr_bitstart_delay();
   input(DS1820_DATAPIN);
   bit_read_delay();

   bBit = input(DS1820_DATAPIN);
   DS1820_EnableInterrupts();

   return (bBit);
}


/*******************************************************************************
 * FUNCTION:   DS1820_WriteBit
 * PURPOSE:    Writes a single bit to the DS1820 device.
 *
 * INPUT:      bBit        value of bit to be written
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
void DS1820_WriteBit(bool bBit)
{
   DS1820_DisableInterrupts();
   
   output_low(DS1820_DATAPIN);
   mstr_bitstart_delay();

   if (bBit != FALSE)
   {
      output_high(DS1820_DATAPIN);
   }

   bit_write_delay();
   output_high(DS1820_DATAPIN);
   
   DS1820_EnableInterrupts();
}


/*******************************************************************************
 * FUNCTION:   DS1820_ReadByte
 * PURPOSE:    Reads a single byte from the DS1820 device.
 *
 * INPUT:      -
 * OUTPUT:     -
 * RETURN:     uint8          byte which has been read from the DS1820
 ******************************************************************************/
uint8 DS1820_ReadByte(void)
{
   uint8 i;
   uint8 value = 0;

   for (i=0 ; i < 8; i++)
   { 
       if ( DS1820_ReadBit() )
      {
         value |= (1 << i);
      }
      //DS1820_DelayUs(120);
      Delay10TCYx(12);
   }
   return(value);
}


/*******************************************************************************
 * FUNCTION:   DS1820_WriteByte
 * PURPOSE:    Writes a single byte to the DS1820 device.
 *
 * INPUT:      val_u8         byte to be written
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
void DS1820_WriteByte(uint8 val_u8)
{
   uint8 i;
   uint8 temp;

   for (i=0; i < 8; i++)      /* writes byte, one bit at a time */
   {
      temp = val_u8 >> i;     /* shifts val right 'i' spaces */
      temp &= 0x01;           /* copy that bit to temp */
      DS1820_WriteBit(temp);  /* write bit in temp into */
   }

   //DS1820_DelayUs(105);
   Delay10TCYx(21);
}



/* -------------------------------------------------------------------------- */
/*                             API Interface                                  */
/* -------------------------------------------------------------------------- */


/*******************************************************************************
 * FUNCTION:   DS1820_AddrDevice
 * PURPOSE:    Addresses a single or all devices on the 1-wire bus.
 *
 * INPUT:      nAddrMethod       use DS1820_CMD_MATCHROM to select a single
 *                               device or DS1820_CMD_SKIPROM to select all
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
void DS1820_AddrDevice(uint8 nAddrMethod)
{
   uint8 i;
   
   if (nAddrMethod == DS1820_CMD_MATCHROM)
   {
      DS1820_WriteByte(DS1820_CMD_MATCHROM);     /* address single devices on bus */
      for (i = 0; i < DS1820_ADDR_LEN; i ++)
      {
         DS1820_WriteByte(nRomAddr_au8[i]);
      }
   }
   else
   {
      DS1820_WriteByte(DS1820_CMD_SKIPROM);     /* address all devices on bus */
   }
}


/*******************************************************************************
 * FUNCTION:   DS1820_FindNextDevice
 * PURPOSE:    Finds next device connected to the 1-wire bus.
 *
 * INPUT:      -
 * OUTPUT:     nRomAddr_au8[]       ROM code of the next device
 * RETURN:     bool                 TRUE if there are more devices on the 1-wire
 *                                  bus, FALSE otherwise
 ******************************************************************************/
bool DS1820_FindNextDevice(void)
{
    uint8 state_u8;
    uint8 byteidx_u8;
    uint8 mask_u8 = 1;
    uint8 bitpos_u8 = 1;
    uint8 nDiscrepancyMarker_u8 = 0;
    bool bit_b;
    bool bStatus;
    bool next_b = FALSE;

    /* init ROM address */
    for (byteidx_u8=0; byteidx_u8 < 8; byteidx_u8 ++)
    {
        nRomAddr_au8[byteidx_u8] = 0x00;
    }

    bStatus = DS1820_Reset();        /* reset the 1-wire */

    if (bStatus || bDoneFlag)        /* no device found */
    {
        nLastDiscrepancy_u8 = 0;     /* reset the search */
        return FALSE;
    }
    
    /* send search rom command */
    DS1820_WriteByte(DS1820_CMD_SEARCHROM);

    byteidx_u8 = 0;
    do
    {
        state_u8 = 0;

        /* read bit */
        if ( DS1820_ReadBit() != 0 )
        {
            state_u8 = 2;
        }
        //DS1820_DelayUs(60);
        Delay10TCYx(12);

        /* read bit complement */
        if ( DS1820_ReadBit() != 0 )
        {
            state_u8 |= 1;
        }
        //DS1820_DelayUs(60)
        Delay10TCYx(12);

        /* description for values of state_u8: */
        /* 00    There are devices connected to the bus which have conflicting */
        /*       bits in the current ROM code bit position. */
        /* 01    All devices connected to the bus have a 0 in this bit position. */
        /* 10    All devices connected to the bus have a 1 in this bit position. */
        /* 11    There are no devices connected to the 1-wire bus. */

        /* if there are no devices on the bus */
        if (state_u8 == 3)
        {
            break;
        }
        else
        {
            /* devices have the same logical value at this position */
            if (state_u8 > 0)
            {
                /* get bit value */
                bit_b = (bool)(state_u8 >> 1);
            }
            /* devices have confilcting bits in the current ROM code */
            else
            {
                /* if there was a conflict on the last iteration */
                if (bitpos_u8 < nLastDiscrepancy_u8)
                {
                    /* take same bit as in last iteration */
                    bit_b = ( (nRomAddr_au8[byteidx_u8] & mask_u8) > 0 );
                }
                else
                {
                    bit_b = (bitpos_u8 == nLastDiscrepancy_u8);
                }

                if (bit_b == 0)
                {
                    nDiscrepancyMarker_u8 = bitpos_u8;
                }
            }

            /* store bit in ROM address */
           if (bit_b != 0)
           {
               nRomAddr_au8[byteidx_u8] |= mask_u8;
           }
           else
           {
               nRomAddr_au8[byteidx_u8] &= ~mask_u8;
           }

           DS1820_WriteBit(bit_b);

           /* increment bit position */
           bitpos_u8 ++;

           /* calculate next mask value */
           mask_u8 = mask_u8 << 1;

           /* check if this byte has finished */
           if (mask_u8 == 0)
           {
               byteidx_u8 ++;  /* advance to next byte of ROM mask */
               mask_u8 = 1;    /* update mask */
           }
        }
    } while (byteidx_u8 < DS1820_ADDR_LEN);


    /* if search was unsuccessful then */
    if (bitpos_u8 < 65)
    {
        /* reset the last discrepancy to 0 */
        nLastDiscrepancy_u8 = 0;
    }
    else
    {
        /* search was successful */
        nLastDiscrepancy_u8 = nDiscrepancyMarker_u8;
        bDoneFlag = (nLastDiscrepancy_u8 == 0);

        /* indicates search is not complete yet, more parts remain */
        next_b = TRUE;
    }

    return next_b;
}


/*******************************************************************************
 * FUNCTION:   DS1820_FindFirstDevice
 * PURPOSE:    Starts the device search on the 1-wire bus.
 *
 * INPUT:      -
 * OUTPUT:     nRomAddr_au8[]       ROM code of the first device
 * RETURN:     bool                 TRUE if there are more devices on the 1-wire
 *                                  bus, FALSE otherwise
 ******************************************************************************/
bool DS1820_FindFirstDevice(void)
{
    nLastDiscrepancy_u8 = 0;
    bDoneFlag = FALSE;

    return ( DS1820_FindNextDevice() );
}


/*******************************************************************************
 * FUNCTION:   DS1820_WriteEEPROM
 * PURPOSE:    Writes to the DS1820 EEPROM memory (2 bytes available).
 *
 * INPUT:      nTHigh         high byte of EEPROM
 *             nTLow          low byte of EEPROM
 * OUTPUT:     -
 * RETURN:     -
 ******************************************************************************/
void DS1820_WriteEEPROM(uint8 nTHigh, uint8 nTLow)
{
    /* --- write to scratchpad ----------------------------------------------- */
    DS1820_Reset();
    DS1820_AddrDevice(DS1820_CMD_MATCHROM);
    DS1820_WriteByte(DS1820_CMD_WRITESCRPAD); /* start conversion */
    DS1820_WriteByte(nTHigh);
    DS1820_WriteByte(nTLow);

    //DS1820_DelayUs(10);
    Delay10TCYx(2);
    DS1820_Reset();
    DS1820_AddrDevice(DS1820_CMD_MATCHROM);
    DS1820_WriteByte(DS1820_CMD_COPYSCRPAD); /* start conversion */

    delay_ms(10);
}


/*******************************************************************************
 * FUNCTION:   DS1820_GetTempRaw
 * PURPOSE:    Get temperature raw value from single DS1820 device.
 *
 *             Scratchpad Memory Layout
 *             Byte  Register
 *             0     Temperature_LSB
 *             1     Temperature_MSB
 *             2     Temp Alarm High / User Byte 1
 *             3     Temp Alarm Low / User Byte 2
 *             4     Reserved
 *             5     Reserved
 *             6     Count_Remain
 *             7     Count_per_C
 *             8     CRC
 *
 *             Temperature calculation for DS18S20 (Family Code 0x10):
 *             =======================================================
 *                                             (Count_per_C - Count_Remain)
 *             Temperature = temp_raw - 0.25 + ----------------------------
 *                                                     Count_per_C
 *
 *             Where temp_raw is the value from the temp_MSB and temp_LSB with
 *             the least significant bit removed (the 0.5C bit).
 *
 *
 *             Temperature calculation for DS18B20 (Family Code 0x28):
 *             =======================================================
 *                      bit7   bit6   bit5   bit4   bit3   bit2   bit1   bit0
 *             LSB      2^3    2^2    2^1    2^0    2^-1   2^-2   2^-3   2^-4
 *                      bit15  bit14  bit13  bit12  bit3   bit2   bit1   bit0
 *             MSB      S      S      S      S      S      2^6    2^5    2^4
 *
 *             The temperature data is stored as a 16-bit sign-extended two�s
 *             complement number in the temperature register. The sign bits (S)
 *             indicate if the temperature is positive or negative: for
 *             positive numbers S = 0 and for negative numbers S = 1.
 *
 * RETURN:     sint16         raw temperature value with a resolution
 *                            of 1/256�C
 ******************************************************************************/
sint16 DS1820_GetTempRaw(void)
{
    uint8 i;
    uint16 temp_u16;
    uint16 highres_u16;
    uint8 scrpad[DS1820_SCRPADMEM_LEN];

    /* --- start temperature conversion -------------------------------------- */
    DS1820_Reset();
    DS1820_AddrDevice(DS1820_CMD_MATCHROM);     /* address the device */
    output_high(DS1820_DATAPIN);
    DS1820_WriteByte(DS1820_CMD_CONVERTTEMP);   /* start conversion */
    //DS1820_DelayMs(DS1820_TEMPCONVERT_DLY);   /* wait for temperature conversion */
    DS1820_DelayMs(750);


    /* --- read sratchpad ---------------------------------------------------- */
    DS1820_Reset();
    DS1820_AddrDevice(DS1820_CMD_MATCHROM);   /* address the device */
    DS1820_WriteByte(DS1820_CMD_READSCRPAD);  /* read scratch pad */

    /* read scratch pad data */
    for (i=0; i < DS1820_SCRPADMEM_LEN; i++)
    {
        scrpad[i] = DS1820_ReadByte();
    }


    /* --- calculate temperature --------------------------------------------- */
    /* Formular for temperature calculation: */
    /* Temp = Temp_read - 0.25 + ((Count_per_C - Count_Remain)/Count_per_C) */

    /* get raw value of temperature (0.5�C resolution) */
    temp_u16 = 0;
    temp_u16 = (uint16)((uint16)scrpad[DS1820_REG_TEMPMSB] << 8);
    temp_u16 |= (uint16)(scrpad[DS1820_REG_TEMPLSB]);

    if (nRomAddr_au8[0] == DS1820_FAMILY_CODE_DS18S20)
    {
        /* get temperature value in 1�C resolution */
        temp_u16 >>= 1;
    
        /* temperature resolution is TEMP_RES (0x100), so 1�C equals 0x100 */
        /* => convert to temperature to 1/256�C resolution */
        temp_u16 = ((uint16)temp_u16 << 8);
    
        /* now substract 0.25�C */
        temp_u16 -= ((uint16)TEMP_RES >> 2);
    
        /* now calculate high resolution */
        highres_u16 = scrpad[DS1820_REG_CNTPERSEC] - scrpad[DS1820_REG_CNTREMAIN];
        highres_u16 = ((uint16)highres_u16 << 8);
        if (scrpad[DS1820_REG_CNTPERSEC])
        {
            highres_u16 = highres_u16 / (uint16)scrpad[DS1820_REG_CNTPERSEC];
        }
    
        /* now calculate result */
        highres_u16 = highres_u16 + temp_u16;
    }
    else
    {
        /* 12 bit temperature value has 0.0625�C resolution */
        /* shift left by 4 to get 1/256�C resolution */
        highres_u16 = temp_u16;
        highres_u16 <<= 4;
    }

    return (highres_u16);
}


/*******************************************************************************
 * FUNCTION:   DS1820_GetTempFloat
 * PURPOSE:    Converts internal temperature value to string (physical value).
 *
 * INPUT:      none
 * OUTPUT:     none
 * RETURN:     float          temperature value with as float value
 ******************************************************************************/
float DS1820_GetTempFloat(void)
{
    return ((float)DS1820_GetTempRaw() / (float)TEMP_RES);
}


/*******************************************************************************
 * FUNCTION:   DS1820_GetTempString
 * PURPOSE:    Converts internal temperature value to string (physical value).
 *
 * INPUT:      tRaw_s16       internal temperature value
 * OUTPUT:     strTemp_pc     user string buffer to write temperature value
 * RETURN:     sint16         temperature value with an internal resolution
 *                            of TEMP_RES
 ******************************************************************************/
void DS1820_GetTempString(sint16 tRaw_s16, char *strTemp_pc)
{
    sint16 tPhyLow_s16;
    sint8 tPhy_s8;

    /* convert from raw value (1/256�C resolution) to physical value */
    tPhy_s8 = (sint8)(tRaw_s16/TEMP_RES);

    /* convert digits from raw value (1/256�C resolution) to physical value */
    /*tPhyLow_u16 = tInt_s16 % TEMP_RES;*/
    tPhyLow_s16 = tRaw_s16 & 0xFF;      /* this operation is the same as */
                                        /* but saves flash memory tInt_s16 % TEMP_RES */
    tPhyLow_s16 = tPhyLow_s16 * 100;
    tPhyLow_s16 = (uint16)tPhyLow_s16 / TEMP_RES;

    /* write physical temperature value to string */
    sprintf(strTemp_pc, "%d.%02d", tPhy_s8, (sint8)tPhyLow_s16);
}

#endif /* _DS1820_H */

