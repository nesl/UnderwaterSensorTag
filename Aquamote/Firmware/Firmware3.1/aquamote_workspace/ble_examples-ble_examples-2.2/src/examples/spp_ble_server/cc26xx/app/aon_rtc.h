
    #ifndef __AON_RTC_H__
    #define __AON_RTC_H__


    #ifdef __cplusplus
    extern "C"
    {
    #endif

    #include <stdbool.h>
    #include <stdint.h>
    #include <inc/hw_types.h>
    #include <inc/hw_memmap.h>
    #include <inc/hw_aon_rtc.h>
    #include <driverlib/debug.h>

    #if !defined(DOXYGEN)
        #define AONRTCCurrentCompareValueGet    NOROM_AONRTCCurrentCompareValueGet
        #define AONRTCCurrent64BitValueGet      NOROM_AONRTCCurrent64BitValueGet
    #endif


    #define AON_RTC_CH_NONE            0x0 // RTC No channel
    #define AON_RTC_CH0                0x1 // RTC Channel 0
    #define AON_RTC_CH1                0x2 // RTC Channel 1
    #define AON_RTC_CH2                0x4 // RTC Channel 2
    #define AON_RTC_ACTIVE             0x8 // RTC Active

   #define AON_RTC_CONFIG_DELAY_NODELAY 0 // NO DELAY
   #define AON_RTC_CONFIG_DELAY_1       1 // Delay of   1 clk cycle
   #define AON_RTC_CONFIG_DELAY_2       2 // Delay of   2 clk cycles
   #define AON_RTC_CONFIG_DELAY_4       3 // Delay of   4 clk cycles
   #define AON_RTC_CONFIG_DELAY_8       4 // Delay of   8 clk cycles
   #define AON_RTC_CONFIG_DELAY_16      5 // Delay of  16 clk cycles
   #define AON_RTC_CONFIG_DELAY_32      6 // Delay of  32 clk cycles
   #define AON_RTC_CONFIG_DELAY_48      7 // Delay of  48 clk cycles
   #define AON_RTC_CONFIG_DELAY_64      8 // Delay of  64 clk cycles
   #define AON_RTC_CONFIG_DELAY_80      9 // Delay of  80 clk cycles
   #define AON_RTC_CONFIG_DELAY_96     10 // Delay of  96 clk cycles
   #define AON_RTC_CONFIG_DELAY_112    11 // Delay of 112 clk cycles
   #define AON_RTC_CONFIG_DELAY_128    12 // Delay of 128 clk cycles
   #define AON_RTC_CONFIG_DELAY_144    13 // Delay of 144 clk cycles


   #define AON_RTC_MODE_CH1_CAPTURE     1 // Capture mode
   #define AON_RTC_MODE_CH1_COMPARE     0 // Compare Mode

   #define AON_RTC_MODE_CH2_CONTINUOUS    1 // Continuous mode
   #define AON_RTC_MODE_CH2_NORMALCOMPARE 0 // Normal compare mode
   __STATIC_INLINE void
   AONRTCEnable(void)
   {
       //
       // Enable RTC.
       //
       HWREGBITW(AON_RTC_BASE + AON_RTC_O_CTL, AON_RTC_CTL_EN_BITN) = 1;
   }
   __STATIC_INLINE void
   AONRTCDisable(void)
   {
       //
       // Disable RTC
       //
       HWREGBITW(AON_RTC_BASE + AON_RTC_O_CTL, AON_RTC_CTL_EN_BITN) = 0;
   }
   __STATIC_INLINE void
   AONRTCReset(void)
   {
       //
       // Reset RTC.
       //
       HWREGBITW(AON_RTC_BASE + AON_RTC_O_CTL, AON_RTC_CTL_RESET_BITN) = 1;
   }
   __STATIC_INLINE bool
   AONRTCActive(void)
   {
       // Read if RTC is enabled
       return(HWREGBITW(AON_RTC_BASE + AON_RTC_O_CTL, AON_RTC_CTL_EN_BITN));
   }
   __STATIC_INLINE bool
   AONRTCChannelActive(uint32_t ui32Channel)
   {
       uint32_t uint32Status = 0;

       if(ui32Channel & AON_RTC_CH0)
       {
           uint32Status = HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH0_EN_BITN);
       }

       if(ui32Channel & AON_RTC_CH1)
       {
           uint32Status = HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH1_EN_BITN);
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           uint32Status = HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH2_EN_BITN);
       }

       return(uint32Status);
   }
   __STATIC_INLINE void
   AONRTCDelayConfig(uint32_t ui32Delay)
   {
       uint32_t ui32Cfg;

       //
       // Check the arguments.
       //
       ASSERT(ui32Delay <= AON_RTC_CONFIG_DELAY_144);


       ui32Cfg =  HWREG(AON_RTC_BASE + AON_RTC_O_CTL);
       ui32Cfg &= ~(AON_RTC_CTL_EV_DELAY_M);
       ui32Cfg |= (ui32Delay << AON_RTC_CTL_EV_DELAY_S);

       HWREG(AON_RTC_BASE + AON_RTC_O_CTL) = ui32Cfg;
   }
   __STATIC_INLINE void
   AONRTCCombinedEventConfig(uint32_t ui32Channels)
   {
       uint32_t ui32Cfg;

       //
       // Check the arguments.
       //
       ASSERT( (ui32Channels & (AON_RTC_CH0 | AON_RTC_CH1 | AON_RTC_CH2)) ||
               (ui32Channels == AON_RTC_CH_NONE) );

       ui32Cfg =  HWREG(AON_RTC_BASE + AON_RTC_O_CTL);
       ui32Cfg &= ~(AON_RTC_CTL_COMB_EV_MASK_M);
       ui32Cfg |= (ui32Channels << AON_RTC_CTL_COMB_EV_MASK_S);

       HWREG(AON_RTC_BASE + AON_RTC_O_CTL) = ui32Cfg;
   }
   __STATIC_INLINE void
   AONRTCEventClear(uint32_t ui32Channel)
   {
       // Check the arguments.
       ASSERT((ui32Channel == AON_RTC_CH0) ||
              (ui32Channel == AON_RTC_CH1) ||
              (ui32Channel == AON_RTC_CH2));

       if(ui32Channel & AON_RTC_CH0)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH0_BITN) = 1;
       }
       if(ui32Channel & AON_RTC_CH1)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH1_BITN) = 1;
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH2_BITN) = 1;
       }
   }

   //*****************************************************************************
   //
   //
   //*****************************************************************************
   __STATIC_INLINE bool
   AONRTCEventGet(uint32_t ui32Channel)
   {
       uint32_t uint32Event = 0;

       // Check the arguments.
       ASSERT((ui32Channel == AON_RTC_CH0) ||
              (ui32Channel == AON_RTC_CH1) ||
              (ui32Channel == AON_RTC_CH2));

       if(ui32Channel & AON_RTC_CH0)
       {
           uint32Event = HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH0_BITN);
       }

       if(ui32Channel & AON_RTC_CH1)
       {
           uint32Event = HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH1_BITN);
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           uint32Event = HWREGBITW(AON_RTC_BASE + AON_RTC_O_EVFLAGS, AON_RTC_EVFLAGS_CH2_BITN);
       }

       return(uint32Event);
   }

  //*****************************************************************************
   //
   //
   //*****************************************************************************
   __STATIC_INLINE uint32_t
   AONRTCSecGet(void)
   {
       //
       // The following read gets the seconds, but also latches the fractional
       // part.
       //
       return(HWREG(AON_RTC_BASE + AON_RTC_O_SEC));
   }

   //*****************************************************************************
   //
   //
   //*****************************************************************************
   __STATIC_INLINE uint32_t
   AONRTCFractionGet(void)
   {
       //
       // Note1: It is recommended to use AON RTCCurrentCompareValueGet() instead
       //        of this function if the <16.16> format is sufficient.
       // Note2: AONRTCSecGet() must be called before this function to get a
       //        consistent reading.
       // Note3: Interrupts must be disabled between the call to AONRTCSecGet() and this
       //        call since there are interrupt functions that reads AON_RTC_O_SEC
       //
       return(HWREG(AON_RTC_BASE + AON_RTC_O_SUBSEC));
   }

   //*****************************************************************************
   //
   //
   //*****************************************************************************
   __STATIC_INLINE uint32_t
   AONRTCSubSecIncrGet(void)
   {
       return(HWREG(AON_RTC_BASE + AON_RTC_O_SUBSECINC));
   }

   //*****************************************************************************
   //
   //
   //*****************************************************************************
   __STATIC_INLINE void
  AONRTCModeCh1Set(uint32_t ui32Mode)
   {
       // Check the arguments.
       ASSERT((ui32Mode == AON_RTC_MODE_CH1_CAPTURE) ||
              (ui32Mode == AON_RTC_MODE_CH1_COMPARE));

     HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH1_CAPT_EN_BITN) = ui32Mode;
 }

   //*****************************************************************************
   //
   //
   //*****************************************************************************
   __STATIC_INLINE uint32_t
   AONRTCModeCh1Get(void)
   {
       return(HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH1_CAPT_EN_BITN));
   }
   __STATIC_INLINE void
   AONRTCModeCh2Set(uint32_t ui32Mode)
   {
       // Check the arguments.
       ASSERT((ui32Mode == AON_RTC_MODE_CH2_CONTINUOUS) ||
              (ui32Mode == AON_RTC_MODE_CH2_NORMALCOMPARE));

       HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH2_CONT_EN_BITN) = ui32Mode;
   }
   __STATIC_INLINE uint32_t
   AONRTCModeCh2Get(void)
   {
       return(HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH2_CONT_EN_BITN));
   }

   __STATIC_INLINE void
   AONRTCChannelEnable(uint32_t ui32Channel)
   {
       // Check the arguments.
       ASSERT((ui32Channel == AON_RTC_CH0) ||
              (ui32Channel == AON_RTC_CH1) ||
              (ui32Channel == AON_RTC_CH2));

       if(ui32Channel & AON_RTC_CH0)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH0_EN_BITN) = 1;
       }

       if(ui32Channel & AON_RTC_CH1)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH1_EN_BITN) = 1;
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH2_EN_BITN) = 1;
       }
   }

   __STATIC_INLINE void
   AONRTCChannelDisable(uint32_t ui32Channel)
   {
       // Check the arguments.
       ASSERT((ui32Channel == AON_RTC_CH0) ||
              (ui32Channel == AON_RTC_CH1) ||
              (ui32Channel == AON_RTC_CH2));

       if(ui32Channel & AON_RTC_CH0)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH0_EN_BITN) = 0;
       }

       if(ui32Channel & AON_RTC_CH1)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH1_EN_BITN) = 0;
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           HWREGBITW(AON_RTC_BASE + AON_RTC_O_CHCTL, AON_RTC_CHCTL_CH2_EN_BITN) = 0;
     }
   }

   __STATIC_INLINE void
   AONRTCCompareValueSet(uint32_t ui32Channel, uint32_t ui32CompValue)
   {
       // Check the arguments.
       ASSERT((ui32Channel == AON_RTC_CH0) ||
              (ui32Channel == AON_RTC_CH1) ||
              (ui32Channel == AON_RTC_CH2));

       if(ui32Channel & AON_RTC_CH0)
       {
           HWREG(AON_RTC_BASE + AON_RTC_O_CH0CMP) = ui32CompValue;
       }

       if(ui32Channel & AON_RTC_CH1)
       {
           HWREG(AON_RTC_BASE + AON_RTC_O_CH1CMP) = ui32CompValue;
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP) = ui32CompValue;
       }
   }

   __STATIC_INLINE uint32_t
   AONRTCCompareValueGet(uint32_t ui32Channel)
   {
       uint32_t ui32Value = 0;

       // Check the arguments
       ASSERT((ui32Channel == AON_RTC_CH0) ||
              (ui32Channel == AON_RTC_CH1) ||
              (ui32Channel == AON_RTC_CH2));

       if(ui32Channel & AON_RTC_CH0)
       {
           ui32Value = HWREG(AON_RTC_BASE + AON_RTC_O_CH0CMP);
       }

       if(ui32Channel & AON_RTC_CH1)
       {
           ui32Value = HWREG(AON_RTC_BASE + AON_RTC_O_CH1CMP);
       }

       if(ui32Channel & AON_RTC_CH2)
       {
           ui32Value = HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMP);
       }

       return(ui32Value);
   }

   extern uint32_t AONRTCCurrentCompareValueGet(void);

   extern uint64_t AONRTCCurrent64BitValueGet(void);

   __STATIC_INLINE void
   AONRTCIncValueCh2Set(uint32_t ui32IncValue)
   {
       HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC) = ui32IncValue;
   }

   __STATIC_INLINE uint32_t
   AONRTCIncValueCh2Get(void)
   {
       return(HWREG(AON_RTC_BASE + AON_RTC_O_CH2CMPINC));
  }

   __STATIC_INLINE uint32_t
   AONRTCCaptureValueCh1Get(void)
   {
       return(HWREG(AON_RTC_BASE + AON_RTC_O_CH1CAPT));
   }


   #if !defined(DRIVERLIB_NOROM) && !defined(DOXYGEN)
       #include <driverlib/rom.h>
       #ifdef ROM_AONRTCCurrentCompareValueGet
           #undef  AONRTCCurrentCompareValueGet
           #define AONRTCCurrentCompareValueGet    ROM_AONRTCCurrentCompareValueGet
       #endif
       #ifdef ROM_AONRTCCurrent64BitValueGet
           #undef  AONRTCCurrent64BitValueGet
           #define AONRTCCurrent64BitValueGet      ROM_AONRTCCurrent64BitValueGet
       #endif
   #endif

   #ifdef __cplusplus
   }
   #endif

   #endif //  __AON_RTC_H__
