#include "timing.h"

#include "91x_lib.h"
#include "custom_math.h"
#include "irq_priority.h"


// =============================================================================
// Private data:

#define F_TIM1 (200000)  // 200kHz

static volatile uint32_t ms_timestamp_ = 0;
static uint32_t micro_wait_multiplier = 1;


// =============================================================================
// Public functions:

void TimingInit(void)
{
  SCU_APBPeriphClockConfig(__TIM01, ENABLE);

  TIM_InitTypeDef tim_init;

  TIM_StructInit(&tim_init);
  tim_init.TIM_Mode = TIM_OCM_CHANNEL_1;
  tim_init.TIM_OC1_Modes = TIM_TIMING;
  tim_init.TIM_Clock_Source = TIM_CLK_APB;
  tim_init.TIM_Prescaler = (SCU_GetPCLKFreqValue() * 1000) / F_TIM1;
  TIM_DeInit(TIM1);
  TIM_Init(TIM1, &tim_init);

  TIM_CounterCmd(TIM1, TIM_START);

  TIM_ITConfig(TIM1, TIM_IT_OC1, ENABLE);
  VIC_Config(TIM1_ITLine, VIC_IRQ, IRQ_PRIORITY_TIMER1);
  VIC_ITCmd(TIM1_ITLine, ENABLE);

  micro_wait_multiplier = 3 * SCU_GetMCLKFreqValue() / (1000 * 24);
}

// -----------------------------------------------------------------------------
// This function returns the current timestamp.
uint32_t GetTimestamp(void)
{
  return ms_timestamp_;
}

// -----------------------------------------------------------------------------
// This function returns a timestamp corresponding to "t" ms in the future. This
// timestamp can be checked against the current timestamp to see if a certain
// amount of time has passed.
uint32_t GetTimestampMillisFromNow(uint32_t t)
{
  return ms_timestamp_ + t + 1;
}

// -----------------------------------------------------------------------------
// This function compares a timestamp to the current timestamp and returns TRUE
// if the timestamp is in the past.
int TimestampInPast(uint32_t t)
{
  return ((int32_t)t - (int32_t)ms_timestamp_) < 0;
}

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred.
uint32_t MillisSinceTimestamp(uint32_t t)
{
  return ms_timestamp_ - t;
}

// -----------------------------------------------------------------------------
// This function delays execution of the program for "t" ms. Functions triggered
// by interrupts will still execute during this period.
void Wait(uint32_t t)
{
  uint32_t timestamp = GetTimestampMillisFromNow(t);
  while (!TimestampInPast(timestamp)) continue;
}

// -----------------------------------------------------------------------------
// This function delays execution of the program for APPROXIMATELY
// "t_microsends" microseconds. Note that time spent in an interrupt routine
// will NOT count towards this delay.
void MicroWait(uint32_t t_microseconds)
{
  // The following loop seems to take 24 cycles... for some reason.
  uint32_t countdown = micro_wait_multiplier * t_microseconds;
  asm(
    "LOOP:\n\t"
    "subs %[countdown], %[countdown], #1\n\t"
    "subs %[countdown], %[countdown], #1\n\t"
    "subs %[countdown], %[countdown], #1\n\t"
    "bne LOOP\n\t"
    : : [countdown] "r" (countdown)
  );
}

// =============================================================================
// Private functions:

void TIM1_IRQHandler(void)
{
  IENABLE;

  if(TIM_GetFlagStatus(TIM1, TIM_FLAG_OC1) == SET)
  {
    TIM_ClearFlag(TIM1, TIM_FLAG_OC1); // clear IRQ pending bit
    TIM1->OC1R += 200;  // F_TIM1 is 200kHz, generate an interrupt every 1 ms
    ms_timestamp_++;
  }

  IDISABLE;
  VIC0->VAR = 0xFF;
}
