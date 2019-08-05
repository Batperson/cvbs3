#include "stm32f4xx.h"
#include "string.h"
#include "macros.h"
#include "bitband.h"
#include "hardware.h"
#include "video_impl.h"
#include "pixel.h"
#include "video.h"

// This is working code for now, later will probably incorporate into PAL/NTSC-specific section.
// Using PAL timing of 4.7uS for sync and 5.8uS for back porch, and 1.5us for front porch.
static const uint16_t pixelStartNanoseconds				= 10500;
static const uint16_t pixelOutputNanoseconds			= 52000;
static const uint16_t shortSyncThresholdNanoseconds		= 10000;
static const uint32_t halfLineThresholdNanoseconds		= 34000;
static const uint32_t fullLineThresholdNanoseconds		= 66000;
static const uint16_t pixelsPerLine						= 300;			// PAL = 768*576, NTSC = 640*480


static VIDEOSTATE state IN_CCM;
static TIMING timing IN_CCM;

// Test code, for test pattern
static PIXEL lineBuf[312] ALIGNED(4);

void initTestPattern()
{
	const int barWidth = 20;
	PIXEL bars[] = { RED, GREEN, BLUE, RGB(3,3,3), RGB(2,2,2), RGB(1,1,1) };
	//PIXEL bars[] = { ARGB(0,3,0,0), ARGB(0,0,3,0), ARGB(0,0,0,3), ARGB(0,3,3,3), ARGB(0,2,2,2), ARGB(0,1,1,1) };

	memset(lineBuf, 0, sizeof(lineBuf));


	for(int i=0; i<sizeof(bars); i++)
		memset(lineBuf + (i * barWidth) + 40, bars[i], barWidth);

	/*
	for(int i=0; i<280; i += 2)
		lineBuf[i] = WHITE;
	*/
}


uint32_t calcNanosecsFromAPB1TimerTicks(uint16_t ticks)
{
	return ((ticks + 1) * (100000000000 /  (SystemCoreClock / 2))) / 100;
}

uint32_t calcNanosecsFromAPB2TimerTicks(uint16_t ticks)
{
	return ((ticks + 1) * (100000000000 /  SystemCoreClock)) / 100;
}

uint32_t calcAPB1TimerPeriod(uint32_t timeNS)
{
	return (((SystemCoreClock / 2) / (1000000000 / timeNS)) - 1);
}

uint32_t calcAPB2TimerPeriod(uint32_t timeNS)
{
	return ((SystemCoreClock / (1000000000 / timeNS)) - 1);
}

uint16_t calcAPB2TimerPeriodFromHz(uint32_t frequency)
{
	return (SystemCoreClock / frequency) - 1;
}

uint16_t calcAPB1TimerPeriodFromHz(uint32_t frequency)
{
	return (((SystemCoreClock / 2) / frequency) - 1);
}

void initTiming()
{
	timing.shortSyncThreshold	= calcAPB2TimerPeriod(shortSyncThresholdNanoseconds);
	timing.halfLineThreshold	= calcAPB2TimerPeriod(halfLineThresholdNanoseconds);
	timing.fullLineThreshold	= calcAPB2TimerPeriod(fullLineThresholdNanoseconds);
	timing.pixelStart			= calcAPB2TimerPeriod(pixelStartNanoseconds);
	timing.pixelPeriod			= calcAPB2TimerPeriod(pixelOutputNanoseconds / pixelsPerLine);
}

void initState()
{
	state.syncCount				= SS_UNKNOWN;
	state.tvStandard			= TVS_UNKNOWN;
	state.lockState				= LS_LOSS_OF_SYNC;
	state.activeLinesPerField	= 0;
	state.blankingLinesPerField = 0;
	state.syncCount				= 0;
	state.videoLine				= 0;
	state.field					= 0;
}

void initRCC()
{
	/*
	 * TIM1 	= sync clock
	 * TIM8 	= pixel clock
	 * GPIOA	= timer pins
	 * GPIOC	= pixel clock (for testing)
	 * GPIOF	= RGB video
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC |RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);
}

void initSyncPort()
{
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Pin 		= GPIO_Pin_9;
	gpio.GPIO_Mode 		= GPIO_Mode_AF;
	gpio.GPIO_Speed 	= GPIO_Speed_100MHz;
	gpio.GPIO_OType 	= GPIO_OType_PP;
	gpio.GPIO_PuPd 		= GPIO_PuPd_UP;

	GPIO_DeInit(GPIOA);
	GPIO_Init(GPIOA, &gpio);
	GPIO_ResetBits(GPIOA, GPIO_Pin_9);

	/* Configure PA9 for Alternate Function 1 (TIM1 CC2), datasheet page 67 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
}

void initPixelPort()
{
	GPIO_InitTypeDef		GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin 	=  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType 	= GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void initSync()
{
	NVIC_InitTypeDef			nvic;
	TIM_TimeBaseInitTypeDef		timb;
	TIM_ICInitTypeDef			icnt;
	TIM_OCInitTypeDef			ocnt;

	TIM_DeInit(TIM1);
	TIM_TimeBaseStructInit(&timb);

	timb.TIM_Prescaler 			= 0;
	timb.TIM_CounterMode 		= TIM_CounterMode_Up;
	timb.TIM_Period 			= 65535;
	timb.TIM_ClockDivision 		= TIM_CKD_DIV1;
	timb.TIM_RepetitionCounter 	= 0;
	TIM_TimeBaseInit(TIM1, &timb);

	// Channels 1 and 2 are input capture, to get the rising and falling edge of CSYNC
	TIM_ICStructInit(&icnt);
	icnt.TIM_Channel 			= TIM_Channel_2;
	icnt.TIM_ICPolarity			= TIM_ICPolarity_Falling;
	icnt.TIM_ICSelection		= TIM_ICSelection_DirectTI;
	icnt.TIM_ICPrescaler		= TIM_ICPSC_DIV1;
	icnt.TIM_ICFilter			= 0;
	TIM_PWMIConfig(TIM1, &icnt);

	TIM_SelectInputTrigger(TIM1, TIM_TS_TI2FP2);
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);

	// Interrupt on capture
	nvic.NVIC_IRQChannel 					= TIM1_CC_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority 	= 0;
	nvic.NVIC_IRQChannelSubPriority 		= 0;
	nvic.NVIC_IRQChannelCmd 				= ENABLE;

	NVIC_Init(&nvic);

	TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);

	// CC3 starts the pixel clock.
	// For CC3, we want: passive level on reload, active level after CC3-match.
	ocnt.TIM_OCMode 			= TIM_OCMode_PWM2;
	ocnt.TIM_Pulse 				= timing.pixelStart;
	ocnt.TIM_OutputState 		= TIM_OutputState_Enable;
	ocnt.TIM_OCPolarity 		= TIM_OCPolarity_High;
	ocnt.TIM_OCIdleState 		= TIM_OCIdleState_Reset;
	TIM_OC3Init(TIM1, &ocnt);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC3Ref);
}

void initPixelClock()
{
	TIM_TimeBaseInitTypeDef		timb;

	TIM_DeInit(TIM8);
	TIM_TimeBaseStructInit(&timb);

	timb.TIM_Prescaler 			= 0;
	timb.TIM_CounterMode 		= TIM_CounterMode_Up;
	timb.TIM_Period 			= timing.pixelPeriod;
	timb.TIM_ClockDivision 		= TIM_CKD_DIV1;
	timb.TIM_RepetitionCounter 	= 0;
	TIM_TimeBaseInit(TIM8, &timb);

	// TIM_SlaveMode_Trigger
	// TIM_SlaveMode_Gated
	TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Gated);
	TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);		// For TIM8, ITR0 = TIM1 TRGO, p565 reference manual
	TIM_DMACmd(TIM8, TIM_DMA_Update, ENABLE);

	// Need this now, since we're not using TIM_SlaveMode_Trigger
	TIM_Cmd(TIM8, ENABLE);

	TIM_OCInitTypeDef			ocnt;

	TIM_OCStructInit(&ocnt);

	ocnt.TIM_OCMode 			= TIM_OCMode_PWM1;
	ocnt.TIM_Pulse 				= timing.pixelPeriod / 2;
	ocnt.TIM_OutputState 		= TIM_OutputState_Enable;
	ocnt.TIM_OutputNState 		= TIM_OutputState_Disable;
	ocnt.TIM_OCPolarity 		= TIM_OCPolarity_Low;
	ocnt.TIM_OCIdleState 		= TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM8, &ocnt);

	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM8, ENABLE);

	// Test code: toggle a GPIO pin as well, so we can observe the pixel clock in the logic analyzer
	// PC6 is TIM8 CH1 alternate function (p69 in datasheet)
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);

	gpio.GPIO_Pin 	= GPIO_Pin_6;
	gpio.GPIO_Mode 	= GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOC, &gpio);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
}

void initPixelDma()
{
	NVIC_InitTypeDef nvic;
	DMA_InitTypeDef	dmai;

	DMA_Cmd(DMA2_Stream1, DISABLE);
	DMA_DeInit(DMA2_Stream1);

	// To be driven by TIM8 Update, we want DMA2 Stream1 Channel 7 (p308, reference manual)
	// Needs to be DMA2 because DMA1 does not have access to the peripherals (http://cliffle.com/article/2015/06/06/pushing-pixels/)
	DMA_StructInit(&dmai);

	// Test code, this will change once we have the frame buffer
	dmai.DMA_Memory0BaseAddr 		= (uint32_t)lineBuf;
	dmai.DMA_BufferSize 			= sizeof(lineBuf);

	dmai.DMA_PeripheralBaseAddr 	= (uint32_t)&GPIOF->ODR;
	dmai.DMA_DIR 					= DMA_DIR_MemoryToPeripheral;
	dmai.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
	dmai.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
	dmai.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	dmai.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Word;
	dmai.DMA_Mode 					= DMA_Mode_Normal;
	dmai.DMA_Priority 				= DMA_Priority_VeryHigh;
	dmai.DMA_Channel 				= DMA_Channel_7;
	dmai.DMA_FIFOMode 				= DMA_FIFOMode_Enable;
	dmai.DMA_FIFOThreshold 			= DMA_FIFOThreshold_HalfFull;
	dmai.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	dmai.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &dmai);
}


static void ALWAYS_INLINE fieldTransition()
{

}

static void ALWAYS_INLINE frameTransition()
{
	toggleLed1();
}

static void ALWAYS_INLINE syncLockChange(u8 lock)
{
	state.lockState				= lock;
}

static void ALWAYS_INLINE tvStandardChange(u8 tvs)
{
	state.tvStandard			= tvs;

	// TODO: Initialize timings, line count
	switch(tvs)
	{
	case TVS_PAL:
		state.activeLinesPerField	= 288;
		state.blankingLinesPerField	= 24;
		break;
	case TVS_NTSC:
		state.activeLinesPerField	= 252;
		state.blankingLinesPerField = 21;
		break;
	}
}

static void ALWAYS_INLINE prepareNextVBILine()
{
	// TODO: Video lines 1-21 or thereabouts occur during the vertical blanking interval and aren't visible
	// on any TV, but we can use them to send a digital data stream similar to Teletext.

	// Reset TIM8 just to ensure that the counter starts off consistently when it is gated on
	TIM8->EGR = TIM_EGR_UG;
	// Disable the DMA request line, then re-enable it to clear any pending request.
	TIM8->DIER &= ~ TIM_DIER_UDE;
	TIM8->DIER |= TIM_DIER_UDE;

	// Clear interrupt flags, DMA won't start if any of these are set.
	DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

	// (Re)-enable DMA for next scanline.
	//DMA2_Stream1->M0AR = (uint32_t)lineBuf;
	//DMA2_Stream1->NDTR = sizeof(lineBuf);
	//DMA2_Stream1->CR |= DMA_SxCR_EN;
}

static void ALWAYS_INLINE prepareNextVideoLine()
{
	// Reset TIM8 just to ensure that the counter starts off consistently when it is gated on
	TIM8->EGR = TIM_EGR_UG;
	// Disable the DMA request line, then re-enable it to clear any pending request.
	TIM8->DIER &= ~ TIM_DIER_UDE;
	TIM8->DIER |= TIM_DIER_UDE;

	// Clear interrupt flags, DMA won't start if any of these are set.
	DMA2->LIFCR = DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;

	ITM_Port32(1)	= state.videoLine;
	ITM_Port32(2) 	= DMA2_Stream1->NDTR;

	// (Re)-enable DMA for next scanline.
	DMA2_Stream1->M0AR = (uint32_t)lineBuf;
	DMA2_Stream1->NDTR = sizeof(lineBuf);
	DMA2_Stream1->CR |= DMA_SxCR_EN;
}

void INTERRUPT TIM1_CC_IRQHandler(void)
{
	// Clear pending interrupt(s), there should only ever be TIM_IT_CC2
	TIM1->SR = 0;

	PULSETYPE pulse			= ((TIM1->CCR2 <= timing.halfLineThreshold) ? HALF_LINE : FULL_LINE)
							+ ((TIM1->CCR1 <= timing.shortSyncThreshold) ? SHORT_SYNC : LONG_SYNC);

	// ITM_Port32(1)	= TIM1->CCR2;	// Line length
	// ITM_Port32(2)	= TIM1->CCR1;	// Sync pulse length

	switch(pulse)
	{
	case HALF_LINE_LONG_SYNC:
		switch(state.syncState)
		{
		case SS_SYNC_PULSE:
			state.syncCount += 1;
			break;

		case SS_PRE_EQU_PULSE:
			// Even field if the pre-equalizing serration begins in the middle of a normal scanline
			// (Which manifests as an extra half-line)
			state.field				= (state.syncCount > state.tvStandard) ? 0 : 1;
			state.syncState			= SS_SYNC_PULSE;
			state.syncCount			= 1;

			fieldTransition();

			if(state.field == 0)
				frameTransition();

			break;

		default:
			goto loss_of_sync;
		}
		break;

	case HALF_LINE_SHORT_SYNC:
		switch(state.syncState)
		{
		case SS_PRE_EQU_PULSE:
			state.syncCount++;
			break;

		case SS_POST_EQU_PULSE:
			if(++state.syncCount == state.tvStandard)
				prepareNextVBILine();

			break;

		case SS_SYNC_PULSE:
			if(state.lockState == LS_LOSS_OF_SYNC)
				syncLockChange(LS_SYNC_LOCK);
			if(state.tvStandard != state.syncCount)
				tvStandardChange(state.syncCount);

			state.syncState			= SS_POST_EQU_PULSE;
			state.syncCount			= 1;

			break;

		case SS_UNKNOWN:
		case SS_ACTIVE_VIDEO:
			state.syncState			= SS_PRE_EQU_PULSE;
			state.syncCount			= 1;
			state.videoLine			= 0;
			break;

		default:
			goto loss_of_sync;
		}
		break;

	case FULL_LINE_SHORT_SYNC:
		switch(state.syncState)
		{
		case SS_ACTIVE_VIDEO:
			state.videoLine++;
			state.syncCount++;

			if(state.videoLine < state.blankingLinesPerField)
				prepareNextVBILine();
			else if(state.videoLine < state.activeLinesPerField)
				prepareNextVideoLine();
			break;

		case SS_POST_EQU_PULSE:
			state.syncState			= SS_ACTIVE_VIDEO;
			state.syncCount			= 1;
			state.videoLine			= 1;
			break;

		default:
			goto loss_of_sync;
		}
		break;

loss_of_sync:
	default:
		state.lockState				= LS_LOSS_OF_SYNC;
		state.syncState				= SS_UNKNOWN;
		state.syncCount				= 0;

		if(state.lockState == LS_SYNC_LOCK)
			syncLockChange(LS_LOSS_OF_SYNC);

		break;
	}
}

void initVideo()
{
	initTiming();
	initState();
	initRCC();
	initSyncPort();
	initPixelPort();
	initSync();
	initPixelClock();
	initPixelDma();

	initTestPattern();

	TIM_Cmd(TIM1, ENABLE);
}
