/*
 * video_impl.h
 *
 *  Created on: 26/12/2017
 */

#ifndef VIDEO_IMPL_H_
#define VIDEO_IMPL_H_

typedef enum {
	SS_UNKNOWN				= 0x00,
	SS_PRE_EQU_PULSE		= 0x01,
	SS_SYNC_PULSE			= 0x02,
	SS_POST_EQU_PULSE		= 0x03,
	SS_ACTIVE_VIDEO			= 0x04
} SYNCSTATE;

typedef enum {
	TVS_UNKNOWN				= 0,
	TVS_PAL					= 5,	// PAL has 5 of each type of blanking pulse
	TVS_NTSC				= 6		// NTSC has 6 of each type of blanking pulse
} TVSTANDARD;

typedef enum {
	FULL_LINE				= 0x00,
	SHORT_SYNC				= 0x00,
	LONG_SYNC				= 0x01,
	HALF_LINE				= 0x02,

	FULL_LINE_SHORT_SYNC	= FULL_LINE | SHORT_SYNC,
	FULL_LINE_LONG_SYNC		= FULL_LINE | LONG_SYNC,
	HALF_LINE_SHORT_SYNC	= HALF_LINE | SHORT_SYNC,
	HALF_LINE_LONG_SYNC		= HALF_LINE | LONG_SYNC
} PULSETYPE;

typedef enum {
	LS_LOSS_OF_SYNC			= 0x00,
	LS_SYNC_LOCK			= 0x01
} LOCKSTATE;

typedef struct {
	u16 pixelStart;
	u16 pixelPeriod;
	u16 shortSyncThreshold;
	u16 halfLineThreshold;
	u16 fullLineThreshold;
} TIMING;

typedef struct {
	u8				tvStandard;
	u8				syncState;
	u8				lockState;
	u8				field;
	u16				syncCount;
	u16				videoLine;
	u16          	activeLinesPerField;
	u16				blankingLinesPerField;

} VIDEOSTATE;

//#define IN_CCM  __attribute__((section(".ccm")))
#define IN_CCM

#endif /* VIDEO_IMPL_H_ */
