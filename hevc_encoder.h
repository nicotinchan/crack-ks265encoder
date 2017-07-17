#ifndef _HEVC_ENCODER_H_
#define _HEVC_ENCODER_H_
#define _GNU_SOURCE
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <pthread.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

/*
same define as QY265xxx in qy265enc.h
*/
typedef enum HEVCTune_tag{
    HEVCTUNE_DEFAULT = 0,
    HEVCTUNE_SELFSHOW = 1,
    HEVCTUNE_GAME = 2,
    HEVCTUNE_MOVIE = 3,
    HEVCTUNE_SCREEN = 4
}HEVCTune;

typedef enum HEVCPreset_tag{
    HEVCPRESET_SUPERFAST = 0,
    HEVCPRESET_VERYFAST = 1,
    HEVCPRESET_FAST = 2,
    HEVCPRESET_MEDIUM = 3,
    HEVCPRESET_SLOW = 4,
    HEVCPRESET_VERYSLOW = 5,
    HEVCPRESET_PLACEBO = 6,
}HEVCPreset;

typedef enum HEVCLatency_tag{
    HEVCLATENCY_ZERO = 0,
    HEVCLATENCY_LOWDELAY = 1,
    HEVCLATENCY_LIVESTREMING = 2,
    HEVCLATENCY_OFFLINE = 3,
}HEVCLatency;

typedef enum HEVCRC_tag {
    HEVCRC_NONE = 0,
    HEVCRC_CBR = 1,
    HEVCRC_ABR = 2,
    HEVCRC_CRF = 3,
} HEVCRC;

typedef struct HEVCConfig {
    int annexb;
    int width;
    int height;
    int frameRate;
    int bitRate; //In kbps
    int vbvBufferSize; 
    int vbvMaxRate;
    int bFrames;
    int qp;
    int crf; //crf value
    int gopSize;
    int qpMin;
    int qpMax;
    int enFrameSkip; //for rate control
    int threads; // number of threads used in encoding
    HEVCTune tune;
    HEVCPreset preset;
    HEVCLatency latency;
    HEVCRC rcType;
    
    //{ VUI
    //vui_parameters_present_flag = 1
    //video_signal_type_present_flag = 1
    int colorRange; //video_full_range_flag
    //colour_description_present_flag = 1
    int colorSpace; //matrix_coefficients
    int colorPrimaries; //colour_primaries
    int colorTrc; //transfer_characteristics
    //}

    uint32_t magicCode; //0x77042485
} HEVCConfig;

typedef enum HEVCFRAMETYPE {
	HEVC_IFRAME  = 0,
	HEVC_PFRAME  = 1,
	HEVC_BFRAME  = 2,
	HEVC_SPFRAME = 3,
	HEVC_UNKNOW  = 4,
}HEVCFRAMETYPE;

#define MAX_NALU_SIZE (1 << 21) // 2MB
typedef struct HevcBitStream 
{
    int type;
    uint8_t *nalu;
    int size;
    int64_t pts;
	int64_t dts;
} HevcBitStream;

#define BUF_SIZE (4096)
typedef struct HevcEncoder {
    uint64_t uuid;
    int64_t bufFrmCnt;
    char iFifoName[64];
    int iFifo;
    char oFifoName[64];
    int oFifo;
    uint8_t ps[BUF_SIZE];
    int psSize;
    int psValid;

    uint8_t getId;
    uint8_t putId;
    int64_t ptsMap[256];

    int writedByte;
    uint8_t buf[BUF_SIZE];
	const uint8_t *nal_start;
	const uint8_t *p;
	const uint8_t *end;
    HEVCConfig config;

    sem_t sem;
    pid_t pid;
    int valid;
} HevcEncoder;

void HevcEncoderConfigDefault(HEVCConfig *config);
HevcEncoder *HevcEncoderAlloc();
void HevcEncoderFree(HevcEncoder **enc);
int HevcEncoderOpen(HevcEncoder *enc, HEVCConfig *config);
/*
return:
-1: no output
0: 1 output
1: more output
*/
int HevcEncoderEncode(HevcEncoder *enc, uint8_t *yuvData, int yuvSize, int64_t pts, /*IN*/
HevcBitStream *bs/*OUT*/);
/*
0: get output
-1: no more output
*/
int HevcEncoderflushAndClose(HevcEncoder *enc, HevcBitStream *bs);
int HevcEncoderGetPs(HevcEncoder *enc, uint8_t *ps);

/**
 * Table 7-3: NAL unit type codes
 */
enum NALUnitType {
    NAL_TRAIL_N    = 0,
    NAL_TRAIL_R    = 1,
    NAL_TSA_N      = 2,
    NAL_TSA_R      = 3,
    NAL_STSA_N     = 4,
    NAL_STSA_R     = 5,
    NAL_RADL_N     = 6,
    NAL_RADL_R     = 7,
    NAL_RASL_N     = 8,
    NAL_RASL_R     = 9,
    NAL_BLA_W_LP   = 16,
    NAL_BLA_W_RADL = 17,
    NAL_BLA_N_LP   = 18,
    NAL_IDR_W_RADL = 19,
    NAL_IDR_N_LP   = 20,
    NAL_CRA_NUT    = 21,
    NAL_VPS        = 32,
    NAL_SPS        = 33,
    NAL_PPS        = 34,
    NAL_AUD        = 35,
    NAL_EOS_NUT    = 36,
    NAL_EOB_NUT    = 37,
    NAL_FD_NUT     = 38,
    NAL_SEI_PREFIX = 39,
    NAL_SEI_SUFFIX = 40,
};

enum SliceType {
    B_SLICE = 0,
    P_SLICE = 1,
    I_SLICE = 2,
};
#define IS_IRAP(t) (t >= 16 && t <= 23)
#define IS_IDR(t) (t == NAL_IDR_W_RADL || t == NAL_IDR_N_LP)
#define IS_PS(t) (t == NAL_SPS || t == NAL_PPS || t == NAL_VPS)

#endif