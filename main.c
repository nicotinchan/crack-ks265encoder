#include "hevc_encoder.h"
#include <stdlib.h>

int main(int argc, char * argv[]) 
{
    int opt;
	char *file_name = NULL;
	int width = 0, height = 0, fps = 0;

	while((opt = getopt( argc, argv, "i:w:h:f:" )) != -1) {
		switch(opt) {
			case 'i':
				printf("option i:'%s'\n",optarg);
				file_name = optarg;
				break;
			case 'w':
				printf("option w:'%s'\n",optarg);
				width = atoi(optarg);
				break;
			case 'h':
				printf("option h:'%s'\n",optarg);
				height = atoi(optarg);
				break;
			case 'f':
				printf("option f:'%s'\n",optarg);
				fps = atoi(optarg);
				break;
			default:
				exit(-1);
		}
	}
    if (width == 0 || height == 0 || fps == 0)
        return -1;

    HEVCConfig config;
    HevcEncoderConfigDefault(&config);
    config.width = width;
    config.height = height;
    config.frameRate = fps;
    config.preset = HEVCPRESET_VERYFAST;
	config.latency = HEVCLATENCY_LIVESTREMING;
    config.rcType = HEVCRC_ABR;
    config.bitRate = 2000;
    config.vbvMaxRate = 2000;
    config.vbvBufferSize = 1000;
    config.gopSize = fps * 3;
    config.threads = 2;
    config.bFrames = 3;
    config.annexb = 1;

    HevcEncoder *enc = HevcEncoderAlloc();
    if(!enc)
        return -1;
    if(HevcEncoderOpen(enc, &config) < 0){
        printf("HevcEncoderOpen\n");
        return -1;
    }

    int yuv_fd = open(file_name, O_RDONLY);
	if(yuv_fd <= 0) {
	    return 0;
	}

    int fd_265 = open("out.265", O_RDWR | O_CREAT, 0644);
	if(fd_265 <= 0) {
	    return 0;
	}

    int nSize = width * height * 3 / 2;
    char *buf  = calloc(1, nSize);
    HevcBitStream *bs = calloc(1, sizeof(HevcBitStream));
    bs->nalu = (uint8_t *)calloc(1, MAX_NALU_SIZE);
    int64_t pts = 0;
    int got_ps = 0;
    int ret;
    int intervalMs = 1000/fps;
    for(;;)
    {
        usleep(intervalMs * 1000); //input at native frame rate 
        if(read(yuv_fd, buf, nSize) <= 0)
            break;
        do{
            ret = HevcEncoderEncode(enc, (uint8_t *)buf, nSize, pts, bs);
            if(ret < 0)
                break;

            if(got_ps == 0) {
                printf("Got ps: size(%d)\n", enc->psSize);
                write(fd_265, enc->ps, (size_t)enc->psSize);
                got_ps = 1;
            }
            write(fd_265, bs->nalu, (size_t)bs->size);
            printf("Got frame 1: size(%d), type(%d), dts(%ld), pts(%ld)\n", \
            bs->size, bs->type, bs->dts, bs->pts);
        } while(ret > 0);
        pts += intervalMs;
    }

    while(HevcEncoderflushAndClose(enc, bs) == 0) {
        write(fd_265, bs->nalu, (size_t)bs->size);
        printf("Got frame 2: size(%d), type(%d), dts(%ld), pts(%ld)\n", \
        bs->size, bs->type, bs->dts, bs->pts);
    }

    free(buf);
    free(bs->nalu);
    free(bs);
    HevcEncoderFree(&enc);
    return 0;
}