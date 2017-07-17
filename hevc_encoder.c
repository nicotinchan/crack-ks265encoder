#include "hevc_encoder.h"
#include <sys/epoll.h>
#include <stdlib.h>
#include <string.h>
#include "golomb.h"
#include "qy265enc.h"

#define APPENCODER "./appencoder"
#define HOOK_SO "./hook.so"
#define MEMBER_OFFSET(type, member) ((unsigned long)(&((type *)0)->member))

static pid_t forkAppEncoder(uint64_t uuid, HEVCConfig *config) 
{
    char buf[128] = {0};
    FILE *fp;

    pid_t pid = fork();
    if (pid == 0) { //child
        sprintf(buf, "/tmp/%lu.gdb", uuid);
		fp = fopen(buf, "w");
		if(fp == NULL)
			exit(-1);

        //parse config
		char confs[4096] = {0};
		strcat(confs, "set environment LD_PRELOAD = "HOOK_SO"\n");
		strcat(confs, "b _ZN10h265_codec15fillDefaultCfgsEPNS_13TEncConfigExtEP14QY265EncConfig\n");
		sprintf(confs + strlen(confs), "r -i /tmp/%lu.in -wdt %d -hgt %d -fr %d -br %d -b /tmp/%lu.out", \
        uuid, config->width, config->height, config->frameRate, config->bitRate, uuid);
        if(config->vbvBufferSize > 0)
            sprintf(confs + strlen(confs), " -vbv-bufsize %d", config->vbvBufferSize);
        if(config->vbvMaxRate > 0)
            sprintf(confs + strlen(confs), " -vbv-maxrate %d", config->vbvMaxRate);
        if(config->bFrames > 0)
            sprintf(confs + strlen(confs), " -bframes %d", config->bFrames);
        if(config->qp > 0)
            sprintf(confs + strlen(confs), " -qp %d", config->qp);
        if(config->crf > 0)
            sprintf(confs + strlen(confs), " -crf %d", config->crf);
        if(config->gopSize > 0)
            sprintf(confs + strlen(confs), " -iper %d", config->gopSize);
        if(config->qpMin > 0)
            sprintf(confs + strlen(confs), " -qpmin %d", config->qpMin);
        if(config->qpMax > 0)
            sprintf(confs + strlen(confs), " -qpmax %d", config->qpMax);
        if(config->enFrameSkip > 0)
            sprintf(confs + strlen(confs), " -frameskip %d", config->enFrameSkip);
        if(config->threads > 0)
            sprintf(confs + strlen(confs), " -threads %d", config->threads);
        if(config->tune > 0)
            sprintf(confs + strlen(confs), " -tune %s", qy265_tunes_names[config->tune]);
        if(config->preset > 0)
            sprintf(confs + strlen(confs), " -preset %s", qy265_preset_names[config->preset]);
        if(config->latency > 0)
            sprintf(confs + strlen(confs), " -latency %s", qy265_latency_names[config->latency]);
        if(config->rcType > 0)
            sprintf(confs + strlen(confs), " -rc %d", config->rcType);
        strcat(confs, "\nset $addr = $rdi\n");
        if(config->colorRange != -1){
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = 1\n", MEMBER_OFFSET(QY265EncConfig, vui_parameters_present_flag));
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = 1\n", MEMBER_OFFSET(QY265EncConfig, vui.video_signal_type_present_flag));
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = %d\n", \
                MEMBER_OFFSET(QY265EncConfig, vui.video_full_range_flag), config->colorRange);
        }
        if(config->colorSpace != -1 \
         && config->colorPrimaries != -1 \
         && config->colorTrc != -1) {
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = 1\n", MEMBER_OFFSET(QY265EncConfig, vui.colour_description_present_flag));
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = %d\n", MEMBER_OFFSET(QY265EncConfig, vui.matrix_coeffs), config->colorSpace);
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = %d\n", MEMBER_OFFSET(QY265EncConfig, vui.colour_primaries), config->colorPrimaries);
            sprintf(confs + strlen(confs), "set *(int*)($addr+%ld) = %d\n", MEMBER_OFFSET(QY265EncConfig, vui.transfer_characteristics), config->colorTrc);
        }
        sprintf(confs + strlen(confs), "dump value /tmp/%lu.pid getpid()\n", uuid);
		strcat(confs, "c\n");
        strcat(confs, "quit\n");
		
        if (fputs(confs, fp) <= 0)
            exit(-1);
        fclose(fp);

        //printf("confs = %s\n", confs);

        char tmp[128] = {0};
        sprintf(tmp, "--command=%s", buf);
		char *argv[] = { "gdb", tmp, APPENCODER, 0 };
		execv("/usr/bin/gdb", &argv[0]);
	}else if (pid < 0) { //error
		printf("fork error \n");
		exit(0);
	}

    sprintf(buf, "/tmp/%lu.pid", uuid);
    while(access(buf, 0)) {
         usleep(10000);
    }
    fp = fopen(buf, "r");
	if(!fp){
        printf("get pid error\n");
		exit(-1);
    }
    fread(&pid, sizeof(pid_t), 1, fp);
    fclose(fp);
	return pid;
}

static void killAppEncoder(uint64_t uuid, pid_t pid)
{
    kill(pid, SIGKILL);

    char buf[128] = {0};
    sprintf(buf, "/tmp/%lu.gdb", uuid);
    unlink(buf);
    sprintf(buf, "/tmp/%lu.pid", uuid);
    unlink(buf);
}

void EpollerInit();
int EpollerAdd(HevcEncoder *enc);
void EpollerDel(HevcEncoder *enc);
void HevcEncoderClose(HevcEncoder *enc)
{    
    EpollerDel(enc);

    killAppEncoder(enc->uuid, enc->pid);

    if(enc->iFifo > 0){
        close(enc->iFifo);
        unlink(enc->iFifoName);
    }
    if(enc->oFifo > 0){
        close(enc->oFifo);
        unlink(enc->oFifoName);
    }

    sem_destroy(&enc->sem);
    enc->valid = 0;
}

int HevcEncoderOpen(HevcEncoder *enc, HEVCConfig *config)
{
    int opts;
    enc->iFifo = -1;
    enc->oFifo = -1;

    EpollerInit();

    if(config->magicCode != 0x77042485)
        return -1;
    memcpy(&enc->config, config, sizeof(HEVCConfig));

    // Initialize semaphore
    sem_init(&enc->sem, 0, 0);

    //create fifo
    sprintf(enc->iFifoName, "/tmp/%lu.in", enc->uuid);
    mkfifo(enc->iFifoName, 0664);
    sprintf(enc->oFifoName, "/tmp/%lu.out", enc->uuid);
    mkfifo(enc->oFifoName, 0664);

    //fork encoder process
    enc->pid = forkAppEncoder(enc->uuid, config);

    //open it, must open oFifo first
    enc->oFifo = open(enc->oFifoName, O_RDONLY);
    if(enc->oFifo <= 0) {
        goto err;
    }
    //set nonblocking
	opts = (fcntl(enc->oFifo, F_GETFL) | O_NONBLOCK);
	fcntl(enc->oFifo, F_SETFL, opts);

    enc->iFifo = open(enc->iFifoName, O_WRONLY);
    if(enc->iFifo <= 0) {
      goto err;
    }
    //set nonblocking
	opts = (fcntl(enc->iFifo, F_GETFL) | O_NONBLOCK);
	fcntl(enc->iFifo, F_SETFL, opts);

    enc->nal_start = enc->p = enc->end = enc->buf;

    if(EpollerAdd(enc) != 0)
        goto err;

    enc->valid = 1;
    return 0;
err:
    HevcEncoderClose(enc);
    return -1;
}

static int HevcEncoderGet(HevcEncoder *enc, HevcBitStream *bs);
int HevcEncoderEncode(HevcEncoder *enc, uint8_t *yuvData, int yuvSize, int64_t pts, /*IN*/
HevcBitStream *bs/*OUT*/)
{
    int ret, flag = 0;

redo:
    ret = HevcEncoderGet(enc, bs);
    if(ret > 0)
        return ret;
    else if(ret == 0 && enc->writedByte > 0)
        return 1;

    if(enc->writedByte > 0 && flag == 0) {
        yuvSize -= enc->writedByte;
        yuvData += enc->writedByte;
    }
    
    while(yuvSize > 0){
        ssize_t wlen;
        wlen = write(enc->iFifo, yuvData, yuvSize);
        if(wlen < 0 && errno != EINTR && errno != EAGAIN) {
            return -1;
        } else if (wlen <= 0) { 
            //waiting for
            sem_wait(&enc->sem);
            flag = 1;
            if(ret < 0)
                goto redo;
            else
                return 1;
        } 
        yuvSize -= wlen;
		yuvData += wlen;
        enc->writedByte += wlen;
    }
    enc->bufFrmCnt++;
    enc->ptsMap[enc->putId++] = pts;
    enc->writedByte = 0;

    return ret;
}

static int naluParse(const uint8_t *nalu, int len, /*IN*/int *type, int *poc/*OUT*/)
{
	const uint8_t *p = nalu + 4; //skip start code 
	int size = len - 4;

	int nal_unit_type = (*(p) >> 1) & 0x3f;
	if(IS_PS(nal_unit_type)){
		return 1;
    }
	
	//int temporal_id = (*(p + 5) & 0x07) - 1;
	//int key_frame;

	//printf("nal_unit_type = %d\n", nal_unit_type);
	
	GetBitContext gb;
	init_get_bits8(&gb, (const uint8_t *)(p + 2), size - 2);
	uint8_t first_slice_in_pic_flag = get_bits1(&gb);

	//uint8_t no_output_of_prior_pics_flag;
	if (IS_IRAP(nal_unit_type)) {
		//key_frame = 1;
		/*no_output_of_prior_pics_flag = */get_bits1(&gb);
	}

	//uint8_t dependent_slice_segment_flag = 0;
	/*uint32_t pps_id = */get_ue_golomb(&gb);
	//printf("pps_id = %u\n", pps_id);
	
	if (!first_slice_in_pic_flag) {
		return -1;
	}

	//num_extra_slice_header_bits = 0;

	int slice_type = get_ue_golomb(&gb);
    if (slice_type == I_SLICE) 
        *type = HEVC_IFRAME;
    else if (slice_type == P_SLICE) 
        *type = HEVC_PFRAME;
    else if (slice_type == B_SLICE) 
        *type = HEVC_BFRAME;
    else
        *type = HEVC_UNKNOW;
	//printf("slice_type = %d\n", slice_type);

	//output_flag_present_flag = 0;
	//separate_colour_plane_flag = 0;
	
	int log2_max_poc_lsb = 8;
	int pic_order_cnt_lsb = 0;
	if (!IS_IDR(nal_unit_type)) {
		pic_order_cnt_lsb = get_bits(&gb, log2_max_poc_lsb);
	}
    *poc = pic_order_cnt_lsb;
	//printf("pic_order_cnt_lsb = %d\n", pic_order_cnt_lsb);

	return 0;
}

static const uint8_t *ff_avc_find_startcode_internal(const uint8_t *p, const uint8_t *end)
{
    const uint8_t *a = p + 4 - ((intptr_t)p & 3);

    for (end -= 3; p < a && p < end; p++) {
        if (p[0] == 0 && p[1] == 0 && p[2] == 1)
            return p;
    }

    for (end -= 3; p < end; p += 4) {
        uint32_t x = *(const uint32_t*)p;
        if ((x - 0x01010101) & (~x) & 0x80808080) { // generic
            if (p[1] == 0) {
                if (p[0] == 0 && p[2] == 1)
                    return p;
                if (p[2] == 0 && p[3] == 1)
                    return p+1;
            }
            if (p[3] == 0) {
                if (p[2] == 0 && p[4] == 1)
                    return p+2;
                if (p[4] == 0 && p[5] == 1)
                    return p+3;
            }
        }
    }

    for (end += 3; p < end; p++) {
        if (p[0] == 0 && p[1] == 0 && p[2] == 1)
            return p;
    }

    return end + 3;
}

static const uint8_t *ff_avc_find_startcode(const uint8_t *p, const uint8_t *end){
    const uint8_t *out= ff_avc_find_startcode_internal(p, end);
    if(p<out && out<end && !out[-1]) out--;
    return out;
}

static int HevcEncoderGet(HevcEncoder *enc, HevcBitStream *bs) 
{
    ssize_t rlen = 0;

    bs->size = 0;
    for (;;) {
        if (enc->end - enc->buf < BUF_SIZE) {
            rlen = read(enc->oFifo, (void *)enc->end, BUF_SIZE - (enc->end - enc->buf));
			if(rlen < 0 && errno != EINTR && errno != EAGAIN) {
                enc->nal_start = enc->p = enc->end = enc->buf;
                return -1;
            } else if (rlen <= 0) {
                if (bs->size > 0) {
                    //printf("%ld, %ld, %ld, %ld\n", (long)enc->buf, (long)enc->nal_start, (long)enc->p, (long)enc->end);
                    //usleep(10000);
                    //continue;
                    //printf("get frame rlen = %d\n", *size);
                    int poc, ret;
                    ret = naluParse(bs->nalu, bs->size, &bs->type, &poc);
                    if (ret < 0)
                        return -1;
                    else if(ret == 1) { //sps/pps/vps
                        if(!enc->psValid){
                            memcpy(enc->ps + enc->psSize, bs->nalu, bs->size);
                            enc->psSize += bs->size;
                        }
                        bs->size = 0;
                        continue;
                    }else {
                        enc->psValid = 1;
                        enc->bufFrmCnt--;
                        //get dts/pts
                        bs->dts = enc->ptsMap[enc->getId++];
                        int nPoc = poc + enc->config.bFrames - 1;
                        bs->pts = enc->ptsMap[(nPoc > 255)?(nPoc-256):nPoc];
                        if(!enc->config.annexb) {
                            uint32_t uSize = bs->size - 4;
                            bs->nalu[0] = (uint8_t)((uSize >> 24) & 0xff);
                            bs->nalu[1] = (uint8_t)((uSize >> 16) & 0xff);
                            bs->nalu[2] = (uint8_t)((uSize >> 8) & 0xff);
                            bs->nalu[3] = (uint8_t)((uSize) & 0xff);
                        }
                        return 0;
                    }
                } else {
                    return -1;
                }
            }
        }

        enc->end += rlen;
        enc->nal_start = enc->p;
        enc->p = ff_avc_find_startcode(enc->p, enc->end);
        while (enc->p < enc->end) {
            size_t frame_len = (size_t)(enc->p - enc->nal_start) + bs->size;
            if (frame_len > 0) {
                memcpy((void *)(bs->nalu + bs->size), enc->nal_start, (size_t)(enc->p - enc->nal_start));
                bs->size += enc->p - enc->nal_start;
                //printf("get frame len = %ld\n", frame_len);

                int poc, ret;
                ret = naluParse(bs->nalu, bs->size, &bs->type, &poc);
                if (ret < 0)
                    return -1;
                else if(ret == 1) { //sps/pps/vps
                    if(!enc->psValid){
                        memcpy(enc->ps + enc->psSize, bs->nalu, bs->size);
                        enc->psSize += bs->size;
                    }
                    bs->size = 0;
                    enc->nal_start = enc->p;
                }else{
                    enc->psValid = 1;
                    enc->bufFrmCnt--;
                    //get dts/pts
                    bs->dts = enc->ptsMap[enc->getId++];
                    int nPoc = poc + enc->config.bFrames - 1;
                    bs->pts = enc->ptsMap[(nPoc > 255)?(nPoc-256):nPoc];
                    if(!enc->config.annexb) {
                        uint32_t uSize = bs->size - 4;
                        bs->nalu[0] = (uint8_t)((uSize >> 24) & 0xff);
                        bs->nalu[1] = (uint8_t)((uSize >> 16) & 0xff);
                        bs->nalu[2] = (uint8_t)((uSize >> 8) & 0xff);
                        bs->nalu[3] = (uint8_t)((uSize) & 0xff);
                    }
                    return 1;
                }
            }
            enc->p = ff_avc_find_startcode(enc->p + 4, enc->end);
        }
        memcpy((void *)(bs->nalu + bs->size), enc->nal_start, (size_t)(enc->end - enc->nal_start));
        bs->size += enc->end - enc->nal_start;
        enc->nal_start = enc->p = enc->end = enc->buf;
    } //for (;;)
}

int HevcEncoderflushAndClose(HevcEncoder *enc, HevcBitStream *bs)
{
    if(enc->valid == 1) {
        EpollerDel(enc);
        sem_destroy(&enc->sem);

        close(enc->iFifo);
        unlink(enc->iFifoName);
        
        //set blocking
        int opts = (fcntl(enc->oFifo, F_GETFL) & (~O_NONBLOCK));
        fcntl(enc->oFifo, F_SETFL, opts);
        //
        enc->valid = 0;
    }

    if(HevcEncoderGet(enc, bs) >= 0 && enc->bufFrmCnt >= 0)
        return 0;
    else {
        killAppEncoder(enc->uuid, enc->pid);

        close(enc->oFifo);
        unlink(enc->oFifoName);

        return -1;
    }
}

int HevcEncoderGetPs(HevcEncoder *enc, uint8_t *ps)
{
    memcpy(ps, enc->ps, (size_t)enc->psSize);
    return enc->psSize;
}

HevcEncoder *HevcEncoderAlloc()
{
    HevcEncoder *enc = calloc(1, sizeof(HevcEncoder));
    if(enc) {
        FILE *fp = fopen("/dev/urandom", "r");
        if(fp) {
            char path[512];
            do {
                fread(&enc->uuid, sizeof(uint64_t), 1, fp);
                sprintf(path, "/tmp/%lu.gdb", enc->uuid);
            }while(!access(path, 0));
            fclose(fp);
        }
    }
    return enc;
}

void HevcEncoderFree(HevcEncoder **enc)
{
    HevcEncoder *pEnc = *enc;
    if(pEnc) {
        if(pEnc->valid == 1)
            HevcEncoderClose(pEnc);
        free(pEnc);
        pEnc = NULL;
    }
        
}

void HevcEncoderConfigDefault(HEVCConfig *config)
{
    if(!config) {
        return;
    }
    memset(config, 0, sizeof(HEVCConfig));
    config->annexb = 1;
    config->colorRange = -1;
    config->colorSpace = -1;
    config->colorPrimaries = -1;
    config->colorTrc = -1;
    config->magicCode = 0x77042485;
}

#define MAX_EVENTS (32)
typedef struct {
   int efd;

   //pthread_mutex_t lock;
   pthread_t tid;
} Epoller;

static Epoller *gEpoller = NULL;

static void *EpollThread(void *arg) 
{
    Epoller *epoller = (Epoller*) arg;
    struct epoll_event events[MAX_EVENTS]; 
    int n, i; 

    for(;;) {
    //pthread_mutex_lock(&epoller->lock);
        n = epoll_wait(epoller->efd, events, MAX_EVENTS, 10); 
        if(n < 0) { 
            perror("epoll_wait");  
            exit(-1);
        }

        for(i = 0; i < n; i++){
            if(events[i].events & EPOLLERR || events[i].events & EPOLLHUP){
                 exit(-1);
            }
            if(events[i].events & (EPOLLIN | EPOLLOUT)) {
                HevcEncoder *enc = (HevcEncoder *)events[i].data.ptr;
                // Post semaphore
                sem_post(&enc->sem);
            }
        }
    //pthread_mutex_unlock(&epoller->lock);
    }

    return NULL;
}

void EpollerInit()
{
    if(gEpoller != NULL)
        return;
    
    /*clean up all of tmp file*/
    system("rm /tmp/*.in;rm /tmp/*.out;rm /tmp/*.gdb;rm /tmp/*.pid");

    gEpoller = (Epoller*)calloc(1, sizeof(Epoller));
    if (!gEpoller)
        exit(-1);

    //pthread_mutex_init(&gEpoller->lock, NULL);
    
    gEpoller->efd = epoll_create(MAX_EVENTS);
    if (gEpoller->efd < 0){
        perror("epoll_create");  
        exit(-1);
    }

    if(pthread_create(&gEpoller->tid, NULL, EpollThread, (void*)gEpoller) != 0){
        perror("pthread_create");  
		exit(-1);
	}
}

void EpollerDel(HevcEncoder *enc) 
{
    epoll_ctl(gEpoller->efd, EPOLL_CTL_DEL, enc->iFifo, NULL);
    epoll_ctl(gEpoller->efd, EPOLL_CTL_DEL, enc->oFifo, NULL);
}

int EpollerAdd(HevcEncoder *enc) 
{
    struct epoll_event ev1, ev2;
    ev1.events = EPOLLIN |EPOLLET;
    ev1.data.ptr = (void*)enc;
    ev2.events = EPOLLOUT|EPOLLET;
    ev2.data.ptr = (void*)enc;

    if(epoll_ctl(gEpoller->efd, EPOLL_CTL_ADD, enc->oFifo, &ev1) == 0 \
      && epoll_ctl(gEpoller->efd, EPOLL_CTL_ADD, enc->iFifo, &ev2) == 0)
        return 0;
    else {
        EpollerDel(enc);
        return -1;
    }
}


