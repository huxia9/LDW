
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "math.h"
#include "hdal.h"
#include "hd_type.h"
#include "vendor_ive.h"
#include "vendor_ai.h"
#include "vendor_ai_cpu/vendor_ai_cpu.h"
#include "vendor_ai_cpu_postproc.h"

#include "vendor_md.h"
#include "libmd.h"
// #include "iplimage.h"

#include "ADAS/AdasDetector.h"
#include "vf_gfx.h"

#include "Drivers/gps.h"//addxjw

///////////////////////////////////////////////////////////////////////////////
#define __MODULE__          adas_lanedetector
#define __DBGLVL__          2//0=OFF, 1=ERROR, 2=TRACE
#define __DBGFLT__          "*" //*=All, [mark]=CustomClass
#include "kwrap/debug.h"

typedef struct _ADAS_NET_PROC {
    ADAS_NET_PROC_CONFIG net_cfg;
    ADAS_MEM_RANGE proc_mem;
    UINT32 proc_id;

    CHAR out_class_labels[MAX_CLASS_NUM * VENDOR_AIS_LBL_LEN];
    ADAS_MEM_RANGE rslt_mem;
    ADAS_MEM_RANGE io_mem;
} ADAS_NET_PROC;

int ai_async = 0;  //0: sync mode will call proc(), 2: sync mode will call proc_buf(), 3: async mode will call push_in_buf() and pull_out_buf()

static ADAS_NET_PROC g_Adas_net[16] = {0};
ADAS_MEM_RANGE MD_share_mem[MD_MEM_BLOCK_NUM] = {0};
ADAS_MEM_RANGE ADAS_share_mem[ADAS_MEM_BLOCK_NUM] = {0};
ADAS_MEM_RANGE NET_BinSize_share_mem[ADAS_NN_RUN_NET_NUM] = {0};
// static BOOL adas_jingzhi = FALSE;
#if LINE_DETECT
static LaneDetector lanedetector = {0};
static Lane left_lane = {0}, right_lane = {0};
static Lane left_line = {0}, right_line = {0};
static Lane left_lane_log[20] = {0},right_lane_log[20] = {0};
static UINT32 left_index = 0, right_index = 0;
static UINT32 horizon_y = 212;
#endif  //LINE_DETECT

#if FCWS
static int fcw_alarm_dist = 250;
void set_fcw_sensitivity(FCW_SENSITIVITY status){
    switch(status){
        case FCW_HIGH:
            fcw_alarm_dist = (horizon_y >= 220) ? 245 : (horizon_y+27);
            break;
        case FCW_MIDDLE:
            fcw_alarm_dist = (horizon_y >= 220) ? 256 : (horizon_y+38);
            break;
        case FCW_LOW:
            fcw_alarm_dist = (horizon_y >= 220) ? 270 : (horizon_y+57);
            break;
        default:
            fcw_alarm_dist = (horizon_y >= 220) ? 245 : (horizon_y+27);
    }
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////
static HD_RESULT ADAS_NET_Binsize_mem_get(ADAS_MEM_RANGE *mem_parm, UINT32 size, UINT32 id)
{
    #if 0 //zmd
    HD_RESULT ret = HD_OK;
    UINT32 pa   = 0;
    void  *va   = NULL;
    HD_COMMON_MEM_VB_BLK blk;

    blk = hd_common_mem_get_block(HD_COMMON_MEM_USER_DEFINIED_POOL + id, size, DDR_ID0);
    if (HD_COMMON_MEM_VB_INVALID_BLK == blk) {
        printf("hd_common_mem_get_block fail\r\n");
        return HD_ERR_NG;
    }
    pa = hd_common_mem_blk2pa(blk);
    if (pa == 0) {
        printf("not get buffer, pa=%08x\r\n", (int)pa);
        return HD_ERR_NOMEM;
    }
    va = hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, pa, size);

    /* Release buffer */
    if (va == 0) {
        ret = hd_common_mem_munmap(va, size);
        if (ret != HD_OK) {
            printf("mem unmap fail\r\n");
            return ret;
        }
    }

    mem_parm->pa = pa;
    mem_parm->va = (UINT32)va;
    mem_parm->size = size;
    mem_parm->blk = blk;
    #endif

    mem_parm->pa = NET_BinSize_share_mem[id].pa;
    mem_parm->va = NET_BinSize_share_mem[id].va;
    mem_parm->size = NET_BinSize_share_mem[id].size;
    //mem_parm->blk = blk;

    return HD_OK;
}

static HD_RESULT ADAS_NET_Binsize_mem_rel(ADAS_MEM_RANGE *mem_parm)
{
    #if 0 //zmd
    HD_RESULT ret = HD_OK;
    /* Release in buffer */
    if (mem_parm->va) {
        ret = hd_common_mem_munmap((void *)mem_parm->va, mem_parm->size);
        if (ret != HD_OK) {
            printf("mem_uninit : (g_mem.va)hd_common_mem_munmap fail.\r\n");
            return ret;
        }
    }
    //ret = hd_common_mem_release_block((HD_COMMON_MEM_VB_BLK)g_mem.pa);
    ret = hd_common_mem_release_block(mem_parm->blk);
    if (ret != HD_OK) {
        printf("mem_uninit : (g_mem.pa)hd_common_mem_release_block fail.\r\n");
        return ret;
    }
    #endif

    mem_parm->pa = 0;
    mem_parm->va = 0;
    mem_parm->size = 0;
    //mem_parm->blk = (UINT32)-1;
    return HD_OK;
}

static HD_RESULT ADAS_mem_alloc(ADAS_MEM_RANGE *mem_parm, CHAR* name, UINT32 size)
{
    HD_RESULT ret = HD_OK;
    UINT32 pa   = 0;
    void  *va   = NULL;

    //alloc private pool
    ret = hd_common_mem_alloc(name, &pa, (void**)&va, size, DDR_ID0);
    if (ret!= HD_OK) {
        DBG_ERR("mem alloc fail(%d) \n", ret);
        return ret;
    }

    mem_parm->pa   = pa;
    mem_parm->va   = (UINT32)va;
    mem_parm->size = size;
    mem_parm->blk  = (UINT32)-1;

    return HD_OK;
}

static HD_RESULT ADAS_mem_free(ADAS_MEM_RANGE *mem_parm)
{
    HD_RESULT ret = HD_OK;

    //free private pool
    ret =  hd_common_mem_free(mem_parm->pa, (void *)mem_parm->va);
    if (ret!= HD_OK) {
        DBG_ERR("mem free fail(%d) \n", ret);
        return ret;
    }

    mem_parm->pa = 0;
    mem_parm->va = 0;
    mem_parm->size = 0;
    mem_parm->blk = (UINT32)-1;

    return HD_OK;
}

static INT32 ADAS_getsize_model(char* filename)
{
    FILE *bin_fd;
    UINT32 bin_size = 0;

    bin_fd = fopen(filename, "rb");
    if (!bin_fd) {
        DBG_ERR("get bin(%s) size fail\n", filename);
        return (-1);
    }

    fseek(bin_fd, 0, SEEK_END);
    bin_size = ftell(bin_fd);
    fseek(bin_fd, 0, SEEK_SET);
    fclose(bin_fd);

    return bin_size;
}

static UINT32 ADAS_load_model(CHAR *filename, UINT32 va)
{
    FILE  *fd;
    UINT32 file_size = 0, read_size = 0;
    const UINT32 model_addr = va;
    //DBG_DUMP("model addr = %08x\r\n", (int)model_addr);

    fd = fopen(filename, "rb");
    if (!fd) {
        DBG_ERR("load model(%s) fail\r\n", filename);
        return 0;
    }

    fseek ( fd, 0, SEEK_END );
    file_size = ALIGN_CEIL_4( ftell(fd) );
    fseek ( fd, 0, SEEK_SET );

    read_size = fread ((void *)model_addr, 1, file_size, fd);
    if (read_size != file_size) {
        DBG_ERR("size mismatch, real = %d, idea = %d\r\n", (int)read_size, (int)file_size);
    }
    fclose(fd);

    printf("load model(%s) ok\r\n", filename);
    return read_size;
}

static UINT32 ADAS_load_part_model(CHAR *filename, UINT32 part_size, UINT32 va)
{
    FILE  *fd;
    UINT32 read_size = 0;
    const UINT32 model_addr = va;

    fd = fopen(filename, "rb");
    if (!fd) {
        DBG_ERR("load model(%s) fail\r\n", filename);
        return 0;
    }

    read_size = fread ((void *)model_addr, 1, part_size, fd);
    if (read_size != part_size) {
        DBG_ERR("size mismatch, real = %d, idea = %d\r\n", (int)read_size, (int)part_size);
    }
    fclose(fd);

    printf("load part_model(%s) ok\r\n", filename);
    return read_size;
}

static HD_RESULT ADAS_load_label(UINT32 addr, UINT32 line_len, const CHAR *filename)
{
    FILE *fd;
    CHAR *p_line = (CHAR *)addr;

    fd = fopen(filename, "r");
    if (!fd) {
        DBG_ERR("load label(%s) fail\r\n", filename);
        return HD_ERR_NG;
    }

    while (fgets(p_line, line_len, fd) != NULL) {
        p_line[strlen(p_line) - 1] = '\0'; // remove newline character
        p_line += line_len;
    }

    if (fd) {
        fclose(fd);
    }

    printf("load label(%s) ok\r\n", filename);
    return HD_OK;
}

HD_RESULT ADAS_network_init(void)
{
    HD_RESULT ret = HD_OK;
    int i;

    // config extend engine plugin, process scheduler
    {
        UINT32 schd = VENDOR_AI_PROC_SCHD_FAIR;
        vendor_ai_cfg_set(VENDOR_AI_CFG_PLUGIN_ENGINE, vendor_ai_cpu1_get_engine());
        vendor_ai_cfg_set(VENDOR_AI_CFG_PROC_SCHD, &schd);
    }

    ret = vendor_ai_init();
    if (ret != HD_OK) {
        DBG_ERR("vendor_ai_init fail=%d\n", ret);
        return ret;
    }

    for (i = 0; i < 16; i++) {
        ADAS_NET_PROC* p_net = g_Adas_net + i;
        p_net->proc_id = i;
    }
    return ret;
}
HD_RESULT ADAS_network_uninit(void)
{
    HD_RESULT ret = HD_OK;
    ret = vendor_ai_uninit();
    if (ret != HD_OK) {
        DBG_ERR("vendor_ai_uninit fail=%d\n", ret);
    }
    return ret;
}

HD_RESULT ADAS_network_set_config(NET_PATH_ID net_path, ADAS_NET_PROC_CONFIG* p_proc_cfg)
{
    HD_RESULT ret = HD_OK;
    ADAS_NET_PROC* p_net = g_Adas_net + net_path;
    UINT32 binsize = p_net->net_cfg.binsize;
    UINT32 proc_id;
    p_net->proc_id = net_path;
    proc_id = p_net->proc_id;

    memcpy((void*)&p_net->net_cfg, (void*)p_proc_cfg, sizeof(ADAS_NET_PROC_CONFIG));
    p_net->net_cfg.binsize = binsize;
    printf("proc_id(%u) set net_cfg: job-opt=(%u,%d), buf-opt(%u)\r\n",
           proc_id,
           p_net->net_cfg.job_method,
           (int)p_net->net_cfg.job_wait_ms,
           p_net->net_cfg.buf_method);

    // set buf opt
    {
        VENDOR_AI_NET_CFG_BUF_OPT cfg_buf_opt = {0};
        cfg_buf_opt.method = p_net->net_cfg.buf_method;
        cfg_buf_opt.ddr_id = DDR_ID0;
        vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_BUF_OPT, &cfg_buf_opt);
    }

    // set job opt
    {
        VENDOR_AI_NET_CFG_JOB_OPT cfg_job_opt = {0};
        cfg_job_opt.method = p_net->net_cfg.job_method;
        cfg_job_opt.wait_ms = p_net->net_cfg.job_wait_ms;
        cfg_job_opt.schd_parm = VENDOR_AI_FAIR_CORE_ALL; //FAIR dispatch to ALL core
        vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_JOB_OPT, &cfg_job_opt);
    }

    return ret;
}
static HD_RESULT ADAS_network_alloc_io_buf(NET_PATH_ID net_path)
{
    HD_RESULT ret = HD_OK;
    ADAS_NET_PROC* p_net = g_Adas_net + net_path;
    UINT32 proc_id = p_net->proc_id;
    VENDOR_AI_NET_CFG_WORKBUF wbuf = {0};

    ret = vendor_ai_net_get(proc_id, VENDOR_AI_NET_PARAM_CFG_WORKBUF, &wbuf);
    if (ret != HD_OK) {
        DBG_ERR("proc_id(%lu) get VENDOR_AI_NET_PARAM_CFG_WORKBUF fail\r\n", proc_id);
        return HD_ERR_FAIL;
    }
    ret = ADAS_mem_alloc(&p_net->io_mem, "ai_io_buf", wbuf.size);
    if (ret != HD_OK) {
        DBG_ERR("proc_id(%lu) alloc ai_io_buf fail\r\n", proc_id);
        return HD_ERR_FAIL;
    }

    wbuf.pa = p_net->io_mem.pa;
    wbuf.va = p_net->io_mem.va;
    wbuf.size = p_net->io_mem.size;
    ret = vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_WORKBUF, &wbuf);
    if (ret != HD_OK) {
        DBG_ERR("proc_id(%lu) set VENDOR_AI_NET_PARAM_CFG_WORKBUF fail\r\n", proc_id);
        return HD_ERR_FAIL;
    }

    printf("alloc_io_buf: work buf, pa = %#lx, va = %#lx, size = %lu\r\n", wbuf.pa, wbuf.va, wbuf.size);

    return ret;
}

static HD_RESULT ADAS_network_free_io_buf(NET_PATH_ID net_path)
{
    HD_RESULT ret = HD_OK;
    ADAS_NET_PROC* p_net = g_Adas_net + net_path;

    if (p_net->io_mem.pa && p_net->io_mem.va) {
        ADAS_mem_free(&p_net->io_mem);
    }
    return ret;
}

HD_RESULT ADAS_network_open(NET_PATH_ID net_path)
{
    HD_RESULT ret = HD_OK;
    ADAS_NET_PROC* p_net = g_Adas_net + net_path;
    UINT32 proc_id;
    p_net->proc_id = net_path;
    proc_id = p_net->proc_id;

    UINT32 loadsize = 0;

    if (strlen(p_net->net_cfg.model_filename) == 0) {
        DBG_ERR("proc_id(%u) input model is null\r\n", proc_id);
        return 0;
    }

    if (p_net->net_cfg.p_share_model == NULL ) {
        ADAS_NET_Binsize_mem_get(&p_net->proc_mem, p_net->net_cfg.binsize, proc_id);
        //load file
        loadsize = ADAS_load_model(p_net->net_cfg.model_filename, p_net->proc_mem.va);

        if (loadsize <= 0) {
            DBG_ERR("proc_id(%u) input model load fail: %s\r\n", proc_id, p_net->net_cfg.model_filename);
            return 0;
        }

        // load label
        ret = ADAS_load_label((UINT32)p_net->out_class_labels, VENDOR_AIS_LBL_LEN, p_net->net_cfg.label_filename);
        if (ret != HD_OK) {
            DBG_ERR("proc_id(%u) load_label fail=%d\n", proc_id, ret);
            return HD_ERR_FAIL;
        }

        // set model
        vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_MODEL, (VENDOR_AI_NET_CFG_MODEL*)&p_net->proc_mem);
    } else {
        //--- this net_path want to share other share_net model ---
        ADAS_NET_PROC* p_share_net = (ADAS_NET_PROC*)p_net->net_cfg.p_share_model;
        VENDOR_AI_NET_INFO net_info = {0};

        // query share_net for necessary size( the size which can't reuse )
        vendor_ai_net_get(p_share_net->proc_id, VENDOR_AI_NET_PARAM_INFO, &net_info);
        if (net_info.model_part1_size == 0) {
            DBG_ERR("proc_id(%u) get share proc_id(%u) necessary size fail\r\n", proc_id, p_share_net->proc_id);
            return HD_ERR_FAIL;
        }

        ret = ADAS_mem_alloc(&p_net->proc_mem, "ai_parm_buf", net_info.model_part1_size);
        if (ret != HD_OK) {
            DBG_ERR("proc_id(%u) alloc ai_parm_buf fail\r\n", proc_id);
            return HD_ERR_FAIL;
        }
        // load file ( load share_net model for necessary part only )
        loadsize = ADAS_load_part_model(p_share_net->net_cfg.model_filename, net_info.model_part1_size, p_net->proc_mem.va);

        if (loadsize <= 0) {
            DBG_ERR("proc_id(%u) input model load fail: %s\r\n", proc_id, p_share_net->net_cfg.model_filename);
            return 0;
        }

        // load label ( load share_net label )
        ret = ADAS_load_label((UINT32)p_net->out_class_labels, VENDOR_AIS_LBL_LEN, p_share_net->net_cfg.label_filename);
        if (ret != HD_OK) {
            DBG_ERR("proc_id(%u) load_label fail=%d\n", proc_id, ret);
            return HD_ERR_FAIL;
        }

        // set model
        vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_MODEL, (VENDOR_AI_NET_CFG_MODEL*)&p_net->proc_mem);
        vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_SHAREMODEL, (VENDOR_AI_NET_CFG_MODEL*)&p_share_net->proc_mem);
    }

    // open
    vendor_ai_net_open(proc_id);

    if ((ret = ADAS_network_alloc_io_buf(net_path)) != HD_OK) {
        DBG_ERR("network alloc buf fail(%d) \n", ret);
        return ret;
    }
    return ret;
}
HD_RESULT ADAS_network_close(NET_PATH_ID net_path)
{
    HD_RESULT ret = HD_OK;
    ADAS_NET_PROC* p_net = g_Adas_net + net_path;
    UINT32 proc_id = p_net->proc_id;

    if ((ret = ADAS_network_free_io_buf(net_path)) != HD_OK)
        return ret;

    // close
    ret = vendor_ai_net_close(proc_id);

    if (p_net->net_cfg.p_share_model == NULL ) {
        ADAS_NET_Binsize_mem_rel(&p_net->proc_mem);  // common buffer
    } else {
        ADAS_mem_free(&p_net->proc_mem); // private buffer
    }

    return ret;
}
INT32 ADAS_network_get_mem_binsize(NET_PATH_ID net_path, void* p_cfg)
{
    ADAS_NET_PROC* p_net = g_Adas_net + net_path;
    UINT32 proc_id;
    ADAS_NET_PROC_CONFIG* p_proc_cfg = (ADAS_NET_PROC_CONFIG*)p_cfg;
    p_net->proc_id = net_path;
    proc_id = p_net->proc_id;

    memcpy((void*)&p_net->net_cfg, (void*)p_proc_cfg, sizeof(ADAS_NET_PROC_CONFIG));
    if (strlen(p_net->net_cfg.model_filename) == 0) {
        DBG_ERR("proc_id(%u) input model is null\r\n", proc_id);
        return 0;
    }

    p_net->net_cfg.binsize = ADAS_getsize_model(p_net->net_cfg.model_filename);
    if (p_net->net_cfg.binsize <= 0) {
        DBG_ERR("proc_id(%u) input model is not exist?\r\n", proc_id);
        return 0;
    }

    printf("proc_id(%u) set net_mem_cfg: model-file(%s), binsize=%d\r\n",
           proc_id,
           p_net->net_cfg.model_filename,
           p_net->net_cfg.binsize);

    printf("proc_id(%u) set net_mem_cfg: label-file(%s)\r\n",
           proc_id,
           p_net->net_cfg.label_filename);

    return p_net->net_cfg.binsize;
}

//float to int
int f2iround(float a)
{
    float tmp = a - (int)a;
    if(tmp >= 0.5)
        return (int)a+1;
    else
        return (int)a;
}

//Determine whether the vehicle is the preceding vehicle
BOOL Is_front_car(float kl, float kr, float bl, float br, int x, int y)
{
    if((kl*x+bl)<y && (kr*x+br)<y) {
        return TRUE;
    } else {
        return FALSE;
    }
}

//Left boundary line initialization
void Left_boundary_init(float* _left_k1, float* _left_b1, float* _left_k2, float* _left_b2)
{
    *_left_k1 = -HEIGHT_CAT*1.0 / RIO_LENGTH;
    *_left_b1 = HEIGHT_CAT;
    *_left_k2 = -HEIGHT_CAT * 1.0 / 50;
    *_left_b2 = HEIGHT_CAT * WIDTH_CAT*1.0 / (2 * 50);
}

//Right boundary line initialization
void Right_boundary_init(float* _right_k1, float* _right_b1, float* _right_k2, float* _right_b2)
{
    *_right_k1 = HEIGHT_CAT*1.0 / RIO_LENGTH;
    *_right_b1 = -(WIDTH_CAT - RIO_LENGTH)*1.0/ RIO_LENGTH* HEIGHT_CAT;
    *_right_k2 = HEIGHT_CAT * 1.0 / 50;
    *_right_b2 = -HEIGHT_CAT * WIDTH_CAT*1.0 / (2 * 50);
}

//update left boundary line
void Set_left_boundary(float _k, float _b, float* _left_k1, float* _left_b1, \
                       float* _left_k2, float* _left_b2)
{
    *_left_k1 = _k;
    *_left_b1 = _b + _k*30;
    *_left_k2 = _k;
    *_left_b2 = _b - _k*30;
}

//update right boundary line
void Set_right_boundary(float _k, float _b, float* _right_k1, float* _right_b1, \
                        float* _right_k2, float* _right_b2)
{
    *_right_k1 = _k;
    *_right_b1 = _b - _k*30;
    *_right_k2 = _k;
    *_right_b2 = _b + _k*30;
}

//Determine whether the pixel is within the lane line area
BOOL In_range(int x, int y, float left_k1, float left_b1, float left_k2, float left_b2, \
              float right_k1, float right_b1, float right_k2, float right_b2)
{
    if(((left_k1*x + left_b1 < y)&&(left_k2*x + left_b2 > y)) || ((right_k1*x + right_b1 < y)&&(right_k2*x + right_b2 > y))) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void set_pre_lane(Lane* pre_lane, Lane* lane)
{
    memcpy(pre_lane, lane, sizeof(Lane));
}

BOOL is_stable_lane(Lane* pre_lane, Lane* lane)
{
    if(fabs(lane->k - pre_lane->k) < 0.08 && fabs(lane->b - pre_lane->b) < 15) {
        return TRUE;
    } else {
        return FALSE;
    }
}

BOOL is_reliable_pre_lane(Lane* pre_lane)
{
    if(pre_lane->Confidence>LANE_CONFIDENCE) {
        return TRUE;
    } else {
        return FALSE;
    }
}

void set_prediction_lane(Lane* pre_lane, Lane* lane)
{
    memcpy(lane, pre_lane, sizeof(Lane));
    // lane->Confidence -= 3;
}


UINT32 which_reliable(Lane* left_lane, Lane* right_lane)
{
    UINT32 ret = 0;	// both reliable
    int x1, x2;
    if(0 != left_lane->k && 0 != right_lane->k) {
        x1 = (HEIGHT_CAT/2 - left_lane->b) / left_lane->k;
        x2 = (HEIGHT_CAT/2 - right_lane->b) / right_lane->k;
        if((x2-x1)>280) {
            if(fabs(left_lane->k) < right_lane->k) {
                ret = 1;
            } else {
                ret = 2;
            }
        } else if((x2-x1)<110) {
            if(fabs(left_lane->k) < right_lane->k) {
                ret = 2;
            } else {
                ret = 1;
            }
        }
        // if(fabs(left_lane->k) > 1.3 && fabs(right_lane->k) > 1.3){
        //     ret = 3;
        // }
    }

    return ret;
}

//update center point x coordinate
void set_center_point(LaneDetector *_lanedetector, int x)
{
    _lanedetector->center_x = x;
}

//set memory to save RIO image data and lines
void create_line(LaneDetector *_lanedetector, ADAS_MEM_RANGE *p_share_mem)
{
    _lanedetector->lines = (line_int_t*)p_share_mem[0].va;
    _lanedetector->left_lines = (line_int_t*)p_share_mem[1].va;
    _lanedetector->right_lines = (line_int_t*)p_share_mem[2].va;
}

//Clear memory to save RIO image data and lines
void clean_line(LaneDetector *_lanedetector)
{
    memset(_lanedetector->lines, 0, 100 * sizeof(line_int_t));
    memset(_lanedetector->left_lines, 0, 50 * sizeof(line_int_t));
    memset(_lanedetector->right_lines, 0, 50 * sizeof(line_int_t));
    _lanedetector->lines_num = 0;
    _lanedetector->left_lines_num = 0;
    _lanedetector->right_lines_num = 0;
}


//Fitting line return confidence
void LineFitLeastSquares(point_int_t* point, int point_num, Lane *Result)
{
    float A = 0.0;
    float B = 0.0;
    float C = 0.0;
    float D = 0.0;
    float E = 0.0;
    float F = 0.0;

    int i;
    for (i = 0; i < point_num; i++) {
        A += point[i].x * point[i].x;
        B += point[i].x;
        C += point[i].x * point[i].y;
        D += point[i].y;
    }

    float a, b, temp = 0;
    temp = (point_num*A - B * B);
    if (temp) {
        a = (point_num*C - B * D) / temp;
        b = (A*D - B * C) / temp;
    } else {
        a = 1;
        b = 0;
    }

    float Xmean, Ymean;
    Xmean = B / point_num;
    Ymean = D / point_num;

    float tempSumXX = 0.0, tempSumYY = 0.0;
    for (i = 0; i < point_num; i++) {
        tempSumXX += (point[i].x - Xmean) * (point[i].x - Xmean);
        tempSumYY += (point[i].y - Ymean) * (point[i].y - Ymean);
        E += (point[i].x - Xmean) * (point[i].y - Ymean);
    }
    F = sqrt(tempSumXX) * sqrt(tempSumYY);

    if(0 == F) {
        F += 0.00000000001;
    }
    float r;
    r = E / F;

    Result->k = a;
    Result->b = b;
    Result->Confidence = r*r*100;
}

//read bmp img file
void readbmpfile(char* name, UINT8* out_img)
{
    if(!(name && out_img)) {
        printf("Error bmpWrite.");
        return;
    }
    UINT32 i;//,length;
    UINT32 width, height;
    FILE *pFile = fopen(name,"rb");
    if(!pFile) {
        printf("Error opening file.\r\n");
        return;
    }
    if(0 != fseek(pFile, 18, 0)) {
        printf("Error seeking file.\r\n");
        goto exit;
    }
    fread(&width, sizeof(UINT32), 1, pFile);
    fread(&height, sizeof(UINT32), 1, pFile);
    if(0 != fseek(pFile, 1078, 0)) {
        printf("Error seeking file.\r\n");
        goto exit;
    }
    UINT8* pdata = out_img + (height - 1) * width;
    for(i = 0; i < height; i++, pdata -= width) {
        fread(pdata, sizeof(UINT8), width, pFile);
    }
exit:
    fclose(pFile);
}

//save bmp img file
void writebmpfile(char* name, UINT8* raw_img, int width, int height)
{
    if(!(name && raw_img)) {
        printf("Error bmpWrite.");
        return;
    }
    int i;
    // FileHeader
    struct Bmp_File_Header file_h;
    file_h.bfType=0x4d42;
    file_h.bfReserved1=0;
    file_h.bfReserved2=0;
    file_h.bfOffBits=1078;
    file_h.bfSize = file_h.bfOffBits + width*height;
    // BmpInfoHeader
    struct Bmp_Info_Header info_h;
    info_h.biSize=40;
    info_h.biWidth=width;
    info_h.biHeight=height;
    info_h.biPlanes=1;
    info_h.biBitCount=8;
    info_h.biCompression=0;
    info_h.biSizeImage=width*height;
    info_h.biXPelsPerMeter=0;
    info_h.biYPelsPerMeter=0;
    info_h.biClrUsed=0;
    info_h.biClrImportant=0;

    Rgb_Quad color_table[256];
    for(i=0; i<256; i++) {
        color_table[i].rgbBlue = i;
        color_table[i].rgbGreen = i;
        color_table[i].rgbRed = i;
        color_table[i].rgbReserved = 0;
    }

    FILE *pFile = fopen(name,"wb");
    if(!pFile) {
        printf("Error opening file.\r\n");
        return;
    }

    printf("write file start!\r\n");
    if(sizeof(file_h) != fwrite(&file_h, 1, sizeof(file_h), pFile)) {
        printf("write file head error!\r\n");
        goto exit;
    }
    if(sizeof(info_h) != fwrite(&info_h, 1, sizeof(info_h), pFile)) {
        printf("write info head error!\r\n");
        goto exit;
    }
    if(sizeof(Rgb_Quad)*256 != fwrite(color_table, 1, sizeof(Rgb_Quad)*256, pFile)) {
        printf("write color_table error!\r\n");
        goto exit;
    }

    for(i = height-1; i>=0; i--) {
        UINT8 *row_data = raw_img+width*i;
        if((size_t)width != fwrite(row_data, 1, width, pFile)) {
            printf("write bmp data error!\r\n");
            goto exit;
        }
    }
    fflush(pFile);
    fsync(fileno(pFile));
    printf("write file end!\r\n");

exit:
    fclose(pFile);
}


void my_crop(void *dst, void *src, UINT32 src_w, UINT32 x, UINT32 y, UINT32 w, UINT32 h){
    char *p_src = (char *)src + src_w * (y-1) + x;
    char *p_dst = (char *)dst;
    UINT32 i;
    for(i = 0; i < h; i++, p_src += src_w, p_dst += w){
        memcpy(p_dst, p_src, w);
    }
    return ;
}

//get RIO and image preprocessing
void edgeDetector(HD_VIDEO_FRAME *video, ADAS_MEM_RANGE *p_share_mem)
{
    int i;
    HD_RESULT ret = HD_OK;
    UINT32 out_lofs =0, out_selection=0;

    #if EDGE_DEBUG
    static int input_num = 1;
    char input_img_path[64],sobel_img_path[64];
    #if LINER_ENHANCE
    char enhance_img_path[64];
    snprintf(enhance_img_path, 64, "//mnt//sd//img//enhance_img//enhance_img_%05d.bmp", input_num);
    #endif
    snprintf(input_img_path, 64, "//mnt//sd//img//input_img//result_img_%05d.bmp", input_num);
    snprintf(sobel_img_path, 64, "//mnt//sd//img//sobel_img//sobel_img_%05d.bmp", input_num);
    input_num++;
    readbmpfile(input_img_path, (UINT8 *)p_share_mem[0].va);
    #endif


    char *src_img = (char *)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, video->phy_addr[0], ADAS_IMG_BUF_SIZE);
    my_crop((void *)p_share_mem[0].va, (void *)src_img, ADAS_IMG_WIDTH, X_CAT, Y_CAT, WIDTH_CAT, HEIGHT_CAT);
    hd_common_mem_munmap((void *)src_img, ADAS_IMG_BUF_SIZE);


    #if LINER_ENHANCE
    UINT8 *en_img = (UINT8 *)p_share_mem[0].va;
    int max = 0, min = 1000;
    for(i = 0; i < HEIGHT_CAT*WIDTH_CAT; i++, en_img++) {
        if(*en_img > max) {
            max = *en_img;
        }
        if(*en_img < min) {
            min = *en_img;
        }
    }

    en_img = (UINT8 *)p_share_mem[0].va;
    for(i = 0; i < HEIGHT_CAT*WIDTH_CAT; i++, en_img++) {
        *en_img = (UINT8)(255.0 * (*en_img - min) / (max - min));
    }

    // writebmpfile(enhance_img_path, (void *)p_share_mem[0].va, WIDTH_CAT, HEIGHT_CAT);
    #endif



    UINT32                             ch = 0;
    VENDOR_IVE_IN_IMG_INFO             ive_input_info = {0};
    VENDOR_IVE_IMG_IN_DMA_INFO         ive_input_addr = {0};
    VENDOR_IVE_IMG_OUT_DMA_INFO        ive_output_addr = {0};
    VENDOR_IVE_GENERAL_FILTER_PARAM    ive_general_filter_param = {0};
    VENDOR_IVE_MEDIAN_FILTER_PARAM     ive_median_filter_param = {0};
    VENDOR_IVE_EDGE_FILTER_PARAM       ive_edge_filter_param = {0};
    VENDOR_IVE_NON_MAX_SUP_PARAM       ive_non_max_sup_param = {0};
    VENDOR_IVE_THRES_LUT_PARAM         ive_thres_lut_param = {0};
    VENDOR_IVE_MORPH_FILTER_PARAM      ive_morph_filter_param = {0};
    VENDOR_IVE_INTEGRAL_IMG_PARAM      ive_integral_img_param = {0};
    VENDOR_IVE_TRIGGER_PARAM           ive_trigger_param = {0};
    VENDOR_IVE_OUTSEL_PARAM            ive_outsel_param = {0};


    // UINT32 general_filter_coef[VENDOR_IVE_GEN_FILT_NUM] = {14,1,1,11,12,12,9,9,12,1};
    // UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] ={{-1,-2,-1,0,0,0,1,2,1},{1,0,-1,2,0,-2,1,0,-1}};
    UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] = {{-1,0,1,-2,0,2,-1,0,1},{1,2,1,0,0,0,-1,-2,-1}};


    ret = vendor_ive_init();
    if (ret != HD_OK) {
        DBG_ERR("err:vendor_ive_init error %d\r\n",ret);
        goto exit;
    }

    // set input & output info
    ive_input_addr.addr = p_share_mem[0].pa;
    ive_output_addr.addr = p_share_mem[1].pa;
    // ive_input_info.width = WIDTH;
    // ive_input_info.height = HEIGHT;
    // ive_input_info.lineofst = WIDTH;
    // out_lofs = WIDTH;
    ive_input_info.width = WIDTH_CAT;
    ive_input_info.height = HEIGHT_CAT;
    ive_input_info.lineofst = WIDTH_CAT;
    out_lofs = WIDTH_CAT;
    out_selection = 3;

    ret = vendor_ive_set_param(VENDOR_IVE_INPUT_INFO, &ive_input_info, ch);
    if (ret != HD_OK) {
        DBG_ERR("err: ive set input info error %d\n", ret);
        goto exit;
    }
    ret = vendor_ive_set_param(VENDOR_IVE_INPUT_ADDR, &ive_input_addr, ch);
    if (ret != HD_OK) {
        DBG_ERR("err: ive set input addr error %d\n", ret);
        goto exit;
    }
    ret = vendor_ive_set_param(VENDOR_IVE_OUTPUT_ADDR, &ive_output_addr, ch);
    if (ret != HD_OK) {
        DBG_ERR("err: ive set output addr error %d\n", ret);
        goto exit;
    }

    // memset((void *)p_share_mem[12].va, 0, WIDTH * HEIGHT);
    memset((void *)p_share_mem[1].va, 0, WIDTH_CAT * HEIGHT_CAT);

    // set func enable
    ive_general_filter_param.enable  = 0;
    ive_median_filter_param.enable  = 1;
    ive_edge_filter_param.enable    = 1;
    ive_non_max_sup_param.enable    = 1;
    ive_thres_lut_param.enable      = 0;
    ive_morph_filter_param.enable   = 0;
    ive_integral_img_param.enable   = 0;
    ive_trigger_param.time_out_ms   = 0;
    ive_trigger_param.wait_end      = 1;
    ive_trigger_param.is_nonblock   = 0;


    // set median filter param
    if (ive_median_filter_param.enable)
        ive_median_filter_param.mode = VENDOR_IVE_MEDIAN;

    // set edge filter param
    ive_edge_filter_param.mode = VENDOR_IVE_BI_DIR;
    ive_edge_filter_param.AngSlpFact = 17;
    for (i=0; i<VENDOR_IVE_EDGE_COEFF_NUM; i++) {
        ive_edge_filter_param.edge_coeff1[i] = edge_filter_coef[0][i];
        ive_edge_filter_param.edge_coeff2[i] = edge_filter_coef[1][i];
    }
    ive_edge_filter_param.edge_shift_bit = 0;


    // set non max sup param
    ive_non_max_sup_param.mag_thres = 90;

    //set output select and output lofset
    ive_outsel_param.OutDataSel = out_selection;
    ive_outsel_param.Outlofs = out_lofs;

    ret = vendor_ive_set_param(VENDOR_IVE_OUTSEL, &ive_outsel_param, ch);
    if (ret != HD_OK) {
        DBG_ERR("err: ive set ive_outsel_param error %d\n", ret);
        goto exit;
    }

    ret = vendor_ive_set_param(VENDOR_IVE_GENERAL_FILTER, &ive_general_filter_param, ch);
    ret = vendor_ive_set_param(VENDOR_IVE_MEDIAN_FILTER, &ive_median_filter_param, ch);
    ret = vendor_ive_set_param(VENDOR_IVE_EDGE_FILTER, &ive_edge_filter_param, ch);
    ret = vendor_ive_set_param(VENDOR_IVE_NON_MAX_SUP, &ive_non_max_sup_param, ch);
    ret = vendor_ive_set_param(VENDOR_IVE_THRES_LUT, &ive_thres_lut_param, ch);
    ret = vendor_ive_set_param(VENDOR_IVE_MORPH_FILTER, &ive_morph_filter_param, ch);
    ret = vendor_ive_set_param(VENDOR_IVE_INTEGRAL_IMG, &ive_integral_img_param, ch);

    // trigger engine
    ret = vendor_ive_trigger(&ive_trigger_param, ch);
    if (ret != HD_OK) {
        DBG_ERR("err: ive trigger engine error %d\n", ret);
        goto exit;
    }

exit:

    ret = vendor_ive_uninit();
    if (HD_OK != ret)
        DBG_ERR("uninit fail, error code = %d\r\n", ret);

    // memcpy(cat_img, (void *)p_share_mem[1].va, WIDTH_CAT * HEIGHT_CAT);

    #if EDGE_DEBUG
    writebmpfile(sobel_img_path, (void *)p_share_mem[1].va, WIDTH_CAT, HEIGHT_CAT);
    #endif

}

//get lane line area
void mask(uint8_t* img, float _left_k1, float _left_b1, float _left_k2, float _left_b2,
          float _right_k1, float _right_b1, float _right_k2, float _right_b2)
{
    int i,j;
    for (i = 0; i < HEIGHT_CAT; i++) {
        for (j = 0; j < WIDTH_CAT; j++) {
            if (FALSE == In_range(j, i, _left_k1, _left_b1, _left_k2, _left_b2,
                                  _right_k1, _right_b1, _right_k2, _right_b2)) {
                img[i*WIDTH_CAT + j] = 0;
            }
        }
    }
}


//hough lines detect
int hough_line_probabilistic(LaneDetector *_lanedetector, int w, int h, int rho,
                             int piece_num, int threshold, int lineLength, int lineGap, int linesMax,ADAS_MEM_RANGE *p_share_mem)
{

    int i, j;
    int offset, offy;

    float theta = PI / piece_num;
    int max_line_length = (int)ceil(sqrt(w*w + h * h));
    int numrho = max_line_length * 2 * rho + 5;

//	int *_accum = new int[numangle*numrho]();
//	unsigned char *_mask = new unsigned char[h*w];
    memset((void*)p_share_mem[2].va, 0, 350*1024);
    memset((void*)p_share_mem[1].va, 0, h*w);
    float* trigtab = (float*)p_share_mem[2].va;
    int8_t *_accum = (int8_t*)p_share_mem[2].va + 2048;
    uint8_t *_mask = (uint8_t*)p_share_mem[1].va;

    // int8_t *_accum = (int8_t*)malloc(piece_num*numrho*sizeof(int8_t));
    // uint8_t *_mask = (uint8_t*)malloc(h*w*sizeof(uint8_t));
    // uint8_t *_image = _lanedetector->graydata_cat;
    uint8_t *_image = (UINT8*)p_share_mem[0].va;
    line_int_t *_lines = _lanedetector->lines;

    // memset(_accum, 0, piece_num*numrho*sizeof(int8_t));


//	std::vector<float> trigtab(numangle * 2);
    // float* trigtab = (float*)malloc(piece_num * 2 * sizeof(float));

    for (int n = 0; n < piece_num; n++) {
        offset = n * 2;
        trigtab[offset] = cos(n*theta);
        trigtab[offset + 1] = sin(n*theta);
    }
    float* ttab = trigtab;
    uint8_t* mdata0 = _mask;
    memset((void*)p_share_mem[3].va, 0, w*h*sizeof(point_int_t));
    point_int_t* nzloc = (point_int_t*)p_share_mem[3].va;
    // point_int_t* nzloc = (point_int_t*)malloc(w*h*sizeof(point_int_t));
    point_int_t* p_nzloc = nzloc;
    int count = 0;
    int num_of_lines = 0;
    // stage 1. collect non-zero image points

    #if LINER_ENHANCE
    int thr = 50;
    #else
    int thr = 35;
    #endif

    for (i = 0; i < h; i++) {
        offy = i*w;
        for (j = 0; j < w; j++) {
            if (_image[offy + j] > thr) {
                _mask[offy + j] = (uint8_t)1;
//				point_int_t temp_point = {j,i};
                (*p_nzloc).x = j;
                (*(p_nzloc++)).y = i;
                count++;
            } else
                _mask[offy + j] = 0;
        }
    }

    // stage 2. process all the points in random order

    int idx,max_val, max_n,k, x0, y0, dx0, dy0, xflag, i1, j1, gap, x, y, dx, dy;
    point_int_t point, line_end[2];
    float a, b;
    int8_t* adata = NULL;
    BOOL good_line;
    int shift = 16;
    for (; count > 0; count--) {
        // choose random point out of the remaining ones
        idx = rand() % count;
        max_val = threshold - 1;
        max_n = 0;
        point = nzloc[idx];
        adata = _accum;
        i = point.y, j = point.x;
        // "remove" it by overriding it with the last element
        nzloc[idx].x = nzloc[count - 1].x;
        nzloc[idx].y = nzloc[count - 1].y;

        // check if it has been excluded already (i.e. belongs to some other line)
        if (!mdata0[i*w + j])
            continue;

        // update accumulator, find the most probable line
        for (int n = 0; n < piece_num; n++, adata += numrho) {
            int r = f2iround((j * ttab[n * 2] + i * ttab[n * 2 + 1])*rho);
            r += max_line_length*rho;
            int val = ++(adata[r]);
            if (max_val < val) {
                max_val = val;
                max_n = n;
// #if ADAS_DEBUG
// 				printf("count = %d n = %d max_val = %d\r\n", count, n, max_val);
// #endif
            }
        }

        // if it is too "weak" candidate, continue with another point
        if (max_val < threshold)
            continue;

        // from the current point walk in each direction
        // along the found line and extract the line segment
        a = -ttab[max_n * 2 + 1];
        b = ttab[max_n * 2];

        if(0 == a) {
            a += 0.00000000001;
        }
        if(0 == b) {
            b += 0.00000000001;
        }
        x0 = j;
        y0 = i;
        if (fabs(a) > fabs(b)) {
            xflag = 1;
            dx0 = a > 0 ? 1 : -1;
            dy0 = (int)f2iround(b*(1 << shift) / fabs(a));
            y0 = (y0 << shift) + (1 << (shift - 1));
        } else {
            xflag = 0;
            dy0 = b > 0 ? 1 : -1;
            dx0 = (int)f2iround(a*(1 << shift) / fabs(b));
            x0 = (x0 << shift) + (1 << (shift - 1));
        }

        for (k = 0; k < 2; k++) {
            gap= 0;
            x = x0;
            y = y0;
            dx = dx0;
            dy = dy0;

            if (k > 0) {
                dx = -dx;
                dy = -dy;
            }

            // walk along the line using fixed-point arithmetics,
            // stop at the image border or in case of too big gap
            for (;; x += dx, y += dy) {
                if (xflag) {
                    j1 = x;
                    i1 = y >> shift;
                } else {
                    j1 = x >> shift;
                    i1 = y;
                }

                if (j1 < 0 || j1 >= w || i1 < 0 || i1 >= h)
                    break;

                // for each non-zero point:
                //    update line end,
                //    clear the mask element
                //    reset the gap
                if (mdata0[i1*w+j1]) {
                    gap = 0;
                    line_end[k].y = i1;
                    line_end[k].x = j1;
                } else if (++gap > lineGap)
                    break;
            }
        }

        if((abs(line_end[1].x - line_end[0].x) >= lineLength) || (abs(line_end[1].y - line_end[0].y) >= lineLength)) {
            good_line = TRUE;
        } else {
            good_line = FALSE;
        }

        for (k = 0; k < 2; k++) {
            x = x0;
            y = y0;
            dx = dx0;
            dy = dy0;

            if (k > 0)
                dx = -dx, dy = -dy;

            // walk along the line using fixed-point arithmetics,
            // stop at the image border or in case of too big gap
            for (;; x += dx, y += dy) {
                if (xflag) {
                    j1 = x;
                    i1 = y >> shift;
                } else {
                    j1 = x >> shift;
                    i1 = y;
                }

                // for each non-zero point:
                //    update line end,
                //    clear the mask element
                //    reset the gap
                if (mdata0[i1*w + j1]) {
                    if (TRUE == good_line) {
                        adata = _accum;
                        for (int n = 0; n < piece_num; n++, adata += numrho) {
                            int r = f2iround((j1 * ttab[n * 2] + i1 * ttab[n * 2 + 1])*rho);
                            r += max_line_length*rho;
                            adata[r]--;
                        }
                    }
                    mdata0[i1*w+j1] = 0;
                }

                if (i1 == line_end[k].y && j1 == line_end[k].x)
                    break;
            }
        }

        if (TRUE == good_line) {
//			line_int_t _lr = {line_end[0].x, line_end[0].y, line_end[1].x, line_end[1].y };
//			_lines.push_back(_lr);
            _lines[num_of_lines].startx = line_end[0].x;
            _lines[num_of_lines].starty = line_end[0].y;
            _lines[num_of_lines].endx = line_end[1].x;
            _lines[num_of_lines].endy = line_end[1].y;
            num_of_lines++;
            if ((int)num_of_lines >= linesMax) {
                break;
            }
        }
    }
    _lanedetector->lines_num = num_of_lines;
//	delete[]_accum;
//	delete[]_mask;

    // free(nzloc);
    // free(trigtab);
    // free(_mask);
    // free(_accum);

    return num_of_lines;
}

//Distinguish left and right lines.  Eliminate interference lines
void lineSeparation(LaneDetector *_lanedetector, ADAS_MEM_RANGE *p_share_mem)
{
    int j = 0;
    float slope_thresh = 0.25;
    memset((void*)p_share_mem[0].va, 0, 50*sizeof(float));
    memset((void*)p_share_mem[1].va, 0, 50*sizeof(line_int_t));
    float* slopes = (float*)p_share_mem[0].va;
    line_int_t* selected_lines = (line_int_t*)p_share_mem[1].va;
    // float* slopes = (float*)malloc(50*sizeof(float));
    // float* bbbs = (float*)malloc(50*sizeof(float));
    // line_int_t* selected_lines = (line_int_t*)malloc(50*sizeof(line_int_t));
    int slopes_num = 0;
    _lanedetector->left_lines_num = 0;
    _lanedetector->right_lines_num = 0;

    line_int_t *line = _lanedetector->lines;
    line_int_t *left_line = _lanedetector->left_lines;
    line_int_t *right_line = _lanedetector->right_lines;
    Lane *pre_left = &(_lanedetector->pre_left_lane);
    Lane *pre_right = &(_lanedetector->pre_right_lane);
    int* left_num = &(_lanedetector->left_lines_num);
    int* right_num = &(_lanedetector->right_lines_num);

    // Calculate the slope of all the detected lines
    for (j = 0; j < _lanedetector->lines_num; j++) {
        // Basic algebra: slope = (y1 - y0)/(x1 - x0)
        float slope = (line[j].endy - line[j].starty) / (line[j].endx - line[j].startx + 0.00001);
        // float bbb = line[j].starty - slope*line[j].startx;

//		DBG_DUMP("slope = %f\r\n", slope);
        // If the slope is too horizontal, discard the line
        // If not, save them  and their respective slope
        if (fabs(slope) > slope_thresh) {
            slopes[slopes_num] = slope;
            // bbbs[slopes_num] = bbb;
            selected_lines[slopes_num].startx = line[j].startx;
            selected_lines[slopes_num].starty = line[j].starty;
            selected_lines[slopes_num].endx = line[j].endx;
            selected_lines[slopes_num].endy = line[j].endy;
            slopes_num++;
        }
    }

    // Split the lines into right and left lines
    for(j = 0; j < slopes_num; j++) {
        // Condition to classify line as left side or right side
        if ((slopes[j] > 0) && (slopes[j] < 10) && (selected_lines[j].endx >= (_lanedetector->center_x-15)) && (selected_lines[j].startx >= (_lanedetector->center_x-15))) {
            if(TRUE == is_reliable_pre_lane(pre_right) && pre_right->k < 0.8) {
                if(fabs(pre_right->k - slopes[j]) < 0.3) {
                    right_line[*right_num].startx = selected_lines[j].startx;
                    right_line[*right_num].starty = selected_lines[j].starty;
                    right_line[*right_num].endx = selected_lines[j].endx;
                    right_line[*right_num].endy = selected_lines[j].endy;
                    _lanedetector->right_lines_num++;
                }
            } else {
                right_line[*right_num].startx = selected_lines[j].startx;
                right_line[*right_num].starty = selected_lines[j].starty;
                right_line[*right_num].endx = selected_lines[j].endx;
                right_line[*right_num].endy = selected_lines[j].endy;
                _lanedetector->right_lines_num++;
            }

        } else if ((slopes[j] < 0) && (slopes[j] > -10) && (selected_lines[j].endx <= (_lanedetector->center_x+15)) && (selected_lines[j].startx <= (_lanedetector->center_x+15))) {
            if(TRUE == is_reliable_pre_lane(pre_left) && pre_left->k > -0.8) {
                if(fabs(pre_left->k - slopes[j]) < 0.3) {
                    left_line[*left_num].startx = selected_lines[j].startx;
                    left_line[*left_num].starty = selected_lines[j].starty;
                    left_line[*left_num].endx = selected_lines[j].endx;
                    left_line[*left_num].endy = selected_lines[j].endy;
                    _lanedetector->left_lines_num++;
                }
            } else {
                left_line[*left_num].startx = selected_lines[j].startx;
                left_line[*left_num].starty = selected_lines[j].starty;
                left_line[*left_num].endx = selected_lines[j].endx;
                left_line[*left_num].endy = selected_lines[j].endy;
                _lanedetector->left_lines_num++;
            }
        }
    }

    // free(slopes);
    // free(bbbs);
    // free(selected_lines);
    #if ADAS_DEBUG
    printf("#####left_lines_num = %d\r\n", _lanedetector->left_lines_num);
    for(int i = 0; i< _lanedetector->left_lines_num; i++) {
        printf("x1 = %d  y1 = %d  x2 = %d  y2 = %d\r\n", left_line[i].startx, left_line[i].starty, left_line[i].endx, left_line[i].endy);
    }
    printf("########################\r\n");
    printf("#####right_lines_num = %d\r\n", _lanedetector->right_lines_num);
    for(int i = 0; i< _lanedetector->right_lines_num; i++) {
        printf("x1 = %d  y1 = %d  x2 = %d  y2 = %d\r\n", right_line[i].startx, right_line[i].starty, right_line[i].endx, right_line[i].endy);
    }
    printf("########################\r\n");
    #endif
}

//Linear clustering
int DBSCAN(line_int_t *_line, float *line_k, int *order, int num)
{
    int i, j;
    float temp;
    int class_num[50];
    int lane_num = 0;

    for(i=0; i<num; i++) {
        line_k[i] = (float)fabs((_line[i].endy - _line[i].starty) / (_line[i].endx - _line[i].startx + 0.00001));
        order[i] = i;
    }

    for(i=0; i<num-1; i++) {
        for(j=i+1; j<num; j++) {
            if(line_k[i]<line_k[j]) {
                temp = line_k[i];
                line_k[i] = line_k[j];
                line_k[j] = temp;
                order[i] = order[i] + order[j];
                order[j] = order[i] - order[j];
                order[i] = order[i] - order[j];
            }
        }
    }

    class_num[0] = 1;

    if(line_k[0]>1 && (line_k[0]- line_k[num-1])>0.5) {
        lane_num = (num>=4?4:num);
    } else {
        for(i=1; i<num; i++) {
            if((line_k[i-1] - line_k[i])>0.06) {
                class_num[i] = class_num[i-1]+1;
            } else {
                class_num[i] = class_num[i-1];
            }
        }


        for(i=1; i<num; i++) {
            if(1 == class_num[i-1] && 2 == class_num[i]) {
                lane_num = (i>=4?4:i);
                break;
            }
        }
        if(num == i) {
            lane_num = (num>=4?4:num);
        }
    }
    return lane_num;
}

//Fitting left and right lane lines
void regression(LaneDetector *_lanedetector, Lane* left_lane, Lane* right_lane, ADAS_MEM_RANGE *p_share_mem)
{
    // point_int_t *right_pts = NULL, *left_pts = NULL;
    int j;

    line_int_t *left_line = _lanedetector->left_lines;
    line_int_t *right_line = _lanedetector->right_lines;
    int left_num = _lanedetector->left_lines_num;
    int right_num = _lanedetector->right_lines_num;

    memset((void*)p_share_mem[0].va, 0, 120*sizeof(point_int_t));
    memset((void*)p_share_mem[1].va, 0, 120*sizeof(point_int_t));
    point_int_t *left_pts = (point_int_t*)p_share_mem[0].va;
    point_int_t *right_pts = (point_int_t*)p_share_mem[1].va;

    memset((void*)p_share_mem[2].va, 0, 50*sizeof(float));
    memset((void*)p_share_mem[3].va, 0, 50*sizeof(int));
    memset((void*)p_share_mem[4].va, 0, 50*sizeof(float));
    memset((void*)p_share_mem[5].va, 0, 50*sizeof(int));
    float *right_line_k = (float *)malloc(right_num*sizeof(float));
    int *right_order = (int *)malloc(right_num*sizeof(int));
    float *left_line_k = (float *)malloc(left_num*sizeof(float));
    int *left_order = (int *)malloc(left_num*sizeof(int));


    // If left lines are being detected, fit a line using all the init and final points of the lines
    if(left_num>1) {
        // left_pts = (point_int_t*)malloc(left_num*2*sizeof(point_int_t));
        int l_num = DBSCAN(left_line, left_line_k, left_order, left_num);

        for (j = 0; j < l_num; j++) {
            left_pts[2*j].x = left_line[left_order[j]].startx;
            left_pts[2*j].y = left_line[left_order[j]].starty;
            left_pts[2*j+1].x = left_line[left_order[j]].endx;
            left_pts[2*j+1].y = left_line[left_order[j]].endy;
        }
        LineFitLeastSquares(left_pts, l_num*2, left_lane);
    } else if(1 == left_num) {
        left_pts[0].x = left_line[0].startx;
        left_pts[0].y = left_line[0].starty;
        left_pts[1].x = left_line[0].endx;
        left_pts[1].y = left_line[0].endy;
        LineFitLeastSquares(left_pts, 2, left_lane);
    }

    // If right lines are being detected, fit a line using all the init and final points of the lines
    if(right_num>1) {
        // right_pts = (point_int_t*)malloc(right_num*2*sizeof(point_int_t));
        int r_num = DBSCAN(right_line, right_line_k, right_order, right_num);

        for(j = 0; j < r_num; j++) {
            right_pts[2*j].x = right_line[right_order[j]].startx;
            right_pts[2*j].y = right_line[right_order[j]].starty;
            right_pts[2*j+1].x = right_line[right_order[j]].endx;
            right_pts[2*j+1].y = right_line[right_order[j]].endy;
        }
        LineFitLeastSquares(right_pts, r_num*2, right_lane);
    } else if(1 == right_num) {
        right_pts[0].x = right_line[0].startx;
        right_pts[0].y = right_line[0].starty;
        right_pts[1].x = right_line[0].endx;
        right_pts[1].y = right_line[0].endy;
        LineFitLeastSquares(right_pts, 2, right_lane);
    }
}

//Reduce false positives
BOOL reduce_false_alarms(int lx, int rx)
{
    BOOL ret = TRUE;
    //Parameter 20 is set empirically and can be modified according to the pitch angle of the camera
    if((lx+rx)<20) { 
        ret = FALSE;
    }
    return ret;
}


//Draw lane lines in the image and save
#if LANE_IMG_SAVE
void draw_line(Lane *left, Lane *right, void* arg)
{
    ADAS_MEM_RANGE *p_share_mem = (ADAS_MEM_RANGE*)arg;
    HD_GFX_DRAW_RECT   rectt[4];
    HD_RESULT           ret;

    memset(rectt, 0, sizeof(rectt));

    if(left->Confidence >= LANE_CONFIDENCE) {
        rectt[0].dst_img.dim.w 			= WIDTH_CAT;
        rectt[0].dst_img.dim.h 			= HEIGHT_CAT;
        rectt[0].dst_img.format 		= HD_VIDEO_PXLFMT_Y8;
        rectt[0].dst_img.p_phy_addr[0] 	= p_share_mem[0].pa;
        rectt[0].dst_img.lineoffset[0]	= WIDTH_CAT;
        rectt[0].color 					= 0x0;
        rectt[0].rect.x 				= (UINT32)((10-left->b)/left->k);
        rectt[0].rect.y 				= (UINT32)10;
        rectt[0].rect.w 				= 6;
        rectt[0].rect.h 				= 6;
        rectt[0].type 					= HD_GFX_RECT_SOLID;

        rectt[1].dst_img.dim.w 			= WIDTH_CAT;
        rectt[1].dst_img.dim.h 			= HEIGHT_CAT;
        rectt[1].dst_img.format 		= HD_VIDEO_PXLFMT_Y8;
        rectt[1].dst_img.p_phy_addr[0] 	= p_share_mem[0].pa;
        rectt[1].dst_img.lineoffset[0]	= WIDTH_CAT;
        rectt[1].color 					= 0x0;
        rectt[1].rect.x 				= (UINT32)((70-left->b)/left->k);
        rectt[1].rect.y 				= (UINT32)70;
        rectt[1].rect.w 				= 6;
        rectt[1].rect.h 				= 6;
        rectt[1].type 					= HD_GFX_RECT_SOLID;

        ret = hd_gfx_draw_rect(&rectt[0]);
        if(ret != HD_OK) {
            DBG_ERR("hd_gfx_draw_line left fail=%d\n", ret);
            goto exit;
        }

        ret = hd_gfx_draw_rect(&rectt[1]);
        if(ret != HD_OK) {
            DBG_ERR("hd_gfx_draw_line left fail=%d\n", ret);
            goto exit;
        }
    }

    if(right->Confidence >= LANE_CONFIDENCE) {
        rectt[2].dst_img.dim.w 			= WIDTH_CAT;
        rectt[2].dst_img.dim.h 			= HEIGHT_CAT;
        rectt[2].dst_img.format 		= HD_VIDEO_PXLFMT_Y8;
        rectt[2].dst_img.p_phy_addr[0] 	= p_share_mem[0].pa;
        rectt[2].dst_img.lineoffset[0]	= WIDTH_CAT;
        rectt[2].color 					= 0x0;
        rectt[2].rect.x 				= (UINT32)((10-right->b)/right->k);
        rectt[2].rect.y 				= (UINT32)10;
        rectt[2].rect.w 				= 6;
        rectt[2].rect.h 				= 6;
        rectt[2].type 					= HD_GFX_RECT_SOLID;

        rectt[3].dst_img.dim.w 			= WIDTH_CAT;
        rectt[3].dst_img.dim.h 			= HEIGHT_CAT;
        rectt[3].dst_img.format 		= HD_VIDEO_PXLFMT_Y8;
        rectt[3].dst_img.p_phy_addr[0] 	= p_share_mem[0].pa;
        rectt[3].dst_img.lineoffset[0]	= WIDTH_CAT;
        rectt[3].color 					= 0x0;
        rectt[3].rect.x 				= (UINT32)((70-right->b)/right->k);
        rectt[3].rect.y 				= (UINT32)70;
        rectt[3].rect.w 				= 6;
        rectt[3].rect.h 				= 6;
        rectt[3].type 					= HD_GFX_RECT_SOLID;


        ret = hd_gfx_draw_rect(&rectt[2]);
        if(ret != HD_OK) {
            DBG_ERR("hd_gfx_draw_line right fail=%d\n", ret);
            goto exit;
        }
        ret = hd_gfx_draw_rect(&rectt[3]);
        if(ret != HD_OK) {
            DBG_ERR("hd_gfx_draw_line right fail=%d\n", ret);
            goto exit;
        }
    }

exit:
    return ;
}

#endif

// scale img
HD_RESULT img1920toimg320(HD_VIDEO_FRAME *video, ADAS_MEM_RANGE *p_share_mem)
{
    HD_RESULT ret = HD_OK;
    HD_GFX_SCALE scale;

    memset(&scale, 0, sizeof(HD_GFX_SCALE));
    scale.src_img.dim.w            = video->dim.w;
    scale.src_img.dim.h            = video->dim.h;
    scale.src_img.format           = video->pxlfmt;
    scale.src_img.p_phy_addr[0]    = video->phy_addr[0];
    scale.src_img.p_phy_addr[1]    = video->phy_addr[1];
    scale.src_img.lineoffset[0]    = video->loff[0];
    scale.src_img.lineoffset[1]    = video->loff[1];
    scale.src_region.x             = 0;
    scale.src_region.y             = 0;
    scale.src_region.w             = video->dim.w;
    scale.src_region.h             = video->dim.h;

    scale.dst_img.dim.w            = MD_IMG_WIDTH;
    scale.dst_img.dim.h            = MD_IMG_HEIGHT;
    scale.dst_img.format           = HD_VIDEO_PXLFMT_YUV420;
    scale.dst_img.p_phy_addr[0]    = p_share_mem[0].pa;
    scale.dst_img.p_phy_addr[1]    = p_share_mem[1].pa;
    scale.dst_img.lineoffset[0]    = MD_IMG_WIDTH;
    scale.dst_img.lineoffset[1]    = MD_IMG_WIDTH;
    scale.dst_region.x             = 0;
    scale.dst_region.y             = 0;
    scale.dst_region.w             = MD_IMG_WIDTH;
    scale.dst_region.h             = MD_IMG_HEIGHT;

    scale.quality                  = HD_GFX_SCALE_QUALITY_BILINEAR;

    ret = hd_gfx_scale_sw(&scale);
    if(ret != HD_OK) {
        DBG_ERR("img1920toimg320 fail=%d\n", ret);
    }

    return ret;
}

void bound_compute(Lane *lines, UINT32 num, Lane *res){
    UINT32 i, j, max_index = 0, min_index = 0;
    for(i = 0; i < 3; i++){
        max_index = i;
        min_index = i;
        for(j = i+1; j < num-i-1; j++){
            if(lines[j].k > lines[max_index].k){
                max_index = j;
            }
            if(lines[j].k < lines[min_index].k){
                min_index = j;
            }
        }
        Lane temp1 = lines[min_index];
        lines[min_index] = lines[i];
        lines[i] = temp1;
        Lane temp2 = lines[max_index];
        lines[max_index] = lines[num-i-1];
        lines[num-i-1] = temp2;        
    }
    float k_sum = 0.0f, b_sum = 0.0f;
    for(i = 3; i < num-3; i++){
        k_sum += lines[i].k;
        b_sum += lines[i].b;
    }
    // printf("k_sum = %f, b_sum = %f\r\n", k_sum, b_sum);
    res->k = k_sum / (num-6);
    res->b = b_sum / (num-6);
}

void bound_set(Lane *ll, Lane *rl, float* kl, float* bl, float* kr, float* br){
    float x0 = (rl->b-ll->b)/(ll->k-rl->k);
    float y0 = (ll->k*x0+ll->b);
    x0 += X_CAT;
    y0 += Y_CAT;
    // printf("x0 = %f, y0 = %f\r\n", x0, y0);
    horizon_y = (UINT32)y0;
    *kl = (y0-300)/80;
    *bl = 300 - 240*(*kl);
    *kr = -1*(*kl);
    *br = 300 - 400*(*kr);
}

//Get the serial number of the preceding vehicle
//Return 100 without the preceding vehicle
UINT32 get_pre_car_index(VENDOR_AI_POSTPROC_RESULT_INFO *net_result, float k1, float b1, float k2, float b2){

    float min_x = 640.0, center;
    UINT32 car_index = 100, index = 0, i;
    UINT32 order[5] = {0};

    // if(1 == LINE_DETECT && left_lane.Confidence>=90 && right_lane.Confidence>=90){
    //      center = lanedetector.center_x + X_CAT;
    //      k1 = left_lane.k;
    //      k2 = right_lane.k;
    //      b1 = left_lane.b+Y_CAT-X_CAT*k1;
    //      b2 = right_lane.b+Y_CAT-X_CAT*k2;
    // }else{
        center = WIDTH/2;
        // k1 = -1;
        // k2 = 1;
        // b1 = 503;
        // b2 = -137;

        // k3 = -2;
        // k4 = 2;
        // b3 = 786;
        // b4 = -494;
    // }

    // if(ii < net_result->result_num){
    for(i=0; i < net_result->result_num && index<5; i++){
        VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[i];
        if(p_rslt->no[0]==1&&p_rslt->score[0]>0.7){
            float w = p_rslt->w * 320 + 0.01;
            float h = p_rslt->h * 180;      
            if(w>15 && (h/w)>0.5 && (h/w)<1.5){
                float pos_x = (p_rslt->x + p_rslt->w/2)*320+160;
                float pos_y = (p_rslt->y + p_rslt->h)*180+120;
                // printf("FCWS: pos_x = %d  pos_y = %d\r\n", pos_x, pos_y);
                if(pos_y >= (horizon_y + 14)){
                    if(Is_front_car(k1, k2, b1, b2, pos_x, pos_y)){
                        // front_car_pos = pos_y;
                        order[index] = i;
                        index++;
                    }                    
                }
            }
        }
    }


    // for(i=0; i < net_result->result_num; i++){
    for(i=0; i < index; i++){
        VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[order[i]];
        float pos_x = (p_rslt->x + p_rslt->w/2)*320+160;
        if(fabs(pos_x-center)<min_x){
            min_x = fabs(pos_x-center);
            car_index = order[i];
        }
    }

    return car_index;
}

//MD Memory cleaning
void md_mem_clean(ADAS_MEM_RANGE *p_share_mem)
{
    for(UINT8 i = 0; i<11; i++) {
        memset((void *)p_share_mem[i].va, 0, p_share_mem[i].size);
    }
}

//MD param set
void my_libmd_param_set(VENDOR_MD_TRIGGER_PARAM* _md_trig_param, LIB_MD_MDT_LIB_INFO* _mdt_lib_param,
                        HD_IRECT *rect, ADAS_MEM_RANGE *p_share_mem, UINT32 _percentage)
{
    HD_RESULT ret = HD_OK;
    _md_trig_param->is_nonblock = 0;
    _md_trig_param->time_out_ms = 0;
    // LibMD motion detection info
    _mdt_lib_param->mdt_info.libmd_enabled = 1;
    _mdt_lib_param->mdt_info.phy_md_x_num = MD_IMG_WIDTH;
    _mdt_lib_param->mdt_info.phy_md_y_num = MD_IMG_HEIGHT;
    _mdt_lib_param->mdt_info.phy_md_rst.p_md_bitmap = (UINT8*)p_share_mem[10].va;
    _mdt_lib_param->mdt_info.phy_md_rst.md_bitmap_sz = MD_IMG_WIDTH*MD_IMG_HEIGHT;
    if ((ret = lib_md_set(0, LIB_MD_MOTION_DETECT_INFO, &_mdt_lib_param->mdt_info)) != HD_OK) {
        DBG_ERR("lib_md_set enable fail, error code = %d\r\n", ret);
        return ;
    }
    // LibMD function enable
    _mdt_lib_param->mdt_enable.globel_md_alarm_detect_en = 0;
    _mdt_lib_param->mdt_enable.subregion_md_alarm_detect_en = 1;
    _mdt_lib_param->mdt_enable.scene_change_alarm_detect_en = 0;
    if ((ret = lib_md_set(0, LIB_MD_AP_ENABLE_PARAM, &_mdt_lib_param->mdt_enable)) != HD_OK) {
        DBG_ERR("lib_md_set enable fail, error code = %d\r\n", ret);
        return ;
    }

    #if 0
    // LibMD global motion alarm
    _mdt_lib_param->mdt_global_param.motion_alarm_th = 50;
    if ((ret = lib_md_set(0, LIB_MD_AP_GLOBAL_MOTION_ALARM_PARAM, &_mdt_lib_param->mdt_global_param)) != HD_OK) {
        printf("lib_md_set global motion alarm param fail, error code = %d\r\n", ret);
        return ;
    }
    #endif
    // LibMD sub-region motion alarm
    _mdt_lib_param->mdt_subregion_param.sub_region_num = 1;
    _mdt_lib_param->mdt_subregion_param.sub_region[0].enabled = 1;
    _mdt_lib_param->mdt_subregion_param.sub_region[0].x_start = rect->x;
    _mdt_lib_param->mdt_subregion_param.sub_region[0].y_start = rect->y;
    _mdt_lib_param->mdt_subregion_param.sub_region[0].x_end = rect->x+rect->w;//160
    _mdt_lib_param->mdt_subregion_param.sub_region[0].y_end = rect->y+rect->h;//90
    _mdt_lib_param->mdt_subregion_param.sub_region[0].alarm_th = _percentage;
    if ((ret = lib_md_set(0, LIB_MD_AP_SUBREGION_MOTION_ALARM_PARAM, &_mdt_lib_param->mdt_subregion_param)) != HD_OK) {
        DBG_ERR("lib_md_set sub-region motion alarm param fail, error code = %d\r\n", ret);
        return ;
    }

    #if 0
    // LibMD scene change alarm
    _mdt_lib_param->mdt_scene_change_param.scene_change_alarm_th = 50;
    if ((ret = lib_md_set(0, LIB_MD_AP_SCENE_CHANGE_ALARM_PARAM, &_mdt_lib_param->mdt_scene_change_param)) != HD_OK) {
        printf("lib_md_set scene change alarm param fail, error code = %d\r\n", ret);
        return ;
    }
    #endif
}

//MD output array to img array
void bc_reorgS1(UINT8* inputS, UINT8* outputS,UINT32 width, UINT32 height)
{
    UINT32 i,j,count,size;
    count=0;
    size = width*height;
    for(j = 0; j < MDBC_ALIGN(size,8)/8; j++) {
        UINT8 c = inputS[j];
        for(i = 0; i < 8; i++) {
            if(count<size) {
                if((c & 0x1) == 1) {
                    outputS[count] = 255;
                } else {
                    outputS[count] = 0;
                }
                // outputS[count] = c & 0x1;
                c = c>>1;
                count++;
            }
        }
    }
}

//Set MD default sensitivity
void md_set_para_default_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
    mdbc_parm->MdmatchPara.lbsp_th    = 0xa;
    mdbc_parm->MdmatchPara.d_colour   = 0xf;
    mdbc_parm->MdmatchPara.r_colour   = 0x1e;
    mdbc_parm->MdmatchPara.d_lbsp     = 0x3;
    mdbc_parm->MdmatchPara.r_lbsp     = 0x5;
    mdbc_parm->MdmatchPara.model_num  = 0x8;
    mdbc_parm->MdmatchPara.t_alpha    = 0x33;
    mdbc_parm->MdmatchPara.dw_shift   = 0x4;
    mdbc_parm->MdmatchPara.dlast_alpha= 0x28;
    mdbc_parm->MdmatchPara.min_match  = 2;
    mdbc_parm->MdmatchPara.dlt_alpha  = 0xa;
    mdbc_parm->MdmatchPara.dst_alpha  = 0x28;
    mdbc_parm->MdmatchPara.uv_thres   = 0x14;
    mdbc_parm->MdmatchPara.s_alpha    = 0x28;
    mdbc_parm->MdmatchPara.dbg_lumDiff= 0x0;
    mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

    mdbc_parm->MorPara.th_ero     = 0x8;
    mdbc_parm->MorPara.th_dil     = 0x0;
    mdbc_parm->MorPara.mor_sel0   = 0x3;
    mdbc_parm->MorPara.mor_sel1   = 0x2;
    mdbc_parm->MorPara.mor_sel2   = 0x1;
    mdbc_parm->MorPara.mor_sel3   = 0x0;

    mdbc_parm->UpdPara.minT           = 0x4;
    mdbc_parm->UpdPara.maxT           = 0x80;
    mdbc_parm->UpdPara.maxFgFrm       = 0xff;
    mdbc_parm->UpdPara.deghost_dth    = 0xa;
    mdbc_parm->UpdPara.deghost_sth    = 0xf0;
    mdbc_parm->UpdPara.stable_frm     = 0x78;
    mdbc_parm->UpdPara.update_dyn     = 0x80;
    mdbc_parm->UpdPara.va_distth      = 10;
    mdbc_parm->UpdPara.t_distth       = 24;
    mdbc_parm->UpdPara.dbg_frmID      = 0x0;
    mdbc_parm->UpdPara.dbg_frmID_en   = 0x0;
    mdbc_parm->UpdPara.dbg_rnd        = 0x0;
    mdbc_parm->UpdPara.dbg_rnd_en     = 0x0;
}

//Set MD high sensitivity
void md_set_para_high_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
    mdbc_parm->MdmatchPara.lbsp_th    = 0x0;
    mdbc_parm->MdmatchPara.d_colour   = 10;
    mdbc_parm->MdmatchPara.r_colour   = 0x1e;
    mdbc_parm->MdmatchPara.d_lbsp     = 3;
    mdbc_parm->MdmatchPara.r_lbsp     = 5;
    mdbc_parm->MdmatchPara.model_num  = 0x8;
    mdbc_parm->MdmatchPara.t_alpha    = 0x33;
    mdbc_parm->MdmatchPara.dw_shift   = 0x4;
    mdbc_parm->MdmatchPara.dlast_alpha= 0x28;
    mdbc_parm->MdmatchPara.min_match  = 2;
    mdbc_parm->MdmatchPara.dlt_alpha  = 0xa;
    mdbc_parm->MdmatchPara.dst_alpha  = 0x28;
    mdbc_parm->MdmatchPara.uv_thres   = 20;
    mdbc_parm->MdmatchPara.s_alpha    = 0x28;
    mdbc_parm->MdmatchPara.dbg_lumDiff= 0x0;
    mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

    mdbc_parm->MorPara.th_ero     = 0x8;
    mdbc_parm->MorPara.th_dil     = 0x0;
    mdbc_parm->MorPara.mor_sel0   = 0x0;
    mdbc_parm->MorPara.mor_sel1   = 0x1;
    mdbc_parm->MorPara.mor_sel2   = 0x2;
    mdbc_parm->MorPara.mor_sel3   = 0x3;

    mdbc_parm->UpdPara.minT           = 0x8;
    mdbc_parm->UpdPara.maxT           = 0x80;
    mdbc_parm->UpdPara.maxFgFrm       = 0x80;
    mdbc_parm->UpdPara.deghost_dth    = 0xf;
    mdbc_parm->UpdPara.deghost_sth    = 0xf0;
    mdbc_parm->UpdPara.stable_frm     = 0x78;
    mdbc_parm->UpdPara.update_dyn     = 0x80;
    mdbc_parm->UpdPara.va_distth      = 32;
    mdbc_parm->UpdPara.t_distth       = 24;
    mdbc_parm->UpdPara.dbg_frmID      = 0x0;
    mdbc_parm->UpdPara.dbg_frmID_en   = 0x0;
    mdbc_parm->UpdPara.dbg_rnd        = 0x0;
    mdbc_parm->UpdPara.dbg_rnd_en     = 0x0;
}

//Set MD medium sensitivity
void md_set_para_medium_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
    mdbc_parm->MdmatchPara.lbsp_th    = 0x0;
    mdbc_parm->MdmatchPara.d_colour   = 15;
    mdbc_parm->MdmatchPara.r_colour   = 0x1e;
    mdbc_parm->MdmatchPara.d_lbsp     = 4;
    mdbc_parm->MdmatchPara.r_lbsp     = 8;
    mdbc_parm->MdmatchPara.model_num  = 0x8;
    mdbc_parm->MdmatchPara.t_alpha    = 25;
    mdbc_parm->MdmatchPara.dw_shift   = 0x4;
    mdbc_parm->MdmatchPara.dlast_alpha= 100;
    mdbc_parm->MdmatchPara.min_match  = 2;
    mdbc_parm->MdmatchPara.dlt_alpha  = 0xa;
    mdbc_parm->MdmatchPara.dst_alpha  = 0x28;
    mdbc_parm->MdmatchPara.uv_thres   = 20;
    mdbc_parm->MdmatchPara.s_alpha    = 100;
    mdbc_parm->MdmatchPara.dbg_lumDiff= 0x0;
    mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

    mdbc_parm->MorPara.th_ero     = 0x8;
    mdbc_parm->MorPara.th_dil     = 0x0;
    mdbc_parm->MorPara.mor_sel0   = 0x0;
    mdbc_parm->MorPara.mor_sel1   = 0x1;
    mdbc_parm->MorPara.mor_sel2   = 0x2;
    mdbc_parm->MorPara.mor_sel3   = 0x3;

    mdbc_parm->UpdPara.minT           = 0x4;
    mdbc_parm->UpdPara.maxT           = 0x40;
    mdbc_parm->UpdPara.maxFgFrm       = 0x80;
    mdbc_parm->UpdPara.deghost_dth    = 50;
    mdbc_parm->UpdPara.deghost_sth    = 205;
    mdbc_parm->UpdPara.stable_frm     = 0x78;
    mdbc_parm->UpdPara.update_dyn     = 0x80;
    mdbc_parm->UpdPara.va_distth      = 32;
    mdbc_parm->UpdPara.t_distth       = 24;
    mdbc_parm->UpdPara.dbg_frmID      = 0x0;
    mdbc_parm->UpdPara.dbg_frmID_en   = 0x0;
    mdbc_parm->UpdPara.dbg_rnd        = 0x0;
    mdbc_parm->UpdPara.dbg_rnd_en     = 0x0;
}

//Set MD low sensitivity
void md_set_para_low_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
    mdbc_parm->MdmatchPara.lbsp_th    = 0x0;
    mdbc_parm->MdmatchPara.d_colour   = 15;
    mdbc_parm->MdmatchPara.r_colour   = 0x1e;
    mdbc_parm->MdmatchPara.d_lbsp     = 5;
    mdbc_parm->MdmatchPara.r_lbsp     = 10;
    mdbc_parm->MdmatchPara.model_num  = 0x8;
    mdbc_parm->MdmatchPara.t_alpha    = 25;
    mdbc_parm->MdmatchPara.dw_shift   = 0x4;
    mdbc_parm->MdmatchPara.dlast_alpha= 100;
    mdbc_parm->MdmatchPara.min_match  = 1;
    mdbc_parm->MdmatchPara.dlt_alpha  = 0xa;
    mdbc_parm->MdmatchPara.dst_alpha  = 0x28;
    mdbc_parm->MdmatchPara.uv_thres   = 20;
    mdbc_parm->MdmatchPara.s_alpha    = 100;
    mdbc_parm->MdmatchPara.dbg_lumDiff= 0x0;
    mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

    mdbc_parm->MorPara.th_ero     = 0x8;
    mdbc_parm->MorPara.th_dil     = 0x0;
    mdbc_parm->MorPara.mor_sel0   = 0x0;
    mdbc_parm->MorPara.mor_sel1   = 0x1;
    mdbc_parm->MorPara.mor_sel2   = 0x2;
    mdbc_parm->MorPara.mor_sel3   = 0x3;

    mdbc_parm->UpdPara.minT           = 0x4;
    mdbc_parm->UpdPara.maxT           = 0x40;
    mdbc_parm->UpdPara.maxFgFrm       = 0x80;
    mdbc_parm->UpdPara.deghost_dth    = 50;
    mdbc_parm->UpdPara.deghost_sth    = 205;
    mdbc_parm->UpdPara.stable_frm     = 0x78;
    mdbc_parm->UpdPara.update_dyn     = 0x80;
    mdbc_parm->UpdPara.va_distth      = 32;
    mdbc_parm->UpdPara.t_distth       = 24;
    mdbc_parm->UpdPara.dbg_frmID      = 0x0;
    mdbc_parm->UpdPara.dbg_frmID_en   = 0x0;
    mdbc_parm->UpdPara.dbg_rnd        = 0x0;
    mdbc_parm->UpdPara.dbg_rnd_en     = 0x0;
}

//Inter-frame parameter adjustment
HD_RESULT md_set_para(ADAS_MEM_RANGE *p_share_mem, UINT32 ping_pong_id, UINT32 mode, UINT32 sensi)
{
    VENDOR_MD_PARAM            mdbc_parm;
    HD_RESULT ret = HD_OK;

    mdbc_parm.mode = mode;
    mdbc_parm.controlEn.update_nei_en = 1;
    mdbc_parm.controlEn.deghost_en    = 1;
    mdbc_parm.controlEn.roi_en0       = 0;
    mdbc_parm.controlEn.roi_en1       = 0;
    mdbc_parm.controlEn.roi_en2       = 0;
    mdbc_parm.controlEn.roi_en3       = 0;
    mdbc_parm.controlEn.roi_en4       = 0;
    mdbc_parm.controlEn.roi_en5       = 0;
    mdbc_parm.controlEn.roi_en6       = 0;
    mdbc_parm.controlEn.roi_en7       = 0;
    mdbc_parm.controlEn.chksum_en     = 0;
    mdbc_parm.controlEn.bgmw_save_bw_en = 0;

    if(ping_pong_id == 0) {
        mdbc_parm.InInfo.uiInAddr0 = p_share_mem[0].pa;
        mdbc_parm.InInfo.uiInAddr1 = p_share_mem[1].pa;
        mdbc_parm.InInfo.uiInAddr2 = p_share_mem[2].pa;
        mdbc_parm.InInfo.uiInAddr3 = p_share_mem[3].pa;
        mdbc_parm.InInfo.uiInAddr4 = p_share_mem[4].pa;
        mdbc_parm.InInfo.uiInAddr5 = p_share_mem[5].pa;
        mdbc_parm.OutInfo.uiOutAddr0 = p_share_mem[6].pa;
        mdbc_parm.OutInfo.uiOutAddr1 = p_share_mem[7].pa;
        mdbc_parm.OutInfo.uiOutAddr2 = p_share_mem[8].pa;
        mdbc_parm.OutInfo.uiOutAddr3 = p_share_mem[9].pa;
    } else {
        mdbc_parm.InInfo.uiInAddr0 = p_share_mem[0].pa;
        mdbc_parm.InInfo.uiInAddr1 = p_share_mem[1].pa;
        mdbc_parm.InInfo.uiInAddr2 = p_share_mem[2].pa;
        mdbc_parm.InInfo.uiInAddr3 = p_share_mem[7].pa;
        mdbc_parm.InInfo.uiInAddr4 = p_share_mem[8].pa;
        mdbc_parm.InInfo.uiInAddr5 = p_share_mem[9].pa;
        mdbc_parm.OutInfo.uiOutAddr0 = p_share_mem[6].pa;
        mdbc_parm.OutInfo.uiOutAddr1 = p_share_mem[3].pa;
        mdbc_parm.OutInfo.uiOutAddr2 = p_share_mem[4].pa;
        mdbc_parm.OutInfo.uiOutAddr3 = p_share_mem[5].pa;
    }

    mdbc_parm.uiLLAddr          = 0x0;
    mdbc_parm.InInfo.uiLofs0    = MDBC_ALIGN(MD_IMG_WIDTH,4);//160;
    mdbc_parm.InInfo.uiLofs1    = MDBC_ALIGN(MD_IMG_WIDTH,4);//160;
    mdbc_parm.Size.uiMdbcWidth  = MD_IMG_WIDTH;
    mdbc_parm.Size.uiMdbcHeight = MD_IMG_HEIGHT;

    switch (sensi) {
    case LOW_SENSI:
        md_set_para_low_sensitivity(&mdbc_parm);
        break;
    case MED_SENSI:
        md_set_para_medium_sensitivity(&mdbc_parm);
        break;
    case HIGH_SENSI:
        md_set_para_high_sensitivity(&mdbc_parm);
        break;
    case DEFAULT_SENSI:
    default:
        md_set_para_default_sensitivity(&mdbc_parm);
        break;
    }

    ret = vendor_md_set(VENDOR_MD_PARAM_ALL, &mdbc_parm);
    if (HD_OK != ret) {
        DBG_ERR("set img fail, error code = %d\r\n", ret);
    }
    return ret;
}

HD_RESULT ADAS_vendor_ai_net_start(UINT32 proc_id)
{
    DBG_DUMP("\033[34m [ZMD:%s:%d]:[%d]\033[0m\r\n",__func__,__LINE__,proc_id);
    return vendor_ai_net_start(proc_id);
}
HD_RESULT ADAS_vendor_ai_net_stop(UINT32 proc_id)
{
    DBG_DUMP("\033[34m [ZMD:%s:%d]:[%d]\033[0m\r\n",__func__,__LINE__,proc_id);
    return vendor_ai_net_stop(proc_id);
}
CHAR *ADAS_lib_md_get_version(VOID)
{
    return lib_md_get_version();
}

//yuv img crop
static void adas_yuv_crop(ADAS_MEM_RANGE *p_share_mem, HD_VIDEO_FRAME *p_video_frame_src,int location)
{
    char *src_img = (char *)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, p_video_frame_src->phy_addr[0], ADAS_IMG_BUF_SIZE*3/2);
    // char *dst_img = (char *)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, p_video_frame_dst->phy_addr[0], 320*320*3/2);
    char *dst_img = (char *)p_share_mem->va;

    //crop Y part
    my_crop((void *)dst_img, (void *)src_img, ADAS_IMG_WIDTH, \
        PERSON_CROP_X_BIAS+location*PERSON_CROP_X_BIAS_UP, PERSON_CROP_Y_BIAS, PERSON_CROP_WIDTH, PERSON_CROP_HEIGHT);

    //crop UV part
    my_crop((void *)(dst_img+PERSON_CROP_BUF_SIZE), (void *)(src_img+ADAS_IMG_BUF_SIZE), ADAS_IMG_WIDTH, \
        PERSON_CROP_X_BIAS+location*PERSON_CROP_X_BIAS_UP, PERSON_CROP_Y_BIAS/2, PERSON_CROP_WIDTH, PERSON_CROP_HEIGHT/2);

    hd_common_mem_munmap((void *)src_img, ADAS_IMG_BUF_SIZE*3/2);
    // hd_common_mem_munmap((void *)dst_img, 320*320*3/2);
}

//Pedestrian detection algorithm entrance
void PCW_Process(UINT32 Id, HD_VIDEO_FRAME *p_video_frame,ADAS_VIDEO_INFO *p_stream,BOOL *adas_xingren_alarm)
{
    static char location=0;
    #if PEOPLE_DETECT
    static MYRECT scale_location= {0};
    // static UINT32 person_flag = 0;
    static UINT32 xingren_alarm_frame_num_l = 0;
    static UINT32 xingren_alarm_frame_num_r = 0;
    // static float people_threshold = 0;
    // static float people_threshold1 = 0;
    static BOOL people_reliable = FALSE;
    static UINT64 pcw_pre_time_ms = 0;
    static UINT64 pcw_cur_time_ms = 0;
    // static BOOL xingren_alarm_limit = FALSE;
    #endif

    static HD_VIDEO_FRAME video_frame_new = {0};
    //video_frame_new.p_next      = NULL;
    //video_frame_new.ddr_id      = 0;
    video_frame_new.pxlfmt      = HD_VIDEO_PXLFMT_YUV420;
    video_frame_new.dim.w       = PERSON_CROP_WIDTH;
    video_frame_new.dim.h       = PERSON_CROP_HEIGHT;
    //video_frame_new.count       = 0;
    //video_frame_new.timestamp   = hd_gettime_us();
    video_frame_new.loff[0]     = PERSON_CROP_WIDTH; // Y
    video_frame_new.loff[1]     = PERSON_CROP_WIDTH; // UV
    video_frame_new.phy_addr[0] = ADAS_share_mem[21].pa;                          // Y
    video_frame_new.phy_addr[1] = ADAS_share_mem[21].pa + PERSON_CROP_BUF_SIZE;  // UV pack

    HD_RESULT ret = HD_OK;
    VENDOR_AI_BUF   in_buf = {0};
    VENDOR_AI_POSTPROC_RESULT_INFO out_buf = {0};
    {
        #if 0 //zmd
        ret = hd_videoproc_pull_out_buf(p_stream->proc_path, &video_frame, -1); // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
        if(ret != HD_OK) {
            printf("hd_videoproc_pull_out_buf fail (%d)\n\r", ret);
            goto skip;
        }
        #endif

        #if 1 //zmd
        location = (location+1)%2;//Control the left and right cropping pictures
        adas_yuv_crop(&ADAS_share_mem[21], p_video_frame, location);

        //prepare input AI_BUF from videoframe
        in_buf.sign = MAKEFOURCC('A','B','U','F');
        in_buf.width = video_frame_new.dim.w;
        in_buf.height = video_frame_new.dim.h;
        in_buf.channel  = HD_VIDEO_PXLFMT_PLANE(video_frame_new.pxlfmt); //conver pxlfmt to channel count
        in_buf.line_ofs = video_frame_new.loff[0];
        in_buf.fmt = video_frame_new.pxlfmt;
        in_buf.pa = video_frame_new.phy_addr[0];
        in_buf.va = 0;
        in_buf.size = video_frame_new.loff[0]*video_frame_new.dim.h*3/2;
        #else
        // //prepare input AI_BUF from videoframe
        in_buf.sign = MAKEFOURCC('A','B','U','F');
        in_buf.width = p_video_frame->dim.w;
        in_buf.height = p_video_frame->dim.h;
        in_buf.channel = HD_VIDEO_PXLFMT_PLANE(p_video_frame->pxlfmt); //conver pxlfmt to channel count
        in_buf.line_ofs = p_video_frame->loff[0];
        in_buf.fmt = p_video_frame->pxlfmt;
        in_buf.pa = p_video_frame->phy_addr[0];
        in_buf.va = 0;
        in_buf.size = p_video_frame->loff[0]*p_video_frame->dim.h*3/2;
        #endif
        if (ai_async == 0) {
            // set input image
            ret = vendor_ai_net_set(p_stream->net_path, VENDOR_AI_NET_PARAM_IN(0, 0), &in_buf);
            if (HD_OK != ret) {
                DBG_ERR("proc_id(%u) set input fail !!\n", p_stream->net_path);
                goto skip;
            }

            // do net proc
            ret = vendor_ai_net_proc(p_stream->net_path);
            if (HD_OK != ret) {
                DBG_ERR("proc_id(%u) proc fail !!\n", p_stream->net_path);
                goto skip;
            }

            // get output result
            ret = vendor_ai_net_get(p_stream->net_path, VENDOR_AI_NET_PARAM_OUT(VENDOR_AI_MAXLAYER, 0), &out_buf);
            if (HD_OK != ret) {
                DBG_ERR("proc_id(%u) get output fail !!\n", p_stream->net_path);
                goto skip;
            }

        } else if (ai_async == 1) {
            // do net proc_buf
            ret = vendor_ai_net_proc_buf(p_stream->net_path, VENDOR_AI_NET_PARAM_IN(0, 0), &in_buf, VENDOR_AI_NET_PARAM_OUT(VENDOR_AI_MAXLAYER, 0), &out_buf);
            if (HD_OK != ret) {
                DBG_ERR("proc_id(%u) proc_buf fail !!\n", p_stream->net_path);
                goto skip;
            }
        }

        // ret = network_dump_out_buf(p_stream->net_path, &out_buf);
        // if (HD_OK != ret) {
        //     printf("proc_id(%u) output dump fail !!\n", p_stream->net_path);
        //     goto skip;
        // }

        #if 0 //zmd
        ret = hd_videoproc_release_out_buf(p_stream->proc_path, &video_frame);
        if(ret != HD_OK) {
            printf("hd_videoproc_release_out_buf fail (%d)\n\r", ret);
            goto skip;
        }
        #endif


        #if PEOPLE_DETECT
   
        // people_threshold = 690;
        // people_threshold1 = 900;

        UINT32 people_num = 0, i;

        //Control the alarm area
        float k1 = -0.7;
        float k2 = 0.7;
        float b1 = 1200;
        float b2 = -98;

        //Analyze the results of the pedestrian network
        for (i = 0; i < out_buf.result_num; i++) {
            VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[i];
            if(p_rslt->no[0]==1 && p_rslt->score[0]>0.8) {
                //Coordinate mapping
                scale_location.x = (UINT32)((p_rslt->x*PERSON_CROP_WIDTH+location*PERSON_CROP_X_BIAS_UP+PERSON_CROP_X_BIAS)*1920/ADAS_IMG_WIDTH);
                scale_location.y = (UINT32)((p_rslt->y*PERSON_CROP_HEIGHT+PERSON_CROP_Y_BIAS)*1080/ADAS_IMG_HEIGHT);
                scale_location.w = (UINT32)((p_rslt->w*PERSON_CROP_WIDTH)*1920/ADAS_IMG_WIDTH);
                scale_location.h = (UINT32)((p_rslt->h*PERSON_CROP_HEIGHT)*1080/ADAS_IMG_HEIGHT);

                #if 0
                personrect[i].x = scale_location.x * 960/1920;
                personrect[i].y = scale_location.y * 240/1080;
                personrect[i].w = scale_location.w * 960/1920;
                personrect[i].h = scale_location.h * 240/1080;
                personrect_num++;
                #endif

                float w = scale_location.w + 0.01;
                float h = scale_location.h;
                if(w>4 && h>0 && (h/w)>1.4 && (h/w)<5 && scale_location.y<900) {
                    float px = scale_location.x + scale_location.w/2;
                    float py = scale_location.y + scale_location.h;
                    people_reliable = FALSE;

                    //Judging the accuracy of pedestrians based on location
                    if(py>650 && py<920 && h < (1.8177*py-1019)){
                        people_reliable = TRUE;
                    }
                    // if(py>=750 && py<800 && scale_location.h<=434){
                    //     people_reliable = TRUE;
                    // }else if(py>=800 && py<850 && scale_location.h<=527){
                    //     people_reliable = TRUE;                       
                    // }else if(py>=700 && py<750 && scale_location.h<=342){
                    //     people_reliable = TRUE;                       
                    // }else if(py>=850 && py<900 && scale_location.h<=620){
                    //     people_reliable = TRUE;                       
                    // }else if(py>=650 && py<700 && scale_location.h<=251){
                    //     people_reliable = TRUE;                       
                    // }
                    if((people_reliable==TRUE) && (k1*px+b1)<py && (k2*px+b2)<py) {
                        people_num++;
                    }
                }
            }
        }

        //Ensure that pedestrians appear in the warning area for two consecutive frames
        switch(location) {
        case 0:
            if(people_num>0) {
                xingren_alarm_frame_num_l++;
            } else {
                xingren_alarm_frame_num_l = 0;
            }
            break;
        case 1:
            if(people_num>0) {
                xingren_alarm_frame_num_r++;
            } else {
                xingren_alarm_frame_num_r = 0;
            }
            break;
        }

        //alarm
        if((xingren_alarm_frame_num_l >= 2) || (xingren_alarm_frame_num_r >= 2)) {
            xingren_alarm_frame_num_l = 0;
            xingren_alarm_frame_num_r = 0;
            pcw_cur_time_ms = hd_gettime_ms();

            //param 5000 control alarm interval time
            if((pcw_cur_time_ms - pcw_pre_time_ms) > 5000) {
                *adas_xingren_alarm = TRUE;
                // person_flag = 80;
                pcw_pre_time_ms = pcw_cur_time_ms;
            }
        }

        #endif  //#if PEOPLE_DETECT
    }
skip:
    return;
}

//Pre-stationary state calculation
static float calculate_mean(int *p_queue, int num){
    int i, count = 0;
    // int del[20];
    float sum = 0;
    float average = 0;

    for(i = 0; i < num; i++){
        if(0 != p_queue[i]){
            sum += p_queue[i];
            count++;
        }
    }

    if(count>0){
        average = sum / count;
    }

    return average;
}

//Pre-stationary state calculation
static float calculate_variance(int *p_queue, int num){
    int i, count = 0, del_num = 0;
    // int del[20];
    float sum = 0;
    float result;
    for(i = 0; i< num; i++){
        if(0 == p_queue[i]){
            count++;
            // del[del_num] = i;
            del_num++;
        }else{
            count = 0;
        }

        if(count >= 3){
            result = 10000;
            goto skip;
        }
    }

    if(del_num >= 4){
        result = 10000;
        goto skip;
    }

    for(i = 0; i < num; i++){
        if(0 != p_queue[i]){
            sum += p_queue[i];
        }
    }

    float average = sum / (num - del_num);

    sum = 0;
    for(i = 0; i < num; i++){
        if(0 != p_queue[i]){
            sum += (p_queue[i] - average)*(p_queue[i] - average);
        }       
    }

    result = sum / (num - del_num);


skip:
    return result;
}

static HD_RESULT fcw_yuv_crop_scale(HD_VIDEO_FRAME *p_video_frame_dst,HD_VIDEO_FRAME *p_video_frame_src)
{
    HD_RESULT ret=0;
    //copy vout0 to common buffer
    HD_GFX_SCALE param_yuv_copy;
    memset(&param_yuv_copy, 0, sizeof(HD_GFX_SCALE));
    param_yuv_copy.src_img.dim.w            = p_video_frame_src->dim.w;
    param_yuv_copy.src_img.dim.h            = p_video_frame_src->dim.h;;
    param_yuv_copy.src_img.format           = p_video_frame_src->pxlfmt;
    param_yuv_copy.src_img.p_phy_addr[0]    = p_video_frame_src->phy_addr[0];//src_pa;
    param_yuv_copy.src_img.p_phy_addr[1]    = p_video_frame_src->phy_addr[1];//src_pa;
    param_yuv_copy.src_img.lineoffset[0]    = p_video_frame_src->loff[0];//src_w 
    param_yuv_copy.src_img.lineoffset[1]    = p_video_frame_src->loff[1];//src_w
    param_yuv_copy.dst_img.dim.w            = p_video_frame_dst->dim.w;//dst_w
    param_yuv_copy.dst_img.dim.h            = p_video_frame_dst->dim.h;//dst_w
    param_yuv_copy.dst_img.format           = p_video_frame_dst->pxlfmt;
    param_yuv_copy.dst_img.p_phy_addr[0]    = p_video_frame_dst->phy_addr[0];//dst_pa;
    param_yuv_copy.dst_img.p_phy_addr[1]    = p_video_frame_dst->phy_addr[1];//dst_pa + 1920 * 1080;
    param_yuv_copy.dst_img.lineoffset[0]    = p_video_frame_dst->loff[0];
    param_yuv_copy.dst_img.lineoffset[1]    = p_video_frame_dst->loff[1];
    param_yuv_copy.src_region.x             = CAR_CROP_X_BIAS;
    param_yuv_copy.src_region.y             = CAR_CROP_Y_BIAS;
    param_yuv_copy.src_region.w             = CAR_CROP_WIDTH;
    param_yuv_copy.src_region.h             = CAR_CROP_HEIGHT;
    param_yuv_copy.dst_region.x             = 0;
    param_yuv_copy.dst_region.y             = 0;
    param_yuv_copy.dst_region.w             = p_video_frame_dst->dim.w;
    param_yuv_copy.dst_region.h             = p_video_frame_dst->dim.h;
    // param_yuv_copy.quality                  = HD_GFX_SCALE_QUALITY_BILINEAR;

    ret = hd_gfx_scale_sw(&param_yuv_copy);
    if(ret != HD_OK){
        printf("hd_gfx_scale fail=%d\n", ret);
        goto exit;
    }

exit:
    return ret;
}

//FCW and SNG algorithm entry
void FCW_Process(UINT32 Id, HD_VIDEO_FRAME *p_video_frame,ADAS_VIDEO_INFO *p_stream,BOOL  *adas_pengzhuang_alarm,BOOL  *adas_qidong_alarm)
{
    #if FCWS
    static BOOL FCW_alarm_limit = FALSE;    //Broadcast restrictions to avoid repeated broadcasts
    static UINT64 fcw_pre_time_ms = 0;
    static UINT64 fcw_cur_time_ms = 0;
    static int fcw_out_alarm_num = 0;   //Count out of alarm zone

    static BOOL car_jingzhi = TRUE;
    static BOOL jingzhi_compute = FALSE;
    static UINT32 jingzhi_pingpong = 0;
    static UINT64 pre_time = 0;
    static BOOL bound_flag = FALSE;
    #endif

    #if SNG
    static HD_IRECT car_crop = {0};

    //MD param
    static UINT32 ping_pong_id = 0,is_Init=0;
    static UINT32 reg_id=0;
    static VENDOR_MD_TRIGGER_PARAM md_trig_param;
    static LIB_MD_MDT_LIB_INFO mdt_lib_param;
    static LIB_MD_MDT_RESULT_INFO lib_md_rst;

    static int wait_frame_num = 0;
    static UINT32 md_status = MD_CLOSE;     //represent MD status
    static UINT32 percentage = 20;          //control SNG sensitivity, the smaller the more sensitive

    static UINT64 sng_pre_time_ms = 0;
    static UINT64 sng_cur_time_ms = 0;  //Record the alarm interval time

    static UINT32 g_sensi = LOW_SENSI;  //Set MD sensitivity
    static BOOL md_init_flag = FALSE;
    static UINT32 sng_alarm_num = 0;
    #endif

    #if FCWS || SNG
    static UINT32 i;

    //Record the state of the previous vehicle
    static int queue[MEMORY_LENGTH];
    static int width_queue[MEMORY_LENGTH];
    static int X_queue[MEMORY_LENGTH]; 

    static float car_width_mean = 0;
    static float car_X_mean = 0;
    static int op = 0;
    static MYRECT record_rect = {0};
    static int pre_static_num = 0;
    static int crop_num = 0;
    static int static_num = 0;
    static int all_num = 0;
    static BOOL record_flag = FALSE;
    static UINT32 car_status = MOVE;
    static UINT32 bufsize = 0;
    static float thresold1 = 1;
    static float thresold2 = 200;   //Two thresholds for the stationary state

    #endif

    #if ADAS_OUTPUT_BMP
    static char ImgFilePath[64];
    static int frmidx = 0;
    #endif

    static float k1 = -0.8, k2 = 0.8, b1 = 476, b2 = -36;

    // static char car_img_path[64];
    // static int index = 0;
    static int car_location = 0;
    HD_RESULT ret = HD_OK;
    VENDOR_AI_BUF   in_buf = {0};
    static VENDOR_AI_POSTPROC_RESULT_INFO out_buf = {0};
    static HD_VIDEO_FRAME video_frame_new = {0};
    //VENDOR_AI_BUF out_buf = {0};


    
    {
        #if 0 //zmd
        ret = hd_videoproc_pull_out_buf(p_stream->proc_path, &video_frame, -1); // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
        if(ret != HD_OK) {
            DBG_ERR("hd_videoproc_pull_out_buf fail (%d)\n\r", ret);
            goto skip;
        }
        #endif

        //Two frames take one frame recognition
        car_location = (car_location+1)%2;
        if(0 == car_location){

            
            // video_frame_new.p_next      = NULL;
            // video_frame_new.ddr_id      = 0;
            video_frame_new.pxlfmt      = HD_VIDEO_PXLFMT_YUV420;
            video_frame_new.dim.w       = ADAS_IMG_WIDTH;
            video_frame_new.dim.h       = ADAS_IMG_HEIGHT;
            // video_frame_new.count       = 0;
            // video_frame_new.timestamp   = hd_gettime_us();
            video_frame_new.loff[0]     = ADAS_IMG_WIDTH; // Y
            video_frame_new.loff[1]     = ADAS_IMG_WIDTH; // UV
            video_frame_new.phy_addr[0] = ADAS_share_mem[20].pa;                          // Y
            video_frame_new.phy_addr[1] = ADAS_share_mem[20].pa + ADAS_IMG_BUF_SIZE;  // UV pack


            // UINT64 begin_time = hd_gettime_us();
            memset(&out_buf, 0, sizeof(out_buf));
            fcw_yuv_crop_scale(&video_frame_new, p_video_frame);

            // snprintf(car_img_path, 64, "//mnt//sd//img//car_crop_%05d.bmp", index++);
            // writebmpfile(car_img_path, (void *)ADAS_share_mem[20].va, ADAS_IMG_WIDTH, ADAS_IMG_HEIGHT);

            // UINT64 end_time = hd_gettime_us();
            // printf("scale time = %lld\n", end_time - begin_time);


            //prepare input AI_BUF from videoframe
            in_buf.sign = MAKEFOURCC('A','B','U','F');
            in_buf.width = video_frame_new.dim.w;
            in_buf.height = video_frame_new.dim.h;
            in_buf.channel  = HD_VIDEO_PXLFMT_PLANE(video_frame_new.pxlfmt); //conver pxlfmt to channel count
            in_buf.line_ofs = video_frame_new.loff[0];
            in_buf.fmt = video_frame_new.pxlfmt;
            in_buf.pa = video_frame_new.phy_addr[0];
            in_buf.va = 0;
            in_buf.size = video_frame_new.loff[0]*video_frame_new.dim.h*3/2; 

            if (ai_async == 0) {
                // set input image
                ret = vendor_ai_net_set(p_stream->net_path, VENDOR_AI_NET_PARAM_IN(0, 0), &in_buf);
                if (HD_OK != ret) {
                    DBG_ERR("proc_id(%u) set input fail !!\n", p_stream->net_path);
                    return;
                }
                // do net proc
                ret = vendor_ai_net_proc(p_stream->net_path);
                if (HD_OK != ret) {
                    DBG_ERR("proc_id(%u) proc fail !!\n", p_stream->net_path);
                    return;
                }
                // get output result
                ret = vendor_ai_net_get(p_stream->net_path, VENDOR_AI_NET_PARAM_OUT(VENDOR_AI_MAXLAYER, 0), &out_buf);
                if (HD_OK != ret) {
                    DBG_ERR("proc_id(%u) get output fail !!\n", p_stream->net_path);
                    return;
                }
            } else if (ai_async == 1) {
                // do net proc_buf
                ret = vendor_ai_net_proc_buf(p_stream->net_path, VENDOR_AI_NET_PARAM_IN(0, 0), &in_buf, VENDOR_AI_NET_PARAM_OUT(VENDOR_AI_MAXLAYER, 0), &out_buf);
                if (HD_OK != ret) {
                    DBG_ERR("proc_id(%u) proc_buf fail !!\n", p_stream->net_path);
                    return;
                }
            }
        }

        // ret = network_dump_out_buf(p_stream->net_path, &out_buf);
        // if (HD_OK != ret) {
        //     printf("proc_id(%u) output dump fail !!\n", p_stream->net_path);
        //     goto skip;
        // }

        float cur_speed;
        BOOL gps_status = GPSMNG_GetSpeed(&cur_speed);
        // if(TRUE == gps_status){
        //     printf("cur_speed = %f\n", cur_speed);
        // }

        ret = img1920toimg320(p_video_frame, ADAS_share_mem);
        if(HD_OK != ret) {
            DBG_ERR("img scale fail !!\n");
            return;
        }


        #if FCWS
        //Determine whether the car is stationary, calculated every 1 second
        UINT64 cur_time = hd_gettime_ms();
        // printf("cur_time = %lld\n", cur_time);
        // printf("pre_time = %lld\n", pre_time);
        // printf("cycle_time = %lld\n", cur_time - pre_time);
        if((cur_time - pre_time) > 1000){
            jingzhi_compute = TRUE; 
        }

        if(jingzhi_compute){
            switch(jingzhi_pingpong){
                case 0:
                    memcpy((void *)ADAS_share_mem[2].va, (void *)ADAS_share_mem[0].va, MD_IMG_WIDTH*MD_CROP_HEIGHT);
                    jingzhi_pingpong++;
                    break;
                case 1:
                case 2:
                    jingzhi_pingpong++;
                    break;
                case 3:
                    {
                        UINT32 re, change_num3 = 0;
                        int kk1, kk2, m, n;
                        UINT8* sss = (UINT8*)ADAS_share_mem[2].va;
                        UINT8* ddd = (UINT8*)ADAS_share_mem[0].va;
                        
                        for(m = 0; m < MD_CROP_WIDTH; m++){
                            for(n = 0; n < MD_CROP_HEIGHT; n++){
                                kk1 = n*MD_IMG_WIDTH + m;
                                kk2 = n*MD_IMG_WIDTH + m + MD_IMG_WIDTH - MD_CROP_WIDTH;
                                re = abs((INT32)sss[kk1] - (INT32)ddd[kk1]);
                                if(re > (UINT32)40){
                                    change_num3++;
                                }
                                re = abs((INT32)sss[kk2] - (INT32)ddd[kk2]);
                                if(re > (UINT32)40){
                                    change_num3++;
                                }
                            }
                        }
                        //param 300 is stationary threshold
                        if(change_num3 > 300){
                            car_jingzhi = FALSE;
                            // printf("car_move car_move car_move car_move car_move car_move car_move car_move\n");
                        }else{
                            if(gps_status == TRUE && cur_speed > 15){
                                car_jingzhi = FALSE;
                                // printf("car_move car_move car_move car_move car_move car_move car_move car_move\n");
                            }else{
                                car_jingzhi = TRUE;
                            }
                            // printf("car_jingzhi car_jingzhi car_jingzhi car_jingzhi car_jingzhi car_jingzhi\n");
                        }
                        jingzhi_compute = FALSE;
                        jingzhi_pingpong = 0;
                        pre_time = hd_gettime_ms();
                    }   
                    break;
                default:
                    break;
            }
        }
        #endif


        #if 0 //zmd
        ret = hd_videoproc_release_out_buf(p_stream->proc_path, &video_frame);
        if(ret != HD_OK) {
            printf("hd_videoproc_release_out_buf fail (%d)\n\r", ret);
            return;
        }
        #endif

        ////--------Calculate the state of the preceding vehicle----------------------
#if LINE_DETECT
    if(bound_flag == FALSE && left_index >= 20 && right_index >= 20){
        bound_compute(left_lane_log, left_index, &left_line);
        bound_compute(right_lane_log, right_index, &right_line);
        // printf("left_line.k = %f, left_line.b = %f\r\n", left_line.k, left_line.b);
        // printf("right_line.k = %f, right_line.b = %f\r\n", right_line.k, right_line.b);
        bound_set(&left_line, &right_line, &k1, &b1, &k2, &b2);
        // printf("k1 = %f, b1 = %f, k2 = %f, b2 = %f\r\n", k1, b1, k2, b2);

        set_fcw_sensitivity(FCW_MIDDLE);
        bound_flag = TRUE;
    }
#endif
        // printf("-------------horizon_y = %d--------------\r\n", horizon_y);
        // printf("k1 = %f, b1 = %f, k2 = %f, b2 = %f\r\n", k1, b1, k2, b2);
        UINT32 car_index = get_pre_car_index(&out_buf, k1, b1, k2, b2);
        UINT32 front_car_pos = 0;
        UINT32 car_width = 0;
        UINT32 car_X = 0;

        // printf("car_index = %d\n", car_index);

        if(car_index < 50){
            VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[car_index];
            front_car_pos = f2iround(((p_rslt->y + p_rslt->h)*180+120)*180/360); 
            car_width = f2iround(p_rslt->w * 180);
            car_X = f2iround((p_rslt->x+p_rslt->w/2) * 180);
        }

        if(car_width > 50){
            width_queue[op] = car_width;
        }else{
            width_queue[op] = 0;
        }
        // printf("width_queue[%d] = %d\n", op, width_queue[op]);

        if(car_X > 0){
            X_queue[op] = car_X;
        }else{
            X_queue[op] = 0;
        }
        // printf("X_queue[%d] = %d\n", op, X_queue[op]);        


        // printf("front_car_pos = %d\n", front_car_pos);
        if(front_car_pos > 125){
            queue[op] = front_car_pos;
        }else{
            queue[op] = 0;    
        }
        // printf("queue[%d] = %d\n", op, queue[op]);
        op = (op+1)%MEMORY_LENGTH;

        float variance = calculate_variance(queue, MEMORY_LENGTH);
        // printf("variance = %f\n", variance);

        car_width_mean = calculate_mean(width_queue, MEMORY_LENGTH);
        // printf("car_width_mean = %f\n", car_width_mean);

        car_X_mean = calculate_mean(X_queue, MEMORY_LENGTH);
        // printf("car_X_mean = %f\n", car_X_mean);
        ////----------------end---------------------------

        ////-----------Determine the static state of the vehicle in front-----------
        switch(car_status){
            case MOVE:
                if(variance <= thresold1){
                    pre_static_num++;
                }else{
                    pre_static_num = 0;
                }
                if(pre_static_num >= 5){
                    pre_static_num = 0;
                    car_status = PRE_STATIC;
                    record_flag = TRUE;
                }
                break;
            case PRE_STATIC:
                if(record_flag && car_index< 50){
                    VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[car_index];
                    float x = (p_rslt->x*320+160)*320/640;
                    float y = (p_rslt->y*180+120)*180/360;
                    float w = p_rslt->w*320*320/640;
                    float h = p_rslt->h*180*180/360;
                    record_rect.x = (UINT32)(x+0.1*w);
                    record_rect.y = (UINT32)(y+0.2*h);
                    record_rect.w = (UINT32)(0.8*w);
                    record_rect.h = (UINT32)(0.8*h);
                    // printf("x = %d, y = %d, w = %d, h = %d\n", record_rect.x, record_rect.y, record_rect.w, record_rect.h);

                    my_crop((void *)ADAS_share_mem[22].va, (void *)ADAS_share_mem[0].va, MD_IMG_WIDTH, record_rect.x, record_rect.y, record_rect.w, record_rect.h);
                    bufsize = record_rect.w*record_rect.h;
                    record_flag = FALSE;
                    crop_num = 1;
                    static_num = 0;
                    all_num = 0;
                }else{
                    //difference
                    if(crop_num == 0){
                        my_crop((void *)ADAS_share_mem[23].va, (void *)ADAS_share_mem[0].va, MD_IMG_WIDTH, record_rect.x, record_rect.y, record_rect.w, record_rect.h);

                        char *ooold = (char *)ADAS_share_mem[22].va;
                        char *nnnew = (char *)ADAS_share_mem[23].va; 
                        int change_num = 0;
                        for(i=0; i<bufsize; i++){
                            int re = fabs((INT32)(ooold[i]) - (INT32)(nnnew[i]));
                            if(re > 30){
                                change_num++;
                            }
                        }

                        memcpy(ooold, nnnew, bufsize);
                        // printf("change_num = %d\n", change_num);
                        //param 0.05 is stationary threshold
                        if(change_num <= bufsize*0.05){
                            static_num++;
                        }
                        all_num++;          
                    }
                    crop_num = (crop_num+1)%3;

                    if(all_num >= 5){
                        if(static_num >= 4){
                            car_status = TRUE_STATIC;
                            // printf("TRUE_STATIC TRUE_STATIC TRUE_STATIC TRUE_STATIC!\n");
                        }else{
                            car_status = MOVE;
                            pre_static_num = 0; 
                        }
                    }                
                }
    
                break;
            case TRUE_STATIC:
                if(variance > thresold2){
                    car_status = MOVE;
                    pre_static_num = 0;
                    // printf("MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE!\n");
                }
                break;
        }
        ////-----------------end------------------------

        #if FCWS
        ////-----Anti-collision alarm judgment of the preceding vehicle---------
        // printf("-------------fcw_alarm_dist = %d--------------\r\n", fcw_alarm_dist);
        if(variance > 2 && car_jingzhi == FALSE){

            int Lower_boundary = f2iround(front_car_pos*ADAS_IMG_WIDTH*1.0/MD_IMG_WIDTH);
            //printf("Lower_boundary = %d\n", Lower_boundary);

            //Alarm distance setting
            int alarm_distance = fcw_alarm_dist;    //Set as 15m
            if(TRUE == gps_status && cur_speed > 60){
                alarm_distance = (horizon_y >= 220 ? 239 : horizon_y+21);   //Set as 30m+
            }

            if(FCW_alarm_limit == FALSE && Lower_boundary >= alarm_distance) {   
                fcw_cur_time_ms = hd_gettime_ms();
                //param 5000 is alarm interval time
                if((fcw_cur_time_ms - fcw_pre_time_ms) > 5000 && (fcw_cur_time_ms - sng_pre_time_ms) > 5000) {
                    *adas_pengzhuang_alarm = TRUE; 
                    FCW_alarm_limit = TRUE;
                    fcw_pre_time_ms = fcw_cur_time_ms;   
                }                                    

            }

            //detects that it is out of the alarm distance 7 times
            //The alarm limit will be lifted
            //avoid repeated broadcasts
            if(Lower_boundary < alarm_distance) {
                fcw_out_alarm_num++;
                if(fcw_out_alarm_num >= 7){
                    FCW_alarm_limit = FALSE;
                }                
            }else{
                fcw_out_alarm_num = 0;
            }
        }
        ////----------------end-----------------------

        #endif

        #if SNG

        ////-------------The vehicle in front starts alarm judgment--------------
        switch(md_status) {
            case MD_CLOSE:
                if(car_status == TRUE_STATIC && md_init_flag == FALSE) {
                    md_status = WAIT_INIT;
                    wait_frame_num = 3;
                }
                break;
            case WAIT_INIT:
                if(wait_frame_num>0) {
                    wait_frame_num--;
                } else {
                    md_status = INIT;
                }
                break;
            case INIT:
                //Initialization parameters and detection area
                if(car_status == TRUE_STATIC && car_index < 50) {
                    VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[car_index];
                    car_crop.x = f2iround((p_rslt->x*CAR_CROP_WIDTH+CAR_CROP_X_BIAS)*MD_IMG_WIDTH/ADAS_IMG_WIDTH);
                    car_crop.y = f2iround((p_rslt->y*CAR_CROP_HEIGHT+CAR_CROP_Y_BIAS)*MD_IMG_HEIGHT/ADAS_IMG_HEIGHT);
                    car_crop.w = f2iround(p_rslt->w*CAR_CROP_WIDTH*MD_IMG_WIDTH/ADAS_IMG_WIDTH);
                    car_crop.h = f2iround(p_rslt->h*CAR_CROP_HEIGHT*MD_IMG_HEIGHT/ADAS_IMG_HEIGHT);

                    //set SNG sensitivity
                    if((car_crop.y+car_crop.h) >= 130){
                        percentage = 18;
                    }else{
                        percentage = 40;
                    }

                    md_mem_clean(MD_share_mem);

                    ret = vendor_md_init();
                    if (HD_OK != ret) {
                        DBG_ERR("md init fail, error code = %d\r\n", ret);
                        break;
                    }
                    // printf("md_init_success\r\n");
                    md_init_flag = TRUE;
                    my_libmd_param_set(&md_trig_param, &mdt_lib_param, &car_crop, MD_share_mem, percentage);
                    md_status = WAIT_TRIGGER;
                    wait_frame_num = 3;
                } else {
                    md_status = MD_CLOSE;
                    md_init_flag = FALSE;
                }
                break;
            case WAIT_TRIGGER:
                if(wait_frame_num>0) {
                    wait_frame_num--;
                } else {
                    md_status = TRIGGER;
                }
                break;
            case TRIGGER:
                //Start SNG detection
                if(car_status == TRUE_STATIC && TRUE == md_init_flag) {
                    // hd_gfx_memcpy(MD_share_mem[0].pa, ADAS_share_mem[0].pa, MD_IMG_BUF_SIZE);
                    // hd_gfx_memcpy(MD_share_mem[1].pa, ADAS_share_mem[1].pa, MD_IMG_BUF_SIZE/2);
                    MD_share_mem[0].pa = ADAS_share_mem[0].pa;
                    MD_share_mem[1].pa = ADAS_share_mem[1].pa;
                    md_set_para(MD_share_mem, ping_pong_id, is_Init, g_sensi);

                    ret = vendor_md_trigger(&md_trig_param);
                    if (HD_OK != ret) {
                        DBG_ERR("trigger fail, error code = %d\r\n", ret);
                        return;
                    }
                    // hd_gfx_memcpy(MD_share_mem[2].pa, ADAS_share_mem[1].pa, MD_IMG_BUF_SIZE/2);
                    memcpy((UINT32 *)MD_share_mem[2].va, (UINT32 *)ADAS_share_mem[1].va, MD_IMG_BUF_SIZE/2);

                    // printf("md_trigger success\r\n");

                    if(1 == is_Init) {
                        bc_reorgS1((UINT8*)MD_share_mem[6].va,(UINT8*)MD_share_mem[10].va, MD_IMG_WIDTH, MD_IMG_HEIGHT);

                        #if ADAS_OUTPUT_BMP
                        frmidx++;
                        printf("save save save save save save save save save save %d\r\n", frmidx);
                        snprintf(ImgFilePath, 64, "//mnt//sd//img//md_output_img//output_%04d.bmp", frmidx);
                        writebmpfile(ImgFilePath, (UINT8*)MD_share_mem[10].va, MD_IMG_WIDTH, MD_IMG_HEIGHT);
                        #endif

                        if ((ret = lib_md_get(0, LIB_MD_RESULT_INFO, &lib_md_rst)) != HD_OK) {
                            DBG_ERR("LIB_MD_RESULT_INFO fail\n");
                        }

                        // if(lib_md_rst.global_motion_alarm == 1) {
                        //     printf("[WRN] global motion alarm\n");
                        // }
                        for(reg_id =0; reg_id<mdt_lib_param.mdt_subregion_param.sub_region_num; reg_id++) {
                            if(lib_md_rst.sub_motion_alarm[reg_id] == 1) {
                                sng_alarm_num++;
                                if(sng_alarm_num>=2){
                                    // *adas_qidong_alarm = TRUE;
                                    md_status = UNINIT;
                                    sng_cur_time_ms = hd_gettime_ms();
                                    if((sng_cur_time_ms - sng_pre_time_ms) > 5000 && car_width < (car_width_mean+1.5) && fabs(car_X_mean-car_X) < 3){
                                        *adas_qidong_alarm = TRUE;
                                        // printf("dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong\r\n");
                                        sng_pre_time_ms = sng_cur_time_ms;
                                    }
                                    // alarm_uninit_flag = TRUE;
                                    // printf("[WRN] sub_region[%d] motion alarm\n", (int)reg_id);                                    
                                }
                            }else{
                                sng_alarm_num = 0;
                            }
                        }
                        // if (lib_md_rst.scene_change_alarm == 1) {
                        //     printf("[WRN] scene change alarm\n");
                        // }
                    }
                    ping_pong_id = (ping_pong_id+1)%2;
                    if(0 == is_Init) is_Init=1;
                } else {
                    md_status = UNINIT;
                }
                break;
            case UNINIT:
                if(TRUE == md_init_flag) {
                    // md_uninit();
                    ret = vendor_md_uninit();
                    if (HD_OK != ret) {
                        DBG_ERR("uninit fail, error code = %d\r\n", ret);
                    }
                    // printf("md uninit success\r\n");
                    md_init_flag = FALSE;
                    ping_pong_id = 0;
                    is_Init = 0;

                }
                md_status = MD_CLOSE;
                car_status = MOVE;
                break;
        }
        ////----------------end---------------------
        #endif

        //printf("\n");        

    }
}

//LDW algorithm entry
void LDW_Process(UINT32 Id, HD_VIDEO_FRAME *p_video_frame,ADAS_VIDEO_INFO *p_stream,BOOL *adas_pianli_alarm)
{
    #if LINE_DETECT
    static int left = 0, right = 0;
    static float left_k1,left_b1,left_k2,left_b2,right_k1,right_b1,right_k2,right_b2;// ROI

    #if LANE_IMG_SAVE
    static int result_img_num = 1;
    static char reslut_img_path[64], mask_img_path[64];
    #endif

    static float seta = PI/2;
    static int x1, x2, x3, x4;
    static BOOL FirstOpen=TRUE;

    static UINT64 lcw_pre_time_ms = 0;
    static UINT64 lcw_cur_time_ms = 0;  //Record LDW alarm interval time
    static BOOL lane_init = TRUE;

    if(FirstOpen) {
        FirstOpen=FALSE;
        create_line(&lanedetector, &ADAS_share_mem[9]);
        Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
        Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
    }
    #endif //LINE_DETECT

    /////Set the speed limit for the algorithm to open, speed > 40km/h
    float cur_speed;
    BOOL gps_status = GPSMNG_GetSpeed(&cur_speed);

    BOOL lcw_flag = TRUE;
    if(TRUE == gps_status){
        if(cur_speed < 40){
            lcw_flag = FALSE;
            if(lane_init){
                Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
                Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2); 
                lanedetector.pre_left_lane.Confidence = 0;
                lanedetector.pre_right_lane.Confidence = 0;
                left_lane.Confidence = 0;
                right_lane.Confidence = 0;
                lane_init = FALSE;
            }
        }else{
            lane_init = TRUE;
        }
    }
    /////--------------end----------------------

    if(TRUE == lcw_flag){
        #if 0 //zmd
        ret = hd_videoproc_pull_out_buf(p_stream->proc_path, &video_frame11, -1);  // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
        if(ret != HD_OK) {
            printf("hd_videoproc_pull_out_buf fail (%d)\n\r", ret);
            return 0;
        }
        #endif


        #if LINE_DETECT
        #if 0 //zmd
        ret = video_frame848to640(p_video_frame, &ADAS_share_mem[20]);
        if(ret != HD_OK) {
            printf("video scale to 640*360 fail (%d)\n\r", ret);
            return 0;
        }
        #endif

        // printf("lane thread start start start start start\r\n");
        left_lane.Confidence = 0;
        right_lane.Confidence = 0;
        left_lane.k = 0.0000001;
        right_lane.k = 0.0000001;
        x1 = WIDTH_CAT/2;
        x2 = WIDTH_CAT/2;
        x3 = WIDTH_CAT/2;
        x4 = WIDTH_CAT/2;

        //edgeDetector(&ADAS_share_mem[20], &ADAS_share_mem[4]);HD_VIDEO_FRAME
        edgeDetector(p_video_frame, &ADAS_share_mem[4]);    //Edge detection

        //get ROI img
        mask((UINT8*)ADAS_share_mem[5].va, left_k1, left_b1, left_k2, left_b2, right_k1, right_b1, right_k2, right_b2);

        #if LANE_IMG_SAVE
        printf("result_img_num = %d\n", result_img_num);
        snprintf(mask_img_path, 64, "//mnt//sd//img//mask_img//mask_img_%05d.bmp", result_img_num);
        writebmpfile(mask_img_path, (UINT8*)ADAS_share_mem[5].va, WIDTH_CAT, HEIGHT_CAT);
        #endif

        clean_line(&lanedetector);

        //detect line
        hough_line_probabilistic(&lanedetector, WIDTH_CAT, HEIGHT_CAT, 2, 180, 12, 10, 30, 60, &ADAS_share_mem[5]);

        #if ADAS_DEBUG
        printf("lines_num = %d\r\n", lanedetector.lines_num);
        #endif

        if (lanedetector.lines_num > 0 && lanedetector.lines_num < 40) {
            lineSeparation(&lanedetector, &ADAS_share_mem[12]); //Straight line classification

            regression(&lanedetector, &left_lane, &right_lane, &ADAS_share_mem[14]);    //Straight line fitting

            if(lanedetector.left_lines_num > 30) {
                left_lane.Confidence = 0;
            }
            if(lanedetector.right_lines_num > 30) {
                right_lane.Confidence = 0;
            }

            if(left_lane.Confidence > 0) {
                #if ADAS_DEBUG
                printf("left lane :\r\nk = %f  b = %f  r = %f\r\n", left_lane.k, left_lane.b, left_lane.Confidence);
                #endif //ADAS_DEBUG
                if(left_lane.k>0) {
                    left_lane.Confidence = 0;
                }
            }
            if (right_lane.Confidence > 0) {
                #if ADAS_DEBUG
                printf("right lane :\r\nk = %f  b = %f  r = %f\r\n", right_lane.k, right_lane.b, right_lane.Confidence);
                #endif //ADAS_DEBUG
                if(right_lane.k < 0) {
                    right_lane.Confidence = 0;
                }
            }
        }

        //Ensure that the slope of the line is not 0
        if(0 == lanedetector.pre_left_lane.k) {
            lanedetector.pre_left_lane.k += 0.00000000001;
        }
        if(0 == lanedetector.pre_right_lane.k) {
            lanedetector.pre_right_lane.k += 0.00000000001;
        }
        if(0 == left_lane.k) {
            left_lane.k += 0.00000000001;
        }
        if(0 == right_lane.k) {
            right_lane.k += 0.00000000001;
        }

        #if LINE_DETECT
        // printf("left_index = %d   right_index = %d\r\n", left_index, right_index);
        // printf("left_lane.Confidence = %f   right_lane.Confidence = %f\r\n", left_lane.Confidence, right_lane.Confidence);
        // printf("left_lane.k = %f   left_lane.b = %f\r\n", left_lane.k, left_lane.b);
        // printf("right_lane.k = %f   right_lane.b = %f\r\n", right_lane.k, right_lane.b);
        if(left_index < 20 && left_lane.Confidence >= 98 && right_lane.Confidence >= 98){
            left_lane_log[left_index].k = left_lane.k;
            left_lane_log[left_index++].b = left_lane.b;
            right_lane_log[right_index].k = right_lane.k;
            right_lane_log[right_index++].b = right_lane.b; 
        }
        #endif

        //Lane line geometric information analysis
        if(left_lane.Confidence >= LANE_CONFIDENCE && right_lane.Confidence >= LANE_CONFIDENCE) {
            int x = (right_lane.b - left_lane.b) / (left_lane.k - right_lane.k);
            lanedetector.center_x = x;
            x1 = x - (int)(left_lane.b / (-left_lane.k));
            x2 = (int)((-right_lane.b) / right_lane.k) - x;
            x3 = x - (int)((left_lane.b - HEIGHT_CAT) / (-left_lane.k));
            x4 = (int)(-(right_lane.b - HEIGHT_CAT) / right_lane.k) - x;
        }

        ////----------update ROI-----------------
        if(left_lane.Confidence >= LANE_CONFIDENCE) {
            set_pre_lane(&(lanedetector.pre_left_lane), &left_lane);
            Set_left_boundary(left_lane.k, left_lane.b, &left_k1, &left_b1, &left_k2, &left_b2);
            left = 0;
        } else {
            lanedetector.pre_left_lane.Confidence -= 4;
            left++;
        }

        if(left >= 3) {
            left = 0;
            lanedetector.center_x = WIDTH_CAT/2;
            Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
        }

        if(right_lane.Confidence >= LANE_CONFIDENCE) {
            set_pre_lane(&(lanedetector.pre_right_lane), &right_lane);
            Set_right_boundary(right_lane.k, right_lane.b, &right_k1, &right_b1, &right_k2, &right_b2);
            right = 0;
        } else {
            lanedetector.pre_right_lane.Confidence -= 4;
            right++;
        }

        if(right >= 3) {
            right = 0;
            lanedetector.center_x = WIDTH_CAT/2;
            Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
        }
        ////---------------end---------------

        //Obtain lane line information according to the previous frame
        if(TRUE == is_reliable_pre_lane(&(lanedetector.pre_left_lane)) \
           && TRUE == is_reliable_pre_lane(&(lanedetector.pre_right_lane))) {

            if(left_lane.Confidence >= LANE_CONFIDENCE && right_lane.Confidence < LANE_CONFIDENCE) {
                if(TRUE == is_stable_lane(&(lanedetector.pre_left_lane), &left_lane)) {
                    set_prediction_lane(&(lanedetector.pre_right_lane), &right_lane);
                }
            } else if(right_lane.Confidence >= LANE_CONFIDENCE && left_lane.Confidence < LANE_CONFIDENCE) {
                if(TRUE == is_stable_lane(&(lanedetector.pre_right_lane), &right_lane)) {
                    set_prediction_lane(&(lanedetector.pre_left_lane), &left_lane);
                }
            }
        }

        #if 1
        ////------------Lane line confidence judgment-------------------
        UINT32 wr = which_reliable(&left_lane, &right_lane);
        switch(wr) {
        case 0:
            break;
        case 1:
            Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
            lanedetector.pre_left_lane.Confidence = 0;
            left_lane.Confidence = 0;
            break;
        case 2:
            Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
            lanedetector.pre_right_lane.Confidence = 0;
            right_lane.Confidence = 0;
            break;
        default:
            break;
        }
        ////----------------end-----------------
        #endif

        /////--------------Lane departure judgment----------------------
        seta = PI/2;
        if(left_lane.Confidence >= LANE_CONFIDENCE && right_lane.Confidence >= LANE_CONFIDENCE) {
            seta = (atan(left_lane.k) + atan(right_lane.k) + PI)/2;
        }
        //param 1.3100114 : Left limit state
        //param 3.3 and 100 : Comprehensive control deviation sensitivity, The bigger the more sensitive
        if((seta - 1.3100114) * 100 < 3.3 && (x1 + x3) < 100) {
            // printf("left left left left left left left left left left left left left left left left!\r\n");
            // *alarmtype_addr = 1;
            if(TRUE == reduce_false_alarms(x1, x2)) { //Reduce false positives
                if(TRUE == gps_status){
                    if(cur_speed > 40){
                        lcw_cur_time_ms = hd_gettime_ms();
                        //param 5000 : Control deviation from alarm interval time
                        if((lcw_cur_time_ms - lcw_pre_time_ms) > 5000){
                            *adas_pianli_alarm = TRUE;
                            // lane_ignore_num = 40; 
                            lcw_pre_time_ms = lcw_cur_time_ms;                        
                        }
                       
                    }
                }else{
                    lcw_cur_time_ms = hd_gettime_ms();
                    //param 5000 : Control deviation from alarm interval time
                    if((lcw_cur_time_ms - lcw_pre_time_ms) > 5000){
                        *adas_pianli_alarm = TRUE;
                        // lane_ignore_num = 40;  
                        lcw_pre_time_ms = lcw_cur_time_ms;                         
                    }
                   
                }
            }
            Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
            Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
            lanedetector.pre_left_lane.Confidence = 0;
            lanedetector.pre_right_lane.Confidence = 0;
        }

        //param 1.8315812 : Right limit state
        //param 3.3 and 100 : Comprehensive control deviation sensitivity, The bigger the more sensitive
        if((1.8315812 - seta) * 100 < 3.3 && (x2 + x4) < 100) { //1.8315812
            // printf("right right right right right right right right right right right right right!\r\n");
            // *alarmtype_addr = 2;
            if(TRUE == reduce_false_alarms(x1, x2)) {//Reduce false positives
                if(TRUE == gps_status){
                    if(cur_speed > 40){
                        lcw_cur_time_ms = hd_gettime_ms();
                        //param 5000 : Control deviation from alarm interval time
                        if((lcw_cur_time_ms - lcw_pre_time_ms) > 5000){
                            *adas_pianli_alarm = TRUE;
                            lcw_pre_time_ms = lcw_cur_time_ms;                        
                        }
                       
                    }
                }else{
                    lcw_cur_time_ms = hd_gettime_ms();
                    //param 5000 : Control deviation from alarm interval time
                    if((lcw_cur_time_ms - lcw_pre_time_ms) > 5000){
                        *adas_pianli_alarm = TRUE; 
                        lcw_pre_time_ms = lcw_cur_time_ms;                         
                    }
                   
                }
            }
            Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
            Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
            lanedetector.pre_left_lane.Confidence = 0;
            lanedetector.pre_right_lane.Confidence = 0;
        }
        /////--------------end----------------------

        #if LANE_IMG_SAVE
        draw_line(&left_lane, &right_lane, &ADAS_share_mem[4]);
        snprintf(reslut_img_path, 64, "//mnt//sd//img//result_img//result_img_%05d.bmp", result_img_num);
        writebmpfile(reslut_img_path, (UINT8*)ADAS_share_mem[4].va, WIDTH_CAT, HEIGHT_CAT);
        result_img_num++;
        #endif

        #endif  //LINE_DETECT

        #if 0 //zmd
        ret = hd_videoproc_release_out_buf(p_stream->proc_path, &video_frame11);  // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
        if(ret != HD_OK) {
            printf("hd_videoproc_release_out_buf fail (%d)\n\r", ret);
            return 0;
        }
        #endif
    }
}

