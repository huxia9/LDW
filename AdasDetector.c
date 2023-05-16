
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "math.h"
#include "hdal.h"
#include "hd_type.h"
#include "vendor_ive.h"
#include "vendor_ai.h"
#include "vendor_ai_cpu/vendor_ai_cpu.h"
#include "vendor_ai_cpu_postproc.h"
#include "comm/hwclock.h"

#include "vendor_md.h"
#include "libmd.h"
// #include "iplimage.h"   

#include "yqlib/AdasDetector.h"
#include "vf_gfx.h"

#include "yqlib/gps.h"//addxjw
#include "yqlib/adas_utils.h" 
#include "yqlib/adas_files.h" 
 
///////////////////////////////////////////////////////////////////////////////
#define __MODULE__ adas_lanedetector
#define __DBGLVL__ 2   //0=OFF, 1=ERROR, 2=TRACE
#define __DBGFLT__ "*" //*=All, [mark]=CustomClass
#include "kwrap/debug.h" 


// TSR 
static int tsr_model_is_America = 0;
static int TSRW = 1200, TSRH = 448, TSRX = 360, TSRY = 220 + G410D_OFFSET;
// static int TSRW = 1260, TSRH = 405, TSRX = 330, TSRY = 220;
static int TSR_IMG_W = 600, TSR_IMG_H = 300, TSR_NUM_W = 300, TSR_NUM_H = 300;
static FLOAT TSR_R_MIN_SIZE = 26.0, TSR_L_MIN_SIZE = 26.0;
static int max_tsr = 3;
static float x_max_tsr1 = 0.5, y_max_tsr1 = 1.0;
static int tsr_offsetx = 0, tsr_offsety = 0;

// static char label_Europe[][20] = {"BackGround", "Limit", "RemoveLimit", "t", "UnLimit", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
char label_Europe[][20] = {"BackGround", "Limit", "RemoveLimit", "UnLimit", "0", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "15"};
char label_America[][20] = {"BackGround", "Limit", "Night", "0", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "15"};

static char txt_buffer_tsr[128];
static int tsr_record_cnt = 1;
static MY_ADAS_RESULT tsr_t_record = {0};

typedef struct _ADAS_NET_PROC {
	ADAS_NET_PROC_CONFIG net_cfg;
	ADAS_MEM_RANGE proc_mem;
	UINT32 proc_id;

	CHAR out_class_labels[MAX_CLASS_NUM * VENDOR_AIS_LBL_LEN];
	ADAS_MEM_RANGE rslt_mem;        
	ADAS_MEM_RANGE io_mem;
} ADAS_NET_PROC;

int ai_async = 0; //0: sync mode will call proc(), 2: sync mode will call proc_buf(), 3: async mode will call push_in_buf() and pull_out_buf()

static ADAS_NET_PROC g_Adas_net[16] = {0};
#if MD_MEM_BLOCK_NUM
ADAS_MEM_RANGE MD_share_mem[MD_MEM_BLOCK_NUM] = {0};
#endif
ADAS_MEM_RANGE ADAS_share_mem[ADAS_MEM_BLOCK_NUM] = {0};
ADAS_MEM_RANGE NET_BinSize_share_mem[ADAS_NN_RUN_NET_NUM] = {0};
static INT32 headstock_offst = 0; //640X360

// static BOOL adas_jingzhi = FALSE;
#if LINE_DETECT
static LaneDetector lanedetector = {0};
static Lane left_lane = {0}, right_lane = {0};

#endif //LINE_DETECT


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

static HD_RESULT ADAS_mem_alloc(ADAS_MEM_RANGE *mem_parm, CHAR *name, UINT32 size)
{
	HD_RESULT ret = HD_OK;
	UINT32 pa = 0;
	void *va = NULL;

	//alloc private pool
	ret = hd_common_mem_alloc(name, &pa, (void **)&va, size, DDR_ID0);
	if (ret != HD_OK) {
		DBG_ERR("mem alloc fail(%d) \n", ret);
		return ret;
	}

	mem_parm->pa = pa;
	mem_parm->va = (UINT32)va;
	mem_parm->size = size;
	mem_parm->blk = (UINT32) - 1;

	return HD_OK;
}

static HD_RESULT ADAS_mem_free(ADAS_MEM_RANGE *mem_parm)
{
	HD_RESULT ret = HD_OK;

	//free private pool
	ret = hd_common_mem_free(mem_parm->pa, (void *)mem_parm->va);
	if (ret != HD_OK) {
		DBG_ERR("mem free fail(%d) \n", ret);
		return ret;
	}

	mem_parm->pa = 0;
	mem_parm->va = 0;
	mem_parm->size = 0;
	mem_parm->blk = (UINT32) - 1;

	return HD_OK;
}

static INT32 ADAS_getsize_model(char *filename)
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
	FILE *fd;
	UINT32 file_size = 0, read_size = 0;
	const UINT32 model_addr = va;
	//DBG_DUMP("model addr = %08x\r\n", (int)model_addr);

	fd = fopen(filename, "rb");
	if (!fd) {
		DBG_ERR("load model(%s) fail\r\n", filename);
		return 0;
	}

	fseek(fd, 0, SEEK_END);
	file_size = ALIGN_CEIL_4(ftell(fd));
	fseek(fd, 0, SEEK_SET);

	read_size = fread((void *)model_addr, 1, file_size, fd);
	if (read_size != file_size) {
		DBG_ERR("size mismatch, real = %d, idea = %d\r\n", (int)read_size, (int)file_size);
	}
	fclose(fd);

	printf("load model(%s) ok\r\n", filename);
	return read_size;
}

static UINT32 ADAS_load_part_model(CHAR *filename, UINT32 part_size, UINT32 va)
{
	FILE *fd;
	UINT32 read_size = 0;
	const UINT32 model_addr = va;

	fd = fopen(filename, "rb");
	if (!fd) {
		DBG_ERR("load model(%s) fail\r\n", filename);
		return 0;
	}

	read_size = fread((void *)model_addr, 1, part_size, fd);
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
		ADAS_NET_PROC *p_net = g_Adas_net + i;
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

HD_RESULT ADAS_network_set_config(NET_PATH_ID net_path, ADAS_NET_PROC_CONFIG *p_proc_cfg)
{
	HD_RESULT ret = HD_OK;
	ADAS_NET_PROC *p_net = g_Adas_net + net_path;
	UINT32 binsize = p_net->net_cfg.binsize;
	UINT32 proc_id;
	p_net->proc_id = net_path;
	proc_id = p_net->proc_id;

	memcpy((void *)&p_net->net_cfg, (void *)p_proc_cfg, sizeof(ADAS_NET_PROC_CONFIG));
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
	ADAS_NET_PROC *p_net = g_Adas_net + net_path;
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
	ADAS_NET_PROC *p_net = g_Adas_net + net_path;

	if (p_net->io_mem.pa && p_net->io_mem.va) {
		ADAS_mem_free(&p_net->io_mem);
	}
	return ret;
}

HD_RESULT ADAS_network_open(NET_PATH_ID net_path)
{
	HD_RESULT ret = HD_OK;
	ADAS_NET_PROC *p_net = g_Adas_net + net_path;
	UINT32 proc_id;
	p_net->proc_id = net_path;
	proc_id = p_net->proc_id;

	UINT32 loadsize = 0;

	if (strlen(p_net->net_cfg.model_filename) == 0) {
		DBG_ERR("proc_id(%u) input model is null\r\n", proc_id);
		return 0;
	}

	if (p_net->net_cfg.p_share_model == NULL) {
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
		vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_MODEL, (VENDOR_AI_NET_CFG_MODEL *)&p_net->proc_mem);
	} else {
		//--- this net_path want to share other share_net model ---
		ADAS_NET_PROC *p_share_net = (ADAS_NET_PROC *)p_net->net_cfg.p_share_model;
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
		vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_MODEL, (VENDOR_AI_NET_CFG_MODEL *)&p_net->proc_mem);
		vendor_ai_net_set(proc_id, VENDOR_AI_NET_PARAM_CFG_SHAREMODEL, (VENDOR_AI_NET_CFG_MODEL *)&p_share_net->proc_mem);
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
	ADAS_NET_PROC *p_net = g_Adas_net + net_path;
	UINT32 proc_id = p_net->proc_id;

	if ((ret = ADAS_network_free_io_buf(net_path)) != HD_OK) {
		return ret;
	}

	// close
	ret = vendor_ai_net_close(proc_id);

	if (p_net->net_cfg.p_share_model == NULL) {
		ADAS_NET_Binsize_mem_rel(&p_net->proc_mem); // common buffer
	} else {
		ADAS_mem_free(&p_net->proc_mem); // private buffer
	}

	return ret;
}
INT32 ADAS_network_get_mem_binsize(NET_PATH_ID net_path, void *p_cfg)
{
	ADAS_NET_PROC *p_net = g_Adas_net + net_path;
	UINT32 proc_id;
	ADAS_NET_PROC_CONFIG *p_proc_cfg = (ADAS_NET_PROC_CONFIG *)p_cfg;
	p_net->proc_id = net_path;
	proc_id = p_net->proc_id;

	memcpy((void *)&p_net->net_cfg, (void *)p_proc_cfg, sizeof(ADAS_NET_PROC_CONFIG));
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
	if (tmp >= 0.5) {
		return (int)a + 1;
	} else {
		return (int)a;
	}
}

//Determine whether the vehicle is the preceding vehicle
BOOL Is_front_car(float kl, float kr, float bl, float br, int x, int y)
{
	if ((kl * x + bl) < y - headstock_offst && (kr * x + br) < y - headstock_offst) {
		return TRUE;
	} else {
		return FALSE;
	}
}


void get_line_params(int x1, int y1, int x2, int y2, float *pk, float *pb)
{
    *pk = (float)(y1 - y2) / (x1 - x2);
    *pb = (float)y1 - (*pk) * x1;
}

//Left boundary line initialization
void Left_boundary_init(float *_left_k1, float *_left_b1, float *_left_k2, float *_left_b2)
{
	*_left_k1 = -HEIGHT_CAT * 1.0 / RIO_LENGTH;          //-80/160
	*_left_b1 = HEIGHT_CAT;                              //80
	*_left_k2 = -HEIGHT_CAT * 1.0 / 50;                  //-80/50
	*_left_b2 = HEIGHT_CAT * WIDTH_CAT * 1.0 / (2 * 50); //80*480/100
	// *_left_k1 = -0.5;
	// *_left_b1 = 50;
	// *_left_k2 = -1;
	// *_left_b2 = 240;

	// float k, b;
	// get_line_params(100, 0, 0, 50, &k, &b);
	// printf("left1 k = %f, b = %f\n", k, b);

	// get_line_params(240, 0, 160, 80, &k, &b);
	// printf("left2 k = %f, b = %f\n", k, b);
	// *_left_k2 = -0.625;
	// *_left_b2 = 180;
}

//Right boundary line initialization
void Right_boundary_init(float *_right_k1, float *_right_b1, float *_right_k2, float *_right_b2)
{
	*_right_k1 = HEIGHT_CAT * 1.0 / RIO_LENGTH;                             //80/160
	*_right_b1 = -(WIDTH_CAT - RIO_LENGTH) * 1.0 / RIO_LENGTH * HEIGHT_CAT; //(480-160)/160*80
	*_right_k2 = HEIGHT_CAT * 1.0 / 50;                                     //80/50
	*_right_b2 = -HEIGHT_CAT * WIDTH_CAT * 1.0 / (2 * 50);                  //80*480/100
	// *_right_k1 = 0.5;
	// *_right_b1 = -190;
	// *_right_k2 = 1;
	// *_right_b2 = -240;

	// float k, b;
	// get_line_params(380, 0, 480, 50, &k, &b);
	// printf("right1 k = %f, b = %f\n", k, b);

	// get_line_params(240, 0, 320, 80, &k, &b);
	// printf("right2 k = %f, b = %f\n", k, b);
	// *_right_k2 = 0.625;
	// *_right_b2 = -120;
}

//update left boundary line
void Set_left_boundary(float _k, float _b, float *_left_k1, float *_left_b1,
					   float *_left_k2, float *_left_b2)
{
	*_left_k1 = _k;
    // *_left_b1 = _b + _k * 40;
    *_left_b1 = _b - 20 / cos(atan(_k));
    *_left_k2 = _k;
    // *_left_b2 = _b - _k * 40;
    *_left_b2 = _b + 20 / cos(atan(_k));
}

//update right boundary line
void Set_right_boundary(float _k, float _b, float *_right_k1, float *_right_b1,
						float *_right_k2, float *_right_b2)
{
	 *_right_k1 = _k;
    // *_right_b1 = _b - _k * 40;
    *_right_b1 = _b - 20 / cos(atan(_k));
    *_right_k2 = _k;
    // *_right_b2 = _b + _k * 40;
    *_right_b2 = _b + 20 / cos(atan(_k));
}

//Determine whether the pixel is within the lane line area
BOOL In_range(int x, int y, float left_k1, float left_b1, float left_k2, float left_b2,
			  float right_k1, float right_b1, float right_k2, float right_b2)
{
	if (((left_k1 * x + left_b1 < y) && (left_k2 * x + left_b2 > y)) || ((right_k1 * x + right_b1 < y) && (right_k2 * x + right_b2 > y))) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void set_pre_lane(Lane *pre_lane, Lane *lane)
{
	memcpy(pre_lane, lane, sizeof(Lane));
}

BOOL is_stable_lane(Lane *pre_lane, Lane *lane)
{
	if (fabs(lane->k - pre_lane->k) < 0.08 && fabs(lane->b - pre_lane->b) < 15) {
		return TRUE;
	} else {
		return FALSE;
	}
}

BOOL is_reliable_pre_lane(Lane *pre_lane)
{
	if (pre_lane->Confidence > LANE_CONFIDENCE) {
		return TRUE;
	} else {
		return FALSE;
	}
}

void set_prediction_lane(Lane *pre_lane, Lane *lane)
{
	memcpy(lane, pre_lane, sizeof(Lane));
	// lane->Confidence -= 3;
}

UINT32 which_reliable(Lane *left_lane, Lane *right_lane)
{
	UINT32 ret = 0; // both reliable
	int x1, x2;
	if (0 != left_lane->k && 0 != right_lane->k) {
		x1 = (HEIGHT_CAT / 2 - left_lane->b) / left_lane->k;
		x2 = (HEIGHT_CAT / 2 - right_lane->b) / right_lane->k;
		if ((x2 - x1) > 280) {
			if (fabs(left_lane->k) < right_lane->k) {
				ret = 1;
			} else {
				ret = 2;
			}
		} else if ((x2 - x1) < 110) {
			if (fabs(left_lane->k) < right_lane->k) {
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

//Request memory to save RIO image data and lines
void create_line(LaneDetector *_lanedetector, ADAS_MEM_RANGE *p_share_mem)
{
	// _lanedetector->graydata_cat = (uint8_t*)malloc(CAT_IMG_SIZE * sizeof(uint8_t));
	// _lanedetector->lines = (line_int_t*)malloc(100 * sizeof(line_int_t));
	// _lanedetector->left_lines = (line_int_t*)malloc(50 * sizeof(line_int_t));
	// _lanedetector->right_lines = (line_int_t*)malloc(50 * sizeof(line_int_t));
	_lanedetector->lines = (line_int_t *)p_share_mem[0].va;
	_lanedetector->left_lines = (line_int_t *)p_share_mem[1].va;
	_lanedetector->right_lines = (line_int_t *)p_share_mem[2].va;
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

//Release memory that save RIO image data and lines
// void releaseGrayimg_and_line(LaneDetector *_lanedetector){
//  // if(_lanedetector->graydata_cat) {
//  //  free(_lanedetector->graydata_cat);
//  // }
//  if(_lanedetector->lines) {
//      free(_lanedetector->lines);
//  }
//  if(_lanedetector->left_lines) {
//      free(_lanedetector->left_lines);
//  }
//  if(_lanedetector->right_lines) {
//      free(_lanedetector->right_lines);
//  }
//  // _lanedetector->graydata_cat = NULL;
//  _lanedetector->lines = NULL;
//  _lanedetector->left_lines = NULL;
//  _lanedetector->right_lines = NULL;
//  _lanedetector->lines_num = 0;
//  _lanedetector->left_lines_num = 0;
//  _lanedetector->right_lines_num = 0;
// }

//Fitting line
void LineFitLeastSquares(point_int_t *point, int point_num, Lane *Result)
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

	// ����б��a�ͽؾ�b
	float a, b, temp = 0;
	temp = (point_num * A - B * B);
	if (temp) {
		// �жϷ�ĸ��Ϊ0
		a = (point_num * C - B * D) / temp;
		b = (A * D - B * C) / temp;
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

	if (0 == F) {
		F += 0.00000000001;
	}
	float r;
	r = E / F;

	Result->k = a;
	Result->b = b;
	Result->Confidence = r * r * 100;
}



void my_crop(void *dst, void *src, UINT32 src_w, UINT32 x, UINT32 y, UINT32 w, UINT32 h)
{
	char *p_src = (char *)src + src_w * (y - 1) + x;
	char *p_dst = (char *)dst;
	UINT32 i;
	for (i = 0; i < h; i++, p_src += src_w, p_dst += w) {
		memcpy(p_dst, p_src, w);
	}
	return;
}

//get RIO and image preprocessing
void edgeDetector(HD_VIDEO_FRAME *video, ADAS_MEM_RANGE *p_share_mem)
{

	int i;
	HD_RESULT ret = HD_OK;
	UINT32 out_lofs = 0, out_selection = 0;
	// UINT8* mor_img = NULL;

	// printf("p_share_mem_addr = 0x%08x\r\n", (UINT32)p_share_mem);
	// printf("input_num = %d\r\n", input_num);
	// #if LANE_IMG_SAVE
	// char output_img_path[64], output_img_path1[64], input_img_path[64];
	// snprintf(input_img_path, 64, "//mnt//sd//output_img//input_img_%05d.bmp", input_num);
	// snprintf(output_img_path, 64, "//mnt//sd//output_img//out_img_%05d.bmp", input_num);
	// snprintf(output_img_path1, 64, "//mnt//sd//output_img//sup_img_%05d.bmp", input_num);
	// #endif

	#if EDGE_DEBUG
	static int input_num = 1;
	char input_img_path[64], sobel_img_path[64];
	#if LINER_ENHANCE
	char enhance_img_path[64];
	snprintf(enhance_img_path, 64, "//mnt//sd//img//enhance_img//enhance_img_%05d.bmp", input_num);
	#endif
	snprintf(input_img_path, 64, "//mnt//sd//img//input_img//result_img_%05d.bmp", input_num);
	snprintf(sobel_img_path, 64, "//mnt//sd//img//sobel_img//sobel_img_%05d.bmp", input_num);
	input_num++;
	readbmpfile(input_img_path, (UINT8 *)p_share_mem[0].va);

	#else

	// char *src_img = (char *)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, video->phy_addr[0], ADAS_IMG_BUF_SIZE);
	// my_crop((void *)p_share_mem[0].va, (void *)src_img, ADAS_IMG_WIDTH, X_CAT, Y_CAT, WIDTH_CAT, HEIGHT_CAT);
	// hd_common_mem_munmap((void *)src_img, ADAS_IMG_BUF_SIZE);

	#if 1
	HD_GFX_SCALE crop;

	memset(&crop, 0, sizeof(HD_GFX_SCALE));
	// crop.src_img.dim.w              = ADAS_IMG_WIDTH;
	// crop.src_img.dim.h              = ADAS_IMG_HEIGHT;
	// crop.src_img.format             = HD_VIDEO_PXLFMT_YUV420;
	// crop.src_img.p_phy_addr[0]      = p_share_mem_src->pa;
	// crop.src_img.p_phy_addr[1]      = p_share_mem_src->pa + ADAS_IMG_BUF_SIZE;
	// crop.src_img.lineoffset[0]      = ADAS_IMG_WIDTH;
	// crop.src_img.lineoffset[1]      = ADAS_IMG_WIDTH;
	crop.src_img.dim.w = video->dim.w;
	crop.src_img.dim.h = video->dim.h;
	// crop.src_img.format = HD_VIDEO_PXLFMT_YUV420;
	crop.src_img.format = HD_VIDEO_PXLFMT_Y8;
	crop.src_img.p_phy_addr[0] = video->phy_addr[0];
	crop.src_img.p_phy_addr[1] = video->phy_addr[1];
	crop.src_img.lineoffset[0] = video->loff[0];
	crop.src_img.lineoffset[1] = video->loff[1];
	// crop.src_region.x               = X_CAT;//80
	// crop.src_region.y               = Y_CAT;//220
	// crop.src_region.w               = WIDTH_CAT;//480
	// crop.src_region.h               = HEIGHT_CAT;//80
	static float ldw_img_rate = (float)5 / 8;
	crop.src_region.x = (int)((int)((640 - 480 * ldw_img_rate) / 2) * video->dim.w / 640);
	crop.src_region.y = (int)(((int)(300 - 80 * ldw_img_rate) + headstock_offst) * video->dim.h / 360);
	crop.src_region.w = (int)(480 * ldw_img_rate * video->dim.w / 640);
	crop.src_region.h = (int)(80 * ldw_img_rate * video->dim.h / 360);

	// printf("crop.src_region.x = %d\n", crop.src_region.x);
	// printf("crop.src_region.y = %d\n", crop.src_region.y);
	// printf("crop.src_region.w = %d\n", crop.src_region.w);
	// printf("crop.src_region.h = %d\n", crop.src_region.h);

	// printf("crop.src_region.x2 = %d\n", (int)(170 * video->dim.w / 640));
	// printf("crop.src_region.y2 = %d\n", (int)((250 + headstock_offst) * video->dim.h / 360));
	// printf("crop.src_region.w2 = %d\n", (int)(300 * video->dim.w / 640));
	// printf("crop.src_region.h2 = %d\n", (int)(50 * video->dim.h / 360));

	crop.dst_img.dim.w = WIDTH_CAT;
	crop.dst_img.dim.h = HEIGHT_CAT;
	crop.dst_img.format = HD_VIDEO_PXLFMT_Y8;
	crop.dst_img.p_phy_addr[0] = p_share_mem[0].pa;
	crop.dst_img.lineoffset[0] = WIDTH_CAT;
	crop.dst_region.x = 0;
	crop.dst_region.y = 0;
	crop.dst_region.w = WIDTH_CAT;
	crop.dst_region.h = HEIGHT_CAT;

	// crop.quality                    = HD_GFX_SCALE_QUALITY_BILINEAR;

	ret = hd_gfx_scale_sw(&crop);
	if (ret != HD_OK) {
		DBG_ERR("hd_gfx_crop_img fail=%d\n", ret);
	}
	#endif
	// static char ldw_img_path[64];
	// static int ldw_name_num = 0;
	// snprintf(ldw_img_path, 64, "//mnt//sd//img//ldw_%05d.bmp", ldw_name_num++);
	// writebmpfile(ldw_img_path, (void *)p_share_mem[0].va, WIDTH_CAT, HEIGHT_CAT);

	#endif

	#if LINER_ENHANCE
	UINT8 *en_img = (UINT8 *)p_share_mem[0].va;
	int max = 0, min = 1000;
	for (i = 0; i < HEIGHT_CAT * WIDTH_CAT; i++, en_img++) {
		if (*en_img > max) {
			max = *en_img;
		}
		if (*en_img < min) {
			min = *en_img;
		}
	}

	en_img = (UINT8 *)p_share_mem[0].va;
	for (i = 0; i < HEIGHT_CAT * WIDTH_CAT; i++, en_img++) {
		*en_img = (UINT8)(255.0 * (*en_img - min) / (max - min));
	}

// writebmpfile(enhance_img_path, (void *)p_share_mem[0].va, WIDTH_CAT, HEIGHT_CAT);
	#endif

	UINT32 ch = 0;
	VENDOR_IVE_IN_IMG_INFO ive_input_info = {0};
	VENDOR_IVE_IMG_IN_DMA_INFO ive_input_addr = {0};
	VENDOR_IVE_IMG_OUT_DMA_INFO ive_output_addr = {0};
	VENDOR_IVE_GENERAL_FILTER_PARAM ive_general_filter_param = {0};
	VENDOR_IVE_MEDIAN_FILTER_PARAM ive_median_filter_param = {0};
	VENDOR_IVE_EDGE_FILTER_PARAM ive_edge_filter_param = {0};
	VENDOR_IVE_NON_MAX_SUP_PARAM ive_non_max_sup_param = {0};
	VENDOR_IVE_THRES_LUT_PARAM ive_thres_lut_param = {0};
	VENDOR_IVE_MORPH_FILTER_PARAM ive_morph_filter_param = {0};
	VENDOR_IVE_INTEGRAL_IMG_PARAM ive_integral_img_param = {0};
	VENDOR_IVE_TRIGGER_PARAM ive_trigger_param = {0};
	VENDOR_IVE_OUTSEL_PARAM ive_outsel_param = {0};

	// UINT32 general_filter_coef[VENDOR_IVE_GEN_FILT_NUM] = {14,1,1,11,12,12,9,9,12,1};
	// UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] ={{-1,-2,-1,0,0,0,1,2,1},{1,0,-1,2,0,-2,1,0,-1}};
	UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] = {{-1, 0, 1, -2, 0, 2, -1, 0, 1}, {1, 2, 1, 0, 0, 0, -1, -2, -1}};

	ret = vendor_ive_init();
	if (ret != HD_OK) {
		DBG_ERR("err:vendor_ive_init error %d\r\n", ret);
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
	ive_general_filter_param.enable = 0;
	ive_median_filter_param.enable = 1;
	ive_edge_filter_param.enable = 1;
	ive_non_max_sup_param.enable = 1;
	ive_thres_lut_param.enable = 0;
	ive_morph_filter_param.enable = 0;
	ive_integral_img_param.enable = 0;
	ive_trigger_param.time_out_ms = 0;
	ive_trigger_param.wait_end = 1;
	ive_trigger_param.is_nonblock = 0;

	// set median filter param
	if (ive_median_filter_param.enable) {
		ive_median_filter_param.mode = VENDOR_IVE_MEDIAN;
	}

	// set edge filter param
	ive_edge_filter_param.mode = VENDOR_IVE_BI_DIR;
	ive_edge_filter_param.AngSlpFact = 17;
	for (i = 0; i < VENDOR_IVE_EDGE_COEFF_NUM; i++) {
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
	if (HD_OK != ret) {
		DBG_ERR("uninit fail, error code = %d\r\n", ret);
	}

	// memcpy(cat_img, (void *)p_share_mem[1].va, WIDTH_CAT * HEIGHT_CAT);

	#if EDGE_DEBUG
	writebmpfile(sobel_img_path, (void *)p_share_mem[1].va, WIDTH_CAT, HEIGHT_CAT);
	#endif

	// UINT8* cat_img = (UINT8*)p_share_mem[1].va;

	// #if LANE_IMG_SAVE

	//  writebmpfile(output_img_path1, cat_img, WIDTH_CAT, HEIGHT_CAT);

	// #if LINER_ENHANCE
	// int thr = 50;
	// #else
	// int thr = 35;
	// #endif

	// for(i = 0; i < HEIGHT_CAT; i++) {
	//     for(j = 0; j < WIDTH_CAT; j++) {
	//         if(cat_img[i * WIDTH_CAT + j] > thr) {
	//             cat_img[i * WIDTH_CAT + j] = 255;
	//         } else {
	//             cat_img[i * WIDTH_CAT + j] = 0;
	//         }
	//     }
	// }

	//  writebmpfile(output_img_path, cat_img, WIDTH_CAT, HEIGHT_CAT);
	//  input_num++;
	// #endif
}

void edgeDetector_headstock(HD_VIDEO_FRAME *video, ADAS_MEM_RANGE *p_share_mem)
{

	int i;
	HD_RESULT ret = HD_OK;
	UINT32 out_lofs = 0, out_selection = 0;
	// UINT8* mor_img = NULL;

	// printf("p_share_mem_addr = 0x%08x\r\n", (UINT32)p_share_mem);
	// printf("input_num = %d\r\n", input_num);
	// #if LANE_IMG_SAVE
	// char output_img_path[64], output_img_path1[64], input_img_path[64];
	// snprintf(input_img_path, 64, "//mnt//sd//output_img//input_img_%05d.bmp", input_num);
	// snprintf(output_img_path, 64, "//mnt//sd//output_img//out_img_%05d.bmp", input_num);
	// snprintf(output_img_path1, 64, "//mnt//sd//output_img//sup_img_%05d.bmp", input_num);
	// #endif

	#if EDGE_DEBUG
	static int input_num = 1;
	char input_img_path[64], sobel_img_path[64];
	#if LINER_ENHANCE
	char enhance_img_path[64];
	snprintf(enhance_img_path, 64, "//mnt//sd//img//enhance_img//enhance_img_%05d.bmp", input_num);
	#endif
	snprintf(input_img_path, 64, "//mnt//sd//img//input_img//result_img_%05d.bmp", input_num);
	snprintf(sobel_img_path, 64, "//mnt//sd//img//sobel_img//sobel_img_%05d.bmp", input_num);
	input_num++;
	readbmpfile(input_img_path, (UINT8 *)p_share_mem[0].va);

	#else

	// char *src_img = (char *)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, video->phy_addr[0], ADAS_IMG_BUF_SIZE);
	// my_crop((void *)p_share_mem[0].va, (void *)src_img, ADAS_IMG_WIDTH, X_CAT, Y_CAT, WIDTH_CAT, HEIGHT_CAT);
	// hd_common_mem_munmap((void *)src_img, ADAS_IMG_BUF_SIZE);

	#if 1
	HD_GFX_SCALE crop;

	memset(&crop, 0, sizeof(HD_GFX_SCALE));
	// crop.src_img.dim.w              = ADAS_IMG_WIDTH;
	// crop.src_img.dim.h              = ADAS_IMG_HEIGHT;
	// crop.src_img.format             = HD_VIDEO_PXLFMT_YUV420;
	// crop.src_img.p_phy_addr[0]      = p_share_mem_src->pa;
	// crop.src_img.p_phy_addr[1]      = p_share_mem_src->pa + ADAS_IMG_BUF_SIZE;
	// crop.src_img.lineoffset[0]      = ADAS_IMG_WIDTH;
	// crop.src_img.lineoffset[1]      = ADAS_IMG_WIDTH;
	crop.src_img.dim.w = video->dim.w;
	crop.src_img.dim.h = video->dim.h;
	crop.src_img.format = HD_VIDEO_PXLFMT_YUV420;
	crop.src_img.p_phy_addr[0] = video->phy_addr[0];
	crop.src_img.p_phy_addr[1] = video->phy_addr[1];
	crop.src_img.lineoffset[0] = video->loff[0];
	crop.src_img.lineoffset[1] = video->loff[1];
	// crop.src_region.x               = X_CAT;
	// crop.src_region.y               = Y_CAT;
	// crop.src_region.w               = WIDTH_CAT;
	// crop.src_region.h               = HEIGHT_CAT;
	crop.src_region.x = (int)(270 * video->dim.w / 640);
	crop.src_region.y = (int)(180 * video->dim.h / 360);
	crop.src_region.w = (int)(80 * video->dim.w / 640);
	crop.src_region.h = (int)(180 * video->dim.h / 360);

	crop.dst_img.dim.w = 80;
	crop.dst_img.dim.h = 180;
	crop.dst_img.format = HD_VIDEO_PXLFMT_Y8;
	crop.dst_img.p_phy_addr[0] = p_share_mem[0].pa;
	crop.dst_img.lineoffset[0] = 80;
	crop.dst_region.x = 0;
	crop.dst_region.y = 0;
	crop.dst_region.w = 80;
	crop.dst_region.h = 180;

	// crop.quality                    = HD_GFX_SCALE_QUALITY_BILINEAR;

	ret = hd_gfx_scale_sw(&crop);
	if (ret != HD_OK) {
		DBG_ERR("hd_gfx_crop_img fail=%d\n", ret);
	}
	#endif
	// static char ldw_img_path[64];
	// static int ldw_name_num = 0;
	// snprintf(ldw_img_path, 64, "//mnt//sd//img//ldw_%05d.bmp", ldw_name_num++);
	// writebmpfile(ldw_img_path, (void *)p_share_mem[0].va, 80, 180);

	#endif

	#if LINER_ENHANCE
	UINT8 *en_img = (UINT8 *)p_share_mem[0].va;
	int max = 0, min = 1000;
	for (i = 0; i < HEIGHT_CAT * WIDTH_CAT; i++, en_img++) {
		if (*en_img > max) {
			max = *en_img;
		}
		if (*en_img < min) {
			min = *en_img;
		}
	}

	en_img = (UINT8 *)p_share_mem[0].va;
	for (i = 0; i < HEIGHT_CAT * WIDTH_CAT; i++, en_img++) {
		*en_img = (UINT8)(255.0 * (*en_img - min) / (max - min));
	}

// writebmpfile(enhance_img_path, (void *)p_share_mem[0].va, WIDTH_CAT, HEIGHT_CAT);
	#endif

	UINT32 ch = 0;
	VENDOR_IVE_IN_IMG_INFO ive_input_info = {0};
	VENDOR_IVE_IMG_IN_DMA_INFO ive_input_addr = {0};
	VENDOR_IVE_IMG_OUT_DMA_INFO ive_output_addr = {0};
	VENDOR_IVE_GENERAL_FILTER_PARAM ive_general_filter_param = {0};
	VENDOR_IVE_MEDIAN_FILTER_PARAM ive_median_filter_param = {0};
	VENDOR_IVE_EDGE_FILTER_PARAM ive_edge_filter_param = {0};
	VENDOR_IVE_NON_MAX_SUP_PARAM ive_non_max_sup_param = {0};
	VENDOR_IVE_THRES_LUT_PARAM ive_thres_lut_param = {0};
	VENDOR_IVE_MORPH_FILTER_PARAM ive_morph_filter_param = {0};
	VENDOR_IVE_INTEGRAL_IMG_PARAM ive_integral_img_param = {0};
	VENDOR_IVE_TRIGGER_PARAM ive_trigger_param = {0};
	VENDOR_IVE_OUTSEL_PARAM ive_outsel_param = {0};

	// UINT32 general_filter_coef[VENDOR_IVE_GEN_FILT_NUM] = {14,1,1,11,12,12,9,9,12,1};
	// UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] ={{-1,-2,-1,0,0,0,1,2,1},{1,0,-1,2,0,-2,1,0,-1}};
	UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] = {{-1, 0, 1, -2, 0, 2, -1, 0, 1}, {1, 2, 1, 0, 0, 0, -1, -2, -1}};

	ret = vendor_ive_init();
	if (ret != HD_OK) {
		DBG_ERR("err:vendor_ive_init error %d\r\n", ret);
		goto exit;
	}

	// set input & output info
	ive_input_addr.addr = p_share_mem[0].pa;
	ive_output_addr.addr = p_share_mem[1].pa;
	// ive_input_info.width = WIDTH;
	// ive_input_info.height = HEIGHT;
	// ive_input_info.lineofst = WIDTH;
	// out_lofs = WIDTH;
	ive_input_info.width = 80;
	ive_input_info.height = 180;
	ive_input_info.lineofst = 80;
	out_lofs = 80;
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
	memset((void *)p_share_mem[1].va, 0, 80 * 180);

	// set func enable
	ive_general_filter_param.enable = 0;
	ive_median_filter_param.enable = 1;
	ive_edge_filter_param.enable = 1;
	ive_non_max_sup_param.enable = 1;
	ive_thres_lut_param.enable = 0;
	ive_morph_filter_param.enable = 0;
	ive_integral_img_param.enable = 0;
	ive_trigger_param.time_out_ms = 0;
	ive_trigger_param.wait_end = 1;
	ive_trigger_param.is_nonblock = 0;

	// set median filter param
	if (ive_median_filter_param.enable) {
		ive_median_filter_param.mode = VENDOR_IVE_MEDIAN;
	}

	// set edge filter param
	ive_edge_filter_param.mode = VENDOR_IVE_BI_DIR;
	ive_edge_filter_param.AngSlpFact = 17;
	for (i = 0; i < VENDOR_IVE_EDGE_COEFF_NUM; i++) {
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
	if (HD_OK != ret) {
		DBG_ERR("uninit fail, error code = %d\r\n", ret);
	}

	// memcpy(cat_img, (void *)p_share_mem[1].va, WIDTH_CAT * HEIGHT_CAT);

	#if EDGE_DEBUG
	writebmpfile(sobel_img_path, (void *)p_share_mem[1].va, WIDTH_CAT, HEIGHT_CAT);
	#endif
}

void edgeDetector_tsr(ADAS_MEM_RANGE *p_share_mem_src, ADAS_MEM_RANGE *p_share_mem_ive)
{

	int i;
	HD_RESULT ret = HD_OK;
	UINT32 out_lofs = 0, out_selection = 0;

	UINT32 ch = 0;
	VENDOR_IVE_IN_IMG_INFO ive_input_info = {0};
	VENDOR_IVE_IMG_IN_DMA_INFO ive_input_addr = {0};
	VENDOR_IVE_IMG_OUT_DMA_INFO ive_output_addr = {0};
	VENDOR_IVE_GENERAL_FILTER_PARAM ive_general_filter_param = {0};
	VENDOR_IVE_MEDIAN_FILTER_PARAM ive_median_filter_param = {0};
	VENDOR_IVE_EDGE_FILTER_PARAM ive_edge_filter_param = {0};
	VENDOR_IVE_NON_MAX_SUP_PARAM ive_non_max_sup_param = {0};
	VENDOR_IVE_THRES_LUT_PARAM ive_thres_lut_param = {0};
	VENDOR_IVE_MORPH_FILTER_PARAM ive_morph_filter_param = {0};
	VENDOR_IVE_INTEGRAL_IMG_PARAM ive_integral_img_param = {0};
	VENDOR_IVE_TRIGGER_PARAM ive_trigger_param = {0};
	VENDOR_IVE_OUTSEL_PARAM ive_outsel_param = {0};

	// UINT32 general_filter_coef[VENDOR_IVE_GEN_FILT_NUM] = {14,1,1,11,12,12,9,9,12,1};
	// UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] ={{-1,-2,-1,0,0,0,1,2,1},{1,0,-1,2,0,-2,1,0,-1}};
	UINT32 edge_filter_coef[2][VENDOR_IVE_EDGE_COEFF_NUM] = {{-1, 0, 1, -2, 0, 2, -1, 0, 1}, {1, 2, 1, 0, 0, 0, -1, -2, -1}};

	ret = vendor_ive_init();
	if (ret != HD_OK) {
		DBG_ERR("err:vendor_ive_init error %d\r\n", ret);
		goto exit;
	}

	// set input & output info
	ive_input_addr.addr = p_share_mem_src[0].pa;
	ive_output_addr.addr = p_share_mem_ive[0].pa;
	// ive_input_info.width = WIDTH;
	// ive_input_info.height = HEIGHT;
	// ive_input_info.lineofst = WIDTH;
	// out_lofs = WIDTH;
	ive_input_info.width = TSR_NUM_W;
	ive_input_info.height = TSR_NUM_H;
	ive_input_info.lineofst = TSR_NUM_W;
	out_lofs = (TSR_NUM_W * 8 + 7) >> 3;
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

	memset((void *)p_share_mem_ive[0].va, 0, TSR_NUM_W * TSR_NUM_H);

	// set func enable
	ive_general_filter_param.enable = 0;
	ive_median_filter_param.enable = 1;
	ive_edge_filter_param.enable = 1;
	ive_non_max_sup_param.enable = 1;
	ive_thres_lut_param.enable = 0;
	ive_morph_filter_param.enable = 0;
	ive_integral_img_param.enable = 0;
	ive_trigger_param.time_out_ms = 0;
	ive_trigger_param.wait_end = 1;
	ive_trigger_param.is_nonblock = 0;

	// set median filter param
	if (ive_median_filter_param.enable) {
		ive_median_filter_param.mode = VENDOR_IVE_MEDIAN;
	}

	// set edge filter param
	ive_edge_filter_param.mode = VENDOR_IVE_BI_DIR;
	ive_edge_filter_param.AngSlpFact = 17;
	for (i = 0; i < VENDOR_IVE_EDGE_COEFF_NUM; i++) {
		ive_edge_filter_param.edge_coeff1[i] = edge_filter_coef[0][i];
		ive_edge_filter_param.edge_coeff2[i] = edge_filter_coef[1][i];
	}
	ive_edge_filter_param.edge_shift_bit = 0;

	// set non max sup param
	ive_non_max_sup_param.mag_thres = 67;

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
	if (HD_OK != ret) {
		DBG_ERR("uninit fail, error code = %d\r\n", ret);
	}

	// static char tsr_ive_img_path[64];
	// static int tsr_ive_name_num = 0;
	// snprintf(tsr_ive_img_path, 64, "//mnt//sd//img//tsr//ive_%05d.bmp", tsr_ive_name_num++);
	// writebmpfile(tsr_ive_img_path, (void *)p_share_mem_ive[0].va, 320, 320);
}

//默认图片最大宽高为300
void my_conv(UINT8 *img, int w, int h, int model)
{
	double GussKernal[9] = {
		0.075114, 0.123841, 0.075114,
		0.123841, 0.204180, 0.123841,
		0.075114, 0.123841, 0.075114
	};

	UINT8 temp_line[2][300];
	int new_pixel;
	memcpy((void *)temp_line[0], img, w * sizeof(UINT8));
	memcpy((void *)temp_line[1], img+w, w * sizeof(UINT8));

	for(int row = 1 ; row < h - 1 ;row++){
		for(int column = 1 ; column < w - 1;column++){
			new_pixel = temp_line[0][column - 1] * GussKernal[0] + temp_line[0][column] * GussKernal[1] + temp_line[0][column + 1] * GussKernal[2] + \
						temp_line[1][column - 1] * GussKernal[3] + temp_line[1][column] * GussKernal[4] + temp_line[1][column + 1] * GussKernal[5];
			new_pixel += img[(row + 1) * w + column - 1] * GussKernal[6] + img[(row + 1) * w + column] * GussKernal[7] + img[(row + 1) * w + column + 1] * GussKernal[8];
			img[row * w + column] = new_pixel;
			if(column == w - 2){
				memcpy((void *)temp_line[0], (void *)temp_line[1], w * sizeof(UINT8));
				memcpy((void *)temp_line[1], (void *)(img + (row + 1) * w), w * sizeof(UINT8));
			}
		}
	}
}

void BilateralFilter(UINT8 *img, int w, int h)
{

	float GussKernal[9] = {
		0.075114, 0.123841, 0.075114,
		0.123841, 0.204180, 0.123841,
		0.075114, 0.123841, 0.075114
	};
	float * table = (float *)ADAS_share_mem[26].va;
	// float GussKernal[9] = {
	// 	table[2], table[1], table[2],
	// 	table[1], table[0], table[1],
	// 	table[2], table[1], table[2]};
	float KN[9], sum_kn;

	UINT8 temp_line[3][300];
	float new_pixel;
	memcpy((void *)temp_line[0], img, w * sizeof(UINT8));
	memcpy((void *)temp_line[1], img+w, w * sizeof(UINT8));
	memcpy((void *)temp_line[2], img+w+w, w * sizeof(UINT8));

	for(int row = 1 ; row < h - 1 ;row++){
		for(int col = 1 ; col < w - 1;col++){
			for(int row_kn = 0, col_kn, diff; row_kn < 3; row_kn++){
				for(col_kn = 0; col < 3; col_kn++){
					diff =  abs(img[row * w + col] - img[(row - 1 + row_kn) * w + col - 1 + col_kn]);
					KN[row_kn * 3 + col_kn] = GussKernal[row_kn * 3 + col_kn] * table[diff];
				}								
			}
			sum_kn = 0;
			for(int idx_kn = 0; idx_kn < 9; idx_kn++){
				sum_kn += KN[idx_kn];
			}
			for(int idx_kn = 0; idx_kn < 9; idx_kn++){
				KN[idx_kn] /= sum_kn;
			}

			new_pixel = temp_line[0][col - 1] * KN[0] + temp_line[0][col] * KN[1] + temp_line[0][col + 1] * KN[2] + \
						temp_line[1][col - 1] * KN[3] + temp_line[1][col] * KN[4] + temp_line[1][col + 1] * KN[5] + \
						temp_line[2][col - 1] * KN[6] + temp_line[2][col] * KN[7] + temp_line[2][col + 1] * KN[8];
			img[row * w + col] = limit0_255((int)new_pixel);
			if(col == w - 2){
				memcpy((void *)temp_line[0], (void *)temp_line[1], w * sizeof(UINT8));
				memcpy((void *)temp_line[1], (void *)temp_line[2], w * sizeof(UINT8));
				memcpy((void *)temp_line[2], (void *)(img + (row + 2) * w), w * sizeof(UINT8));
			}
		}
	}
}

void MedianFilter(UINT8 *img, int w, int h)
{
	UINT8 temp_line[2][300];
	int new_pixel;
	memcpy((void *)temp_line[0], img, w * sizeof(UINT8));
	memcpy((void *)temp_line[1], img+w, w * sizeof(UINT8));

	for(int row = 1 ; row < h - 1 ;row++){
		for(int column = 1 ; column < w - 1;column++){
			new_pixel = temp_line[0][column - 1] + temp_line[0][column] + temp_line[0][column + 1] + temp_line[1][column - 1] + temp_line[1][column]  + temp_line[1][column + 1] + img[(row + 1) * w + column - 1] + img[(row + 1) * w + column] + img[(row + 1) * w + column + 1];
			img[row * w + column] = new_pixel / 9;
			if(column == w - 2){
				memcpy((void *)temp_line[0], (void *)temp_line[1], w * sizeof(UINT8));
				memcpy((void *)temp_line[1], (void *)(img + (row + 1) * w), w * sizeof(UINT8));
			}
		}
	}
}


void HistogramEqualization(UINT8 *img, int w, int h, int block_numx, int block_numy, int th, int max)
{
	int blockw = w / block_numx, blockh = h / block_numy;
	int pixel_index_offset;
	for(int block_index = 0; block_index < block_numx * block_numy; block_index++){
		int gray_hist_src[256] = {0}, gray_hist_lut[256] = {0};
		int pixel_total = 0;
		float gray_level_proportion_cumulative[256] = {0};
		pixel_index_offset = (block_index % block_numx) * blockw + (block_index / block_numx) * blockh * w;
		for(int row = 0; row < blockh; row++){
			for(int col = 0;col < blockw; col++){
				if(img[row * w + col + pixel_index_offset] <= th){
					gray_hist_src[img[row * w + col + pixel_index_offset]]++;
					pixel_total++;
				}
			}
		}
		gray_level_proportion_cumulative[0] = (float)gray_hist_src[0] / pixel_total;
		for (int i = 1; i < 255; i++) {
			gray_level_proportion_cumulative[i] = gray_level_proportion_cumulative[i - 1] + (float)gray_hist_src[i] / pixel_total;
		}
		gray_level_proportion_cumulative[255] = 1;
		for(int i = 0; i < 256; i++){
			gray_hist_lut[i] = gray_level_proportion_cumulative[i] * max;

			for(int row = 0; row < blockh; row++){
				for(int col = 0;col < blockw; col++){
					if(img[row * w + col + pixel_index_offset] <= th){
						img[row * w + col + pixel_index_offset] = gray_hist_lut[img[row * w + col + pixel_index_offset]];
					}
				}
			}
		}
	}
	
}

void HistogramEqualization1(UINT8 *img, int w, int h, int th, int max)
{
	int gray_hist_src[256] = {0}, gray_hist_lut[256] = {0};
	int pixel_total = 0;
	float gray_level_proportion_cumulative[256] = {0};


	for(int row = 0; row < h; row++){
		for(int col = 0;col < w; col++){
			if(img[row * w + col] <= th){
				gray_hist_src[img[row * w + col]]++;
				pixel_total++;
			}
		}
	}

	gray_level_proportion_cumulative[0] = (float)gray_hist_src[0] / pixel_total;
	for (int i = 1; i < 255; i++) {
		gray_level_proportion_cumulative[i] = gray_level_proportion_cumulative[i - 1] + (float)gray_hist_src[i] / pixel_total;
	}
	gray_level_proportion_cumulative[255] = 1;

	for(int i = 0; i < 256; i++){
		gray_hist_lut[i] = gray_level_proportion_cumulative[i] * max;
	}

	pixel_total = w * h;
	for(int index_pixel = 0; index_pixel < pixel_total; index_pixel++)
	{	
		if(img[index_pixel] <= th)
		img[index_pixel] = gray_hist_lut[img[index_pixel]];
	}
}

void HistogramEqualization2(UINT8 *img, int w, int h)
{
	int pixel_tatal = w * h;
	for(int i = 0; i < pixel_tatal; i++){
		if(img[i] < 80)
			img[i] = img[i] * 1.5 + 10;
	}
}


//get lane line area
void mask(uint8_t *img, float _left_k1, float _left_b1, float _left_k2, float _left_b2,
		  float _right_k1, float _right_b1, float _right_k2, float _right_b2)
{
	int i, j;
	for (i = 0; i < HEIGHT_CAT; i++) {
		for (j = 0; j < WIDTH_CAT; j++) {
			if (FALSE == In_range(j, i, _left_k1, _left_b1, _left_k2, _left_b2,
								  _right_k1, _right_b1, _right_k2, _right_b2)) {
				img[i * WIDTH_CAT + j] = 0;
			}
		}
	}
}

int get_Otsu_threshold(ADAS_MEM_RANGE *p_share_mem, int w, int h)
{
	int gray_level_histogram[256] = {0}, pixel_num = w * h, threshold_best = 0;
	float gray_level_proportion[256] = {0};

	for (int line = 0; line < h; line++) {
		for (int cloumn = 0; cloumn < w; cloumn++) {
			gray_level_histogram[*(UINT8 *)(p_share_mem[0].va + line * w + cloumn)]++;
		}
	}

	for (int i = 0; i < 256; i++) {
		gray_level_proportion[i] = (float)gray_level_histogram[i] / pixel_num;
	}

	float w0, w1, u0, u1, u0temp, u1temp, deltaTmp, deltaMax = 0;
	for (int threshold = 1; threshold < 255; threshold++) {
		w0 = 0, w1 = 0, u0 = 0, u1 = 0, u0temp = 0, u1temp = 0, deltaTmp = 0;
		for (int gray_level = 0; gray_level < 256; gray_level++) {
			if (gray_level <= threshold) {
				w0 += gray_level_proportion[gray_level];
				u0temp += gray_level * gray_level_proportion[gray_level];
			} else {
				w1 += gray_level_proportion[gray_level];
				u1temp += gray_level * gray_level_proportion[gray_level];
			}
		}

		if (w0 && w1) {
			u0 = u0temp / w0;
			u1 = u1temp / w1;

			deltaTmp = (float)(w0 * w1 * ((u0 - u1) * (u0 - u1)));
			if (deltaTmp > deltaMax) {
				deltaMax = deltaTmp;
				threshold_best = threshold;
			}
		}
	}
	return threshold_best;
}



void get_Otsu_threshold_lane_divide(ADAS_MEM_RANGE *p_share_mem, int w, int h, int x_lane_divide, int * th_left, int * th_right, float k1, float k2, float k3, float k4, float b1, float b2, float b3, float b4)
{
	int gray_level_histogram_left[256] = {0}, gray_level_histogram_right[256] = {0}, pixel_num_left = x_lane_divide * h, pixel_num_right = (w - x_lane_divide) * h, threshold_best = 0;
	float gray_level_proportion_left[256] = {0}, gray_level_proportion_right[256] = {0};

	for (int line = 0; line < h; line++) {
		for (int column = 0; column < w; column++) {
			if(column < x_lane_divide){
				if(k1 * column + b1 < line && k2 * column + b2 > line)
					gray_level_histogram_left[*(UINT8 *)(p_share_mem[0].va + line * w + column)]++;
			}
			else{
				if(k3 * column + b3 > line && k4 * column + b4 < line)
					gray_level_histogram_right[*(UINT8 *)(p_share_mem[0].va + line * w + column)]++;
			}

		}
	}

	for (int i = 0; i < 256; i++) {
		gray_level_proportion_left[i] = (float)gray_level_histogram_left[i] / pixel_num_left;
		gray_level_proportion_right[i] = (float)gray_level_histogram_right[i] / pixel_num_right;
	}

	float w0, w1, u0, u1, u0temp, u1temp, deltaTmp, deltaMax = 0;
	for (int threshold = 10; threshold < 255; threshold++) {
		w0 = 0, w1 = 0, u0 = 0, u1 = 0, u0temp = 0, u1temp = 0, deltaTmp = 0;
		for (int gray_level = 0; gray_level < 256; gray_level++) {
			if (gray_level <= threshold) {
				w0 += gray_level_proportion_left[gray_level];
				u0temp += gray_level * gray_level_proportion_left[gray_level];
			} else {
				w1 += gray_level_proportion_left[gray_level];
				u1temp += gray_level * gray_level_proportion_left[gray_level];
			}
		}

		if (w0 && w1) {
			u0 = u0temp / w0;
			u1 = u1temp / w1;

			deltaTmp = (float)(w0 * w1 * ((u0 - u1) * (u0 - u1)));
			if (deltaTmp > deltaMax) {
				deltaMax = deltaTmp;
				threshold_best = threshold;
			}
		}
	}
	*th_left = threshold_best;

	deltaMax = 0;
	for (int threshold = 10; threshold < 255; threshold++) {
		w0 = 0, w1 = 0, u0 = 0, u1 = 0, u0temp = 0, u1temp = 0, deltaTmp = 0;
		for (int gray_level = 0; gray_level < 256; gray_level++) {
			if (gray_level <= threshold) {
				w0 += gray_level_proportion_right[gray_level];
				u0temp += gray_level * gray_level_proportion_right[gray_level];
			} else {
				w1 += gray_level_proportion_right[gray_level];
				u1temp += gray_level * gray_level_proportion_right[gray_level];
			}
		}

		if (w0 && w1) {
			u0 = u0temp / w0;
			u1 = u1temp / w1;

			deltaTmp = (float)(w0 * w1 * ((u0 - u1) * (u0 - u1)));
			if (deltaTmp > deltaMax) {
				deltaMax = deltaTmp;
				threshold_best = threshold;
			}
		}
	}
	*th_right = threshold_best;
}

// static int headstock_offst = 0;
#define HEADSTOCK_RECORD_NUM 50
#define LINEWAVERANGE 3
void headstock_offst_update(ADAS_MEM_RANGE *p_share_mem, int w, int h)
{
	static int headstock_record[HEADSTOCK_RECORD_NUM] = {0};
	static int headstock_record_location = 0;
	static BOOL headstock_record_ini = 1;
	if (headstock_record_ini) {
		for (int i = 0; i < HEADSTOCK_RECORD_NUM; i++) {
			headstock_record[i] = 120;
		}
	headstock_record_ini = 0;
	}

	int score[180 - LINEWAVERANGE] = {0}, brightness_threshold = 40;
	UINT8 temp = 0, temp2 = 0;

	brightness_threshold = get_Otsu_threshold(p_share_mem, w, h);
	// printf("brightness_threshold = %d\n", brightness_threshold);

	for (int line = 0; line < h - LINEWAVERANGE; line++) {
		for (int column = 0; column < w; column++) {
			temp = 0;
			temp2 = 0;
			for (int line2 = 0; line2 < LINEWAVERANGE; line2++) {
				temp = *(UINT8 *)(p_share_mem[0].va + (line + line2) * w + column);

				if (temp > brightness_threshold) {
					temp = 1;
					// *(UINT8 *)(p_share_mem[0].va + (line + line2) * w + column) = 255;
				} else {
					temp = 0;
					// *(UINT8 *)(p_share_mem[0].va + (line + line2) * w + column) = 0;
				}
				temp2 += temp;
			}
			if (temp2 > 0) {
				temp2 = 1;
			}
			score[line] += (int)temp2;
		}
	}
	int max_score = 0, max_score_index = 0;
	for (int i = 0; i < h - LINEWAVERANGE; i++) {
		if (score[i] > max_score) {
			max_score = score[i];
			max_score_index = i;
		}
		// printf("index = %d, score = %d\n", i, score[i]);
	}
	// printf("find_headstock max_score = %d, index = %d\n", max_score, max_score_index);
	headstock_record[headstock_record_location++] = max_score_index;
	if (headstock_record_location == HEADSTOCK_RECORD_NUM) {
		headstock_record_location = 0;
	}
	int mean_headstock_record = 0;
	for (int i = 0; i < HEADSTOCK_RECORD_NUM; i++) {
		mean_headstock_record += headstock_record[i];
		// printf("headstock_record[i] = %d\n", headstock_record[i]);
	}
	mean_headstock_record /= HEADSTOCK_RECORD_NUM;

	headstock_offst = mean_headstock_record - 120;
	// headstock_offst = 0;
	// printf("headstock_offst = %d\n", headstock_offst);

	// // static int write_file_headstock = 0;
	// // static char numInfo[256];
	// static int num_pic = 0;
	// static char src_img_path[64], ive_img_path[64];
	// // if (write_file_headstock++ == 10)
	// {
	//     snprintf(src_img_path, 64, "//mnt//sd//src_img_%08d.bmp", num_pic);
	//     snprintf(ive_img_path, 64, "//mnt//sd//ive_img_%08d.bmp", num_pic);
	//     writebmpfile(src_img_path, (UINT8 *)ADAS_share_mem[24].va, 80, 180);
	//     // writebmpfile(ive_img_path, (UINT8 *)ADAS_share_mem[25].va, 80, 180);
	//     writebmpfile(ive_img_path, (UINT8 *)p_share_mem[0].va, 80, 180);
	//     // snprintf(numInfo, 256, "num_pic = %08d, mean = %03d, headstock_offst = %d\r\n", num_pic, mean, (int)headstock_offst);
	//     // write_numInfo("//mnt//sd//out_img_info.txt", numInfo);
	//     // write_file_headstock = 0;
	//     num_pic++;
	// }

	return;
}

//hough lines detect
int hough_line_probabilistic(LaneDetector *_lanedetector, int w, int h, int rho,
							 int piece_num, int threshold, int lineLength, int lineGap, int linesMax, ADAS_MEM_RANGE *p_share_mem)
{

	int i, j;
	int offset, offy;

	float theta = PI / piece_num;
	int max_line_length = (int)ceil(sqrt(w * w + h * h));
	int numrho = max_line_length * 2 * rho + 5;

	//  int *_accum = new int[numangle*numrho]();
	//  unsigned char *_mask = new unsigned char[h*w];
	memset((void *)p_share_mem[2].va, 0, 350 * 1024);
	memset((void *)p_share_mem[1].va, 0, h * w);
	float *trigtab = (float *)p_share_mem[2].va;
	int8_t *_accum = (int8_t *)p_share_mem[2].va + 2048;
	uint8_t *_mask = (uint8_t *)p_share_mem[1].va;

	// int8_t *_accum = (int8_t*)malloc(piece_num*numrho*sizeof(int8_t));
	// uint8_t *_mask = (uint8_t*)malloc(h*w*sizeof(uint8_t));
	// uint8_t *_image = _lanedetector->graydata_cat;
	uint8_t *_image = (UINT8 *)p_share_mem[0].va;
	line_int_t *_lines = _lanedetector->lines;

	// memset(_accum, 0, piece_num*numrho*sizeof(int8_t));

	//  std::vector<float> trigtab(numangle * 2);
	// float* trigtab = (float*)malloc(piece_num * 2 * sizeof(float));

	for (int n = 0; n < piece_num; n++) {
		offset = n * 2;
		trigtab[offset] = cos(n * theta);
		trigtab[offset + 1] = sin(n * theta);
	}
	float *ttab = trigtab;
	uint8_t *mdata0 = _mask;
	memset((void *)p_share_mem[3].va, 0, w * h * sizeof(point_int_t));
	point_int_t *nzloc = (point_int_t *)p_share_mem[3].va;
	// point_int_t* nzloc = (point_int_t*)malloc(w*h*sizeof(point_int_t));
	point_int_t *p_nzloc = nzloc;
	int count = 0;
	int num_of_lines = 0;
	// stage 1. collect non-zero image points

	#if LINER_ENHANCE
	int thr = 50;
	#else
	int thr = 35;
	#endif

	for (i = 0; i < h; i++) {
		offy = i * w;
		for (j = 0; j < w; j++) {
			if (_image[offy + j] > thr) {
				_mask[offy + j] = (uint8_t)1;
				//              point_int_t temp_point = {j,i};
				(*p_nzloc).x = j;
				(*(p_nzloc++)).y = i;
				count++;
			} else {
				_mask[offy + j] = 0;
			}
		}
	}

	// stage 2. process all the points in random order

	int idx, max_val, max_n, k, x0, y0, dx0, dy0, xflag, i1, j1, gap, x, y, dx, dy;
	point_int_t point, line_end[2];
	float a, b;
	int8_t *adata = NULL;
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
		if (!mdata0[i * w + j]) {
			continue;
		}

		// update accumulator, find the most probable line
		for (int n = 0; n < piece_num; n++, adata += numrho) {
			int r = f2iround((j * ttab[n * 2] + i * ttab[n * 2 + 1]) * rho);
			r += max_line_length * rho;
			int val = ++(adata[r]);
			if (max_val < val) {
				max_val = val;
				max_n = n;
				// #if ADAS_DEBUG
				//              printf("count = %d n = %d max_val = %d\r\n", count, n, max_val);
				// #endif
			}
		}

		// if it is too "weak" candidate, continue with another point
		if (max_val < threshold) {
			continue;
		}

		// from the current point walk in each direction
		// along the found line and extract the line segment
		a = -ttab[max_n * 2 + 1];
		b = ttab[max_n * 2];

		if (0 == a) {
			a += 0.00000000001;
		}
		if (0 == b) {
			b += 0.00000000001;
		}
		x0 = j;
		y0 = i;
		if (fabs(a) > fabs(b)) {
			xflag = 1;
			dx0 = a > 0 ? 1 : -1;
			dy0 = (int)f2iround(b * (1 << shift) / fabs(a));
			y0 = (y0 << shift) + (1 << (shift - 1));
		} else {
			xflag = 0;
			dy0 = b > 0 ? 1 : -1;
			dx0 = (int)f2iround(a * (1 << shift) / fabs(b));
			x0 = (x0 << shift) + (1 << (shift - 1));
		}

		for (k = 0; k < 2; k++) {
			gap = 0;
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

				if (j1 < 0 || j1 >= w || i1 < 0 || i1 >= h) {
					break;
				}

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				if (mdata0[i1 * w + j1]) {
					gap = 0;
					line_end[k].y = i1;
					line_end[k].x = j1;
				} else if (++gap > lineGap) {
					break;
				}
			}
		}

		if ((abs(line_end[1].x - line_end[0].x) >= lineLength) || (abs(line_end[1].y - line_end[0].y) >= lineLength)) {
			good_line = TRUE;
		} else {
			good_line = FALSE;
		}

		for (k = 0; k < 2; k++) {
			x = x0;
			y = y0;
			dx = dx0;
			dy = dy0;

			if (k > 0) {
				dx = -dx, dy = -dy;
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

				// for each non-zero point:
				//    update line end,
				//    clear the mask element
				//    reset the gap
				if (mdata0[i1 * w + j1]) {
					if (TRUE == good_line) {
						adata = _accum;
						for (int n = 0; n < piece_num; n++, adata += numrho) {
							int r = f2iround((j1 * ttab[n * 2] + i1 * ttab[n * 2 + 1]) * rho);
							r += max_line_length * rho;
							adata[r]--;
						}
					}
					mdata0[i1 * w + j1] = 0;
				}

				if (i1 == line_end[k].y && j1 == line_end[k].x) {
					break;
				}
			}
		}

		if (TRUE == good_line) {
			//          line_int_t _lr = {line_end[0].x, line_end[0].y, line_end[1].x, line_end[1].y };
			//          _lines.push_back(_lr);
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
	//  delete[]_accum;
	//  delete[]_mask;

	// free(nzloc);
	// free(trigtab);
	// free(_mask);
	// free(_accum);

	return num_of_lines;
}

//////12.19_20.53,,tttttttttttttttttthis;tomorrow go go go on
//Distinguish left and right lines
void lineSeparation(LaneDetector *_lanedetector, ADAS_MEM_RANGE *p_share_mem)
{
	int j = 0;
	float slope_thresh = 0.25;
	memset((void *)p_share_mem[0].va, 0, 50 * sizeof(float));
	memset((void *)p_share_mem[1].va, 0, 50 * sizeof(line_int_t));
	float *slopes = (float *)p_share_mem[0].va;
	line_int_t *selected_lines = (line_int_t *)p_share_mem[1].va;
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
	int *left_num = &(_lanedetector->left_lines_num);
	int *right_num = &(_lanedetector->right_lines_num);

	// Calculate the slope of all the detected lines
	for (j = 0; j < _lanedetector->lines_num; j++) {
		// Basic algebra: slope = (y1 - y0)/(x1 - x0)
		float slope = (line[j].endy - line[j].starty) / (line[j].endx - line[j].startx + 0.00001);
		// float bbb = line[j].starty - slope*line[j].startx;

		//      DBG_DUMP("slope = %f\r\n", slope);
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
	for (j = 0; j < slopes_num; j++) {
		// Condition to classify line as left side or right side
		if ((slopes[j] > 0) && (slopes[j] < 10) && (selected_lines[j].endx >= (_lanedetector->center_x - 15)) && (selected_lines[j].startx >= (_lanedetector->center_x - 15))) {
			if (TRUE == is_reliable_pre_lane(pre_right) && pre_right->k < 0.8) {
				if (fabs(pre_right->k - slopes[j]) < 0.3) {
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
		} else if ((slopes[j] < 0) && (slopes[j] > -10) && (selected_lines[j].endx <= (_lanedetector->center_x + 15)) && (selected_lines[j].startx <= (_lanedetector->center_x + 15))) {
			if (TRUE == is_reliable_pre_lane(pre_left) && pre_left->k > -0.8) {
				if (fabs(pre_left->k - slopes[j]) < 0.3) {
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
	for (int i = 0; i < _lanedetector->left_lines_num; i++) {
		printf("x1 = %d  y1 = %d  x2 = %d  y2 = %d\r\n", left_line[i].startx, left_line[i].starty, left_line[i].endx, left_line[i].endy);
	}
	printf("########################\r\n");
	printf("#####right_lines_num = %d\r\n", _lanedetector->right_lines_num);
	for (int i = 0; i < _lanedetector->right_lines_num; i++) {
		printf("x1 = %d  y1 = %d  x2 = %d  y2 = %d\r\n", right_line[i].startx, right_line[i].starty, right_line[i].endx, right_line[i].endy);
	}
	printf("########################\r\n");
	#endif
}

int DBSCAN(line_int_t *_line, float *line_k, int *order, int num)
{
	int i, j;
	float temp;
	int class_num[50];
	int lane_num = 0;

	for (i = 0; i < num; i++) {
		line_k[i] = (float)fabs((_line[i].endy - _line[i].starty) / (_line[i].endx - _line[i].startx + 0.00001));
		order[i] = i;
	}

	for (i = 0; i < num - 1; i++) {
		for (j = i + 1; j < num; j++) {
			if (line_k[i] < line_k[j]) {
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

	if (line_k[0] > 1 && (line_k[0] - line_k[num - 1]) > 0.5) {
		lane_num = (num >= 4 ? 4 : num);
	} else {
		for (i = 1; i < num; i++) {
			if ((line_k[i - 1] - line_k[i]) > 0.06) {
				class_num[i] = class_num[i - 1] + 1;
			} else {
				class_num[i] = class_num[i - 1];
			}
		}

		for (i = 1; i < num; i++) {
			if (1 == class_num[i - 1] && 2 == class_num[i]) {
				lane_num = (i >= 4 ? 4 : i);
				break;
			}
		}
		if (num == i) {
			lane_num = (num >= 4 ? 4 : num);
		}
	}

	return lane_num;
}

//Fitting left and right lane lines
void regression(LaneDetector *_lanedetector, Lane *left_lane, Lane *right_lane, ADAS_MEM_RANGE *p_share_mem)
{
	// point_int_t *right_pts = NULL, *left_pts = NULL;
	int j;

	line_int_t *left_line = _lanedetector->left_lines;
	line_int_t *right_line = _lanedetector->right_lines;
	int left_num = _lanedetector->left_lines_num;
	int right_num = _lanedetector->right_lines_num;

	memset((void *)p_share_mem[0].va, 0, 120 * sizeof(point_int_t));
	memset((void *)p_share_mem[1].va, 0, 120 * sizeof(point_int_t));
	point_int_t *left_pts = (point_int_t *)p_share_mem[0].va;
	point_int_t *right_pts = (point_int_t *)p_share_mem[1].va;

	memset((void *)p_share_mem[2].va, 0, 50 * sizeof(float));
	memset((void *)p_share_mem[3].va, 0, 50 * sizeof(int));
	memset((void *)p_share_mem[4].va, 0, 50 * sizeof(float));
	memset((void *)p_share_mem[5].va, 0, 50 * sizeof(int));
	float *right_line_k = (float *)malloc(right_num * sizeof(float));
	int *right_order = (int *)malloc(right_num * sizeof(int));
	float *left_line_k = (float *)malloc(left_num * sizeof(float));
	int *left_order = (int *)malloc(left_num * sizeof(int));

	// If left lines are being detected, fit a line using all the init and final points of the lines
	if (left_num > 1) {
		// left_pts = (point_int_t*)malloc(left_num*2*sizeof(point_int_t));
		int l_num = DBSCAN(left_line, left_line_k, left_order, left_num);

		for (j = 0; j < l_num; j++) {
			left_pts[2 * j].x = left_line[left_order[j]].startx;
			left_pts[2 * j].y = left_line[left_order[j]].starty;
			left_pts[2 * j + 1].x = left_line[left_order[j]].endx;
			left_pts[2 * j + 1].y = left_line[left_order[j]].endy;
		}
		LineFitLeastSquares(left_pts, l_num * 2, left_lane);
	} else if (1 == left_num) {
		left_pts[0].x = left_line[0].startx;
		left_pts[0].y = left_line[0].starty;
		left_pts[1].x = left_line[0].endx;
		left_pts[1].y = left_line[0].endy;
		LineFitLeastSquares(left_pts, 2, left_lane);
	}

	// If right lines are being detected, fit a line using all the init and final points of the lines
	if (right_num > 1) {
		// right_pts = (point_int_t*)malloc(right_num*2*sizeof(point_int_t));
		int r_num = DBSCAN(right_line, right_line_k, right_order, right_num);

		for (j = 0; j < r_num; j++) {
			right_pts[2 * j].x = right_line[right_order[j]].startx;
			right_pts[2 * j].y = right_line[right_order[j]].starty;
			right_pts[2 * j + 1].x = right_line[right_order[j]].endx;
			right_pts[2 * j + 1].y = right_line[right_order[j]].endy;
		}
		LineFitLeastSquares(right_pts, r_num * 2, right_lane);
	} else if (1 == right_num) {
		right_pts[0].x = right_line[0].startx;
		right_pts[0].y = right_line[0].starty;
		right_pts[1].x = right_line[0].endx;
		right_pts[1].y = right_line[0].endy;
		LineFitLeastSquares(right_pts, 2, right_lane);
	}

	// if(NULL != left_pts){
	//  free(left_pts);
	// }
	// if(NULL != right_pts){
	//  free(right_pts);
	// }
}

BOOL reduce_false_alarms(int lx, int rx)
{
	BOOL ret = TRUE;
	if ((lx + rx) < 20) {
		ret = FALSE;
	}

	return ret;
}

#if 0
BOOL limit_judgment(char *mem, int length)
{

	int count = 0, i = 0;
	for (i = 0; i < length; i++) {
		if (mem[i] != 0) {
			count++;
		}
	}
	if (count * 1.0 / length > 0.75) {
		return TRUE;
	} else {
		return FALSE;
	}
}
#endif

#if LANE_IMG_SAVE
void draw_line(Lane *left, Lane *right, void *arg)
{
	ADAS_MEM_RANGE *p_share_mem = (ADAS_MEM_RANGE *)arg;
	HD_GFX_DRAW_RECT rectt[4];
	HD_RESULT ret;

	memset(rectt, 0, sizeof(rectt));

	if (left->Confidence >= LANE_CONFIDENCE) {
		rectt[0].dst_img.dim.w = WIDTH_CAT;
		rectt[0].dst_img.dim.h = HEIGHT_CAT;
		rectt[0].dst_img.format = HD_VIDEO_PXLFMT_Y8;
		rectt[0].dst_img.p_phy_addr[0] = p_share_mem[0].pa;
		rectt[0].dst_img.lineoffset[0] = WIDTH_CAT;
		rectt[0].color = 0x0;
		rectt[0].rect.x = (UINT32)((10 - left->b) / left->k);
		rectt[0].rect.y = (UINT32)10;
		rectt[0].rect.w = 6;
		rectt[0].rect.h = 6;
		rectt[0].type = HD_GFX_RECT_SOLID;

		rectt[1].dst_img.dim.w = WIDTH_CAT;
		rectt[1].dst_img.dim.h = HEIGHT_CAT;
		rectt[1].dst_img.format = HD_VIDEO_PXLFMT_Y8;
		rectt[1].dst_img.p_phy_addr[0] = p_share_mem[0].pa;
		rectt[1].dst_img.lineoffset[0] = WIDTH_CAT;
		rectt[1].color = 0x0;
		rectt[1].rect.x = (UINT32)((70 - left->b) / left->k);
		rectt[1].rect.y = (UINT32)70;
		rectt[1].rect.w = 6;
		rectt[1].rect.h = 6;
		rectt[1].type = HD_GFX_RECT_SOLID;

		ret = hd_gfx_draw_rect(&rectt[0]);
		if (ret != HD_OK) {
			DBG_ERR("hd_gfx_draw_line left fail=%d\n", ret);
			goto exit;
		}

		ret = hd_gfx_draw_rect(&rectt[1]);
		if (ret != HD_OK) {
			DBG_ERR("hd_gfx_draw_line left fail=%d\n", ret);
			goto exit;
		}
	}

	if (right->Confidence >= LANE_CONFIDENCE) {
		rectt[2].dst_img.dim.w = WIDTH_CAT;
		rectt[2].dst_img.dim.h = HEIGHT_CAT;
		rectt[2].dst_img.format = HD_VIDEO_PXLFMT_Y8;
		rectt[2].dst_img.p_phy_addr[0] = p_share_mem[0].pa;
		rectt[2].dst_img.lineoffset[0] = WIDTH_CAT;
		rectt[2].color = 0x0;
		rectt[2].rect.x = (UINT32)((10 - right->b) / right->k);
		rectt[2].rect.y = (UINT32)10;
		rectt[2].rect.w = 6;
		rectt[2].rect.h = 6;
		rectt[2].type = HD_GFX_RECT_SOLID;

		rectt[3].dst_img.dim.w = WIDTH_CAT;
		rectt[3].dst_img.dim.h = HEIGHT_CAT;
		rectt[3].dst_img.format = HD_VIDEO_PXLFMT_Y8;
		rectt[3].dst_img.p_phy_addr[0] = p_share_mem[0].pa;
		rectt[3].dst_img.lineoffset[0] = WIDTH_CAT;
		rectt[3].color = 0x0;
		rectt[3].rect.x = (UINT32)((70 - right->b) / right->k);
		rectt[3].rect.y = (UINT32)70;
		rectt[3].rect.w = 6;
		rectt[3].rect.h = 6;
		rectt[3].type = HD_GFX_RECT_SOLID;

		ret = hd_gfx_draw_rect(&rectt[2]);
		if (ret != HD_OK) {
			DBG_ERR("hd_gfx_draw_line right fail=%d\n", ret);
			goto exit;
		}
		ret = hd_gfx_draw_rect(&rectt[3]);
		if (ret != HD_OK) {
			DBG_ERR("hd_gfx_draw_line right fail=%d\n", ret);
			goto exit;
		}
	}

exit:
	return;
}

#endif

#if 0 //XJW
HD_RESULT video_frame848to640(HD_VIDEO_FRAME *video, ADAS_MEM_RANGE *p_share_mem)
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

	scale.dst_img.dim.w            = ADAS_IMG_WIDTH;
	scale.dst_img.dim.h            = ADAS_IMG_HEIGHT;
	scale.dst_img.format           = HD_VIDEO_PXLFMT_YUV420;
	scale.dst_img.p_phy_addr[0]    = p_share_mem[0].pa;
	scale.dst_img.p_phy_addr[1]    = p_share_mem[0].pa + ADAS_IMG_BUF_SIZE;
	scale.dst_img.lineoffset[0]    = ADAS_IMG_WIDTH;
	scale.dst_img.lineoffset[1]    = ADAS_IMG_WIDTH;
	scale.dst_region.x             = 0;
	scale.dst_region.y             = 0;
	scale.dst_region.w             = ADAS_IMG_WIDTH;
	scale.dst_region.h             = ADAS_IMG_HEIGHT;

	scale.quality                  = HD_GFX_SCALE_QUALITY_BILINEAR;

	ret = hd_gfx_scale_sw(&scale);
	if (ret != HD_OK) {
		DBG_ERR("video_frame848to640 fail=%d\n", ret);
	}

	return ret;
}

#endif

HD_RESULT img1920toimg320(HD_VIDEO_FRAME *video, ADAS_MEM_RANGE *p_share_mem)
{
	HD_RESULT ret = HD_OK;
	HD_GFX_SCALE scale;

	memset(&scale, 0, sizeof(HD_GFX_SCALE));
	scale.src_img.dim.w = video->dim.w;
	scale.src_img.dim.h = video->dim.h;
	scale.src_img.format = video->pxlfmt;
	scale.src_img.p_phy_addr[0] = video->phy_addr[0];
	scale.src_img.p_phy_addr[1] = video->phy_addr[1];
	scale.src_img.lineoffset[0] = video->loff[0];
	scale.src_img.lineoffset[1] = video->loff[1];
	scale.src_region.x = 0;
	scale.src_region.y = 0;
	scale.src_region.w = video->dim.w;
	scale.src_region.h = video->dim.h;

	scale.dst_img.dim.w = MD_IMG_WIDTH;
	scale.dst_img.dim.h = MD_IMG_HEIGHT;
	scale.dst_img.format = HD_VIDEO_PXLFMT_YUV420;
	scale.dst_img.p_phy_addr[0] = p_share_mem[0].pa;
	scale.dst_img.p_phy_addr[1] = p_share_mem[1].pa;
	scale.dst_img.lineoffset[0] = MD_IMG_WIDTH;
	scale.dst_img.lineoffset[1] = MD_IMG_WIDTH;
	scale.dst_region.x = 0;
	scale.dst_region.y = 0;
	scale.dst_region.w = MD_IMG_WIDTH;
	scale.dst_region.h = MD_IMG_HEIGHT;

	scale.quality = HD_GFX_SCALE_QUALITY_BILINEAR;

	ret = hd_gfx_scale_sw(&scale);
	if (ret != HD_OK) {
		DBG_ERR("img1920toimg320 fail=%d\n", ret);
	}

	return ret;
}

// HD_RESULT img_crop(ADAS_MEM_RANGE *src_mem, ADAS_MEM_RANGE *dst_mem)
// {
//     HD_RESULT ret = HD_OK;
//     HD_GFX_SCALE crop;

//     memset(&crop, 0, sizeof(HD_GFX_SCALE));
//     crop.src_img.dim.w              = MD_IMG_WIDTH;
//     crop.src_img.dim.h              = MD_IMG_HEIGHT;
//     crop.src_img.format             = HD_VIDEO_PXLFMT_YUV420;
//     crop.src_img.p_phy_addr[0]      = src_mem[0].pa;
//     crop.src_img.p_phy_addr[1]      = src_mem[1].pa;
//     crop.src_img.lineoffset[0]      = MD_IMG_WIDTH;
//     crop.src_img.lineoffset[1]      = MD_IMG_WIDTH;
//     crop.src_region.x               = 0;
//     crop.src_region.y               = 0;
//     crop.src_region.w               = MD_IMG_WIDTH;
//     crop.src_region.h               = MD_CROP_HEIGHT;

//     crop.dst_img.dim.w              = MD_IMG_WIDTH;
//     crop.dst_img.dim.h              = MD_CROP_HEIGHT;
//     crop.dst_img.format             = HD_VIDEO_PXLFMT_Y8;
//     crop.dst_img.p_phy_addr[0]       = dst_mem->pa;
//     crop.dst_img.lineoffset[0]      = MD_IMG_WIDTH;
//     crop.dst_region.x               = 0;
//     crop.dst_region.y               = 0;
//     crop.dst_region.w               = MD_IMG_WIDTH;
//     crop.dst_region.h               = MD_CROP_HEIGHT;

//     // crop.quality                    = HD_GFX_SCALE_QUALITY_BILINEAR;

//     ret = hd_gfx_scale_sw(&crop);
//     if(ret != HD_OK) {
//         DBG_ERR("img_crop fail=%d\n", ret);
//     }

//     return ret;
// }

// UINT32 get_front_car_pos(VENDOR_AI_POSTPROC_RESULT_INFO *net_result, Lane* left_lane, Lane* right_lane, int lane_center)
// {
//     float k1, k2, b1, b2;
//     UINT32 i, ii = 100, min_x = 640, center, front_car_pos = 0;

//     if(left_lane->Confidence>=96 && right_lane->Confidence>=96) {
//         center = lane_center + X_CAT;
//         k1 = left_lane->k;
//         k2 = right_lane->k;
//         b1 = left_lane->b+Y_CAT-X_CAT*k1+15*fabs(left_lane->k);
//         b2 = right_lane->b+Y_CAT-X_CAT*k2+15*fabs(right_lane->k);
//     } else {
//         center = WIDTH/2;
//         k1 = -1;
//         k2 = 1;
//         b1 = 503;
//         b2 = -137;
//     }

//     for(i=0; i < net_result->result_num; i++) {
//         VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[i];
//         if(p_rslt->no[0]==1&&p_rslt->score[0]>0.7) {
//             float w = p_rslt->w * ADAS_IMG_WIDTH + 0.01;
//             float h = p_rslt->h * HEIGHT;
//             if(w>25 && w<400 && (h/w)>0.5 && (h/w)<1.5 && h<300) {
//                 int pos_x = f2iround(w/2 + p_rslt->x * ADAS_IMG_WIDTH);
//                 if((UINT32)abs(pos_x-center)<min_x) {
//                     min_x = abs(pos_x-center);
//                     ii = i;
//                 }
//             }

//         }
//     }

//     if(ii < net_result->result_num) {
//         VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[ii];
//         int pos_x = f2iround((p_rslt->x + p_rslt->w/2)*ADAS_IMG_WIDTH);
//         int pos_y = f2iround((p_rslt->y + p_rslt->h)*ADAS_IMG_HEIGHT);
//         // printf("FCWS: pos_x = %d  pos_y = %d\r\n", pos_x, pos_y);
//         if(pos_y > 300) {
//             if(pos_x > 203 && pos_x < 437) {
//                 front_car_pos = pos_y;
//             }
//         } else {
//             if(Is_front_car(k1, k2, b1, b2, pos_x, pos_y)) {
//                 front_car_pos = pos_y;
//             }
//         }
//     }

//     return front_car_pos;

// }

// UINT32 get_front_car_pos_nolane(VENDOR_AI_POSTPROC_RESULT_INFO *net_result)
// {
//     float k1, k2, b1, b2;
//     UINT32 i, ii = 100, min_x = 640, center, front_car_pos = 0;

//     center = WIDTH/2;
//     k1 = -1;
//     k2 = 1;
//     b1 = 503;
//     b2 = -137;

//     for(i=0; i < net_result->result_num; i++) {
//         VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[i];
//         if(p_rslt->no[0]==1&&p_rslt->score[0]>0.7) {
//             // float w = p_rslt->w * ADAS_IMG_WIDTH + 0.01;
//             // float h = p_rslt->h * ADAS_IMG_HEIGHT;
//             float w = p_rslt->w * CAR_CROP_WIDTH + 0.01;
//             float h = p_rslt->h * CAR_CROP_HEIGHT;
//             if(w>20 && w<300 && (h/w)>0.5 && (h/w)<1.6 && h<160) {
//                 // int pos_x = f2iround(w/2 + p_rslt->x * ADAS_IMG_WIDTH);
//                 int pos_x = f2iround(w/2 + p_rslt->x * CAR_CROP_WIDTH+CAR_CROP_X_BIAS);
//                 if((UINT32)abs(pos_x-center)<min_x) {
//                     min_x = abs(pos_x-center);
//                     ii = i;
//                 }
//             }

//         }
//     }

//     if(ii < net_result->result_num) {
//         VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[ii];
//         // int pos_x = f2iround((p_rslt->x + p_rslt->w/2)*ADAS_IMG_WIDTH);
//         // int pos_y = f2iround((p_rslt->y + p_rslt->h)*ADAS_IMG_HEIGHT);
//         int pos_x = f2iround((p_rslt->x + p_rslt->w/2)*CAR_CROP_WIDTH+CAR_CROP_X_BIAS);
//         int pos_y = f2iround((p_rslt->y + p_rslt->h)*CAR_CROP_HEIGHT+CAR_CROP_Y_BIAS);
//         // printf("FCWS: pos_x = %d  pos_y = %d\r\n", pos_x, pos_y);
//         if(pos_y > 300) {
//             if(pos_x > 203 && pos_x < 437) {
//                 front_car_pos = pos_y;
//             }
//         } else {
//             if(Is_front_car(k1, k2, b1, b2, pos_x, pos_y)) {
//                 front_car_pos = pos_y;
//             }
//         }

//     }

//     return front_car_pos;
// }

// BOOL have_pre_car(VENDOR_AI_POSTPROC_RESULT_INFO *net_result, int *car_index)
// {
//     UINT32 i, iii = 100, min_x = 640, center = 320;
//     float k11 = -0.66667;
//     float k22 = 0.66667;
//     float b11 = 400;
//     float b22 = -20;
//     BOOL flag = FALSE;
//     for(i=0; i < net_result->result_num; i++) {
//         VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[i];
//         if(p_rslt->no[0]==1 && p_rslt->score[0]>0.7) {
//             // float w = p_rslt->w * ADAS_IMG_WIDTH + 0.01;
//             // float h = p_rslt->h * ADAS_IMG_HEIGHT;
//             float w = p_rslt->w * CAR_CROP_WIDTH + 0.01;
//             float h = p_rslt->h * CAR_CROP_HEIGHT;
//             if(w>50 && w<320 && h<180 && (h/w)>0.5 && (h/w)<1.5) {
//                 // int pos_x = f2iround(w/2 + p_rslt->x * ADAS_IMG_WIDTH);
//                 int pos_x = f2iround(w/2 + p_rslt->x * CAR_CROP_WIDTH+CAR_CROP_X_BIAS);
//                 if((UINT32)abs(pos_x-center)<min_x) {
//                     min_x = (UINT32)abs(pos_x-center);
//                     iii = i;
//                 }
//             }

//         }
//     }//for

//     if(iii < net_result->result_num) {
//         VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[iii];
//         // int pos_x = f2iround((p_rslt->x + p_rslt->w/2)*ADAS_IMG_WIDTH);
//         // int pos_y = f2iround((p_rslt->y + p_rslt->h)*ADAS_IMG_HEIGHT);
//         int pos_x = f2iround((p_rslt->x + p_rslt->w/2)*CAR_CROP_WIDTH+CAR_CROP_X_BIAS);
//         int pos_y = f2iround((p_rslt->y + p_rslt->h)*CAR_CROP_HEIGHT+CAR_CROP_Y_BIAS);
//         printf("SNG: pos_x = %d  pos_y = %d\r\n", pos_x, pos_y);
//         if(TRUE == Is_front_car(k11, k22, b11, b22, pos_x, pos_y)) {
//             *car_index = iii;
//             flag = TRUE;
//         }
//     }

//     return flag;
// }

UINT32 get_pre_car_index(VENDOR_AI_POSTPROC_RESULT_INFO *net_result)
{
	// printf("%s, result_num=%lu\n", __func__, net_result->result_num);
	float k1, k2, b1, b2 /*, k3, k4, b3, b4*/, min_x = 640.0, center;
	UINT32 car_index = 100, index = 0, i;
	UINT32 order[5] = {0};

	// if(1 == LINE_DETECT && left_lane.Confidence>=90 && right_lane.Confidence>=90){
	//      center = lanedetector.center_x + X_CAT;
	//      k1 = left_lane.k;
	//      k2 = right_lane.k;
	//      b1 = left_lane.b+Y_CAT-X_CAT*k1;
	//      b2 = right_lane.b+Y_CAT-X_CAT*k2;
	// }else{
	center = WIDTH / 2;
	k1 = 1.5;
	k2 = -1.5;
	b1 = -270;
	b2 = 690;

	// k3 = -2;
	// k4 = 2;
	// b3 = 786;
	// b4 = -494;
	// }

	// if(ii < net_result->result_num){
	// for (i = 0; i < net_result->result_num && index < 5; i++)
	// {
	//     VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[i];
	//     if (p_rslt->no[0] == 1 && p_rslt->score[0] > 0.7)
	//     {
	//         float w = p_rslt->w * 320 + 0.01;
	//         float h = p_rslt->h * 180;
	//         if (w > 15 && (h / w) > 0.5 && (h / w) < 1.5)
	//         {
	//             float pos_x = (p_rslt->x + p_rslt->w / 2) * 320 + 160;
	//             float pos_y = (p_rslt->y + p_rslt->h) * 180 + 120;
	//             // printf("FCWS: pos_x = %d  pos_y = %d\r\n", pos_x, pos_y);
	//             if (pos_y >= Y_CAT)
	//             {
	//                 if (Is_front_car(k1, k2, b1, b2, pos_x, pos_y))
	//                 {
	//                     // front_car_pos = pos_y;
	//                     order[index] = i;
	//                     index++;
	//                 }
	//             }
	//             else if (pos_y < Y_CAT && pos_y > (Y_CAT - 30))
	//             {
	//                 if (Is_front_car(k3, k4, b3, b4, pos_x, pos_y))
	//                 {
	//                     // front_car_pos = pos_y;
	//                     order[index] = i;
	//                     index++;
	//                 }
	//             }
	//         }
	//     }
	// }
	for (i = 0; i < net_result->result_num && index < 5; i++) {
		VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[i];
		// printf("\tidx=%d, conf=%f, cls=%lu, %f, %f, %f, %f\n", i, p_rslt->score[0], p_rslt->no[0],p_rslt->x, p_rslt->y, p_rslt->w, p_rslt->h);
		if (p_rslt->no[0] == 2 && p_rslt->score[0] > 0.9 && p_rslt->w < 0.5 && p_rslt->h < 0.5) {
			// printf("get_pre_car_index p_rslt->score[0] = %f, p_rslt->x = %f, p_rslt->y = %f, p_rslt->w = %f, p_rslt->h = %f\n", p_rslt->score[0], p_rslt->x * 640, p_rslt->y * 360, p_rslt->w * 640, p_rslt->h * 360);
			// float w = p_rslt->w * 320 + 0.01;
			// float h = p_rslt->h * 180;
			float w = p_rslt->w * CAR_CROP_WIDTH + 0.01;
			float h = p_rslt->h * CAR_CROP_HEIGHT;
			// printf("\t\t w=%f, h=%f\n", w, h);
			if (w > 15 && (h / w) > 0.5 && (h / w) < 1.5) {
				// float pos_x = (p_rslt->x + p_rslt->w / 2) * 320 + 160;
				// float pos_y = (p_rslt->y + p_rslt->h) * 180 + 120;
				float pos_x = (p_rslt->x + p_rslt->w / 2) * CAR_CROP_WIDTH * 640 / 1920 + CAR_CROP_X_BIAS * 640 / 1920;
				float pos_y = (p_rslt->y + p_rslt->h) * CAR_CROP_HEIGHT * 360 / 1080 + CAR_CROP_Y_BIAS * 360 / 1080;
				// printf("\t\tFCWS: pos_x = %f  pos_y = %f\r\n", pos_x, pos_y);
				if (pos_y >= 255) {
					if (Is_front_car(k1, k2, b1, b2, pos_x, pos_y)) {
						// printf("\t\t%d in front car\n", index);
						// front_car_pos = pos_y;
						order[index] = i;
						index++;
					}
				}
				// else if (pos_y < Y_CAT && pos_y > (Y_CAT - 30))
				// {
				//     if (Is_front_car(k3, k4, b3, b4, pos_x, pos_y))
				//     {
				//         // front_car_pos = pos_y;
				//         order[index] = i;
				//         index++;
				//     }
				// }
			}
		}
	}

	// for(i=0; i < net_result->result_num; i++){
	for (i = 0; i < index; i++) {
		VENDOR_AI_POSTPROC_RESULT *p_rslt = &net_result->p_result[order[i]];
		// float pos_x = (p_rslt->x + p_rslt->w / 2) * 320 + 160;
		float pos_x = (p_rslt->x + p_rslt->w / 2) * CAR_CROP_WIDTH * 640 / 1920 + CAR_CROP_X_BIAS * 640 / 1920;
		if (fabs(pos_x - center) < min_x) {
			min_x = fabs(pos_x - center);
			car_index = order[i];
		}
	}

	return car_index;
}

void md_mem_clean(ADAS_MEM_RANGE *p_share_mem)
{
	for (UINT8 i = 0; i < 11; i++) {
		memset((void *)p_share_mem[i].va, 0, p_share_mem[i].size);
	}
}

void my_libmd_param_set(VENDOR_MD_TRIGGER_PARAM *_md_trig_param, LIB_MD_MDT_LIB_INFO *_mdt_lib_param,
						HD_IRECT *rect, ADAS_MEM_RANGE *p_share_mem, UINT32 _percentage)
{
	HD_RESULT ret = HD_OK;
	_md_trig_param->is_nonblock = 0;
	_md_trig_param->time_out_ms = 0;
	// LibMD motion detection info
	_mdt_lib_param->mdt_info.libmd_enabled = 1;
	_mdt_lib_param->mdt_info.phy_md_x_num = MD_IMG_WIDTH;
	_mdt_lib_param->mdt_info.phy_md_y_num = MD_IMG_HEIGHT;
	_mdt_lib_param->mdt_info.phy_md_rst.p_md_bitmap = (UINT8 *)p_share_mem[10].va;
	_mdt_lib_param->mdt_info.phy_md_rst.md_bitmap_sz = MD_IMG_WIDTH * MD_IMG_HEIGHT;
	if ((ret = lib_md_set(0, LIB_MD_MOTION_DETECT_INFO, &_mdt_lib_param->mdt_info)) != HD_OK) {
		DBG_ERR("lib_md_set enable fail, error code = %d\r\n", ret);
		return;
	}
	// LibMD function enable
	_mdt_lib_param->mdt_enable.globel_md_alarm_detect_en = 0;
	_mdt_lib_param->mdt_enable.subregion_md_alarm_detect_en = 1;
	_mdt_lib_param->mdt_enable.scene_change_alarm_detect_en = 0;
	if ((ret = lib_md_set(0, LIB_MD_AP_ENABLE_PARAM, &_mdt_lib_param->mdt_enable)) != HD_OK) {
		DBG_ERR("lib_md_set enable fail, error code = %d\r\n", ret);
		return;
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
	_mdt_lib_param->mdt_subregion_param.sub_region[0].x_end = rect->x + rect->w; //160
	_mdt_lib_param->mdt_subregion_param.sub_region[0].y_end = rect->y + rect->h; //90
	_mdt_lib_param->mdt_subregion_param.sub_region[0].alarm_th = _percentage;
	if ((ret = lib_md_set(0, LIB_MD_AP_SUBREGION_MOTION_ALARM_PARAM, &_mdt_lib_param->mdt_subregion_param)) != HD_OK) {
		DBG_ERR("lib_md_set sub-region motion alarm param fail, error code = %d\r\n", ret);
		return;
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

void bc_reorgS1(UINT8 *inputS, UINT8 *outputS, UINT32 width, UINT32 height)
{
	UINT32 i, j, count, size;
	count = 0;
	size = width * height;
	for (j = 0; j < MDBC_ALIGN(size, 8) / 8; j++) {
		UINT8 c = inputS[j];
		for (i = 0; i < 8; i++) {
			if (count < size) {
				if ((c & 0x1) == 1) {
					outputS[count] = 255;
				} else {
					outputS[count] = 0;
				}
				// outputS[count] = c & 0x1;
				c = c >> 1;
				count++;
			}
		}
	}
}

void md_set_para_default_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
	mdbc_parm->MdmatchPara.lbsp_th = 0xa;
	mdbc_parm->MdmatchPara.d_colour = 0xf;
	mdbc_parm->MdmatchPara.r_colour = 0x1e;
	mdbc_parm->MdmatchPara.d_lbsp = 0x3;
	mdbc_parm->MdmatchPara.r_lbsp = 0x5;
	mdbc_parm->MdmatchPara.model_num = 0x8;
	mdbc_parm->MdmatchPara.t_alpha = 0x33;
	mdbc_parm->MdmatchPara.dw_shift = 0x4;
	mdbc_parm->MdmatchPara.dlast_alpha = 0x28;
	mdbc_parm->MdmatchPara.min_match = 2;
	mdbc_parm->MdmatchPara.dlt_alpha = 0xa;
	mdbc_parm->MdmatchPara.dst_alpha = 0x28;
	mdbc_parm->MdmatchPara.uv_thres = 0x14;
	mdbc_parm->MdmatchPara.s_alpha = 0x28;
	mdbc_parm->MdmatchPara.dbg_lumDiff = 0x0;
	mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

	mdbc_parm->MorPara.th_ero = 0x8;
	mdbc_parm->MorPara.th_dil = 0x0;
	mdbc_parm->MorPara.mor_sel0 = 0x3;
	mdbc_parm->MorPara.mor_sel1 = 0x2;
	mdbc_parm->MorPara.mor_sel2 = 0x1;
	mdbc_parm->MorPara.mor_sel3 = 0x0;

	mdbc_parm->UpdPara.minT = 0x4;
	mdbc_parm->UpdPara.maxT = 0x80;
	mdbc_parm->UpdPara.maxFgFrm = 0xff;
	mdbc_parm->UpdPara.deghost_dth = 0xa;
	mdbc_parm->UpdPara.deghost_sth = 0xf0;
	mdbc_parm->UpdPara.stable_frm = 0x78;
	mdbc_parm->UpdPara.update_dyn = 0x80;
	mdbc_parm->UpdPara.va_distth = 10;
	mdbc_parm->UpdPara.t_distth = 24;
	mdbc_parm->UpdPara.dbg_frmID = 0x0;
	mdbc_parm->UpdPara.dbg_frmID_en = 0x0;
	mdbc_parm->UpdPara.dbg_rnd = 0x0;
	mdbc_parm->UpdPara.dbg_rnd_en = 0x0;
}

void md_set_para_high_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
	mdbc_parm->MdmatchPara.lbsp_th = 0x0;
	mdbc_parm->MdmatchPara.d_colour = 10;
	mdbc_parm->MdmatchPara.r_colour = 0x1e;
	mdbc_parm->MdmatchPara.d_lbsp = 3;
	mdbc_parm->MdmatchPara.r_lbsp = 5;
	mdbc_parm->MdmatchPara.model_num = 0x8;
	mdbc_parm->MdmatchPara.t_alpha = 0x33;
	mdbc_parm->MdmatchPara.dw_shift = 0x4;
	mdbc_parm->MdmatchPara.dlast_alpha = 0x28;
	mdbc_parm->MdmatchPara.min_match = 2;
	mdbc_parm->MdmatchPara.dlt_alpha = 0xa;
	mdbc_parm->MdmatchPara.dst_alpha = 0x28;
	mdbc_parm->MdmatchPara.uv_thres = 20;
	mdbc_parm->MdmatchPara.s_alpha = 0x28;
	mdbc_parm->MdmatchPara.dbg_lumDiff = 0x0;
	mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

	mdbc_parm->MorPara.th_ero = 0x8;
	mdbc_parm->MorPara.th_dil = 0x0;
	mdbc_parm->MorPara.mor_sel0 = 0x0;
	mdbc_parm->MorPara.mor_sel1 = 0x1;
	mdbc_parm->MorPara.mor_sel2 = 0x2;
	mdbc_parm->MorPara.mor_sel3 = 0x3;

	mdbc_parm->UpdPara.minT = 0x8;
	mdbc_parm->UpdPara.maxT = 0x80;
	mdbc_parm->UpdPara.maxFgFrm = 0x80;
	mdbc_parm->UpdPara.deghost_dth = 0xf;
	mdbc_parm->UpdPara.deghost_sth = 0xf0;
	mdbc_parm->UpdPara.stable_frm = 0x78;
	mdbc_parm->UpdPara.update_dyn = 0x80;
	mdbc_parm->UpdPara.va_distth = 32;
	mdbc_parm->UpdPara.t_distth = 24;
	mdbc_parm->UpdPara.dbg_frmID = 0x0;
	mdbc_parm->UpdPara.dbg_frmID_en = 0x0;
	mdbc_parm->UpdPara.dbg_rnd = 0x0;
	mdbc_parm->UpdPara.dbg_rnd_en = 0x0;
}

void md_set_para_medium_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
	mdbc_parm->MdmatchPara.lbsp_th = 0x0;
	mdbc_parm->MdmatchPara.d_colour = 15;
	mdbc_parm->MdmatchPara.r_colour = 0x1e;
	mdbc_parm->MdmatchPara.d_lbsp = 4;
	mdbc_parm->MdmatchPara.r_lbsp = 8;
	mdbc_parm->MdmatchPara.model_num = 0x8;
	mdbc_parm->MdmatchPara.t_alpha = 25;
	mdbc_parm->MdmatchPara.dw_shift = 0x4;
	mdbc_parm->MdmatchPara.dlast_alpha = 100;
	mdbc_parm->MdmatchPara.min_match = 2;
	mdbc_parm->MdmatchPara.dlt_alpha = 0xa;
	mdbc_parm->MdmatchPara.dst_alpha = 0x28;
	mdbc_parm->MdmatchPara.uv_thres = 20;
	mdbc_parm->MdmatchPara.s_alpha = 100;
	mdbc_parm->MdmatchPara.dbg_lumDiff = 0x0;
	mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

	mdbc_parm->MorPara.th_ero = 0x8;
	mdbc_parm->MorPara.th_dil = 0x0;
	mdbc_parm->MorPara.mor_sel0 = 0x0;
	mdbc_parm->MorPara.mor_sel1 = 0x1;
	mdbc_parm->MorPara.mor_sel2 = 0x2;
	mdbc_parm->MorPara.mor_sel3 = 0x3;

	mdbc_parm->UpdPara.minT = 0x4;
	mdbc_parm->UpdPara.maxT = 0x40;
	mdbc_parm->UpdPara.maxFgFrm = 0x80;
	mdbc_parm->UpdPara.deghost_dth = 50;
	mdbc_parm->UpdPara.deghost_sth = 205;
	mdbc_parm->UpdPara.stable_frm = 0x78;
	mdbc_parm->UpdPara.update_dyn = 0x80;
	mdbc_parm->UpdPara.va_distth = 32;
	mdbc_parm->UpdPara.t_distth = 24;
	mdbc_parm->UpdPara.dbg_frmID = 0x0;
	mdbc_parm->UpdPara.dbg_frmID_en = 0x0;
	mdbc_parm->UpdPara.dbg_rnd = 0x0;
	mdbc_parm->UpdPara.dbg_rnd_en = 0x0;
}

void md_set_para_low_sensitivity(VENDOR_MD_PARAM *mdbc_parm)
{
	mdbc_parm->MdmatchPara.lbsp_th = 0x0;
	mdbc_parm->MdmatchPara.d_colour = 15;
	mdbc_parm->MdmatchPara.r_colour = 0x1e;
	mdbc_parm->MdmatchPara.d_lbsp = 5;
	mdbc_parm->MdmatchPara.r_lbsp = 10;
	mdbc_parm->MdmatchPara.model_num = 0x8;
	mdbc_parm->MdmatchPara.t_alpha = 25;
	mdbc_parm->MdmatchPara.dw_shift = 0x4;
	mdbc_parm->MdmatchPara.dlast_alpha = 100;
	mdbc_parm->MdmatchPara.min_match = 1;
	mdbc_parm->MdmatchPara.dlt_alpha = 0xa;
	mdbc_parm->MdmatchPara.dst_alpha = 0x28;
	mdbc_parm->MdmatchPara.uv_thres = 20;
	mdbc_parm->MdmatchPara.s_alpha = 100;
	mdbc_parm->MdmatchPara.dbg_lumDiff = 0x0;
	mdbc_parm->MdmatchPara.dbg_lumDiff_en = 0x0;

	mdbc_parm->MorPara.th_ero = 0x8;
	mdbc_parm->MorPara.th_dil = 0x0;
	mdbc_parm->MorPara.mor_sel0 = 0x0;
	mdbc_parm->MorPara.mor_sel1 = 0x1;
	mdbc_parm->MorPara.mor_sel2 = 0x2;
	mdbc_parm->MorPara.mor_sel3 = 0x3;

	mdbc_parm->UpdPara.minT = 0x4;
	mdbc_parm->UpdPara.maxT = 0x40;
	mdbc_parm->UpdPara.maxFgFrm = 0x80;
	mdbc_parm->UpdPara.deghost_dth = 50;
	mdbc_parm->UpdPara.deghost_sth = 205;
	mdbc_parm->UpdPara.stable_frm = 0x78;
	mdbc_parm->UpdPara.update_dyn = 0x80;
	mdbc_parm->UpdPara.va_distth = 32;
	mdbc_parm->UpdPara.t_distth = 24;
	mdbc_parm->UpdPara.dbg_frmID = 0x0;
	mdbc_parm->UpdPara.dbg_frmID_en = 0x0;
	mdbc_parm->UpdPara.dbg_rnd = 0x0;
	mdbc_parm->UpdPara.dbg_rnd_en = 0x0;
}

HD_RESULT md_set_para(ADAS_MEM_RANGE *p_share_mem, UINT32 ping_pong_id, UINT32 mode, UINT32 sensi)
{
	VENDOR_MD_PARAM mdbc_parm;
	HD_RESULT ret = HD_OK;

	mdbc_parm.mode = mode;
	mdbc_parm.controlEn.update_nei_en = 1;
	mdbc_parm.controlEn.deghost_en = 1;
	mdbc_parm.controlEn.roi_en0 = 0;
	mdbc_parm.controlEn.roi_en1 = 0;
	mdbc_parm.controlEn.roi_en2 = 0;
	mdbc_parm.controlEn.roi_en3 = 0;
	mdbc_parm.controlEn.roi_en4 = 0;
	mdbc_parm.controlEn.roi_en5 = 0;
	mdbc_parm.controlEn.roi_en6 = 0;
	mdbc_parm.controlEn.roi_en7 = 0;
	mdbc_parm.controlEn.chksum_en = 0;
	mdbc_parm.controlEn.bgmw_save_bw_en = 0;

	if (ping_pong_id == 0) {
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

	mdbc_parm.uiLLAddr = 0x0;
	mdbc_parm.InInfo.uiLofs0 = MDBC_ALIGN(MD_IMG_WIDTH, 4); //160;
	mdbc_parm.InInfo.uiLofs1 = MDBC_ALIGN(MD_IMG_WIDTH, 4); //160;
	mdbc_parm.Size.uiMdbcWidth = MD_IMG_WIDTH;
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
	DBG_DUMP("\033[34m [ZMD:%s:%d]:[%d]\033[0m\r\n", __func__, __LINE__, proc_id);
	return vendor_ai_net_start(proc_id);
}
HD_RESULT ADAS_vendor_ai_net_stop(UINT32 proc_id)
{
	DBG_DUMP("\033[34m [ZMD:%s:%d]:[%d]\033[0m\r\n", __func__, __LINE__, proc_id);
	return vendor_ai_net_stop(proc_id);
}
CHAR *ADAS_lib_md_get_version(VOID)
{
	return lib_md_get_version();
}

  

int circle_color_judge(int which_tsr, int model, line_int_t area_number)
{
	UINT8 * TSR_IMG = (UINT8 *)ADAS_share_mem[21].va;
	int red_continuous = 0;
	int idx_y, index_u = -1,  index_v = -1; 
	// BOOL result = FALSE;

	int row_start, row_end, col_start, col_end;

	row_start = (PADDING_UP_KEY) / (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * TSR_NUM_H;
	row_end = (1 + PADDING_UP_KEY) / (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * TSR_NUM_H - 1;
	col_start = (PADDING_LEFT_KEY) / (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_NUM_W + which_tsr * TSR_NUM_W;
	col_end = (1 + PADDING_LEFT_KEY) / (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_NUM_W - 1 + which_tsr * TSR_NUM_W;

	for(int row = row_start; row < row_end; row++){
		for(int col = col_start; col < col_end; col++){
			if(row < area_number.starty || row > area_number.endy || col < area_number.startx || col > area_number.endx){
				idx_y = row * TSR_IMG_W + col;
				get_index_uv(TSR_IMG_W, TSR_IMG_H, idx_y, &index_u, &index_v);
				if(hsv_judge(TSR_IMG[idx_y], TSR_IMG[index_u], TSR_IMG[index_v], model)){
					red_continuous++;
					TSR_IMG[idx_y] = 0;
				}
			}
		}
	}
	// draw_rect_img(TSR_IMG, TSR_IMG_W, TSR_IMG_H, col_start, row_start, col_end - col_start, row_end - row_start);
	// draw_rect_img(TSR_IMG, TSR_IMG_W, TSR_IMG_H, area_number.startx, area_number.starty, area_number.endx - area_number.startx, area_number.endy - area_number.starty);
	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/output/RECT_%05d.bmp", tsr_record_cnt - 1);
	// write24bmpfile(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_IMG_H);

	// if(red_continuous > 4000)//TSR_NUM_W * TSR_NUM_H / 9)
	// 	result = TRUE;

	// snprintf(txt_buffer_tsr, 64, "\tRed circle%d : %s, %s cnt:%d\n", which_tsr, result?"true": "false", (model == HSV_RED_MODEL)?"red":"black", red_continuous);
	// write_txt_tsr(txt_buffer_tsr);
	// return result?red_continuous:0;
	return red_continuous;
}


void LUT(UINT8 * table, UINT8 * img, int size)
{
	for(int i = 0; i < size; i++){
		if(img[i] < 60)
			img[i] = table[img[i]];
	}
}

static float th_tsr_number[2][20];
static BOOL tsr_non_standard_flag = FALSE;

static HD_RESULT TSR_Copy1(HD_VIDEO_FRAME *p_video_frame_dst, HD_VIDEO_FRAME *p_video_frame_src, char model_tsr_copy, VENDOR_AI_POSTPROC_RESULT_INFO * p_out_buf, int tsr_cnt, int *p_tsr_index, HD_URECT *p_rect_tsr)
{
	// HD_GFX_SCALE param;
	HD_RESULT ret = 0;
	//copy vout0 to common buffer
	HD_GFX_SCALE param_yuv_copy1;
	memset(&param_yuv_copy1, 0, sizeof(HD_GFX_SCALE));
	param_yuv_copy1.src_img.dim.w = p_video_frame_src->dim.w;
	param_yuv_copy1.src_img.dim.h = p_video_frame_src->dim.h;
	param_yuv_copy1.src_img.format = p_video_frame_src->pxlfmt;
	param_yuv_copy1.src_img.p_phy_addr[0] = p_video_frame_src->phy_addr[0]; //src_pa;
	param_yuv_copy1.src_img.p_phy_addr[1] = p_video_frame_src->phy_addr[1]; //src_pa;
	param_yuv_copy1.src_img.lineoffset[0] = p_video_frame_src->loff[0];     //src_w
	param_yuv_copy1.src_img.lineoffset[1] = p_video_frame_src->loff[1];     //src_w
	param_yuv_copy1.dst_img.dim.w = p_video_frame_dst->dim.w;               //dst_w
	param_yuv_copy1.dst_img.dim.h = p_video_frame_dst->dim.h;               //dst_w
	param_yuv_copy1.dst_img.format = p_video_frame_dst->pxlfmt;
	param_yuv_copy1.dst_img.p_phy_addr[0] = p_video_frame_dst->phy_addr[0]; //dst_pa;
	param_yuv_copy1.dst_img.p_phy_addr[1] = p_video_frame_dst->phy_addr[1]; //dst_pa + 1920 * 1080;
	param_yuv_copy1.dst_img.lineoffset[0] = p_video_frame_dst->loff[0];
	param_yuv_copy1.dst_img.lineoffset[1] = p_video_frame_dst->loff[1];

	if (TSR_NUMBER_AREA == model_tsr_copy){	
		VENDOR_AI_POSTPROC_RESULT *p_rslt;
		int row_tsr, col_tsr;
		memset((void *)ADAS_share_mem[21].va, 0, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H);
		memset((void *)(ADAS_share_mem[21].va + TSR_IMG_W * TSR_IMG_H), 128, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H / 2);
		param_yuv_copy1.dst_region.w = TSR_NUM_W;
		param_yuv_copy1.dst_region.h = TSR_NUM_H;
		snprintf(txt_buffer_tsr, 128, "\t%s:\n", __func__);
		write_txt_tsr(txt_buffer_tsr);
		for(int i = 0;i < MIN(tsr_cnt, max_tsr);i++){	
				p_rslt = &p_out_buf->p_result[p_tsr_index[i]];
				if(tsr_non_standard_flag){
					int w_img = TSRW * TSR_IMG_H / TSRH, h_img = TSR_IMG_H;
					param_yuv_copy1.src_region.x = (p_rslt->x - PADDING_LEFT_KEY * p_rslt->w) * w_img + tsr_offsetx;
					param_yuv_copy1.src_region.y = (p_rslt->y - PADDING_UP_KEY * p_rslt->h) * h_img + tsr_offsety;
					param_yuv_copy1.src_region.w = (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * p_rslt->w * w_img;
					param_yuv_copy1.src_region.h = (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * p_rslt->h * h_img;		
				}else{
					// param_yuv_copy1.src_region.x = (p_rslt->x - PADDING_LEFT_KEY * p_rslt->w) * TSRW + TSRX;
					// param_yuv_copy1.src_region.y = (p_rslt->y - PADDING_UP_KEY * p_rslt->h) * TSRH + TSRY;
					// param_yuv_copy1.src_region.w = (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * p_rslt->w * TSRW;
					// param_yuv_copy1.src_region.h = (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * p_rslt->h * TSRH;

					param_yuv_copy1.src_region.x = p_rect_tsr[i].x - ceil(PADDING_LEFT_KEY * p_rect_tsr[i].w);
					param_yuv_copy1.src_region.y = p_rect_tsr[i].y - ceil(PADDING_UP_KEY * p_rect_tsr[i].h);
					param_yuv_copy1.src_region.w = ceil(p_rect_tsr[i].w * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY));
					param_yuv_copy1.src_region.h = ceil(p_rect_tsr[i].h * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY));
				}

				row_tsr = i / (TSR_IMG_W / TSR_NUM_W);
				col_tsr = i % (TSR_IMG_W / TSR_NUM_W);
				param_yuv_copy1.dst_region.x = col_tsr * TSR_NUM_W;
				param_yuv_copy1.dst_region.y = row_tsr * TSR_NUM_H;

				ret = hd_gfx_scale(&param_yuv_copy1);
				if (ret != HD_OK) {
					snprintf(txt_buffer_tsr, 128, "\t\t%s_TSR_NUMBER_AREA failed ret = %d\n", __func__, ret);
					write_txt_tsr(txt_buffer_tsr);
					printf("hd_gfx_scale fail=%d\n", ret);
					goto exit;
				}
				
				snprintf(txt_buffer_tsr, 128, "\t\t%s : conf=%f, x=%d, y=%d, w=%d, h=%d\n", tsr_model_is_America?label_America[p_rslt->no[0]]:label_Europe[p_rslt->no[0]], p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
				write_txt_tsr(txt_buffer_tsr);
		}
		// if(MIN(tsr_cnt, max_tsr) == 1){
		// 	if(tsr_non_standard_flag){
		// 			int w_img = TSRW * TSR_IMG_H / TSRH, h_img = TSR_IMG_H;
		// 			param_yuv_copy1.src_region.x = (p_rslt->x - 2 * PADDING_LEFT_KEY * p_rslt->w) * w_img + tsr_offsetx;
		// 			param_yuv_copy1.src_region.y = (p_rslt->y -2 *  PADDING_UP_KEY * p_rslt->h) * h_img + tsr_offsety;
		// 			param_yuv_copy1.src_region.w = (1 + 2 * PADDING_LEFT_KEY + 2 * PADDING_RIGHT_KEY) * p_rslt->w * w_img;
		// 			param_yuv_copy1.src_region.h = (1 + 2 * PADDING_UP_KEY + 2 * PADDING_DOWN_KEY) * p_rslt->h * h_img;		
		// 		}else{
		// 			param_yuv_copy1.src_region.x = (p_rslt->x + 0.1 * p_rslt->w) * TSRW + TSRX;
		// 			param_yuv_copy1.src_region.y = (p_rslt->y - 0.1 * p_rslt->h) * TSRH + TSRY;
		// 			param_yuv_copy1.src_region.w = (1 + 0.2) * p_rslt->w * TSRW;
		// 			param_yuv_copy1.src_region.h = (1 + 0.2) * p_rslt->h * TSRH;
		// 		}

		// 		if (param_yuv_copy1.src_region.x < 0) {
		// 			param_yuv_copy1.src_region.x = 0;
		// 		}

		// 		if (param_yuv_copy1.src_region.y < 0) {
		// 			param_yuv_copy1.src_region.y = 0;
		// 		}

		// 		if ((param_yuv_copy1.src_region.w + param_yuv_copy1.src_region.x) >= 1920) {
		// 			param_yuv_copy1.src_region.x = 1920 - param_yuv_copy1.src_region.w;
		// 		}

		// 		if ((param_yuv_copy1.src_region.h + param_yuv_copy1.src_region.y) >= 1080) {
		// 			param_yuv_copy1.src_region.y = 1080 - param_yuv_copy1.src_region.h;
		// 		}

		// 		param_yuv_copy1.dst_region.x = TSR_NUM_W;
		// 		param_yuv_copy1.dst_region.y = 0;

		// 		ret = hd_gfx_scale(&param_yuv_copy1);
		// 		if (ret != HD_OK) {
		// 			printf("hd_gfx_scale fail=%d\n", ret);
		// 			goto exit;
		// 		}
		// 		snprintf(txt_buffer_tsr, 128, "\t\t%s multiscale: conf=%f, x=%d, y=%d, w=%d, h=%d\n", tsr_model_is_America?label_America[p_rslt->no[0]]:label_Europe[p_rslt->no[0]], p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
		// 		write_txt_tsr(txt_buffer_tsr);
		// }
	}else if(TSR_STANDART_AREA == model_tsr_copy){
		param_yuv_copy1.src_region.x = TSRX; 
		param_yuv_copy1.src_region.y = TSRY;
		param_yuv_copy1.src_region.w = TSRW; 
		param_yuv_copy1.src_region.h = TSRH;
		param_yuv_copy1.dst_region.x = 0;
		param_yuv_copy1.dst_region.y = 0;
		param_yuv_copy1.dst_region.w = p_video_frame_dst->dim.w;
		param_yuv_copy1.dst_region.h = p_video_frame_dst->dim.h;
		ret = hd_gfx_scale(&param_yuv_copy1);
		if (ret != HD_OK) {
			printf("hd_gfx_scale fail=%d\n", ret);
			goto exit;
		}
	}else if(TSR_UNLIMIT_AREA == model_tsr_copy){
		memset((void *)ADAS_share_mem[21].va, 0, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H);
		memset((void *)(ADAS_share_mem[21].va + TSR_IMG_W * TSR_IMG_H), 128, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H / 2);
		param_yuv_copy1.dst_region.x = 0;
		param_yuv_copy1.dst_region.y = 0;
		param_yuv_copy1.dst_region.w = TSR_NUM_W;
		param_yuv_copy1.dst_region.h = TSR_NUM_H;
		VENDOR_AI_POSTPROC_RESULT *p_rslt;
	
		p_rslt = &p_out_buf->p_result[0];
		param_yuv_copy1.src_region.x = (p_rslt->x - PADDING_LEFT_KEY * p_rslt->w) * TSRW + TSRX;
		param_yuv_copy1.src_region.y = (p_rslt->y - PADDING_UP_KEY * p_rslt->h) * TSRH + TSRY;
		param_yuv_copy1.src_region.w = (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * p_rslt->w * TSRW;
		param_yuv_copy1.src_region.h = (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * p_rslt->h * TSRH;

		ret = hd_gfx_scale(&param_yuv_copy1);
		if (ret != HD_OK) {
			printf("hd_gfx_scale fail=%d\n", ret);
		}
		goto exit;
	}else if(TSR_NONSTANDARD_AREA == model_tsr_copy){
		param_yuv_copy1.src_region.x = tsr_offsetx; 
		param_yuv_copy1.src_region.y = tsr_offsety;
		param_yuv_copy1.src_region.w = TSRW * TSR_IMG_H / TSRH; 
		param_yuv_copy1.src_region.h = TSR_IMG_H;
		param_yuv_copy1.dst_region.x = 0;
		param_yuv_copy1.dst_region.y = 0;
		param_yuv_copy1.dst_region.w = p_video_frame_dst->dim.w;
		param_yuv_copy1.dst_region.h = p_video_frame_dst->dim.h;
		ret = hd_gfx_scale(&param_yuv_copy1);
		if (ret != HD_OK) {
			printf("hd_gfx_scale fail=%d\n", ret);
			goto exit;
		}
	}else if(TSR_AMERICA_AREA == model_tsr_copy){
		VENDOR_AI_POSTPROC_RESULT *p_rslt;
		memset((void *)ADAS_share_mem[21].va, 0, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H);
		memset((void *)(ADAS_share_mem[21].va + TSR_IMG_W * TSR_IMG_H), 128, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H / 2);
		param_yuv_copy1.dst_region.w = TSR_NUM_W;
		param_yuv_copy1.dst_region.h = TSR_NUM_H;
		param_yuv_copy1.dst_region.y = 0;
		snprintf(txt_buffer_tsr, 128, "\t%s:\n", __func__);
		write_txt_tsr(txt_buffer_tsr);
		for(int i = 0;i < MIN(tsr_cnt, max_tsr);i++){	
			p_rslt = &p_out_buf->p_result[p_tsr_index[i]];
			if(p_rslt->no[0] == TSR_NO_A_LIMIT){
				param_yuv_copy1.src_region.x = p_rect_tsr[i].x - ceil(PADDING_A_LIMIT_LEFT * p_rect_tsr[i].w);
				param_yuv_copy1.src_region.y = p_rect_tsr[i].y - ceil(PADDING_A_LIMIT_UP * p_rect_tsr[i].h);
				// param_yuv_copy1.src_region.w = ceil(p_rect_tsr[i].w * (1 + PADDING_A_LIMIT_LEFT + PADDING_A_LIMIT_RIGHT));
				// param_yuv_copy1.src_region.h = ceil(p_rect_tsr[i].h * (1 + PADDING_A_LIMIT_UP + PADDING_A_LIMIT_DOWN));
				param_yuv_copy1.src_region.w = p_rect_tsr[i].w + ceil(PADDING_A_LIMIT_LEFT * p_rect_tsr[i].w) + ceil(PADDING_A_LIMIT_RIGHT * p_rect_tsr[i].w);
				param_yuv_copy1.src_region.h = p_rect_tsr[i].h + ceil(PADDING_A_LIMIT_UP * p_rect_tsr[i].h) + ceil(PADDING_A_LIMIT_DOWN * p_rect_tsr[i].h);
			}else{
				param_yuv_copy1.src_region.x = p_rect_tsr[i].x - ceil(PADDING_A_NIGHT_LEFT * p_rect_tsr[i].w);
				param_yuv_copy1.src_region.y = p_rect_tsr[i].y - ceil(PADDING_A_NIGHT_UP * p_rect_tsr[i].h);
				param_yuv_copy1.src_region.w = ceil(p_rect_tsr[i].w * (1 + PADDING_A_NIGHT_LEFT + PADDING_A_NIGHT_RIGHT));
				param_yuv_copy1.src_region.h = ceil(p_rect_tsr[i].h * (1 + PADDING_A_NIGHT_UP + PADDING_A_NIGHT_DOWN));
			}

			param_yuv_copy1.dst_region.x = i * TSR_NUM_W;
			ret = hd_gfx_scale(&param_yuv_copy1);
			if (ret != HD_OK) {
				printf("hd_gfx_scale fail=%d\n", ret);
			}
		} 
	}


	// if(model_tsr_copy == 2)
	// 	GussianFilter((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_SIZE, TSR_IMG_SIZE);
	// if(model_tsr_copy == 2)
	// printf("hd_gfx_scale ok\n");
exit:
	return ret;
}

HD_RESULT TSR_Copy_Backword(HD_VIDEO_FRAME *p_video_frame_dst, HD_VIDEO_FRAME *p_video_frame_src, MYRECT *p_rect_backward, int *p_tsr_value)
{
	// HD_GFX_SCALE param;
	HD_RESULT ret = 0;
	//copy vout0 to common buffer
	HD_GFX_SCALE param_yuv_copy1;
	memset(&param_yuv_copy1, 0, sizeof(HD_GFX_SCALE));
	param_yuv_copy1.src_img.dim.w = p_video_frame_src->dim.w;
	param_yuv_copy1.src_img.dim.h = p_video_frame_src->dim.h;
	param_yuv_copy1.src_img.format = p_video_frame_src->pxlfmt;
	param_yuv_copy1.src_img.p_phy_addr[0] = p_video_frame_src->phy_addr[0]; //src_pa;
	param_yuv_copy1.src_img.p_phy_addr[1] = p_video_frame_src->phy_addr[1]; //src_pa;
	param_yuv_copy1.src_img.lineoffset[0] = p_video_frame_src->loff[0];     //src_w
	param_yuv_copy1.src_img.lineoffset[1] = p_video_frame_src->loff[1];     //src_w
	param_yuv_copy1.dst_img.dim.w = p_video_frame_dst->dim.w;               //dst_w
	param_yuv_copy1.dst_img.dim.h = p_video_frame_dst->dim.h;               //dst_w
	param_yuv_copy1.dst_img.format = p_video_frame_dst->pxlfmt;
	param_yuv_copy1.dst_img.p_phy_addr[0] = p_video_frame_dst->phy_addr[0]; //dst_pa;
	param_yuv_copy1.dst_img.p_phy_addr[1] = p_video_frame_dst->phy_addr[1]; //dst_pa + 1920 * 1080;
	param_yuv_copy1.dst_img.lineoffset[0] = p_video_frame_dst->loff[0];
	param_yuv_copy1.dst_img.lineoffset[1] = p_video_frame_dst->loff[1];
	
	int row_tsr, col_tsr;
	memset((void *)ADAS_share_mem[21].va, 0, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H);
	memset((void *)(ADAS_share_mem[21].va + TSR_IMG_W * TSR_IMG_H), 128, sizeof(UINT8) * TSR_IMG_W * TSR_IMG_H / 2);
	param_yuv_copy1.dst_region.w = TSR_NUM_W;
	param_yuv_copy1.dst_region.h = TSR_NUM_H;
	for(int i = 0, cnt = 0; i < 20; i++){	
		if(p_tsr_value[i] < 10 || p_tsr_value[i] % 5 !=0)
			continue;
		param_yuv_copy1.src_region.x = p_rect_backward[i].x;
		param_yuv_copy1.src_region.y = p_rect_backward[i].y;
		param_yuv_copy1.src_region.w = p_rect_backward[i].w;
		param_yuv_copy1.src_region.h = p_rect_backward[i].h;

		row_tsr = i / (TSR_IMG_W / TSR_NUM_W);
		col_tsr = i % (TSR_IMG_W / TSR_NUM_W);
		param_yuv_copy1.dst_region.x = col_tsr * TSR_NUM_W;
		param_yuv_copy1.dst_region.y = row_tsr * TSR_NUM_H;

		ret = hd_gfx_scale(&param_yuv_copy1);
		if (ret != HD_OK) {
			printf("hd_gfx_scale TSR_Copy_Backword fail=%d\n", ret);
			return ret;
		}
		cnt++;
		if(cnt == 2)
			break;
	}
	return ret;
}



int UnLimtFilter(VENDOR_AI_POSTPROC_RESULT *p_rslt, HD_VIDEO_FRAME *p_video_frame_dst, HD_VIDEO_FRAME *p_video_frame_src, BOOL crop_flag, VENDOR_AI_POSTPROC_RESULT_INFO *p_out_buf)
{	
	snprintf(txt_buffer_tsr, 64, "\tUnLimtFilter:\n");
	write_txt_tsr(txt_buffer_tsr);
	HD_URECT temp_HD_URECT;
	if(crop_flag){
		int temp=0;
		TSR_Copy1(p_video_frame_dst, p_video_frame_src, TSR_UNLIMIT_AREA, p_out_buf, temp, &temp, &temp_HD_URECT);
	}

	UINT8 * img_src = (UINT8 *)ADAS_share_mem[21].va;

	int gray_level_histogram[256] = {0}, pixel_num = TSR_NUM_W* TSR_NUM_H / 4, threshold_best = 0;
	float gray_level_proportion[256] = {0};

	for (int line = TSR_NUM_H / 4; line < TSR_NUM_H * 3 / 4; line++) {
		for (int cloumn = TSR_NUM_W / 4; cloumn < TSR_NUM_W * 3 / 4; cloumn++) {
			gray_level_histogram[img_src[line * TSR_IMG_W + cloumn]]++;
			// img_src[line * TSR_IMG_SIZE + cloumn] = 255;
		}
	}

	for (int i = 0; i < 256; i++) {
		gray_level_proportion[i] = (float)gray_level_histogram[i] / pixel_num;
	}

	float w0, w1, u0, u1, u0temp, u1temp, deltaTmp, deltaMax = 0;
	for (int threshold = 1; threshold < 255; threshold++) {
		w0 = 0, w1 = 0, u0 = 0, u1 = 0, u0temp = 0, u1temp = 0, deltaTmp = 0;
		for (int gray_level = 0; gray_level < 256; gray_level++) {
			if (gray_level <= threshold) {
				w0 += gray_level_proportion[gray_level];
				u0temp += gray_level * gray_level_proportion[gray_level];
			} else {
				w1 += gray_level_proportion[gray_level];
				u1temp += gray_level * gray_level_proportion[gray_level];
			}
		}

		if (w0 && w1) {
			u0 = u0temp / w0;
			u1 = u1temp / w1;

			deltaTmp = (float)(w0 * w1 * ((u0 - u1) * (u0 - u1)));
			if (deltaTmp > deltaMax) {
				deltaMax = deltaTmp;
				threshold_best = threshold;
			}
		}
	}

	w0 = 0, w1 = 0;
	for (int gray_level = 0; gray_level < 256; gray_level++) {
		if (gray_level <= threshold_best) {
			w0 += gray_level_proportion[gray_level];
		} else {
			w1 += gray_level_proportion[gray_level];
		}
	}
	// printf("w0 = %f, w1 = %f\n", w0, w1);
	if(w0 > 0.4 || w0 < 0.1){
		snprintf(txt_buffer_tsr, 64, "\t\t1003, w0 = %f\n", w0);
		write_txt_tsr(txt_buffer_tsr);
		return 1003;
	}
		
	int below_thr = 0;
	for (int line = TSR_NUM_H / 4; line < TSR_NUM_H * 3 / 4; line++) {
		for (int cloumn = TSR_NUM_W / 4; cloumn < TSR_NUM_W * 3 / 4; cloumn++) {
			if(line > -(FLOAT)cloumn * TSR_NUM_H / TSR_NUM_W + TSR_NUM_H * 3 / 4 && line < -(FLOAT)cloumn * TSR_NUM_H / TSR_NUM_W + TSR_NUM_H * 5 / 4){
				if(img_src[line * TSR_IMG_W+ cloumn] <= threshold_best)
					below_thr++;
			}
		}
	}
	if(below_thr < 0.9 * w0 * TSR_NUM_W * TSR_NUM_H / 4){
		printf("======== error1005 : w0 = %f, below_thr = %f\n", w0, (float)below_thr / (TSR_NUM_W * TSR_NUM_H / 4));
		snprintf(txt_buffer_tsr, 64, "\t\t1005, w0 = %f, below_thr = %f\n", w0,  (float)below_thr / (TSR_NUM_W * TSR_NUM_H / 4));
		write_txt_tsr(txt_buffer_tsr);
		return 1005;
	}
	printf("======== 1005ok : w0 = %f, below_thr = %f\n", w0, (float)below_thr / (TSR_NUM_W * TSR_NUM_H  / 4));

	int index_u, index_v, cnt_white = 0, cnt_white2 = 0, cnt_white3 = 0, cnt_black = 0;
	int s1_qualified = 0, s2_qualified = 0, s3_qualified = 0;
	for(int row = TSR_NUM_H / 4;row < TSR_NUM_H *3 / 8; row++){
		for(int col = TSR_NUM_W / 4;col < TSR_NUM_W *3 /8;col++){
			get_index_uv(TSR_IMG_W, TSR_IMG_H, row * TSR_IMG_W + col, &index_u, &index_v);
			if(hsv_judge(img_src[row * TSR_IMG_W + col], img_src[index_u], img_src[index_v], HSV_GRAY_MODEL + HSV_WHITE_MODEL)){
				cnt_white++;
				// img_src[row * TSR_IMG_SIZE + col] = 255;
			}
			if(img_src[row * TSR_IMG_W + col] > threshold_best)
				s1_qualified++;
		}
	}
	
	for(int row = TSR_NUM_H * 7 / 16; row < TSR_NUM_H * 9 / 16;row++){
		for(int col = TSR_NUM_W * 7 / 16;col < TSR_NUM_W * 9 / 16;col++){
			get_index_uv(TSR_IMG_W, TSR_IMG_H, row * TSR_IMG_W + col, &index_u, &index_v);
			if(hsv_judge(img_src[row * TSR_IMG_W + col], img_src[index_u], img_src[index_v], HSV_GRAY_MODEL + HSV_WHITE_MODEL)){
				cnt_white2++;
				// img_src[row * TSR_IMG_SIZE + col] = 255;
			}else if(hsv_judge(img_src[row * TSR_IMG_W + col], img_src[index_u], img_src[index_v], HSV_BLACK_MODEL)){
				cnt_black++;
				// img_src[row * TSR_IMG_SIZE + col] = 0;
			}
			if(img_src[row * TSR_IMG_W + col] <= threshold_best)
				s2_qualified++;
		}
	}

	for(int row = TSR_NUM_H * 10 / 16; row < TSR_NUM_H * 12 / 16;row++){
		for(int col = TSR_NUM_W * 10 / 16;col < TSR_NUM_W * 12 / 16;col++){
			get_index_uv(TSR_IMG_W, TSR_IMG_H, row * TSR_IMG_W + col, &index_u, &index_v);
			if(hsv_judge(img_src[row * TSR_IMG_W + col], img_src[index_u], img_src[index_v], HSV_GRAY_MODEL + HSV_WHITE_MODEL)){
				cnt_white3++;
				// img_src[row * TSR_IMG_SIZE + col] = 255;
			}
			if(img_src[row * TSR_IMG_W + col] > threshold_best)
				s3_qualified++;
		}
	}

	snprintf(txt_buffer_tsr, 128, "\t\tcnt_white = %d, cnt_black = %d, cnt_white2 = %d, cnt_white3 = %d\n", cnt_white, cnt_black, cnt_white2, cnt_white3);
	write_txt_tsr(txt_buffer_tsr);

	if(!(cnt_white >= TSR_NUM_W * TSR_NUM_H/ 128 
		&& ((cnt_white2 >= TSR_NUM_W * TSR_NUM_H / 512 && cnt_white2 <= TSR_NUM_W * TSR_NUM_H / 96) || (cnt_black >= TSR_NUM_W * TSR_NUM_H / 512))
		&& cnt_white3 >= TSR_NUM_W * TSR_NUM_H / 128))
		return 1002;

	if(!(s1_qualified >=TSR_NUM_W * TSR_NUM_H / 96 
		&& s2_qualified >= TSR_NUM_W * TSR_NUM_H / 512 && s2_qualified <= TSR_NUM_W * TSR_NUM_H / 128 
		&& s3_qualified >=TSR_NUM_W * TSR_NUM_H / 96))
		return 1004;

	printf("cnt_white = %d, cnt_black = %d, cnt_white2 = %d, cnt_white3 = %d\n",cnt_white, cnt_black, cnt_white2, cnt_white3);
	return 1001;
}



BOOL AI_Process(HD_VIDEO_FRAME *  p_video_frame_new, ADAS_VIDEO_INFO * p_stream, VENDOR_AI_POSTPROC_RESULT_INFO *p_out_buf)
{	
	// write_tsr_time();
	// ms_start_tsr = hd_gettime_ms();
	// UINT32 time_start = hd_gettime_ms();
	HD_RESULT ret;
	VENDOR_AI_BUF in_buf;
	//prepare input AI_BUF from videoframe
	in_buf.sign = MAKEFOURCC('A', 'B', 'U', 'F');
	in_buf.width = p_video_frame_new->dim.w;
	in_buf.height = p_video_frame_new->dim.h;
	in_buf.channel = HD_VIDEO_PXLFMT_PLANE(p_video_frame_new->pxlfmt); //conver pxlfmt to channel count
	in_buf.line_ofs = p_video_frame_new->loff[0];
	in_buf.fmt = p_video_frame_new->pxlfmt;
	in_buf.pa = p_video_frame_new->phy_addr[0];
	in_buf.va = 0;
	in_buf.size = p_video_frame_new->loff[0] * p_video_frame_new->dim.h * 3 / 2;

	if (ai_async == 0) {
		// set input image
		ret = vendor_ai_net_set(p_stream->net_path, VENDOR_AI_NET_PARAM_IN(0, 0), &in_buf);
		if (HD_OK != ret) {
			DBG_ERR("proc_id(%u) set input fail !!\n", p_stream->net_path);
			return FALSE;
		}

		// do net proc
		ret = vendor_ai_net_proc(p_stream->net_path);
		if (HD_OK != ret) {
			DBG_ERR("proc_id(%u) proc fail !!\n", p_stream->net_path);
			return FALSE;
		}

		// get output result
		ret = vendor_ai_net_get(p_stream->net_path, VENDOR_AI_NET_PARAM_OUT(VENDOR_AI_MAXLAYER, 0), p_out_buf);
		if (HD_OK != ret) {
			DBG_ERR("proc_id(%u) get output fail !!\n", p_stream->net_path);
			return FALSE;
		}
	} else if (ai_async == 1) {
		// do net proc_buf
		ret = vendor_ai_net_proc_buf(p_stream->net_path, VENDOR_AI_NET_PARAM_IN(0, 0), &in_buf, VENDOR_AI_NET_PARAM_OUT(VENDOR_AI_MAXLAYER, 0), p_out_buf);
		if (HD_OK != ret) {
			DBG_ERR("proc_id(%u) proc_buf fail !!\n", p_stream->net_path);
			return FALSE;
		}
	}
	// printf("Time ai process = %ld\n",hd_gettime_ms()-time_start);
	// snprintf(txt_buffer_tsr, 128, "\tAI process :%ldms\n", hd_gettime_ms() - time_start);
	// write_txt_tsr(txt_buffer_tsr);
	return TRUE;
}



// id, 图片, 视频流, 声音值
void TSR_Set_America_Type(UINT8 type)
{
	set_txt_output_path("/mnt/sd/tsr/txt/tsr_infos.txt");
	tsr_model_is_America = type;
	if(tsr_model_is_America){
		for(int idx = 0; idx < 20; idx++){
			th_tsr_number[TSR_AMERICA_DAY_TH_IDX][idx] = 0.6;
			th_tsr_number[TSR_AMERICA_NIGHT_TH_IDX][idx] = 0.6;
		}
		TSR_IMG_W = 600;
		TSR_IMG_H = 300;
		TSR_NUM_W = 300;
		TSR_NUM_H = 300;
		// TSRW = 1200, TSRH = 450;
	}else{
		for(int idx = 0; idx < 20; idx++){
			th_tsr_number[TSR_EUROPE_LIMIT_TH_IDX][idx] = 0.6;
			th_tsr_number[TSR_EUROPE_REMOVELIMIT_TH_IDX][idx] = 0.6;
		}
		// th_tsr_number[TSR_EUROPE_LIMIT_TH_IDX][1] = 0.0;
		// th_tsr_number[TSR_EUROPE_REMOVELIMIT_TH_IDX][1] = 0.0;
		TSR_IMG_W = 600;
		TSR_IMG_H = 300; 
		TSR_NUM_W = 300;
		TSR_NUM_H = 300;
		// TSRW = 1440, TSRH = 540;		
	}
	max_tsr = (TSR_IMG_W / TSR_NUM_W) * (TSR_IMG_H / TSR_NUM_H);
	x_max_tsr1 = (float)TSR_NUM_W / TSR_IMG_W;
	y_max_tsr1 = (float)TSR_NUM_H / TSR_IMG_H;
	// TSRX = (1920 - TSRW) / 2;
	// TSRY = G410D_OFFSET + (1080 - 300 - TSRH) * 2 / 3;
	snprintf(txt_buffer_tsr, 64, "\n\n\t---------%s record---------\n", tsr_model_is_America?"America":"Europe");
	write_txt_tsr(txt_buffer_tsr);
	// Gauss_Lut_init();
	// snprintf(txt_buffer_tsr, 64, "\n");
	// write_txt_tsr(txt_buffer_tsr);
	// float * table = (float *)ADAS_share_mem[26].va;
	// for(int i = 0; i < 256; i++){
	// 	snprintf(txt_buffer_tsr, 64, "\t%d   %f\n", i, table[i]);
	// 	write_txt_tsr(txt_buffer_tsr);
	// }

}
UINT8 TSR_Get_America_Type(void)
{
	return tsr_model_is_America;
}

void tsr_min_size_update(void)
{
	static struct tm Curr_DateTime = {0};
	Curr_DateTime = hwclock_get_time(TIME_ID_CURRENT);

	if(7 < Curr_DateTime.tm_hour && Curr_DateTime.tm_hour < 18){
		TSR_R_MIN_SIZE = 20.0;
		TSR_L_MIN_SIZE = 20.0;
	}else{
		TSR_R_MIN_SIZE = 30.0;
		TSR_L_MIN_SIZE = 30.0;
	}
	
}

void tsr_offset_update(VENDOR_AI_POSTPROC_RESULT *p_rslt)
{
	int x = p_rslt->x * TSRW + TSRX, y = p_rslt->y * TSRH + TSRY;
	int w = p_rslt->w * TSRW, h = p_rslt->h * TSRH;
	int w_img = TSRW * TSR_IMG_H / TSRH, h_img = TSR_IMG_H;
	tsr_offsety = 0;
	if(p_rslt->x < 0.34){
		if(x + w > w_img){
			tsr_offsetx = x + w - w_img;
		}else{
			tsr_offsetx = 0;
		}
		if(y - h_img / 2 > 0){
			tsr_offsety = y - h_img / 2;
		}		
	}else if(p_rslt->x > 0.66){
		if(x + w_img < 1920){
			tsr_offsetx = x;
		}else{
			tsr_offsetx = 1920 - w_img;		
		}

		if(y - h_img / 2 > 0){
			tsr_offsety = y - h_img / 2;
		}
	}else{
		tsr_offsetx = x + w / 2 - w_img / 2;
		if(y + h > h_img){
			tsr_offsety = y + h - h_img;
		}
	}
	// tsr_offsetx = x;
	// tsr_offsety = y;
}

void tsr_init(HD_VIDEO_FRAME *p_frame21, HD_VIDEO_FRAME *p_frame3, HD_IRECT *rect_dst_egde)
{
	//p_frame21.p_next      = NULL; 
	//p_frame21.ddr_id      = 0;
	p_frame21->pxlfmt = HD_VIDEO_PXLFMT_YUV420;
	p_frame21->dim.w = TSR_IMG_W;
	p_frame21->dim.h = TSR_IMG_H; 
	//p_frame21.count       = 0;
	//p_frame21.timestamp   = hd_gettime_us();
	p_frame21->loff[0] = TSR_IMG_W;                                                  // Y
	p_frame21->loff[1] = TSR_IMG_W;                                                  // UV
	p_frame21->phy_addr[0] = ADAS_share_mem[21].pa;                                          // Y
	p_frame21->phy_addr[1] = ADAS_share_mem[21].pa + TSR_IMG_W * TSR_IMG_H; // UV pack

	//p_frame3.p_next      = NULL; 
	//p_frame3.ddr_id      = 0;
	p_frame3->pxlfmt = HD_VIDEO_PXLFMT_YUV420;
	p_frame3->dim.w = TSR_EDGE_DST_W * TSR_EDGE_BACKGROUND_RATE;
	p_frame3->dim.h = TSR_EDGE_DST_H * TSR_EDGE_BACKGROUND_RATE; 
	//p_frame3.count       = 0;
	//p_frame3.timestamp   = hd_gettime_us();
	p_frame3->loff[0] = TSR_EDGE_DST_W * TSR_EDGE_BACKGROUND_RATE;                                                  // Y
	p_frame3->loff[1] = TSR_EDGE_DST_W * TSR_EDGE_BACKGROUND_RATE;                                                  // UV
	p_frame3->phy_addr[0] = ADAS_share_mem[3].pa;                                          // Y
	p_frame3->phy_addr[1] = ADAS_share_mem[3].pa + TSR_EDGE_DST_W * TSR_EDGE_DST_H * TSR_EDGE_BACKGROUND_RATE * TSR_EDGE_BACKGROUND_RATE; // UV pack
	memset((void *)ADAS_share_mem[21].va, 0, TSR_IMG_W * TSR_IMG_H * 3 / 2);
	memset((void *)ADAS_share_mem[3].va, 128, TSR_EDGE_DST_W * TSR_EDGE_DST_H * TSR_EDGE_BACKGROUND_RATE * TSR_EDGE_BACKGROUND_RATE * 3 / 2);

	rect_dst_egde->x = 0;
	rect_dst_egde->y = 0;
	rect_dst_egde->w = TSR_EDGE_DST_W * TSR_EDGE_BACKGROUND_RATE;
	rect_dst_egde->h = TSR_EDGE_DST_H * TSR_EDGE_BACKGROUND_RATE;
}

void TSR_Process(UINT32 Id, HD_VIDEO_FRAME *p_video_frame, ADAS_VIDEO_INFO *p_stream, ALG_ADAS_RESULT *p_result)
{
	
	static HD_VIDEO_FRAME frame21 = {0}, frame3;
	static UINT8 tsr_is_init = FALSE;
	static MY_ADAS_RESULT tsr_result = {0};
	static int tsr_cnt0 = 0;
	static HD_IRECT rect_dst_egde = {0};
	static UINT8 *img_tsr_src, *img3;

	
	if(tsr_is_init == FALSE){
		tsr_is_init = TRUE;
		tsr_init(&frame21, &frame3, &rect_dst_egde);	
		img3 = (UINT8 *)ADAS_share_mem[3].va;
		#if TSR_DEB == 0
			img_tsr_src = (UINT8 *)hd_common_mem_mmap(HD_COMMON_MEM_MEM_TYPE_CACHE, p_video_frame->phy_addr[0], p_video_frame->dim.w * p_video_frame->dim.h * 3 / 2);
		#else
			
			img21 = (UINT8 *)ADAS_share_mem[21].va;
		#endif
	}else{
		#if TSR_DEB == 0
			hd_common_mem_flush_cache((void *)img_tsr_src, p_video_frame->dim.w * p_video_frame->dim.h * 3 / 2);
		#endif
	}
	
	tsr_result_control(p_result, &tsr_result, &tsr_t_record, tsr_cnt0);
	VENDOR_AI_POSTPROC_RESULT_INFO out_buf = {0};
	HD_URECT rect_tsr[20];
	tsr_result.value = 0;
	tsr_cnt0 = 0;
	int tsr_cnt = 0;
	int temp_int;
	#if TSR_DEB 
		static int deb_init = FALSE;
		static int bin_num = 1, bin_num_last;
		static char input_name[64]; 
		bin_num_last = bin_num;
		srand((unsigned)hd_gettime_ms());
		// bin_num += rand() % 5 + 1;
		bin_num ++;
		snprintf(input_name, 64, "/mnt/sd/tsr/test/%d.yuv", bin_num_last);
		if(!readyuvfile(input_name, (UINT8 *)ADAS_share_mem[26].va, 1920, 1080))
			return;
		static HD_VIDEO_FRAME frame_input = {0};
		if(deb_init == FALSE){
			frame_input.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
			frame_input.dim.w = 1920;
			frame_input.dim.h = 1080; 
			frame_input.loff[0] = 1920;                                                   // Y
			frame_input.loff[1] = 1920;                                                  // UV
			frame_input.phy_addr[0] = ADAS_share_mem[26].pa;                                          // Y
			frame_input.phy_addr[1] = ADAS_share_mem[26].pa + 1920 * 1080; // UV pack
			img_tsr_src = (UINT8 *)ADAS_share_mem[26].va;
			deb_init = TRUE;
		}
		TSR_Copy1(&frame21, &frame_input, TSR_STANDART_AREA, &out_buf, temp_int, &temp_int, rect_tsr);
		// static HD_VIDEO_FRAME frame_26 = {0};
		// frame_26.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
		// frame_26.dim.w = TSR_IMG_W;
		// frame_26.dim.h = TSR_IMG_H; 
		// frame_26.loff[0] = TSR_IMG_W;                                                   // Y
		// frame_26.loff[1] = TSR_IMG_W;                                                  // UV
		// frame_26.phy_addr[0] = ADAS_share_mem[26].pa;                                          // Y
		// frame_26.phy_addr[1] = ADAS_share_mem[26].pa + TSR_IMG_W * TSR_IMG_H; // UV pack	
		// if(bin_num % 2 != 0){
		// 	UINT32 time_start = hd_gettime_ms();
		// 	// HistogramEqualization2((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_NUM_H);
		// 	// MedianFilter((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_NUM_H);
		// 	LUT(gamma_table_04546, (UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W * TSR_IMG_H);
		// 	printf("Time HistogramEqualization = %ld\n",hd_gettime_ms()-time_start);
		// }
	#else	
		if(tsr_non_standard_flag){		
			TSR_Copy1(&frame21, p_video_frame, TSR_NONSTANDARD_AREA, &out_buf, temp_int, &temp_int, rect_tsr);	

			// static char non_standard_img_path[64];
			// static int name_num = 1;
			// snprintf(non_standard_img_path, 512, "/mnt/sd/img/tsr/ns%05d.bmp", name_num++);
			// write24bmpfile(non_standard_img_path, (UINT8 *)ADAS_share_mem[21].va, video_frame_new.dim.w, video_frame_new.dim.h);
			// tsr_non_standard_flag = FALSE;
			// return;

		}else{
			TSR_Copy1(&frame21, p_video_frame, TSR_STANDART_AREA, &out_buf, temp_int, &temp_int, rect_tsr);	
		}
	#endif
	
	if(AI_Process(&frame21, p_stream, &out_buf) == FALSE){
		printf("error : tsr ai process1 fail\n");
		return;
	}


	#if TSR_DEB
		VENDOR_AI_POSTPROC_RESULT *p_r_deb;
		for (UINT32 i = 0; i < out_buf.result_num; i++) {
			p_r_deb = &out_buf.p_result[i];
			draw_rect_img(img21, TSR_IMG_W, TSR_IMG_H,  W_Img(p_r_deb->x), H_Img(p_r_deb->y), W_Img(p_r_deb->w), H_Img(p_r_deb->h));
			// printf("%d : x=%d, y=%d, w=%d, h=%d\n", i, W_Img(p_r_deb->x), H_Img(p_r_deb->y), W_Img(p_r_deb->w), H_Img(p_r_deb->h));			
		}
		snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/round1/%05d_%d_%d.yuv420sp", tsr_record_cnt, TSR_IMG_W, TSR_IMG_H);
		writeyuv420sp(txt_buffer_tsr, img21, TSR_IMG_W, TSR_IMG_H);
		// write24bmpfile(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, frame21.dim.w, frame21.dim.h);
		// HistogramEqualization((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_NUM_H);
		// snprintf(non_standard_img_path, 512, "/mnt/sd/img/tsr/ns%05d.bmp", name_num++);
		// write24bmpfile(non_standard_img_path, (UINT8 *)ADAS_share_mem[21].va, frame21.dim.w, frame21.dim.h);
		// printf("out_buf.result_num = %d\n", out_buf.result_num);
		// return;
	#endif

	// if(out_buf.result_num)
	// 	tsr_min_size_update();
	
	static int PHaveLabel[20];
	int tsr_index[20] = {-1}, tsr_type[20] = {-1};//tsr_type用于记录第一轮得到tsr的结果，不保存在第二轮识别后将丢失结果

	
	// ms_start_tsr = hd_gettime_ms();
	if (tsr_model_is_America) {
		for (UINT32 i = 0, frist_write = 0; i < out_buf.result_num; i++) {
			VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[i];
			if (p_rslt->no[0] == TSR_NO_A_LIMIT || p_rslt->no[0] == TSR_NO_A_NIGHT) {
				if(frist_write == 0)frist_write = write_tsr_time(&tsr_record_cnt);
				if (p_rslt->score[0] > 0.2) {
					tsr_index[tsr_cnt] = i;
					tsr_type[tsr_cnt] = p_rslt->no[0];
					tsr_cnt++;
					snprintf(txt_buffer_tsr, 128, "\t%s : conf=%f, x=%d, y=%d, w=%d, h=%d\n", label_America[p_rslt->no[0]], p_rslt->score[0], TSRX + W_Rec(p_rslt->x), TSRY + H_Rec(p_rslt->y), W_Rec(p_rslt->w), H_Rec(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
				}else{
					snprintf(txt_buffer_tsr, 128, "\t%s : conf=%f, x=%d, y=%d, w=%d, h=%d, conf less than the confidence\n", label_America[p_rslt->no[0]], p_rslt->score[0], TSRX + W_Rec(p_rslt->x), TSRY + H_Rec(p_rslt->y), W_Rec(p_rslt->w), H_Rec(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
				}
			}
		}
	} else {
		for (UINT32 i = 0, frist_write = 0; i < out_buf.result_num; i++) {
			VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[i];
			if (p_rslt->no[0] == TSR_NO_E_LIMIT || p_rslt->no[0] == TSR_NO_E_REMOVELIMIT){	
				if(frist_write == 0)frist_write = write_tsr_time(&tsr_record_cnt);			 
				if(p_rslt->no[0] == TSR_NO_E_REMOVELIMIT){					
					if(p_rslt->w * TSRW> (FLOAT)TSR_R_MIN_SIZE && p_rslt->h * TSRH > (FLOAT)TSR_R_MIN_SIZE){
						// printf("============== Find RemoveLimit ============\n");
						// printf("score = %f\n", p_rslt->score[0]);						
						snprintf(txt_buffer_tsr, 128, "\tRemoveLimit : conf=%f, x=%d, y=%d, w=%d, h=%d\n", p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
						write_txt_tsr(txt_buffer_tsr);
					}else{
						snprintf(txt_buffer_tsr, 128, "\tRemoveLimit --> Drop cause size: conf=%f, x=%d, y=%d, w=%d, h=%d\n", p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
						write_txt_tsr(txt_buffer_tsr);
						continue;
					}
				}else if(p_rslt->no[0] == TSR_NO_E_LIMIT){
					if(p_rslt->w * TSRW < (FLOAT)TSR_L_MIN_SIZE || p_rslt->h * TSRH < (FLOAT)TSR_L_MIN_SIZE){
						snprintf(txt_buffer_tsr, 128, "\tLimit --> Drop cause size: conf=%f, x=%d, y=%d, w=%d, h=%d\n", p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
						write_txt_tsr(txt_buffer_tsr);
						continue;
					}else{
						snprintf(txt_buffer_tsr, 128, "\tLimit : conf=%f, x=%d, y=%d, w=%d, h=%d\n", p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
						write_txt_tsr(txt_buffer_tsr);
					}
				}
		
				if (p_rslt->score[0] > 0.2) {			 // && p_rslt->w / p_rslt->h > 0.8 * TSRH / TSRW && p_rslt->w / p_rslt->h < 1.25 * TSRH / TSRW		
					// Location1Scroe = p_rslt->score[0] + p_rslt->no[0];
					tsr_index[tsr_cnt] = i;
					tsr_type[tsr_cnt] = p_rslt->no[0];
					tsr_cnt++;
				}else{
					snprintf(txt_buffer_tsr, 128, "\t^\t|\t| conf=%f less than th\n", p_rslt->score[0]);
					write_txt_tsr(txt_buffer_tsr);
				}
			} else if((p_rslt->no[0] == TSR_NO_E_UNLIMIT) && p_rslt->w / p_rslt->h > 0.8 * TSRH / TSRW && p_rslt->w / p_rslt->h < 1.25 * TSRH / TSRW){      
				if(p_rslt->w > 40.0 / TSRW && p_rslt->h > 40.0 / TSRH  && p_rslt->score[0] > 0.3 && out_buf.result_num == 1){
					if(frist_write == 0)frist_write = write_tsr_time(&tsr_record_cnt);
					int flag_unlimit = UnLimtFilter(p_rslt, &frame21, p_video_frame, TRUE, &out_buf);
					if(1001 == flag_unlimit){
						printf("class is UnLimt, conf = %f\n", p_rslt->score[0]);
						// result->tsr_result.tsr_postproc_result.x = p_rslt->x * TSRW + TSRX;
						// result->tsr_result.tsr_postproc_result.y = p_rslt->y * TSRH + TSRY;
						// result->tsr_result.tsr_postproc_result.w = p_rslt->w * TSRW;
						// result->tsr_result.tsr_postproc_result.h = p_rslt->h * TSRH;
						tsr_result.rect.x = p_rslt->x * TSRW + TSRX;
						tsr_result.rect.y = p_rslt->y * TSRH + TSRY;
						tsr_result.rect.w = p_rslt->w * TSRW;
						tsr_result.rect.h = p_rslt->h * TSRH;
					}else{
						printf("UnLimt filtered out by %d, conf = %f\n", flag_unlimit, p_rslt->score[0]);
					}
					
					snprintf(txt_buffer_tsr, 128, "\tUnLimit:conf = %f, x=%d, y=%d, w=%d, h=%d, flag_unlimit=%d\n", p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH), flag_unlimit);
					write_txt_tsr(txt_buffer_tsr);
					// result->tsr_result.tsr_status = flag_unlimit;
					tsr_result.value = flag_unlimit;
					tsr_result.time_record = hd_gettime_ms();
					snprintf(txt_buffer_tsr, 20, "\tresult: %d\n", flag_unlimit);
					write_txt_tsr(txt_buffer_tsr);
				}
				continue;
			}/*else{
				snprintf(txt_buffer_tsr, 128, "\t%s : conf=%f, x=%d, y=%d, w=%d, h=%d ---In first recognit\n", label_Europe[p_rslt->no[0]], p_rslt->score[0], (int)(TSRX + p_rslt->x * TSRW), (int)(TSRY + p_rslt->y * TSRH), (int)(p_rslt->w * TSRW), (int)(p_rslt->h * TSRH));
				write_txt_tsr(txt_buffer_tsr);
			}*/
		}
	}

	if(tsr_cnt == 0)
		return;
	// for(int i = 0; i < tsr_cnt;i++){
	// 	snprintf(txt_buffer_tsr, 128, "\ttsr_idx%d = %d, tsr_type%d = %d\n", i, tsr_index[i], i, tsr_type[i]);
	// 	write_txt_tsr(txt_buffer_tsr);
	// }
	myNMS(&out_buf, tsr_index, tsr_type, &tsr_cnt, 0.2, tsr_model_is_America);
	// snprintf(txt_buffer_tsr, 128, "\n");
	// write_txt_tsr(txt_buffer_tsr);
	// for(int i = 0; i < tsr_cnt;i++){
	// 	snprintf(txt_buffer_tsr, 128, "\ttsr_idx%d = %d, tsr_type%d = %d\n", i, tsr_index[i], i, tsr_type[i]);
	// 	write_txt_tsr(txt_buffer_tsr);
	// }


	if(tsr_cnt == 0){//NMS后tsrcnt可能为0
		snprintf(txt_buffer_tsr, 64, "\tAfter NMS, tsr_cnt == 0\n");
		write_txt_tsr(txt_buffer_tsr);
		return;	
	}
	tsr_cnt0 = tsr_cnt;

	if(!tsr_non_standard_flag) tsr_offset_update(&out_buf.p_result[0]);

	for(int i = 0; i < MIN(max_tsr, tsr_cnt); i++){
	rect_tsr[i].x = floor(out_buf.p_result[tsr_index[i]].x * TSRW) + (UINT32)TSRX;
	rect_tsr[i].y = floor(out_buf.p_result[tsr_index[i]].y * TSRH) + (UINT32)TSRY;
	rect_tsr[i].w = (UINT32)(ceil((out_buf.p_result[tsr_index[i]].x + out_buf.p_result[tsr_index[i]].w) * TSRW)) - rect_tsr[i].x + (UINT32)TSRX;
	rect_tsr[i].h = (UINT32)(ceil((out_buf.p_result[tsr_index[i]].y + out_buf.p_result[tsr_index[i]].h) * TSRH)) - rect_tsr[i].y + (UINT32)TSRY;
	}
	align_rect(rect_tsr, MIN(max_tsr, tsr_cnt), 4);



	// if(tsr_cnt == 1){
	// 	tsr_cnt = 2;
	// 	rect_tsr[1] = rect_tsr[0];
	// }			
	// if(tsr_non_standard_flag){
	// 	int w_img = TSRW * TSR_IMG_H / TSRH, h_img = TSR_IMG_H;
	// 	result->tsr_result.tsr_postproc_result.x = out_buf.p_result[0].x * w_img + tsr_offsetx;
	// 	result->tsr_result.tsr_postproc_result.y = out_buf.p_result[0].y * h_img + tsr_offsety;
	// 	result->tsr_result.tsr_postproc_result.w = out_buf.p_result[0].w * w_img;
	// 	result->tsr_result.tsr_postproc_result.h = out_buf.p_result[0].h * h_img;
	// }else{
	// 	result->tsr_result.tsr_postproc_result.x = out_buf.p_result[0].x * TSRW + TSRX;
	// 	result->tsr_result.tsr_postproc_result.y = out_buf.p_result[0].y * TSRH + TSRY;
	// 	result->tsr_result.tsr_postproc_result.w = out_buf.p_result[0].w * TSRW;
	// 	result->tsr_result.tsr_postproc_result.h = out_buf.p_result[0].h * TSRH;
	// }

	#if TSR_DEB
		if(TSR_MODEL_EUROPE == tsr_model_is_America)	  
			TSR_Copy1(&frame21, &frame_input, TSR_NUMBER_AREA, &out_buf, tsr_cnt, tsr_index, rect_tsr);		
		else
			TSR_Copy1(&frame21, &frame_input, TSR_AMERICA_AREA, &out_buf, tsr_cnt, tsr_index, rect_tsr);	
		// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/src_frame_21/21_%05d_%05d.bmp", bin_num_last, tsr_record_cnt - 1);
		// write24bmpfile(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, frame21.dim.w, frame21.dim.h);
	#else
		if(TSR_MODEL_EUROPE == tsr_model_is_America)
			TSR_Copy1(&frame21, p_video_frame, TSR_NUMBER_AREA, &out_buf, tsr_cnt, tsr_index, rect_tsr);
		else
			TSR_Copy1(&frame21, p_video_frame, TSR_AMERICA_AREA, &out_buf, tsr_cnt, tsr_index, rect_tsr);
		// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/copy_%05d.yuv420sp", tsr_record_cnt - 1);
		// writeyuv420sp(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, frame21.dim.w, frame21.dim.h);
	#endif

	
	if(AI_Process(&frame21, p_stream, &out_buf) == FALSE){
		printf("error : tsr ai process2 fail\n");
		return;
	}

	#if TSR_DEB
		VENDOR_AI_POSTPROC_RESULT *p_r;
		for(UINT32 idx = 0; idx < out_buf.result_num; idx++){
			p_r = &out_buf.p_result[idx];
			draw_rect_img(img21, TSR_IMG_W, TSR_IMG_H, W_Img(p_r->x), H_Img(p_r->y), W_Img(p_r->w), H_Img(p_r->h));
			// printf("%d : x=%d, y=%d, w=%d, h=%d\n", idx, W_Img(p_r->x), H_Img(p_r->y), W_Img(p_r->w), H_Img(p_r->h));	
		}
		// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/output/n%05d_%05d.bmp", bin_num_last, tsr_record_cnt - 1);
		// write24bmpfile(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, frame21.dim.w, frame21.dim.h);
		snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/round2/%05d_%d_%d.yuv420sp", tsr_record_cnt - 1, TSR_IMG_W, TSR_IMG_H);
		writeyuv420sp(txt_buffer_tsr, img21, TSR_IMG_W, TSR_IMG_H);
	#endif
 
	int tsr_value[20] = {0};
	int num_not_satisfy_cnt[20] = {0}, num_not_satisfy_idx[20] = {0};
	int labelcnt = 0;
	int tsr_type2[20] = {0};
	if (TSR_MODEL_AMERICA == tsr_model_is_America) {
		for (UINT32 i = 0, idx_temp; i < out_buf.result_num; i++) {
			VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[i];
			int idx_tsr = (int)(p_rslt->x / x_max_tsr1) + (int)(TSR_IMG_W / TSR_NUM_W) * (int)(p_rslt->y / y_max_tsr1);
			if(tsr_class_is_number(p_rslt->no[0], TSR_MODEL_AMERICA)){
				idx_temp = (tsr_type[idx_tsr] == TSR_NO_A_LIMIT)?TSR_AMERICA_DAY_TH_IDX:TSR_AMERICA_NIGHT_TH_IDX;
				if (p_rslt->score[0] > th_tsr_number[idx_temp][atoi(label_America[p_rslt->no[0]])]) {//数字通常在图片下方，上方字母可能会误识别为数字，需对y做限制	
					snprintf(txt_buffer_tsr, 128, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d\n", label_America[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
					// printf("class is %s, conf = %f\n", label_America[p_rslt->no[0]], p_rslt->score[0]);
					PHaveLabel[labelcnt] = i;
					labelcnt += 1;
				}else{
					num_not_satisfy_cnt[idx_tsr]++;
					num_not_satisfy_idx[idx_tsr] = i;
					snprintf(txt_buffer_tsr, 128, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d, conf less than the confidence\n", label_America[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
				}
			}			
		}
	}else{
		for (UINT32 i = 0, idx_temp, idx_tsr; i < out_buf.result_num; i++) {
			VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[i];
			/*if (p_rslt->no[0] == TSR_NO_E_T) {
				printf("============== Find WeightLimit ============\n");
				printf("class is %s, conf = %f\n", label_Europe[p_rslt->no[0]], p_rslt->score[0]);
				snprintf(txt_buffer_tsr, 256, "\tt, conf = %f, x=%d, y=%d, w=%d, h=%d\n", p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
				write_txt_tsr(txt_buffer_tsr);
				result->tsr_result.tsr_status = 0;
				draw_rect_img((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_IMG_H, W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
				snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/t_%05d.yuv420sp", tsr_record_cnt - 1);
				writeyuv420sp(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, frame21.dim.w, frame21.dim.h);
				return;
			} else */if (tsr_class_is_number(p_rslt->no[0], TSR_MODEL_EUROPE)) {
				if(p_rslt->w > 0.5){
					snprintf(txt_buffer_tsr, 64, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d, w too large\n", label_Europe[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
					continue;
				}
				if(p_rslt->h < 0.05){
					snprintf(txt_buffer_tsr, 128, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d, h too small\n", label_Europe[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
					continue;
				}
				idx_tsr = (int)(p_rslt->x / x_max_tsr1) + (int)(TSR_IMG_W / TSR_NUM_W) * (int)(p_rslt->y / y_max_tsr1);
				idx_temp = (tsr_type[idx_tsr] == TSR_NO_E_LIMIT)?TSR_EUROPE_LIMIT_TH_IDX:TSR_EUROPE_REMOVELIMIT_TH_IDX;
				if (p_rslt->score[0] < th_tsr_number[idx_temp][atoi(label_Europe[p_rslt->no[0]])]){
					num_not_satisfy_cnt[idx_tsr]++;
					num_not_satisfy_idx[idx_tsr] = i;
					snprintf(txt_buffer_tsr, 128, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d, below the threshold value\n", label_Europe[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);							
					printf("class is %s, conf = %f, below the threshold value\n", label_Europe[p_rslt->no[0]], p_rslt->score[0]);
					continue;
				}
				int x_int = W_Img(p_rslt->x);
				int xw_int = W_Img(p_rslt->x + p_rslt->w);
				if(x_int / TSR_NUM_W != xw_int / TSR_NUM_W){
					snprintf(txt_buffer_tsr, 128, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d, number have two dates\n", label_Europe[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
					write_txt_tsr(txt_buffer_tsr);
					continue;
				}			
				snprintf(txt_buffer_tsr, 64, "\t%s, conf = %f, x=%d, y=%d, w=%d, h=%d\n", label_Europe[p_rslt->no[0]], p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
				write_txt_tsr(txt_buffer_tsr);
				printf("class is %s, conf = %f\n", label_Europe[p_rslt->no[0]], p_rslt->score[0]);
				PHaveLabel[labelcnt] = i;
				labelcnt += 1;				
			}else{
				idx_tsr = (int)(p_rslt->x / x_max_tsr1) + (int)(TSR_IMG_W / TSR_NUM_W) * (int)(p_rslt->y / y_max_tsr1); 
				tsr_type2[idx_tsr] = p_rslt->no[0];
				snprintf(txt_buffer_tsr, 128, "\t%s, idx_tsr = %lu, conf = %f, x=%d, y=%d, w=%d, h=%d\n", label_Europe[p_rslt->no[0]], idx_tsr, p_rslt->score[0], W_Img(p_rslt->x), H_Img(p_rslt->y), W_Img(p_rslt->w), H_Img(p_rslt->h));
				write_txt_tsr(txt_buffer_tsr);
			}
		} 
	}


	// NMS(&out_buf, PHaveLabel, &labelcnt, 0.9);

	// int max_index_tsr1 = labelcnt - 1;//初始化第一个限速牌最后一个数字的下标（单限速牌情况）
	int last_index_tsrnum[20], first_index_tsrnum[20];
	for(int i = 0; i < 20; i++){
		first_index_tsrnum[i] = -1;
		last_index_tsrnum[i] = -1;		
	}
	if (labelcnt > 0) {
		//将PHaveLabel里的标签按x排序
		for (int i = 0, temp; i < labelcnt - 1; i++) {
			for (int j = 0; j < labelcnt - 1 - i; j++) {
				if (out_buf.p_result[PHaveLabel[j]].x > out_buf.p_result[PHaveLabel[j + 1]].x) {
					temp = PHaveLabel[j];
					PHaveLabel[j] = PHaveLabel[j + 1];
					PHaveLabel[j + 1] = temp;
				}
			}
		}	
		//多限速牌计算max_index_tsr1
		for(int i = 0, which; i < labelcnt; i++){
			which = (int)(out_buf.p_result[PHaveLabel[i]].x / x_max_tsr1);
			if(-1 == first_index_tsrnum[which]){
				first_index_tsrnum[which] = i;
			}
			last_index_tsrnum[which] = i;	

			//防止数字坐标越界
			if(out_buf.p_result[PHaveLabel[i]].x < 0){
			out_buf.p_result[PHaveLabel[i]].w += out_buf.p_result[PHaveLabel[i]].x;
			out_buf.p_result[PHaveLabel[i]].x = 0;
			}
			if(out_buf.p_result[PHaveLabel[i]].y < 0){
				out_buf.p_result[PHaveLabel[i]].h += out_buf.p_result[PHaveLabel[i]].y;
				out_buf.p_result[PHaveLabel[i]].y = 0;
			}	
			if(out_buf.p_result[PHaveLabel[i]].x + out_buf.p_result[PHaveLabel[i]].w > (which + 1) * x_max_tsr1){
				out_buf.p_result[PHaveLabel[i]].w = (which + 1) * x_max_tsr1 - out_buf.p_result[PHaveLabel[i]].x;
			}
			if(out_buf.p_result[PHaveLabel[i]].y + out_buf.p_result[PHaveLabel[i]].h > y_max_tsr1){
				out_buf.p_result[PHaveLabel[i]].h = y_max_tsr1 - out_buf.p_result[PHaveLabel[i]].y;
			}
		}	

		for(int i = 0; i < MIN(tsr_cnt, max_tsr); i++){
			snprintf(txt_buffer_tsr, 64, "\tfist%d = %d\tlast = %d\n",i, first_index_tsrnum[i], last_index_tsrnum[i]);
			write_txt_tsr(txt_buffer_tsr);
		}
 
		snprintf(txt_buffer_tsr, 64, "\ttsrcnt = %d\tlabelcnt = %d\n",tsr_cnt, labelcnt);
		write_txt_tsr(txt_buffer_tsr);

		for(int idx_tsr = 0; idx_tsr < MIN(tsr_cnt, max_tsr); idx_tsr++){
			tsr_value[idx_tsr] = calc_tsr_result(&out_buf, first_index_tsrnum[idx_tsr], last_index_tsrnum[idx_tsr], PHaveLabel, idx_tsr, tsr_model_is_America);	
		}
		
		for(int i = 0; i < MIN(tsr_cnt, max_tsr); i++){
			snprintf(txt_buffer_tsr, 64, "\tresult%d : %d",i, tsr_value[i]);
			write_txt_tsr(txt_buffer_tsr);
		}
		snprintf(txt_buffer_tsr, 64, "\n");
		write_txt_tsr(txt_buffer_tsr);
		
	} /*else if (labelcnt == 1 && out_buf.p_result[PHaveLabel[0]].no[0] == zero_start + 5) {
		result->tsr_result.tsr_status = 5;
	}*/else if(0 == labelcnt){
		// result->tsr_result.tsr_status = 3000;
		snprintf(txt_buffer_tsr, 64, "\tNo number was detected\n");
		write_txt_tsr(txt_buffer_tsr);
		return;
	}
	
	for(int which_tsr = 0; which_tsr < MIN(max_tsr, tsr_cnt); which_tsr++){
		if(num_not_satisfy_cnt[which_tsr]){
			// snprintf(txt_buffer_tsr, 64, "\tnot_cnt%d = %d\n", which_tsr, num_not_satisfy_cnt[which_tsr]);
			// write_txt_tsr(txt_buffer_tsr);
			if(num_not_satisfy_cnt[which_tsr] == 1 && tsr_model_is_America == TSR_MODEL_EUROPE && last_index_tsrnum[which_tsr] - first_index_tsrnum[which_tsr] > 0){
				int class_notsatisfy = out_buf.p_result[num_not_satisfy_idx[which_tsr]].no[0];
				int class_may_reapeat = out_buf.p_result[PHaveLabel[first_index_tsrnum[which_tsr]]].no[0];
				int class_may_reapeat2 = out_buf.p_result[PHaveLabel[last_index_tsrnum[which_tsr]]].no[0];
				// snprintf(txt_buffer_tsr, 64, "\t%s\t%s\n", label_Europe[class_notsatisfy], label_Europe[class_may_reapeat]);
				// write_txt_tsr(txt_buffer_tsr);
				if(atoi(label_Europe[class_may_reapeat]) - atoi(label_Europe[class_notsatisfy]) == 10){
					snprintf(txt_buffer_tsr, 64, "\ttsr%d not satisfy cause repeat\n", which_tsr);
					write_txt_tsr(txt_buffer_tsr);
					continue;
				}
				if(atoi(label_Europe[class_may_reapeat2]) - atoi(label_Europe[class_notsatisfy]) == 10){
					snprintf(txt_buffer_tsr, 64, "\ttsr%d not satisfy cause repeat\n", which_tsr);
					write_txt_tsr(txt_buffer_tsr);
					continue;
				}
				if(atoi(label_Europe[class_may_reapeat]) == 11 && atoi(label_Europe[class_notsatisfy]) == 10){
					snprintf(txt_buffer_tsr, 64, "\ttsr%d not satisfy cause repeat\n", which_tsr);
					write_txt_tsr(txt_buffer_tsr);
					continue;
				}
			}			
			snprintf(txt_buffer_tsr, 64, "\tresult%d, Number have low conf, %d -> 0\n", which_tsr, tsr_value[which_tsr]);
			write_txt_tsr(txt_buffer_tsr);
			tsr_value[which_tsr] = 0;
		}else if(tsr_value[which_tsr] >= 1000){
			snprintf(txt_buffer_tsr, 64, "\ttsr%d's number >= 1000!\n", which_tsr);
			write_txt_tsr(txt_buffer_tsr);
			tsr_value[which_tsr] = 0;
		}else if(tsr_value[which_tsr] < 10 && tsr_value[which_tsr] != 0){
			snprintf(txt_buffer_tsr, 64, "\ttsr%d's number < 10!\n", which_tsr);
			write_txt_tsr(txt_buffer_tsr);
			tsr_value[which_tsr] = 0;
		}
	}
	{
		int sum_tsr = 0;
		for(int idx = 0; idx < MIN(max_tsr, tsr_cnt); idx++){
			sum_tsr += tsr_value[idx];
		}
		if(sum_tsr == 0){
			snprintf(txt_buffer_tsr, 64, "\tAll TSR value are 0!\n");
			write_txt_tsr(txt_buffer_tsr);
			return;
		}
	}

	#if TSR_DEB 
		snprintf(txt_buffer_tsr, 64, "\tyuv num = %d\n", bin_num_last);
		write_txt_tsr(txt_buffer_tsr);
	#endif
	
	// //Limit bbox backward pass
	// MYRECT rect_tsr_backward[20];
	// for(int which_tsr = 0; which_tsr < MIN(max_tsr, tsr_cnt); which_tsr++){
	// 	if(tsr_value[which_tsr] <= 5 || tsr_value[which_tsr] % 5 != 0){
	// 		continue;
	// 	}
	// 	VENDOR_AI_POSTPROC_RESULT *p_rslt_first = &out_buf.p_result[PHaveLabel[first_index_tsrnum[which_tsr]]];			
	// 	VENDOR_AI_POSTPROC_RESULT *p_rslt_last = &out_buf.p_result[PHaveLabel[last_index_tsrnum[which_tsr]]];
	// 	float w = p_rslt_last->x + p_rslt_last->w - p_rslt_first->x;
	// 	float h1 = p_rslt_last->y + p_rslt_last->h - p_rslt_first->y;
	// 	float h2 = p_rslt_first->y + p_rslt_first->h - p_rslt_last->y;
	// 	float h = MAX(h1, h2);
	// 	float x = p_rslt_first->x;		
	// 	if(x > 0.5)
	// 		x -= 0.5;
	// 	float w_ratio = (x * TSR_IMG_W * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) - PADDING_LEFT_KEY * TSR_NUM_W) / TSR_NUM_W;
	// 	float h_ratio = (p_rslt_first->y * TSR_IMG_H * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) - PADDING_UP_KEY * TSR_NUM_H) / TSR_NUM_H;
	// 	float rectity_left = tsr_value[which_tsr]>=100?TSR_RECTIFY_HUNDRED_LEFT:TSR_RECTIFY_LEFT;
	// 	float rectify_right = tsr_value[which_tsr]>=100?TSR_RECTIFY_HUNDRED_RIGHT:TSR_RECTIFY_RIGHT;
	// 	rect_tsr_backward[which_tsr].w = rect_tsr[which_tsr].w * w * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_IMG_W / TSR_NUM_W + 2;
	// 	rect_tsr_backward[which_tsr].h = rect_tsr[which_tsr].h * h * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * TSR_IMG_H / TSR_NUM_H + 2;
	// 	rect_tsr_backward[which_tsr].x = rect_tsr[which_tsr].x + rect_tsr[which_tsr].w * w_ratio - rect_tsr_backward[which_tsr].w * rectity_left;
	// 	rect_tsr_backward[which_tsr].y = rect_tsr[which_tsr].y + rect_tsr[which_tsr].h * h_ratio - rect_tsr_backward[which_tsr].h * TSR_RECTIFY_TOP;
	// 	rect_tsr_backward[which_tsr].w *= 1 + rectity_left + rectify_right;
	// 	rect_tsr_backward[which_tsr].h *= 1 + TSR_RECTIFY_TOP + TSR_RECTIFY_BOTTOM; 

	// 	snprintf(txt_buffer_tsr, 128, "\tx %lu --> %lu\n", rect_tsr[which_tsr].x, rect_tsr_backward[which_tsr].x);
	// 	write_txt_tsr(txt_buffer_tsr);
	// 	snprintf(txt_buffer_tsr, 128, "\ty %lu --> %lu\n", rect_tsr[which_tsr].y, rect_tsr_backward[which_tsr].y);
	// 	write_txt_tsr(txt_buffer_tsr);
	// 	snprintf(txt_buffer_tsr, 128, "\tw %lu --> %lu\n", rect_tsr[which_tsr].w, rect_tsr_backward[which_tsr].w);
	// 	write_txt_tsr(txt_buffer_tsr);
	// 	snprintf(txt_buffer_tsr, 128, "\th %lu --> %lu\n", rect_tsr[which_tsr].h, rect_tsr_backward[which_tsr].h);
	// 	write_txt_tsr(txt_buffer_tsr);
	// }
	// VENDOR_AI_POSTPROC_RESULT *p_r;
	// for (UINT32 i = 0; i < out_buf.result_num; i++) {
	// 	p_r = &out_buf.p_result[i];
	// 	draw_rect_img((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_IMG_H,  W_Img(p_r->x), H_Img(p_r->y), W_Img(p_r->w), H_Img(p_r->h));	
	// }
	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/%05da.yuv420sp", tsr_record_cnt - 1);
	// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
	// TSR_Copy_Backword(&frame21, p_video_frame, rect_tsr_backward, tsr_value);
	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/%05db.yuv420sp", tsr_record_cnt - 1);
	// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
	// return;
	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/src/%05d.yuv420sp", tsr_record_cnt - 1);
	// writeyuv420sp(txt_buffer_tsr, img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h);
	//Error Filtering
	// ms_start_tsr = hd_gettime_ms();
	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/%05d.yuv420sp", tsr_record_cnt - 1);
	// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
	if (tsr_model_is_America == TSR_MODEL_EUROPE) {
		for(int which_tsr = 0; which_tsr < MIN(max_tsr, tsr_cnt); which_tsr++){
			if(which_tsr ==1 && tsr_cnt0 != tsr_cnt && tsr_value[0] != 0){
				snprintf(txt_buffer_tsr, 128, "\tmultiscale0 is ok, stop to recognize multiscale1\n");
				write_txt_tsr(txt_buffer_tsr);
				tsr_value[1] = 0;
				break;
			}
			snprintf(txt_buffer_tsr, 128, "\n");
			write_txt_tsr(txt_buffer_tsr);
			if(tsr_value[which_tsr] <= 5 || tsr_value[which_tsr] % 5 != 0){
				if(tsr_value[which_tsr] > 0){
					snprintf(txt_buffer_tsr, 64, "\tresult%d %d -> 0 value <= 5 or Not a multiple of 5\n", which_tsr, tsr_value[which_tsr]);
					write_txt_tsr(txt_buffer_tsr);
					tsr_value[which_tsr] = 0;
				}
				continue;
			}
			VENDOR_AI_POSTPROC_RESULT *p_rslt_first = &out_buf.p_result[PHaveLabel[first_index_tsrnum[which_tsr]]];
			VENDOR_AI_POSTPROC_RESULT *p_rslt_second = &out_buf.p_result[PHaveLabel[1 + first_index_tsrnum[which_tsr]]];			
			VENDOR_AI_POSTPROC_RESULT *p_rslt_last = &out_buf.p_result[PHaveLabel[last_index_tsrnum[which_tsr]]];
			HD_IRECT inner, outer;
			float w_n = p_rslt_last->x + p_rslt_last->w - p_rslt_first->x;
			float y_min = 1.0, y_max = 0.0;			
			for(int idx = first_index_tsrnum[which_tsr]; idx <= last_index_tsrnum[which_tsr]; idx++){
				if(out_buf.p_result[PHaveLabel[idx]].y < y_min){
					y_min = out_buf.p_result[PHaveLabel[idx]].y;
				}
				if(out_buf.p_result[PHaveLabel[idx]].y + out_buf.p_result[PHaveLabel[idx]].h > y_max){
					y_max = out_buf.p_result[PHaveLabel[idx]].y + out_buf.p_result[PHaveLabel[idx]].h;
				}
			}
			float h_n = y_max - y_min;
			float x_n = p_rslt_first->x, y_n = y_min;		
			if(x_n > 0.5)
				x_n -= 0.5;
			float w_ratio = (x_n * TSR_IMG_W * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) - PADDING_LEFT_KEY * TSR_NUM_W) / TSR_NUM_W;
			float h_ratio = (y_n * TSR_IMG_H * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) - PADDING_UP_KEY * TSR_NUM_H) / TSR_NUM_H;
			float rectity_left = tsr_value[which_tsr]>=100?TSR_RECTIFY_HUNDRED_LEFT:TSR_RECTIFY_LEFT;
			float rectify_right = tsr_value[which_tsr]>=100?TSR_RECTIFY_HUNDRED_RIGHT:TSR_RECTIFY_RIGHT;
			inner.x = rect_tsr[which_tsr].x + floor(rect_tsr[which_tsr].w * w_ratio);
			inner.y = rect_tsr[which_tsr].y + floor(rect_tsr[which_tsr].h * h_ratio);
			w_ratio = ((x_n + w_n) * TSR_IMG_W * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) - PADDING_LEFT_KEY * TSR_NUM_W) / TSR_NUM_W;
			h_ratio = ((y_n + h_n) * TSR_IMG_H * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) - PADDING_UP_KEY * TSR_NUM_H) / TSR_NUM_H;
			// inner.w = rect_tsr[which_tsr].w * w_n * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_IMG_W / TSR_NUM_W;
			inner.w = rect_tsr[which_tsr].x + ceil(rect_tsr[which_tsr].w * w_ratio) - inner.x;
			// inner.h = rect_tsr[which_tsr].h * h_n * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * TSR_IMG_H / TSR_NUM_H;
			inner.h = rect_tsr[which_tsr].y + ceil(rect_tsr[which_tsr].h * h_ratio) - inner.y;
			outer.x = inner.x - inner.w * rectity_left;
			outer.y = inner.y - inner.h * TSR_RECTIFY_TOP;
			outer.w = inner.w * (1 + rectity_left + rectify_right);
			outer.h = inner.h * (1 + TSR_RECTIFY_TOP + TSR_RECTIFY_BOTTOM); 
			snprintf(txt_buffer_tsr, 128, "\tinner x=%ld, y=%ld, w=%ld, h=%ld\n\touter x=%ld, y=%ld, w=%ld, h=%ld\n", inner.x, inner.y, inner.w, inner.h, outer.x, outer.y, outer.w, outer.h);
			write_txt_tsr(txt_buffer_tsr);


			// draw_rect_img(img_tsr_src, 1920, 1080, inner.x, inner.y, inner.w, inner.h);
			// draw_rect_img(img_tsr_src, 1920, 1080, rect_tsr[which_tsr].x, rect_tsr[which_tsr].y, rect_tsr[which_tsr].w, rect_tsr[which_tsr].h);
			// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/test/%05d_%d_%d_%d.yuv420sp", tsr_record_cnt - 1, which_tsr, 1920, 1080);
			// writeyuv420sp(txt_buffer_tsr, img_tsr_src, 1920, 1080);
			// break;
			{//HightLlimit and noise
				if(last_index_tsrnum[which_tsr] - first_index_tsrnum[which_tsr] > 0){
					// HightLimit
					if (p_rslt_first->h > p_rslt_last->h * 5 / 4) {
						printf("============== Find HighLimit ============\n");
						tsr_value[which_tsr] = 0;
						snprintf(txt_buffer_tsr, 128, "\tTSR error: result%d: 3002, HightLimit, h1/h2=%f\n", which_tsr, p_rslt_first->h/p_rslt_second->h);
						write_txt_tsr(txt_buffer_tsr);
						continue;
					}
					// noise
					if(p_rslt_first->h < 0.2 || p_rslt_second->h < 0.2){
						printf("============== Find noise like Limit h < 0.2 ============\n");
						tsr_value[which_tsr] = 0;
						snprintf(txt_buffer_tsr, 128, "\tTSR error: result%d: 3004, Find noise like Limit h < 0.2\n", which_tsr);
						write_txt_tsr(txt_buffer_tsr);
						continue;
					}
					if(p_rslt_first->h < p_rslt_first->w || p_rslt_second->h < p_rslt_second->w){
						printf("============== Find noise like Limit h < w ============\n");
						tsr_value[which_tsr] = 0;
						snprintf(txt_buffer_tsr, 128, "\tTSR error: result%d: 3005\n", which_tsr);
						write_txt_tsr(txt_buffer_tsr);
						continue;
					}
					if( last_index_tsrnum[which_tsr] > first_index_tsrnum[which_tsr] &&
						(p_rslt_second->x - p_rslt_first->x - p_rslt_first->w > (p_rslt_second->w + p_rslt_first->w) / 3
						|| p_rslt_last->x - p_rslt_second->x - p_rslt_second->w > (p_rslt_last->w + p_rslt_second->w) / 3)){
						printf("============== Find noise like Limit num1 num2 distance too large ============\n");
						tsr_value[which_tsr] = 0;
						snprintf(txt_buffer_tsr, 128, "\tTSR error: result%d: 3006, d=%f, max=%f\n", which_tsr, p_rslt_second->x - p_rslt_first->x - p_rslt_first->w, (p_rslt_second->w + p_rslt_first->w) / 4);
						write_txt_tsr(txt_buffer_tsr);
						continue;
					}
					// if(p_rslt_first->h / p_rslt_second->h < 0.8 || p_rslt_second->h / p_rslt_first->h < 0.8){
					// 	printf("============== Find noise like Limit h1/h2<0.8 ============\n");
					// 	tsr_value[which_tsr] = 0;
					// 	snprintf(txt_buffer_tsr, 128, "\tresult%d: 3007\n", which_tsr);
					// 	write_txt_tsr(txt_buffer_tsr);
					// 	continue;
					// }
					// if(tsr_value[which_tsr] == 110){
					// 	if(last_index_tsrnum[which_tsr] - first_index_tsrnum[which_tsr] == 2){//100、70
					// 		if(p_rslt_second->x < p_rslt_first->x + p_rslt_first->w * 0.35){
					// 			tsr_value[which_tsr] = 0;
					// 			snprintf(txt_buffer_tsr, 128, "\tp_rslt_second->x < p_rslt_first->x + p_rslt_first->w * 0.35\n");
					// 			write_txt_tsr(txt_buffer_tsr);
					// 			continue;
					// 		}
					// 	}
					// }
				}								 
			}	 

			// edges color
			{	
				// line_int_t area_number;
				// area_number.startx = W_Img(p_rslt_first->x);
				// area_number.endx = W_Img(p_rslt_last->x + p_rslt_last->w);
				// int temp_y1, temp_y2;
				// temp_y1 = H_Img(p_rslt_first->y);
				// temp_y2 = H_Img(p_rslt_last->y);
				// area_number.starty = MIN(temp_y1, temp_y2);
				// temp_y1 = H_Img(p_rslt_first->y + p_rslt_first->h);
				// temp_y2 = H_Img(p_rslt_last->y + p_rslt_last->h);
				// area_number.endy = MAX(temp_y1, temp_y2);			
				// int red_cnt = circle_color_judge(which_tsr, HSV_RED_MODEL, area_number);
				// snprintf(txt_buffer_tsr, 64, "\tcolor%d: red cnt = %d\n", which_tsr, red_cnt);
				// write_txt_tsr(txt_buffer_tsr);
				
				// draw_rect_img(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, inner.x, inner.y, inner.w, inner.h);
				// draw_rect_img(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, outer.x, outer.y, outer.w, outer.h);
				// img_tsr_src[outer.y * 1920 + outer.x + (int)(outer.w * 0.25)] = 255;
				// img_tsr_src[outer.y * 1920 + outer.x + (int)(outer.w * 0.75)] = 255;
				// img_tsr_src[(outer.y + (int)(outer.h * 0.25)) * 1920 + outer.x] = 255;
				// img_tsr_src[(outer.y + (int)(outer.h * 0.75)) * 1920 + outer.x] = 255;
				// img_tsr_src[(outer.y + outer.h) * 1920 + outer.x + (int)(outer.w * 0.25)] = 255;
				// img_tsr_src[(outer.y + outer.h) * 1920 + outer.x + (int)(outer.w * 0.75)] = 255;
				// img_tsr_src[(outer.y + (int)(outer.h * 0.25)) * 1920 + outer.x + outer.w] = 255;
				// img_tsr_src[(outer.y + (int)(outer.h * 0.75)) * 1920 + outer.x + outer.w] = 255;
				// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/src/%05d_%d.yuv420sp", tsr_record_cnt - 1, which_tsr);
				// writeyuv420sp(txt_buffer_tsr, img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h);
				// return;
				
				int red_cnt = get_color_cnt_between_two_rect(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, &outer, &inner, HSV_RED_MODEL);
				// snprintf(txt_buffer_tsr, 64, "\t\tcolor%d: red cnt = %d, %d\n", which_tsr, red_cnt, (int)(red_cnt * 300 * 300 / outer.w / outer.h));
				// write_txt_tsr(txt_buffer_tsr);
				// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/E/%05d.yuv420sp", tsr_record_cnt - 1);
				// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
				// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/src/%05d.yuv420sp", tsr_record_cnt - 1);
				// writeyuv420sp(txt_buffer_tsr, img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h);
				// return;
				
				double red_percent = (double)red_cnt / (double)(outer.w * outer.h);
				snprintf(txt_buffer_tsr, 128, "\tred_percent = %lf\n", red_percent);
				write_txt_tsr(txt_buffer_tsr);
				if(tsr_type[which_tsr] == TSR_NO_E_REMOVELIMIT){
					if(red_percent > 0.045){
						if(red_percent > 0.12){
							tsr_type[which_tsr] = TSR_NO_E_LIMIT;
							snprintf(txt_buffer_tsr, 64, "\tcolor%d, red enough RemoveLimit --> Limit\n", which_tsr);
							write_txt_tsr(txt_buffer_tsr);
						}else{
							if(tsr_type2[which_tsr] == TSR_NO_E_LIMIT){
								tsr_type[which_tsr] = TSR_NO_E_LIMIT;
								snprintf(txt_buffer_tsr, 64, "\tcolor%d, tsr_type2 is Limit, RemoveLimit --> Limit\n", which_tsr);
								write_txt_tsr(txt_buffer_tsr);
							}else{
								tsr_value[which_tsr] = 0;
								printf("============== RemoveLimit have red edge ============\n");
								snprintf(txt_buffer_tsr, 64, "\tTSR error: color%d: RemoveLimit have red edge\n", which_tsr);
								write_txt_tsr(txt_buffer_tsr);
								// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/color/c_%05d_%d_%d.yuv420sp", tsr_record_cnt - 1, TSR_IMG_W, TSR_IMG_H);
								// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
								continue;
							}							
						}											
					}
				}else{
					if(red_percent < 0.045){
						// int cyan_cnt = circle_color_judge(which_tsr, HSV_CYAN_MODEL, area_number);
						int cyan_cnt = get_color_cnt_between_two_rect(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, &outer, &inner, HSV_CYAN_MODEL);
						double cyan_percent = (double)cyan_cnt / (double)(outer.w * outer.h);
						snprintf(txt_buffer_tsr, 64, "\tcolor%d: cyan_percent = %lf\n", which_tsr, cyan_percent);
						write_txt_tsr(txt_buffer_tsr);
						int flag_keep_limit = 0;
						if(cyan_percent > 0.12){
							// int blue_cnt = get_color_cnt_in_rect((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_IMG_H, &area_number, HSV_BLUE_MODEL);
							// int blue_cnt = get_color_cnt_between_two_rect(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, &outer, &inner, HSV_BLUE_MODEL);
							int black_cnt = get_color_cnt_in_rect(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, &inner, HSV_BLACK_MODEL);
							double black_percent = (double)black_cnt / (double)(inner.w * inner.h);
							snprintf(txt_buffer_tsr, 64, "\tcolor%d: black_percent = %lf\n", which_tsr, black_percent);
							write_txt_tsr(txt_buffer_tsr);
							if(black_percent > 0.1){
								snprintf(txt_buffer_tsr, 64, "\tcolor%d: save limit\n", which_tsr);
								write_txt_tsr(txt_buffer_tsr);
								flag_keep_limit = 1;
							}
						}
						// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/src/src_%05d_%d_%d.yuv420sp", tsr_record_cnt - 1, 1920, 1080);
						// writeyuv420sp(txt_buffer_tsr, img_tsr_src, 1920, 1080);
						if(flag_keep_limit == 0){ 
							tsr_value[which_tsr] = 0;
							snprintf(txt_buffer_tsr, 64, "\tTSR error: color%d: Limit have no red edge\n", which_tsr);
							write_txt_tsr(txt_buffer_tsr);
							// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/color/c_%05d_%d_%d.yuv420sp", tsr_record_cnt - 1, TSR_IMG_W, TSR_IMG_H);
							// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
							continue;
						}
					}
				}
			}

			if(tsr_type[which_tsr] == TSR_NO_E_REMOVELIMIT){
				tsr_value[which_tsr] += 1000;
			}else{							
				// WeightLimit
				VENDOR_AI_POSTPROC_RESULT *p_last = &out_buf.p_result[PHaveLabel[last_index_tsrnum[which_tsr]]];
				VENDOR_AI_POSTPROC_RESULT *p_last_last = &out_buf.p_result[PHaveLabel[last_index_tsrnum[which_tsr] - 1]];
					
				// UINT8 * img26 = (UINT8 *)ADAS_share_mem[26].va;			
				// int edge_threshold = get_Otsu_threshold_rect(img21, TSR_IMG_W, W_Img(p_last->x), H_Img(p_last->y), W_Img(p_last->w), H_Img(p_last->h));
				// int edge_threshold = 128;

				// // tsr_crop((UINT8 *)(ADAS_share_mem[3].va), (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, 0, 0, TSR_NUM_W, TSR_NUM_H);
				// tsr_crop_and_trans_hsv(img21, img3, TSR_IMG_W, TSR_IMG_H, TSR_NUM_W * which_tsr, 0, TSR_NUM_W, TSR_NUM_H);
				// MedianFilter(img3, TSR_NUM_W, TSR_NUM_H);
				// UINT32 time_start = hd_gettime_ms();
				// edgeDetector_tsr(&ADAS_share_mem[3], &ADAS_share_mem[26]);
				// snprintf(txt_buffer_tsr, 128, "\ttime = %lu\n", hd_gettime_ms() - time_start);
				// write_txt_tsr(txt_buffer_tsr);
				// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/ive/before_%05d_%d_%dyyy.yyy", tsr_record_cnt - 1, TSR_NUM_W, TSR_NUM_H);
				// writeyyy(txt_buffer_tsr, img3, TSR_NUM_W, TSR_NUM_H);
				// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/ive/%05d_%d_%dyyy.yyy", tsr_record_cnt - 1, TSR_NUM_W, TSR_NUM_H);
				// writeyyy(txt_buffer_tsr, img26, TSR_NUM_W, TSR_NUM_H);
				// int edge_threshold_ive = get_Otsu_threshold_rect(img26, TSR_NUM_W, 0, 0, TSR_NUM_W, TSR_NUM_H);
				// for(int idx = 0; idx < TSR_NUM_W * TSR_NUM_H; idx++){
				// 	img26[idx] = img26[idx] > edge_threshold_ive?255:0;
				// }
				// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/ive/bin%05d_%d_%dyyy.yyy", tsr_record_cnt - 1, TSR_NUM_W, TSR_NUM_H);
				// writeyyy(txt_buffer_tsr, img26, TSR_NUM_W, TSR_NUM_H);
				 

				{	
					HD_IRECT rect_edge_right, rect_edge_left;
					float x_f = p_last->x;
					if(x_f > 0.5) x_f -= 0.5;
					float w_ratio = (x_f + p_last->w) * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_IMG_W / TSR_NUM_W - PADDING_LEFT_KEY;
					// float w_ratio = x_f * (1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_IMG_W / TSR_NUM_W - PADDING_LEFT_KEY;     
					rect_edge_right.x = rect_tsr[which_tsr].x + rect_tsr[which_tsr].w * w_ratio + 1;
					float h_ratio = p_last->y * (1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * TSR_IMG_H / TSR_NUM_H - PADDING_UP_KEY;
					rect_edge_right.y = rect_tsr[which_tsr].y + rect_tsr[which_tsr].h * h_ratio;
					float w_last_num = p_last->w;
					if(p_last_last->x + p_last_last->w > p_last->x){
						if(atoi(label_Europe[p_last_last->no[0]]) - atoi(label_Europe[p_last->no[0]]) != 10){
							w_last_num -= (p_last_last->x + p_last_last->w - p_last->x); 
						}
					}
					rect_edge_right.w = TSR_EDGE_BACKGROUND_RATE * rect_tsr[which_tsr].w * w_last_num * ((1 + PADDING_LEFT_KEY + PADDING_RIGHT_KEY) * TSR_IMG_W / TSR_NUM_W - PADDING_LEFT_KEY);
					// rect_edge_right.h = TSR_EDGE_BACKGROUND_RATE * 0.5 * rect_tsr[which_tsr].h * p_last->h * ((1 + PADDING_UP_KEY + PADDING_DOWN_KEY) * TSR_IMG_H / TSR_NUM_H - PADDING_LEFT_KEY);
					rect_edge_right.w = align_i(rect_edge_right.w, 4);
					rect_edge_right.h = rect_edge_right.w;

					rect_edge_left.y = inner.y;
					rect_edge_left.w = rect_edge_right.w;
					rect_edge_left.h = rect_edge_right.h;
					rect_edge_left.x = inner.x - rect_edge_left.w;
					#if TSR_DEB 
						TSR_Copy_Edge(&frame_input, &frame3, &rect_edge_left, &rect_edge_right, &rect_dst_egde);
					#else
						TSR_Copy_Edge(p_video_frame, &frame3, &rect_edge_left, &rect_edge_right, &rect_dst_egde);
					#endif

					int red_continuous;
					int w_img3 = TSR_EDGE_DST_W * TSR_EDGE_BACKGROUND_RATE, h_img3 = TSR_EDGE_DST_H * TSR_EDGE_BACKGROUND_RATE;
					int cnt_not_red_right = 0, cnt_not_red_left = 0, flag_line_not_red, aver_d_right = 0, aver_d_left = 0;
					int idx_y, idx_u, idx_v;
					int cnt_after_red = 0, cnt_dy_error;
					int offset_col = 0;
					int col_last = 0, col_min, col_max, col_first_red;
					int flag_update_col_min = TRUE;
					// int col_y_min;
					
					int d_right[TSR_EDGE_DST_H] = {0}, d_left[TSR_EDGE_DST_H] = {0};
					for (int row = TSR_EDGE_ROW_START; row < TSR_EDGE_DST_H + TSR_EDGE_ROW_START; row++) {
						red_continuous = 0, flag_line_not_red = TRUE, cnt_after_red = 0, flag_update_col_min = TRUE, cnt_dy_error = 0;
						for (int col = w_img3 / 2 + offset_col; col < w_img3 / 2 + TSR_EDGE_DST_W; col++) {
							idx_y = row * w_img3 + col;
							get_index_uv(w_img3, h_img3, idx_y, &idx_u, &idx_v);
							if(hsv_judge(img3[idx_y], img3[idx_u], img3[idx_v], HSV_RED_MODEL)){
								if(flag_line_not_red == FALSE){
									// if(img3[idx_y] < img3[row * w_img3 + col_y_min]){
									// 	col_y_min = col;
									// }
									if(flag_update_col_min){
										if(img3[idx_y] + TSR_EDGE_DY < img3[row * w_img3 + col_min])
											col_min = col;
										else{
											if(col_min != col_first_red)
												cnt_dy_error++;
											if(cnt_dy_error == TSR_EDGE_DY_ERROR)
												flag_update_col_min = FALSE;
										}											
									}


									if(col_last == 0){
										if(++cnt_after_red == TSR_EDGE_LEN_AFTER_RED0)
											break;
									}else{
										if(++cnt_after_red == TSR_EDGE_LEN_AFTER_RED)
											break;
									}							
								}else{
									red_continuous++;
									if(red_continuous == TSR_EDGE_TH_CONTINUE){																							
										flag_line_not_red = FALSE;
										// col_y_min = col;
										col_min = col;
										col_first_red = col;
									}
								}
							}else{
								if(flag_line_not_red){
									red_continuous = 0;
								}else{	
									break;
								}
							}							
						}
						
						if(flag_line_not_red){
							cnt_not_red_right++;
						}else{
							// if(col_last != 0)
							// 	col_y_min = MIN(col_y_min, col_last + 1);
							// col_last = col_y_min;							
							// img3[row * w_img3 + col_y_min] = 255;
							// d_right[row - TSR_EDGE_ROW_START] = col_y_min;
							// offset_col = col_y_min - w_img3 / 2 - TSR_EDGE_TH_CONTINUE + 1;
							d_right[row - TSR_EDGE_ROW_START] = col_min;
						}
					}
					
					
					offset_col = 0, col_last = 0;
					for(int row = TSR_EDGE_ROW_START; row < TSR_EDGE_DST_H + TSR_EDGE_ROW_START; row++){
						red_continuous = 0, flag_line_not_red = TRUE, cnt_after_red = 0, flag_update_col_min = TRUE, cnt_dy_error = 0;
						for(int col = w_img3 / 2 - 1 - offset_col; col > w_img3 / 2 - 1 - TSR_EDGE_DST_W; col--){
							idx_y = row * w_img3 + col;
							get_index_uv(w_img3, h_img3, idx_y, &idx_u, &idx_v);
							if(hsv_judge(img3[idx_y], img3[idx_u], img3[idx_v], HSV_RED_MODEL)){
								if(flag_line_not_red == FALSE){
									// if(img3[idx_y] < img3[row * w_img3 + col_y_min]){
									// 	col_y_min = col;
									// } 

									if(flag_update_col_min){
										if(img3[idx_y] + TSR_EDGE_DY < img3[row * w_img3 + col_max])
											col_max = col;
										else{
											if(col_max != col_first_red)
												cnt_dy_error++;
											if(cnt_dy_error == TSR_EDGE_DY_ERROR)
												flag_update_col_min = FALSE;
										}											
									}

									if(col_last == 0){
										if(++cnt_after_red == TSR_EDGE_LEN_AFTER_RED0)
											break;
									}else{
										if(++cnt_after_red == TSR_EDGE_LEN_AFTER_RED)
											break;
									}
								}else{
									red_continuous++;								
									if(red_continuous == TSR_EDGE_TH_CONTINUE){																												
										flag_line_not_red = FALSE;
										// col_y_min = col;
										col_max = col;
										col_first_red = col;
									}
								}
							}else{
								if(flag_line_not_red){
									red_continuous = 0;
								}else{
									break;
								}
							}
						}
						if(flag_line_not_red){
							cnt_not_red_left++;
						}else{
							// if(col_last != 0) 
							// 	col_y_min = MAX(col_y_min, col_last - 1);
							// col_last = col_y_min;
							// img3[row * w_img3 + col_y_min] = 255;
							// d_left[row - TSR_EDGE_ROW_START] = col_y_min;	
							// offset_col = w_img3 / 2 - col_y_min - TSR_EDGE_TH_CONTINUE;
							d_left[row - TSR_EDGE_ROW_START] = col_max;
						}
					}
					// for(int i = 0; i < TSR_EDGE_DST_H; i++){
					// 	snprintf(txt_buffer_tsr, 128, "\t%d\t%d\t%d\n", i, d_right[i], d_left[i]);
					// 	write_txt_tsr(txt_buffer_tsr); 
					// }
					cnt_not_red_right += tsr_edge_arc_exception_weaken(d_right, TSR_EDGE_DST_H - cnt_not_red_right);
					cnt_not_red_left += tsr_edge_arc_exception_weaken(d_left, TSR_EDGE_DST_H - cnt_not_red_left);
					#if TSR_DEB
						for(int row = 0; row < TSR_EDGE_DST_H; row++){
							if(d_right[row] != 0)
								img3[row * w_img3 + d_right[row]] = 255;
							if(d_left[row] != 0)
								img3[row * w_img3 + d_left[row]] = 255;
						}																											
																																
					#endif
					qsort(d_right, TSR_EDGE_DST_H, sizeof(d_right[0]), cmp_int);
					qsort(d_left, TSR_EDGE_DST_H, sizeof(d_left[0]), cmp_int);

					for(int i = cnt_not_red_left; i < MIN(TSR_EDGE_DST_H, cnt_not_red_left + TSR_EDGE_NUM_AVER); i++){
						aver_d_left += d_left[i];
					}
					for(int i = MAX(TSR_EDGE_DST_H - TSR_EDGE_NUM_AVER, cnt_not_red_right); i < TSR_EDGE_DST_H; i++){
						aver_d_right += d_right[i];
					}
					if(TSR_EDGE_DST_H == cnt_not_red_left)
						aver_d_left = 185;
					else
						aver_d_left /= MIN(TSR_EDGE_NUM_AVER, TSR_EDGE_DST_H - cnt_not_red_left);
					if(TSR_EDGE_DST_H == cnt_not_red_right)
						aver_d_right = rect_dst_egde.w;
					else
						aver_d_right /= MIN(TSR_EDGE_NUM_AVER, TSR_EDGE_DST_H - cnt_not_red_right);


					int d_rl = aver_d_right + aver_d_left - w_img3;

					#if TSR_DEB
						snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/edge/%05d_%d_%d_%d_%ld_%ld.yuv420sp", tsr_record_cnt - 1, which_tsr, aver_d_left, aver_d_right, rect_dst_egde.w, rect_dst_egde.h);
						writeyuv420sp(txt_buffer_tsr, img3, rect_dst_egde.w, rect_dst_egde.h);
					#endif

					if(aver_d_right > w_img3 / 2 + TSR_EDGE_TH_R_TRIGGER && d_rl >= TSR_EDGE_TH_D){
						tsr_t_record.rect = rect_tsr[which_tsr];
						tsr_t_record.time_record = hd_gettime_ms();
						tsr_t_record.value = tsr_value[which_tsr];
						// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/d_%05d_%d_%d_%d_%dyyy.yyy", tsr_record_cnt - 1, which_tsr, max_d, ROI_1_w, ROI_1_h);
						// writeyyy(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[3].va), ROI_1_w, ROI_1_h);
						// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/d21_%05d_%d_%d_%d.yuv420sp", tsr_record_cnt - 1, which_tsr, TSR_IMG_W, TSR_IMG_H);
						// writeyuv420sp(txt_buffer_tsr, img21, TSR_IMG_W, TSR_IMG_H);
						// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/d3_%05d_%d_%ld_%ld.yuv420sp", tsr_record_cnt - 1, which_tsr, rect_dst_egde.w, rect_dst_egde.h);
						// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[3].va), rect_dst_egde.w, rect_dst_egde.h);
						snprintf(txt_buffer_tsr, 128, "\tTSR error: d%d:find t, %d -->0, d_rl = %d, aver_d_left = %d, aver_d_right = %d, cnt_not_red_left = %d, cnt_not_red_right = %d\n", which_tsr, tsr_value[which_tsr], d_rl, aver_d_left, aver_d_right, cnt_not_red_left, cnt_not_red_right);
						write_txt_tsr(txt_buffer_tsr);
						tsr_value[which_tsr] = 0;
						continue;
					}else{
						snprintf(txt_buffer_tsr, 128, "\td%d: d_rl = %d, aver_d_left = %d, aver_d_right = %d, cnt_not_red_left = %d, cnt_not_red_right = %d\n", which_tsr, d_rl, aver_d_left, aver_d_right, cnt_not_red_left, cnt_not_red_right);
						write_txt_tsr(txt_buffer_tsr);
						// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/d_%05d_%d_%d_%d_%d_%dyyy.yyy", tsr_record_cnt - 1, which_tsr, max_d, edge_threshold, ROI_1_w, ROI_1_h);
						// writeyyy(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[3].va), ROI_1_w, ROI_1_h);
						// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/d_%05d_%d_%d_%d.yuv420sp", tsr_record_cnt - 1, which_tsr, max_d, edge_threshold);
						// writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
					}												
				}

				//One hundred missing judge
				if(tsr_value[which_tsr] <= 30){
					// int x_max = TSR_IMG_W * p_rslt_first->x;
					// int x_min = MAX(which_tsr * 300, x_max - TSR_IMG_W * p_rslt_first->w * 0.6);
					// int h0 = p_rslt_first->y * TSR_IMG_H, h1 = (p_rslt_first->y + p_rslt_first->h) * TSR_IMG_H;;
					// int index_u, index_v;
					// int cnt_black = 0;
					// for(int col = x_min; col < x_max; col++){
					// 	for(int line = h0; line < h1; line++){
					// 		get_index_uv(TSR_IMG_W, TSR_IMG_H, line * TSR_IMG_W + col, &index_u, &index_v);
					// 		cnt_black += hsv_judge(img21[line * TSR_IMG_W + col], img21[index_u], img21[index_v], HSV_BLACK_MODEL);
					// 		// if(img21[line * TSR_IMG_W + col] < edge_threshold)
					// 		// 	cnt_black++;
					// 	}
					// }

					int x_max = inner.x;
					int x_min = x_max - inner.w * 0.3;
					int y_min = inner.y;
					int y_max = inner.y + inner.h;
					int idx_y, idx_u, idx_v;
					int cnt_black = 0;
					for(int row = y_min; row < y_max; row++){
						for(int col = x_min; col < x_max; col++){
							idx_y = row * p_video_frame->dim.w + col;
							get_index_uv(p_video_frame->dim.w, p_video_frame->dim.h, idx_y, &idx_u, &idx_v);
							if(hsv_judge(img_tsr_src[idx_y], img_tsr_src[idx_u], img_tsr_src[idx_v], HSV_BLACK_MODEL)){
								cnt_black++;
							}
						}
					}
					// draw_rect_img(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, x_min, y_min, x_max - x_min, y_max - y_min);
					// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/src/h%05d.yuv420sp", tsr_record_cnt - 1);
					// writeyuv420sp(txt_buffer_tsr, img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h);

					double black_percent = (double)cnt_black / (x_max - x_min) / (y_max - y_min);
					snprintf(txt_buffer_tsr, 128, "\th%d: black_percent = %lf\n", which_tsr, black_percent);
					write_txt_tsr(txt_buffer_tsr);
					if(black_percent > 0.08){
						snprintf(txt_buffer_tsr, 128, "\tTSR error: h%d: %d --> 0\n", which_tsr, tsr_value[which_tsr]);
						write_txt_tsr(txt_buffer_tsr);
						tsr_value[which_tsr] = 0;
					}	
					// snprintf(txt_buffer_tsr, 128, "\th%d: cnt_black=%d, x_min=%d, x_max=%d\n", which_tsr, cnt_black, x_min, x_max);
					// write_txt_tsr(txt_buffer_tsr);	

					// draw_rect_img((UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_IMG_H, x_min, h0, x_max - x_min, h1 - h0);

					// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/hundred/h_%05d_%d.yuv420sp", tsr_record_cnt - 1, which_tsr);
					// writeyuv420sp(txt_buffer_tsr, (UINT8 *)ADAS_share_mem[21].va, TSR_IMG_W, TSR_IMG_H);
				}											
			}
		}
	}else{
		for(int which_tsr = 0; which_tsr < MIN(max_tsr, tsr_cnt); which_tsr++){
			snprintf(txt_buffer_tsr, 128, "\n");
			write_txt_tsr(txt_buffer_tsr);
			if(tsr_value[which_tsr] <= 5 || tsr_value[which_tsr] % 5 != 0){
				if(tsr_value[which_tsr] > 0){
					snprintf(txt_buffer_tsr, 64, "\tresult%d %d -> 0 value <= 5 or Not a multiple of 5\n", which_tsr, tsr_value[which_tsr]);
					write_txt_tsr(txt_buffer_tsr);
					tsr_value[which_tsr] = 0;
				}
				continue;
			}
			VENDOR_AI_POSTPROC_RESULT *p_rslt_first = &out_buf.p_result[PHaveLabel[first_index_tsrnum[which_tsr]]];
			VENDOR_AI_POSTPROC_RESULT *p_rslt_second = &out_buf.p_result[PHaveLabel[1 + first_index_tsrnum[which_tsr]]];			
			VENDOR_AI_POSTPROC_RESULT *p_rslt_last = &out_buf.p_result[PHaveLabel[last_index_tsrnum[which_tsr]]];
			HD_IRECT inner;
			if( last_index_tsrnum[which_tsr] > first_index_tsrnum[which_tsr] &&
				(p_rslt_second->x - p_rslt_first->x - p_rslt_first->w > (p_rslt_second->w + p_rslt_first->w) / 3
				|| p_rslt_last->x - p_rslt_second->x - p_rslt_second->w > (p_rslt_last->w + p_rslt_second->w) / 3)){
				printf("============== Find noise like Limit num1 num2 distance too large ============\n");
				tsr_value[which_tsr] = 0;
				snprintf(txt_buffer_tsr, 128, "\tresult%d: 3006, d=%f, max=%f\n", which_tsr, p_rslt_second->x - p_rslt_first->x - p_rslt_first->w, (p_rslt_second->w + p_rslt_first->w) / 4);
				write_txt_tsr(txt_buffer_tsr);
				continue;
			}

			float w_n = p_rslt_last->x + p_rslt_last->w - p_rslt_first->x;
			float y_min = 1.0, y_max = 0.0;			
			for(int idx = first_index_tsrnum[which_tsr]; idx <= last_index_tsrnum[which_tsr]; idx++){
				if(out_buf.p_result[PHaveLabel[idx]].y < y_min){
					y_min = out_buf.p_result[PHaveLabel[idx]].y;
				}
				if(out_buf.p_result[PHaveLabel[idx]].y + out_buf.p_result[PHaveLabel[idx]].h > y_max){
					y_max = out_buf.p_result[PHaveLabel[idx]].y + out_buf.p_result[PHaveLabel[idx]].h;
				}
			}

			float h_n = y_max - y_min;
			float x_n = p_rslt_first->x, y_n = y_min;		
			if(x_n > 0.5)x_n -= 0.5;
			float alpha_u, alpha_d, alpha_l, alpha_r;
			if(tsr_type[which_tsr] == TSR_NO_A_LIMIT){
				alpha_u = PADDING_A_LIMIT_UP;
				alpha_d = PADDING_A_LIMIT_DOWN;
				alpha_l = PADDING_A_LIMIT_LEFT;
				alpha_r = PADDING_A_LIMIT_RIGHT;
			}else{
				alpha_u = PADDING_A_NIGHT_UP;
				alpha_d = PADDING_A_NIGHT_DOWN;
				alpha_l = PADDING_A_NIGHT_LEFT;
				alpha_r = PADDING_A_NIGHT_RIGHT;
			}

			// float w_ratio = (x_n * TSR_IMG_W * (1 + alpha_l + alpha_r) - alpha_l * TSR_NUM_W) / TSR_NUM_W;
			// float h_ratio = (y_n * TSR_IMG_H * (1 + alpha_u + alpha_d) - alpha_u * TSR_NUM_H) / TSR_NUM_H;
			float w_ratio = ((UINT32)(x_n * TSR_IMG_W) + ceil(x_n * TSR_IMG_W * alpha_l) + ceil(x_n * TSR_IMG_W * alpha_r) - ceil(alpha_l * TSR_NUM_W)) / TSR_NUM_W;
			float h_ratio = ((UINT32)(y_n * TSR_IMG_H) + ceil(y_n * TSR_IMG_H * alpha_u) + ceil(y_n * TSR_IMG_H * alpha_d) - ceil(alpha_u * TSR_NUM_H)) / TSR_NUM_H;
			inner.x = rect_tsr[which_tsr].x + (UINT32)(rect_tsr[which_tsr].w * w_ratio);
			inner.y = rect_tsr[which_tsr].y + (UINT32)(rect_tsr[which_tsr].h * h_ratio);
			w_ratio = ((x_n + w_n) * TSR_IMG_W * (1 + alpha_l + alpha_r) - alpha_l * TSR_NUM_W) / TSR_NUM_W;
			h_ratio = ((y_n + h_n) * TSR_IMG_H * (1 + alpha_u + alpha_d) - alpha_u * TSR_NUM_H) / TSR_NUM_H;
			inner.w = rect_tsr[which_tsr].x + (UINT32)ceil(rect_tsr[which_tsr].w * w_ratio) - inner.x;
			inner.h = rect_tsr[which_tsr].y + (UINT32)ceil(rect_tsr[which_tsr].h * h_ratio) - inner.y;

			// for(int idx = first_index_tsrnum[which_tsr]; idx <= last_index_tsrnum[which_tsr]; idx++){
			// 	int x0, y0, w0, h0;
			// 	x_n = out_buf.p_result[PHaveLabel[idx]].x;		
			// 	if(x_n > 0.5)x_n -= 0.5;
			// 	w_ratio = (x_n * TSR_IMG_W * (1 + alpha_l + alpha_r) - alpha_l * TSR_NUM_W) / TSR_NUM_W;
			// 	h_ratio = (out_buf.p_result[PHaveLabel[idx]].y * TSR_IMG_H * (1 + alpha_u + alpha_d) - alpha_u * TSR_NUM_H) / TSR_NUM_H;
			// 	x0 = rect_tsr[which_tsr].x + rect_tsr[which_tsr].w * w_ratio;
			// 	y0 = rect_tsr[which_tsr].y + rect_tsr[which_tsr].h * h_ratio;
			// 	w_ratio = ((out_buf.p_result[PHaveLabel[idx]].x + out_buf.p_result[PHaveLabel[idx]].w) * TSR_IMG_W * (1 + alpha_l + alpha_r) - alpha_l * TSR_NUM_W) / TSR_NUM_W;
			// 	h_ratio = ((out_buf.p_result[PHaveLabel[idx]].y + out_buf.p_result[PHaveLabel[idx]].h) * TSR_IMG_H * (1 + alpha_u + alpha_d) - alpha_u * TSR_NUM_H) / TSR_NUM_H;
			// 	w0 = rect_tsr[which_tsr].x + ceil(rect_tsr[which_tsr].w * w_ratio) - x0;
			// 	h0 = rect_tsr[which_tsr].y + ceil(rect_tsr[which_tsr].h * h_ratio) - y0;
			// 	draw_rect_img(img_tsr_src, 1920, 1080, x0, y0, w0, h0);
			// }
			snprintf(txt_buffer_tsr, 128, "\tresult%d: inner_number x=%ld, y=%ld, w=%ld, h=%ld\n", which_tsr, inner.x, inner.y, inner.w, inner.h);
			write_txt_tsr(txt_buffer_tsr);

			int cnt_black_white = get_color_cnt_in_rect(img_tsr_src, p_video_frame->dim.w, p_video_frame->dim.h, &inner, HSV_BLACK_MODEL + HSV_WHITE_MODEL + HSV_GRAY_MODEL);
			double ratio_black_white = (double)cnt_black_white / (double)(inner.w * inner.h);
			snprintf(txt_buffer_tsr, 128, "\tresult%d: cnt_black_white=%d, ratio_black_white=%lf\n", which_tsr, cnt_black_white, ratio_black_white);
			write_txt_tsr(txt_buffer_tsr);
			draw_rect_img(img_tsr_src, 1920, 1080, inner.x, inner.y, inner.w, inner.h);
			if(ratio_black_white < 0.5){
				tsr_value[which_tsr] = 0;
				continue;
			}

			

			

			
		}
		// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/input/round3/%05d_%d_%d.yuv420sp", tsr_record_cnt - 1, 1920, 1080);
		// writeyuv420sp(txt_buffer_tsr, img_tsr_src, 1920, 1080);
	}
  

	snprintf(txt_buffer_tsr, 64, "\n");
	write_txt_tsr(txt_buffer_tsr);
	for(int idx_tsr = 0; idx_tsr < MIN(tsr_cnt, max_tsr); idx_tsr++){
		snprintf(txt_buffer_tsr, 64, "\tresult%d: %d", idx_tsr, tsr_value[idx_tsr]);
		write_txt_tsr(txt_buffer_tsr);
	}
	snprintf(txt_buffer_tsr, 64, "\n");
	write_txt_tsr(txt_buffer_tsr);

	int max_tsr_value = tsr_value[0], max_tsr_value_idx = 0;
	for(int idx_tsr = 1; idx_tsr < MIN(tsr_cnt, max_tsr); idx_tsr++){
		if(tsr_value[idx_tsr] > max_tsr_value){
			max_tsr_value = tsr_value[idx_tsr];
			max_tsr_value_idx = idx_tsr;
		}
	}

	tsr_result.value = tsr_value[max_tsr_value_idx];
	if(tsr_result.value == 0){
		tsr_result.rect.x = 0;
		tsr_result.rect.y = 0;
		tsr_result.rect.w = 0;
		tsr_result.rect.h = 0;
	}else{
		tsr_result.rect.x = rect_tsr[max_tsr_value_idx].x;
		tsr_result.rect.y = rect_tsr[max_tsr_value_idx].y;
		tsr_result.rect.w = rect_tsr[max_tsr_value_idx].w;
		tsr_result.rect.h = rect_tsr[max_tsr_value_idx].h; 
		tsr_result.time_record = hd_gettime_ms();
	}

	// result->tsr_result.tsr_status = tsr_value[max_tsr_value_idx];
	// snprintf(txt_buffer_tsr, 128, "\tFinal result %s: %lu\n", result->tsr_result.tsr_status?"got":"noise", result->tsr_result.tsr_status);
	// write_txt_tsr(txt_buffer_tsr);

	// if(result->tsr_result.tsr_status == 0){
	// 	result->tsr_result.tsr_postproc_result.x = 0;
	// 	result->tsr_result.tsr_postproc_result.y = 0;
	// 	result->tsr_result.tsr_postproc_result.w = 0;
	// 	result->tsr_result.tsr_postproc_result.h = 0;
	// }else{
	// 	result->tsr_result.tsr_postproc_result.x = rect_tsr[max_tsr_value_idx].x;
	// 	result->tsr_result.tsr_postproc_result.y = rect_tsr[max_tsr_value_idx].y;
	// 	result->tsr_result.tsr_postproc_result.w = rect_tsr[max_tsr_value_idx].w;
	// 	result->tsr_result.tsr_postproc_result.h = rect_tsr[max_tsr_value_idx].h;
	// }
	
	
	// // if(tsr_model_is_America){
	// 	VENDOR_AI_POSTPROC_RESULT *p_r;
	// 	for(int idx = 0; idx < labelcnt; idx++){
	// 		p_r = &out_buf.p_result[PHaveLabel[idx]];
	// 		draw_rect_img((UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H, W_Img(p_r->x), H_Img(p_r->y), W_Img(p_r->w), H_Img(p_r->h));
	// 	}
	// 	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/America/every_result/%05d_%d_%d.yuv420sp", tsr_record_cnt - 1, tsr_value[0], tsr_value[1]);
	// 	snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/first/%05d.yuv420sp", tsr_record_cnt - 1);
	// 	writeyuv420sp(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, TSR_IMG_H);
	// 	// tsr_crop((UINT8 *)(ADAS_share_mem[3].va), (UINT8 *)(ADAS_share_mem[21].va), TSR_IMG_W, 0, 0, TSR_NUM_W, TSR_NUM_H);
		
	// 	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/d_%05d_%d_%dyyy.yyy", tsr_record_cnt - 1, TSR_NUM_W, TSR_NUM_H);
	// 	// writeyyy(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[3].va), TSR_NUM_W, TSR_NUM_H);
	// 	// UINT32 time_start = hd_gettime_ms();
	// 	// BilateralFilter((UINT8 *)(ADAS_share_mem[3].va), TSR_NUM_W, TSR_NUM_H);
	// 	// snprintf(txt_buffer_tsr, 128, "\ttime = %lu\n", hd_gettime_ms() - time_start);
	// 	// write_txt_tsr(txt_buffer_tsr);
	// 	// snprintf(txt_buffer_tsr, 128, "/mnt/sd/tsr/img/Europe/edge/b_%05d_%d_%dyyy.yyy", tsr_record_cnt - 1, TSR_NUM_W, TSR_NUM_H);
	// 	// writeyyy(txt_buffer_tsr, (UINT8 *)(ADAS_share_mem[3].va), TSR_NUM_W, TSR_NUM_H);
	// // }
	
	return;
}





#if FCWS || SNG
#if SNG
static float calculate_mean(int *p_queue, int num)
{
	int i, count = 0;
	// int del[20];
	float sum = 0;
	float average = 0;

	for (i = 0; i < num; i++) {
		if (0 != p_queue[i]) {
			sum += p_queue[i];
			count++;
		}
	}

	if (count > 0) {
		average = sum / count;
	}

	return average;
}
#endif

static float calculate_variance(int *p_queue, int num)
{
	int i, count = 0, del_num = 0;
	// int del[20];
	float sum = 0;
	float result;
	for (i = 0; i < num; i++) {
		if (0 == p_queue[i]) {
			count++;
			// del[del_num] = i;
			del_num++;
		} else {
			count = 0;
		}

		if (count >= 3) {
			result = 10000;
			goto skip;
		}
	}

	if (del_num >= 4) {
		result = 10000;
		goto skip;
	}

	for (i = 0; i < num; i++) {
		if (0 != p_queue[i]) {
			sum += p_queue[i];
		}
	}

	float average = sum / (num - del_num);

	sum = 0;
	for (i = 0; i < num; i++) {
		if (0 != p_queue[i]) {
			sum += (p_queue[i] - average) * (p_queue[i] - average);
		}
	}

	result = sum / (num - del_num);

skip:
	return result;
}
#endif

static HD_RESULT fcw_yuv_crop_scale(HD_VIDEO_FRAME *p_video_frame_dst, HD_VIDEO_FRAME *p_video_frame_src)
{
	HD_RESULT ret = 0;
	//copy vout0 to common buffer
	HD_GFX_SCALE param_yuv_copy;
	memset(&param_yuv_copy, 0, sizeof(HD_GFX_SCALE));
	param_yuv_copy.src_img.dim.w = p_video_frame_src->dim.w;
	param_yuv_copy.src_img.dim.h = p_video_frame_src->dim.h;
	param_yuv_copy.src_img.format = p_video_frame_src->pxlfmt;
	param_yuv_copy.src_img.p_phy_addr[0] = p_video_frame_src->phy_addr[0]; //src_pa;
	param_yuv_copy.src_img.p_phy_addr[1] = p_video_frame_src->phy_addr[1]; //src_pa;
	param_yuv_copy.src_img.lineoffset[0] = p_video_frame_src->loff[0];     //src_w
	param_yuv_copy.src_img.lineoffset[1] = p_video_frame_src->loff[1];     //src_w
	param_yuv_copy.dst_img.dim.w = p_video_frame_dst->dim.w;               //dst_w
	param_yuv_copy.dst_img.dim.h = p_video_frame_dst->dim.h;               //dst_w
	param_yuv_copy.dst_img.format = p_video_frame_dst->pxlfmt;
	param_yuv_copy.dst_img.p_phy_addr[0] = p_video_frame_dst->phy_addr[0]; //dst_pa;
	param_yuv_copy.dst_img.p_phy_addr[1] = p_video_frame_dst->phy_addr[1]; //dst_pa + 1920 * 1080;
	param_yuv_copy.dst_img.lineoffset[0] = p_video_frame_dst->loff[0];
	param_yuv_copy.dst_img.lineoffset[1] = p_video_frame_dst->loff[1];
	param_yuv_copy.src_region.x = CAR_CROP_X_BIAS;//160
	param_yuv_copy.src_region.y = CAR_CROP_Y_BIAS;//120
	param_yuv_copy.src_region.w = CAR_CROP_WIDTH;//320
	param_yuv_copy.src_region.h = CAR_CROP_HEIGHT;//180
	// param_yuv_copy.src_region.x = (INT32)(0.25 * p_video_frame_src->dim.w);
	// // param_yuv_copy.src_region.y = (INT32)(p_video_frame_src->dim.h / 3);
	// param_yuv_copy.src_region.y = (INT32)((INT32)(p_video_frame_src->dim.h / 3) + (INT32)(headstock_offst * 3));
	// param_yuv_copy.src_region.w = (INT32)(0.5 * p_video_frame_src->dim.w);
	// param_yuv_copy.src_region.h = (INT32)(0.5 * p_video_frame_src->dim.h);
	// for (int i = 0; i < 10; i++)
	//     printf("=======fcw x = %d, y = %d, w = %d, h = %d==========\n", (INT32)(0.25 * p_video_frame_src->dim.w), (INT32)(p_video_frame_src->dim.h / 3), (INT32)(0.5 * p_video_frame_src->dim.w), (INT32)(0.5 * p_video_frame_src->dim.h));
	param_yuv_copy.dst_region.x = 0;
	param_yuv_copy.dst_region.y = 0;
	param_yuv_copy.dst_region.w = p_video_frame_dst->dim.w;
	param_yuv_copy.dst_region.h = p_video_frame_dst->dim.h;
	// param_yuv_copy.quality                  = HD_GFX_SCALE_QUALITY_BILINEAR;

	ret = hd_gfx_scale_sw(&param_yuv_copy);
	if (ret != HD_OK) {
		printf("hd_gfx_scale fail=%d\n", ret);
		goto exit;
	}

exit:
	return ret;
}

void FCW_Process(UINT32 Id, HD_VIDEO_FRAME *p_video_frame, ADAS_VIDEO_INFO *p_stream, ALG_ADAS_RESULT *result)
{
	// return; 
	// static int FCW_Process_cnt = 0;
	// printf("=======FCW_Process_cnt = %d =========\n", FCW_Process_cnt++);
	//printf("=======FCW_Process_cnt ==========\n");
	// return;
	#if FCWS
	// static UINT32 FCWS_flag = 0;
	static BOOL FCW_alarm_limit = FALSE;
	// static char fcw_jilu[MEMORY_LENGTH] = {0};
	// static int sum_fcw = 0, index = 0;
	static UINT64 fcw_pre_time_ms = 0;
	static UINT64 fcw_cur_time_ms = 0;
	// static int fcw_in_alarm_num = 0;
	static int fcw_out_alarm_num = 0;

	static BOOL car_jingzhi = TRUE;
	static BOOL jingzhi_compute = FALSE;
	static UINT32 jingzhi_pingpong = 0;
	static UINT64 pre_time = 0;
	#endif

	#if SNG
	// BOOL pre_frame = TRUE;
	static HD_IRECT car_crop = {0};
	// UINT32 sng_flag = 0/*, crop_car_num = 0, car_img_loff = 200*/;
	static UINT32 ping_pong_id = 0, is_Init = 0;
	static UINT32 reg_id = 0;
	static VENDOR_MD_TRIGGER_PARAM md_trig_param;
	static LIB_MD_MDT_LIB_INFO mdt_lib_param;
	static LIB_MD_MDT_RESULT_INFO lib_md_rst;
	// UINT32 count = 0;
	static int wait_frame_num = 0;
	static UINT32 md_status = MD_CLOSE;
	static UINT32 percentage = 20;

	static UINT64 sng_pre_time_ms = 0;
	static UINT64 sng_cur_time_ms = 0;

	// static BOOL pre_jingzhi = FALSE;
	static UINT32 g_sensi = LOW_SENSI; //DEFAULT_SENSI;
	// static BOOL have_pre_car = FALSE;
	static BOOL md_init_flag = FALSE;
	static UINT32 sng_alarm_num = 0;
// static BOOL alarm_uninit_flag = FALSE;
// static UINT32 SNG_flag = 0;
// static int index = 0;
// static char newimg[64], oldimg[64], yuantu[64];
	#endif

	#if FCWS || SNG
	static int queue[MEMORY_LENGTH];
	static int op = 0;
	#if SNG
	static UINT32 i;
	static int width_queue[MEMORY_LENGTH];
	static int X_queue[MEMORY_LENGTH];
	static float car_width_mean = 0;
	static float car_X_mean = 0;
	static MYRECT record_rect = {0};
	static int pre_static_num = 0;
	static int crop_num = 0;
	static int static_num = 0;
	static int all_num = 0;
	static BOOL record_flag = FALSE;
	static UINT32 car_status = MOVE;
	// static int move_num = 0;
	static UINT32 bufsize = 0;
	// static BOOL move_flag = FALSE;
	// static UINT64 time = 0;
	static float thresold1 = 1;
	static float thresold2 = 200;
	#endif

// static UINT32 cycle_num = 1;
// static BOOL cycle_flag = FALSE;
	#endif

	#if ADAS_OUTPUT_BMP
	static char ImgFilePath[64];
	static int frmidx = 0;
	#endif

	static int car_location = 0;
	HD_RESULT ret = HD_OK;
	VENDOR_AI_BUF in_buf = {0};
	static VENDOR_AI_POSTPROC_RESULT_INFO out_buf = {0};
	static HD_VIDEO_FRAME video_frame_new = {0};
	//VENDOR_AI_BUF out_buf = {0};

	{
		#if 0 //zmd
		ret = hd_videoproc_pull_out_buf(p_stream->proc_path, &video_frame, -1); // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
		if (ret != HD_OK) {
			DBG_ERR("hd_videoproc_pull_out_buf fail (%d)\n\r", ret);
			goto skip;
		}
		#endif

		//prepare input AI_BUF from videoframe
		// in_buf.sign = MAKEFOURCC('A','B','U','F');
		// in_buf.width = p_video_frame->dim.w;
		// in_buf.height = p_video_frame->dim.h;
		// in_buf.channel = HD_VIDEO_PXLFMT_PLANE(p_video_frame->pxlfmt); //conver pxlfmt to channel count
		// in_buf.line_ofs = p_video_frame->loff[0];
		// in_buf.fmt = p_video_frame->pxlfmt;
		// in_buf.pa = p_video_frame->phy_addr[0];
		// in_buf.va = 0;
		// in_buf.size = p_video_frame->loff[0]*p_video_frame->dim.h*3/2;
		car_location = (car_location + 1) % 2;
		if (0 == car_location) {

			// video_frame_new.p_next      = NULL;
			// video_frame_new.ddr_id      = 0;
			video_frame_new.pxlfmt = HD_VIDEO_PXLFMT_YUV420;
			video_frame_new.dim.w = FCW_MODEL_WIDTH;
			video_frame_new.dim.h = FCW_MODEL_HEIGHT;
			// video_frame_new.count       = 0;
			// video_frame_new.timestamp   = hd_gettime_us();
			video_frame_new.loff[0] = FCW_MODEL_WIDTH;                                // Y
			video_frame_new.loff[1] = FCW_MODEL_WIDTH;                                // UV
			video_frame_new.phy_addr[0] = ADAS_share_mem[20].pa;                     // Y
			video_frame_new.phy_addr[1] = ADAS_share_mem[20].pa + FCW_MODEL_BUF_SIZE; // UV pack

			// UINT64 begin_time = hd_gettime_us();
			memset(&out_buf, 0, sizeof(out_buf));
			fcw_yuv_crop_scale(&video_frame_new, p_video_frame);

			// static char car_img_path[64];
			// static int car_img_num = 0;
			// snprintf(car_img_path, 64, "/mnt/sd/tsr/car_src/%05d_300_300.yuv420sp", car_img_num++);	
			// writeyuv420sp(car_img_path, (UINT8 *)ADAS_share_mem[20].va, 300, 300);
			// static char car_img_path[64];
			// static int car_img_num = 0;
			// snprintf(car_img_path, 64, "/mnt/sd/img/car/car_crop_%05d.bmp", car_img_num++);
			// writebmpfile(car_img_path, (void *)ADAS_share_mem[20].va, ADAS_IMG_WIDTH, ADAS_IMG_HEIGHT);

			// UINT64 end_time = hd_gettime_us();
			// printf("scale time = %lld\n", end_time - begin_time);

			//prepare input AI_BUF from videoframe
			in_buf.sign = MAKEFOURCC('A', 'B', 'U', 'F');
			in_buf.width = video_frame_new.dim.w;
			in_buf.height = video_frame_new.dim.h;
			in_buf.channel = HD_VIDEO_PXLFMT_PLANE(video_frame_new.pxlfmt); //conver pxlfmt to channel count
			in_buf.line_ofs = video_frame_new.loff[0];
			in_buf.fmt = video_frame_new.pxlfmt;
			in_buf.pa = video_frame_new.phy_addr[0];
			in_buf.va = 0;
			in_buf.size = video_frame_new.loff[0] * video_frame_new.dim.h * 3 / 2;

			// for (int i = 0; i < 10; i++)
			// {
			//     printf("========= net = %u =======\n", p_stream->net_path);
			// }

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
		} else {
			return;
		}

		// if(out_buf.result_num){
		// 	for(UINT32 i = 0; i < out_buf.result_num; i++){
		// 		VENDOR_AI_POSTPROC_RESULT *p_r_deb = &out_buf.p_result[i];
		// 		// draw_rect_img((UINT8 *)ADAS_share_mem[20].va, 300, 300, p_r_deb->x * 300,  p_r_deb->y * 300,  p_r_deb->w * 300,  p_r_deb->h * 300);
		// 		printf("fcw class = %lu\n", p_r_deb->no[0]);
		// 	}
		// }
		// static char car_img_path[64];
		// static int car_img_num = 0;
		// snprintf(car_img_path, 64, "/mnt/sd/tsr/car/%05d_300_300.yuv420sp", car_img_num++);	
		// writeyuv420sp(car_img_path, (UINT8 *)ADAS_share_mem[20].va, 300, 300);

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
		if (HD_OK != ret) {
			DBG_ERR("img scale fail !!\n");
			return;
		}

		#if FCWS
		UINT64 cur_time = hd_gettime_ms();
		// printf("cur_time = %lld\n", cur_time);
		// printf("pre_time = %lld\n", pre_time);
		// printf("cycle_time = %lld\n", cur_time - pre_time);
		if ((cur_time - pre_time) > 1000) {
			jingzhi_compute = TRUE;
		}

		if (jingzhi_compute) {
			switch (jingzhi_pingpong) {
			case 0:
				memcpy((void *)ADAS_share_mem[2].va, (void *)ADAS_share_mem[0].va, MD_IMG_WIDTH * MD_CROP_HEIGHT);
				jingzhi_pingpong++;
				break;
			case 1:
			case 2:
				jingzhi_pingpong++;
				break;
			case 3: {
					UINT32 re, change_num3 = 0;
					int kk1, kk2, m, n;
					UINT8 *sss = (UINT8 *)ADAS_share_mem[2].va;
					UINT8 *ddd = (UINT8 *)ADAS_share_mem[0].va;

					for (m = 0; m < MD_CROP_WIDTH; m++) {
						for (n = 0; n < MD_CROP_HEIGHT; n++) {
							kk1 = n * MD_IMG_WIDTH + m;
							kk2 = n * MD_IMG_WIDTH + m + MD_IMG_WIDTH - MD_CROP_WIDTH;
							re = abs((INT32)sss[kk1] - (INT32)ddd[kk1]);
							if (re > (UINT32)40) {
								change_num3++;
							}
							re = abs((INT32)sss[kk2] - (INT32)ddd[kk2]);
							if (re > (UINT32)40) {
								change_num3++;
							}
						}
					}

					if (change_num3 > 300) {
						car_jingzhi = FALSE;
						// printf("car_move car_move car_move car_move car_move car_move car_move car_move\n");
					} else {
						if (gps_status == TRUE && cur_speed > 15) {
							car_jingzhi = FALSE;
							// printf("car_move car_move car_move car_move car_move car_move car_move car_move\n");
						}
						car_jingzhi = TRUE;
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
		if (ret != HD_OK) {
			printf("hd_videoproc_release_out_buf fail (%d)\n\r", ret);
			return;
		}
		#endif
		UINT32 car_index = get_pre_car_index(&out_buf);
		UINT32 front_car_pos = 0;
		if (car_index != 100) {
			// result->fcw_result.fcw_postproc_result.x = (UINT32)(out_buf.p_result[car_index].x * 960 + 480);
			// result->fcw_result.fcw_postproc_result.y = (UINT32)(out_buf.p_result[car_index].y * 540 + 360);
			// result->fcw_result.fcw_postproc_result.w = (UINT32)(out_buf.p_result[car_index].w * 960);
			// result->fcw_result.fcw_postproc_result.h = (UINT32)(out_buf.p_result[car_index].h * 540);
			result->fcw_result.fcw_postproc_result.x = (UINT32)(out_buf.p_result[car_index].x * CAR_CROP_WIDTH + CAR_CROP_X_BIAS);
			result->fcw_result.fcw_postproc_result.y = (UINT32)(out_buf.p_result[car_index].y * CAR_CROP_HEIGHT + CAR_CROP_Y_BIAS);
			result->fcw_result.fcw_postproc_result.w = (UINT32)(out_buf.p_result[car_index].w * CAR_CROP_WIDTH);
			result->fcw_result.fcw_postproc_result.h = (UINT32)(out_buf.p_result[car_index].h * CAR_CROP_HEIGHT);
			// printf("car_index = %d\n", car_index);
		}
		else{
			result->fcw_result.fcw_postproc_result.x = 0;
			result->fcw_result.fcw_postproc_result.y = 0;
			result->fcw_result.fcw_postproc_result.w = 0;
			result->fcw_result.fcw_postproc_result.h = 0;
		}

		#if 0
		if (car_index != 100) {
			int car_x, car_y, car_w, car_h;
			car_x = (int)(out_buf.p_result[car_index].x * 640);
			car_y = (int)(out_buf.p_result[car_index].y * 360);
			car_w = (int)(out_buf.p_result[car_index].w * 640);
			car_h = (int)(out_buf.p_result[car_index].h * 360);

			printf("score = %f, car_x = %d, car_y = %d, car_w = %d, car_h = %d\n", out_buf.p_result[car_index].score[0], car_x, car_y, car_w, car_h);
			// printf("car_x = %f, car_y = %f, car_w = %f, car_h = %f\n", out_buf.p_result[car_index].x, out_buf.p_result[car_index].y, out_buf.p_result[car_index].w, out_buf.p_result[car_index].h);
			for (int line = 0; line < car_h; line++) {
				// memset((UINT8 *)ADAS_share_mem[20].va + (car_y + line) * 640 + car_x, 0, sizeof(UINT8) * car_w);
				for (int column = 0; column < car_w; column++) {
					if (line == 0 || line == car_h - 1 || column == 0 || column == car_w - 1) {
						*((UINT8 *)ADAS_share_mem[20].va + (car_y + line) * 640 + car_x + column) = 0;
					}
				}
			}
			static char car_img_path[64];
			static int car_img_num = 0;
			snprintf(car_img_path, 64, "//mnt//sd//img//car//car_crop_%05d.bmp", car_img_num++);
			writebmpfile(car_img_path, (UINT8 *)ADAS_share_mem[20].va, ADAS_IMG_WIDTH, ADAS_IMG_HEIGHT);
		}
		#endif
		#if SNG
		UINT32 car_width = 0;
		UINT32 car_X = 0;
		#endif

		if (car_index < 50) {
			VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[car_index];
			front_car_pos = f2iround(((p_rslt->y + p_rslt->h) * 180 + 120) * 180 / 360);
			// printf("front_car_pos = %lu\n", front_car_pos);
			// front_car_pos = f2iround(((p_rslt->y + p_rslt->h)* CAR_CROP_HEIGHT * 360 / 1080 + CAR_CROP_Y_BIAS * 360 / 1080) * 180 / 360);
			#if SNG
			car_width = f2iround(p_rslt->w * 180);
			car_X = f2iround((p_rslt->x + p_rslt->w / 2) * 180);
			#endif
		}

		if (front_car_pos > 125) {
			queue[op] = front_car_pos;
		} else {
			queue[op] = 0;
		}
		// printf("queue[%d] = %d\n", op, queue[op]);
		op = (op + 1) % MEMORY_LENGTH;
		float variance = calculate_variance(queue, MEMORY_LENGTH);
		// printf("variance = %f\n", variance);

		// printf("width_queue[%d] = %d\n", op, width_queue[op]);
		#if SNG
		if (car_width > 50) {
			width_queue[op] = car_width;
		} else {
			width_queue[op] = 0;
		}

		if (car_X > 0) {
			X_queue[op] = car_X;
		} else {
			X_queue[op] = 0;
		}
		// printf("X_queue[%d] = %d\n", op, X_queue[op]);

		// printf("front_car_pos = %d\n", front_car_pos);

		car_width_mean = calculate_mean(width_queue, MEMORY_LENGTH);
		// printf("car_width_mean = %f\n", car_width_mean);

		car_X_mean = calculate_mean(X_queue, MEMORY_LENGTH);
		// printf("car_X_mean = %f\n", car_X_mean);

		// for (int i = 0; i < 5; i++)
		// {
		//     printf("=================== car_status = %u ============\n", car_status);
		// }
		switch (car_status) {
		case MOVE:
			if (variance <= thresold1) {
				pre_static_num++;
			} else {
				pre_static_num = 0;
			}
			if (pre_static_num >= 5) {
				pre_static_num = 0;
				car_status = PRE_STATIC;
				// printf("PRE_STATIC PRE_STATIC PRE_STATIC PRE_STATIC!\n");
				record_flag = TRUE;
			}
			break;
		case PRE_STATIC:
			if (record_flag && car_index < 50) {
				VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[car_index];
				float x = (p_rslt->x * 320 + 160) * 320 / 640;
				float y = (p_rslt->y * 180 + 120) * 180 / 360;
				float w = p_rslt->w * 320 * 320 / 640;
				float h = p_rslt->h * 180 * 180 / 360;
				record_rect.x = (UINT32)(x + 0.1 * w);
				record_rect.y = (UINT32)(y + 0.2 * h);
				record_rect.w = (UINT32)(0.8 * w);
				record_rect.h = (UINT32)(0.8 * h);
				// printf("x = %d, y = %d, w = %d, h = %d\n", record_rect.x, record_rect.y, record_rect.w, record_rect.h);

				my_crop((void *)ADAS_share_mem[22].va, (void *)ADAS_share_mem[0].va, MD_IMG_WIDTH, record_rect.x, record_rect.y, record_rect.w, record_rect.h);
				bufsize = record_rect.w * record_rect.h;
				record_flag = FALSE;
				crop_num = 1;
				static_num = 0;
				all_num = 0;
			} else {
				if (crop_num == 0) {
					my_crop((void *)ADAS_share_mem[23].va, (void *)ADAS_share_mem[0].va, MD_IMG_WIDTH, record_rect.x, record_rect.y, record_rect.w, record_rect.h);

					char *ooold = (char *)ADAS_share_mem[22].va;
					char *nnnew = (char *)ADAS_share_mem[23].va;
					int change_num = 0;
					for (i = 0; i < bufsize; i++) {
						int re = fabs((INT32)(ooold[i]) - (INT32)(nnnew[i]));
						if (re > 30) {
							change_num++;
						}
					}

					memcpy(ooold, nnnew, bufsize);
					// printf("change_num = %d\n", change_num);
					if (change_num <= bufsize * 0.05) {
						static_num++;
					}
					all_num++;
				}
				crop_num = (crop_num + 1) % 3;

				if (all_num >= 5) {
					if (static_num >= 4) {
						car_status = TRUE_STATIC;
						//printf("TRUE_STATIC TRUE_STATIC TRUE_STATIC TRUE_STATIC!\n");
					} else {
						car_status = MOVE;
						pre_static_num = 0;
					}
				}
			}

			break;
		case TRUE_STATIC:
			if (variance > thresold2) {
				car_status = MOVE;
				pre_static_num = 0;
				//printf("MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE MOVE!\n");
			}
			break;
		}
		#endif

		#if FCWS
		if (variance > 2 && car_jingzhi == FALSE) {

			int Lower_boundary = f2iround(front_car_pos * ADAS_IMG_WIDTH * 1.0 / MD_IMG_WIDTH);
			// printf("Lower_boundary = %d\n", Lower_boundary);

			int alarm_distance = -2;
			if (TRUE == gps_status && cur_speed > 60) {
				alarm_distance = -25;
			}

			if (FCW_alarm_limit == FALSE && Lower_boundary >= (Y_CAT + alarm_distance)) {
				fcw_cur_time_ms = hd_gettime_ms();
				// if ((fcw_cur_time_ms - fcw_pre_time_ms) > 5000 && (fcw_cur_time_ms - sng_pre_time_ms) > 5000)
				if ((fcw_cur_time_ms - fcw_pre_time_ms) > 5000) {
					result->fcw_result.fcw_status = TRUE;
					FCW_alarm_limit = TRUE;
					fcw_pre_time_ms = fcw_cur_time_ms;
					// for (int i = 0; i < 10; i++)
					// {
					//     printf("============= *adas_pengzhuang_alarm = TRUE =======\n");
					// }
				}
			}

			if (Lower_boundary < (Y_CAT + alarm_distance)) {
				fcw_out_alarm_num++;
				if (fcw_out_alarm_num >= 7) {
					FCW_alarm_limit = FALSE;
				}
			} else {
				fcw_out_alarm_num = 0;
			}
		}

		#endif

		// if(SNG_flag == 0){

		#if SNG
		switch (md_status) {
		case MD_CLOSE:
			if (car_status == TRUE_STATIC && FALSE == md_init_flag) {
				md_status = WAIT_INIT;
				wait_frame_num = 3;
			}
			break;
		case WAIT_INIT:
			if (wait_frame_num > 0) {
				wait_frame_num--;
			} else {
				md_status = INIT;
			}
			break;
		case INIT:
			if (car_status == TRUE_STATIC && car_index < 50) {
				VENDOR_AI_POSTPROC_RESULT *p_rslt = &out_buf.p_result[car_index];
				car_crop.x = f2iround((p_rslt->x * CAR_CROP_WIDTH + CAR_CROP_X_BIAS) * MD_IMG_WIDTH / ADAS_IMG_WIDTH);
				car_crop.y = f2iround((p_rslt->y * CAR_CROP_HEIGHT + CAR_CROP_Y_BIAS) * MD_IMG_HEIGHT / ADAS_IMG_HEIGHT);
				car_crop.w = f2iround(p_rslt->w * CAR_CROP_WIDTH * MD_IMG_WIDTH / ADAS_IMG_WIDTH);
				car_crop.h = f2iround(p_rslt->h * CAR_CROP_HEIGHT * MD_IMG_HEIGHT / ADAS_IMG_HEIGHT);

				if ((car_crop.y + car_crop.h) >= 130) {
					percentage = 18;
				} else {
					percentage = 40;
				}
				// UINT32 percentage = 170 - (car_crop.y+car_crop.h);
				//printf("percentage = %d\n", percentage);

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
			if (wait_frame_num > 0) {
				wait_frame_num--;
			} else {
				md_status = TRIGGER;
			}
			break;
		case TRIGGER:
			if (car_status == TRUE_STATIC && TRUE == md_init_flag) {
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
				memcpy((UINT32 *)MD_share_mem[2].va, (UINT32 *)ADAS_share_mem[1].va, MD_IMG_BUF_SIZE / 2);

				// printf("md_trigger success\r\n");

				if (1 == is_Init) {
					bc_reorgS1((UINT8 *)MD_share_mem[6].va, (UINT8 *)MD_share_mem[10].va, MD_IMG_WIDTH, MD_IMG_HEIGHT);

					#if ADAS_OUTPUT_BMP
					frmidx++;
					printf("save save save save save save save save save save %d\r\n", frmidx);
					snprintf(ImgFilePath, 64, "//mnt//sd//img//md_output_img//output_%04d.bmp", frmidx);
					writebmpfile(ImgFilePath, (UINT8 *)MD_share_mem[10].va, MD_IMG_WIDTH, MD_IMG_HEIGHT);
					#endif

					if ((ret = lib_md_get(0, LIB_MD_RESULT_INFO, &lib_md_rst)) != HD_OK) {
						DBG_ERR("LIB_MD_RESULT_INFO fail\n");
					}

					// if(lib_md_rst.global_motion_alarm == 1) {
					//     printf("[WRN] global motion alarm\n");
					// }
					for (reg_id = 0; reg_id < mdt_lib_param.mdt_subregion_param.sub_region_num; reg_id++) {
						if (lib_md_rst.sub_motion_alarm[reg_id] == 1) {
							sng_alarm_num++;
							if (sng_alarm_num >= 2) {
								// *adas_qidong_alarm = TRUE;
								md_status = UNINIT;
								sng_cur_time_ms = hd_gettime_ms();
								if ((sng_cur_time_ms - sng_pre_time_ms) > 5000 && car_width < (car_width_mean + 1.5) && fabs(car_X_mean - car_X) < 3) {
									result->fcw_result.sng_status = TRUE;
									// for (int i = 0; i < 10; i++)
									// {
									//     printf("dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong dong\r\n");
									// }
									sng_pre_time_ms = sng_cur_time_ms;
								}
								// alarm_uninit_flag = TRUE;
								// printf("[WRN] sub_region[%d] motion alarm\n", (int)reg_id);
							}
						} else {
							sng_alarm_num = 0;
						}
					}
					// if (lib_md_rst.scene_change_alarm == 1) {
					//     printf("[WRN] scene change alarm\n");
					// }
				}
				ping_pong_id = (ping_pong_id + 1) % 2;
				if (0 == is_Init) {
					is_Init = 1;
				}
			} else {
				md_status = UNINIT;
			}
			break;
		case UNINIT:
			if (TRUE == md_init_flag) {
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

		#endif

		//printf("\n");
	}
}

void LDW_Process(UINT32 Id, HD_VIDEO_FRAME *p_video_frame, ADAS_VIDEO_INFO *p_stream, ALG_ADAS_RESULT *result)
{
	// static int LDW_Process_cnt = 0;
	// printf("=======LDW_Process_cnt = %d =========\n", LDW_Process_cnt++);
	//printf("=======LDW_Process_cnt ==========\n");
	// return;
	#if LINE_DETECT
	static int left = 0, right = 0;
	static float left_k1, left_b1, left_k2, left_b2, right_k1, right_b1, right_k2, right_b2;
	// int i, j, pos_x = 0, pos_y = 0, center, min_x, Lower_boundary = 0;
	//float k1, k2, b1, b2;
	// LANE_BOOL first_flag = TRUE;
	// _MEM_RANGE *p_share_mem = (_MEM_RANGE *)arg;

	static float seta = PI / 2;
	static int x1, x2, x3, x4;
	static BOOL FirstOpen = TRUE;

	static UINT64 lcw_pre_time_ms = 0;
	static UINT64 lcw_cur_time_ms = 0;
	static BOOL lane_init = TRUE;

	if (FirstOpen) {
		FirstOpen = FALSE;
		create_line(&lanedetector, &ADAS_share_mem[9]);
		Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
		Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
	}
	#endif //LINE_DETECT

	float cur_speed;
	BOOL gps_status = GPSMNG_GetSpeed(&cur_speed);

	BOOL lcw_flag = TRUE;
	if (TRUE == gps_status) {
		if (cur_speed < 40) {
			lcw_flag = FALSE;
			if (lane_init) {
				Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
				Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
				lanedetector.pre_left_lane.Confidence = 0;
				lanedetector.pre_right_lane.Confidence = 0;
				left_lane.Confidence = 0;
				right_lane.Confidence = 0;
				lane_init = FALSE;
			}
		} else {
			lane_init = TRUE;
		}
	}

	if (TRUE == lcw_flag) {
		#if 0 //zmd
		ret = hd_videoproc_pull_out_buf(p_stream->proc_path, &video_frame11, -1);  // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
		if (ret != HD_OK) {
			printf("hd_videoproc_pull_out_buf fail (%d)\n\r", ret);
			return 0;
		}
		#endif

		#if LINE_DETECT
		#if 0 //zmd
		ret = video_frame848to640(p_video_frame, &ADAS_share_mem[20]);
		if (ret != HD_OK) {
			printf("video scale to 640*360 fail (%d)\n\r", ret);
			return 0;
		}
		#endif

		// printf("lane thread start start start start start\r\n");
		left_lane.Confidence = 0;
		right_lane.Confidence = 0;
		left_lane.k = 0.0000001;
		right_lane.k = 0.0000001;
		x1 = WIDTH_CAT / 2;
		x2 = WIDTH_CAT / 2;
		x3 = WIDTH_CAT / 2;
		x4 = WIDTH_CAT / 2;

		//edgeDetector(&ADAS_share_mem[20], &ADAS_share_mem[4]);HD_VIDEO_FRAME

#define USE_INPUT_IMG 0
		#if USE_INPUT_IMG
		static char lwd_input_img_path[64];
		static int lwd_input_img_num = 1;
		snprintf(lwd_input_img_path, 64, "//mnt//sd//img//input_img//%05d.bmp", lwd_input_img_num++);
		readbmpfile(lwd_input_img_path, (UINT8 *)ADAS_share_mem[26].va);

		static HD_VIDEO_FRAME sd_input_frame = {0};
		sd_input_frame.pxlfmt = HD_VIDEO_PXLFMT_Y8;
		sd_input_frame.dim.w = 1920;
		sd_input_frame.dim.h = 1080;
		sd_input_frame.loff[0] = 1920;                      // Y
		sd_input_frame.phy_addr[0] = ADAS_share_mem[26].pa; // Y
		edgeDetector(&sd_input_frame, &ADAS_share_mem[4]);
		edgeDetector_headstock(&sd_input_frame, &ADAS_share_mem[24]);

		#else
		edgeDetector(p_video_frame, &ADAS_share_mem[4]);
		// edgeDetector_headstock(p_video_frame, &ADAS_share_mem[24]);
		#endif
		// headstock_offst_update(&ADAS_share_mem[25], 80, 180);

		// static int input_num = 1;
		// static char src_img_path[64]; 
		// static char ive_img_path[64];
		// snprintf(src_img_path, 64, "//mnt//sd//img//ldw//src_img_%05d.bmp", input_num);
		// snprintf(ive_img_path, 64, "//mnt//sd//img//ldw//ive//ive_img_%05d.bmp", input_num++);
		// writebmpfile(src_img_path, (UINT8 *)ADAS_share_mem[4].va, WIDTH_CAT, HEIGHT_CAT);
		// writebmpfile(ive_img_path, (UINT8 *)ADAS_share_mem[5].va, WIDTH_CAT, HEIGHT_CAT);

		// static int input_num = 1;
		// static char src_img_path[64], ive_img_path[64];
		// snprintf(src_img_path, 64, "//mnt//sd//img//src//src_img_%05d.bmp", input_num);
		// snprintf(ive_img_path, 64, "//mnt//sd//img//ive//ive_img_%05d.bmp", input_num++);
		// writebmpfile(src_img_path, (UINT8 *)ADAS_share_mem[24].va, 80, 180);
		// writebmpfile(ive_img_path, (UINT8 *)ADAS_share_mem[25].va, 80, 180);

		// int threshhold = get_Otsu_threshold(&ADAS_share_mem[5], 480, 80);

		int x_lane_divide = (int)((right_b2 - left_b2) / (left_k2 - right_k2)), th_left, th_right;
		get_Otsu_threshold_lane_divide(&ADAS_share_mem[5], 480, 80, x_lane_divide, &th_left, &th_right, left_k1, left_k2, right_k2, right_k1, left_b1, left_b2, right_b2, right_b1);
		// th_left = MAX(35, th_left);
		// th_right = MAX(35, th_right);
		#if LANE_IMG_SAVE
		static int ldw_img_num = 1;
		static char ldw_img_path[64]; 
		snprintf(ldw_img_path, 64, "/mnt/sd/img/ldw/%08d_%d_%d.bmp", ldw_img_num++, th_left, th_right);
		writebmp_ive_mask_result(ldw_img_path, (UINT8 *)ADAS_share_mem[5].va, WIDTH_CAT, HEIGHT_CAT, 0);
		#endif
		mask((UINT8 *)ADAS_share_mem[5].va, left_k1, left_b1, left_k2, left_b2, right_k1, right_b1, right_k2, right_b2);

		

		
		// static int mask_img_num = 1;
		// static char mask_img_path[64];
		// snprintf(mask_img_path, 64, "//mnt//sd//img//ldw//mask//mask_img_%05d.bmp", mask_img_num++);

        // int threshhold = get_Otsu_threshold(&ADAS_share_mem[5], 480, 80);
		// if(threshhold < 50)
		// 	threshhold = 50;
        // for (int i = 0; i < 480 * 80; i++)
        // {
        //     if (*((UINT8 *)ADAS_share_mem[5].va + i) > threshhold)
        //     {
        //         *((UINT8 *)ADAS_share_mem[5].va + i) = 255;
        //     }
        //     else
        //     {
        //         *((UINT8 *)ADAS_share_mem[5].va + i) = 0;
        //     }
        // }
		for(int line = 0;line < 80; line++)
		{
			for(int column = 0;column < 480; column++)
			{
				if(column < x_lane_divide)
				{
					if(*((UINT8 *)ADAS_share_mem[5].va + line * 480 + column) > th_left)
						*((UINT8 *)ADAS_share_mem[5].va + line * 480 + column) = 255;
					else
						*((UINT8 *)ADAS_share_mem[5].va + line * 480 + column) = 0;
				}
				else{
					if(*((UINT8 *)ADAS_share_mem[5].va + line * 480 + column) > th_right)
						*((UINT8 *)ADAS_share_mem[5].va + line * 480 + column) = 255;
					else
						*((UINT8 *)ADAS_share_mem[5].va + line * 480 + column) = 0;
				}
			}
		}

		#if LANE_IMG_SAVE
		// writebmpfile(mask_img_path, (UINT8 *)ADAS_share_mem[5].va, WIDTH_CAT, HEIGHT_CAT);
		writebmp_ive_mask_result(ldw_img_path, (UINT8 *)ADAS_share_mem[5].va, WIDTH_CAT, HEIGHT_CAT, 1);
		#endif

		clean_line(&lanedetector);

		hough_line_probabilistic(&lanedetector, WIDTH_CAT, HEIGHT_CAT, 2, 180, 12, 10, 30, 60, &ADAS_share_mem[5]);

		#if ADAS_DEBUG
		printf("lines_num = %d\r\n", lanedetector.lines_num);
		#endif

		if (lanedetector.lines_num > 0 && lanedetector.lines_num < 40) {
			lineSeparation(&lanedetector, &ADAS_share_mem[12]);

			regression(&lanedetector, &left_lane, &right_lane, &ADAS_share_mem[14]);

			if (lanedetector.left_lines_num > 30) {
				left_lane.Confidence = 0;
			}
			if (lanedetector.right_lines_num > 30) {
				right_lane.Confidence = 0;
			}

			if (left_lane.Confidence > 0) {
				#if ADAS_DEBUG
				printf("left lane :\r\nk = %f  b = %f  r = %f\r\n", left_lane.k, left_lane.b, left_lane.Confidence);
				#endif //ADAS_DEBUG
				if (left_lane.k > 0) {
					left_lane.Confidence = 0;
				}
			}
			if (right_lane.Confidence > 0) {
				#if ADAS_DEBUG
				printf("right lane :\r\nk = %f  b = %f  r = %f\r\n", right_lane.k, right_lane.b, right_lane.Confidence);
				#endif //ADAS_DEBUG
				if (right_lane.k < 0) {
					right_lane.Confidence = 0;
				}
			}
		}

		if (0 == lanedetector.pre_left_lane.k) {
			lanedetector.pre_left_lane.k += 0.00000000001;
		}
		if (0 == lanedetector.pre_right_lane.k) {
			lanedetector.pre_right_lane.k += 0.00000000001;
		}
		if (0 == left_lane.k) {
			left_lane.k += 0.00000000001;
		}
		if (0 == right_lane.k) {
			right_lane.k += 0.00000000001;
		}

		if (left_lane.Confidence >= LANE_CONFIDENCE && right_lane.Confidence >= LANE_CONFIDENCE) {
			int x = (right_lane.b - left_lane.b) / (left_lane.k - right_lane.k);
			lanedetector.center_x = x;
			x1 = x - (int)(left_lane.b / (-left_lane.k));
			x2 = (int)((-right_lane.b) / right_lane.k) - x;
			x3 = x - (int)((left_lane.b - HEIGHT_CAT) / (-left_lane.k));
			x4 = (int)(-(right_lane.b - HEIGHT_CAT) / right_lane.k) - x;
		}

		if (left_lane.Confidence >= LANE_CONFIDENCE) {
			set_pre_lane(&(lanedetector.pre_left_lane), &left_lane);
			Set_left_boundary(left_lane.k, left_lane.b, &left_k1, &left_b1, &left_k2, &left_b2);
			left = 0;
		} else {
			lanedetector.pre_left_lane.Confidence -= 4;
			left++;
		}

		if (left >= 3) {
			left = 0;
			lanedetector.center_x = WIDTH_CAT / 2;
			Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
		}

		if (right_lane.Confidence >= LANE_CONFIDENCE) {
			set_pre_lane(&(lanedetector.pre_right_lane), &right_lane);
			Set_right_boundary(right_lane.k, right_lane.b, &right_k1, &right_b1, &right_k2, &right_b2);
			right = 0;
		} else {
			lanedetector.pre_right_lane.Confidence -= 4;
			right++;
		}

		if (right >= 3) {
			right = 0;
			lanedetector.center_x = WIDTH_CAT / 2;
			Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
		}

		if (TRUE == is_reliable_pre_lane(&(lanedetector.pre_left_lane)) && TRUE == is_reliable_pre_lane(&(lanedetector.pre_right_lane))) {

			if (left_lane.Confidence >= LANE_CONFIDENCE && right_lane.Confidence < LANE_CONFIDENCE) {
				if (TRUE == is_stable_lane(&(lanedetector.pre_left_lane), &left_lane)) {
					set_prediction_lane(&(lanedetector.pre_right_lane), &right_lane);
				}
			} else if (right_lane.Confidence >= LANE_CONFIDENCE && left_lane.Confidence < LANE_CONFIDENCE) {
				if (TRUE == is_stable_lane(&(lanedetector.pre_right_lane), &right_lane)) {
					set_prediction_lane(&(lanedetector.pre_left_lane), &left_lane);
				}
			}
		}

		#if 1
		UINT32 wr = which_reliable(&left_lane, &right_lane);
		switch (wr) {
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
		#endif

		seta = PI / 2;
		if (left_lane.Confidence >= LANE_CONFIDENCE && right_lane.Confidence >= LANE_CONFIDENCE) {
			seta = (atan(left_lane.k) + atan(right_lane.k) + PI) / 2;
		}

		#if 0
		index %= MEMORY_LENGTH;
		if (left_lane.Confidence >= LANE_CONFIDENCE) {
			left_seta[index] = 1;
		} else {
			left_seta[index] = 0;
		}
		if (right_lane.Confidence >= LANE_CONFIDENCE) {
			right_seta[index] = 1;
		} else {
			right_seta[index] = 0;
		}
		index++;

		left_alarm_limit = limit_judgment(left_seta, MEMORY_LENGTH);
		right_alarm_limit = limit_judgment(right_seta, MEMORY_LENGTH);
		#endif

		//if ((seta - 1.3100114) * 100 < 3.3 && (x1 + x3) < 100)
		if ((seta - 1.3100114) * 100 < 4 && (x1 + x3) < 120) {
			// printf("left left left left left left left left left left left left left left left left!\r\n");
			// *alarmtype_addr = 1;
			// printf("=========== LDW seta = %f, threshould = %f ============\n", seta, (seta - 1.3100114) * 100);
			if (TRUE == reduce_false_alarms(x1, x2)) {
				if (TRUE == gps_status) {
					if (cur_speed > 40) {
						lcw_cur_time_ms = hd_gettime_ms();
						if ((lcw_cur_time_ms - lcw_pre_time_ms) > 5000)

						{

							result->ldw_result.ldw_status = TRUE;
							// lane_ignore_num = 40;

							lcw_pre_time_ms = lcw_cur_time_ms;
						}
					}
				} else {
					lcw_cur_time_ms = hd_gettime_ms();
					if ((lcw_cur_time_ms - lcw_pre_time_ms) > 5000) {
						result->ldw_result.ldw_status = TRUE;
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

		//if ((1.8315812 - seta) * 100 < 3.3 && (x2 + x4) < 100)
		if ((1.8315812 - seta) * 100 < 4 && (x2 + x4) < 120) {
			//1.8315812
			// printf("right right right right right right right right right right right right right!\r\n");
			// *alarmtype_addr = 2;
			if (TRUE == reduce_false_alarms(x1, x2)) {
				if (TRUE == gps_status) {
					if (cur_speed > 40) {
						lcw_cur_time_ms = hd_gettime_ms();
						if ((lcw_cur_time_ms - lcw_pre_time_ms) > 5000) {
							result->ldw_result.ldw_status = TRUE;
							lcw_pre_time_ms = lcw_cur_time_ms;
						}
					}
				} else {
					lcw_cur_time_ms = hd_gettime_ms();
					if ((lcw_cur_time_ms - lcw_pre_time_ms) > 5000) {
						result->ldw_result.ldw_status = TRUE;
						lcw_pre_time_ms = lcw_cur_time_ms;
					}
				}
			}
			Left_boundary_init(&left_k1, &left_b1, &left_k2, &left_b2);
			Right_boundary_init(&right_k1, &right_b1, &right_k2, &right_b2);
			lanedetector.pre_left_lane.Confidence = 0;
			lanedetector.pre_right_lane.Confidence = 0;
		}

		#if LANE_IMG_SAVE
		// static char reslut_img_path[64];
		// static int result_img_num = 1;
		draw_line(&left_lane, &right_lane, &ADAS_share_mem[4]);
		// snprintf(reslut_img_path, 64, "//mnt//sd//img//ldw//result//result_img_%05d.bmp", result_img_num++);
		// writebmpfile(reslut_img_path, (UINT8 *)ADAS_share_mem[4].va, WIDTH_CAT, HEIGHT_CAT);
		if(result->ldw_result.ldw_status){
			printf("%s\n", ldw_img_path);
			writebmp_ive_mask_result(ldw_img_path, (UINT8 *)ADAS_share_mem[4].va, WIDTH_CAT, HEIGHT_CAT, 2);
		}
		else
			writebmp_ive_mask_result(ldw_img_path, (UINT8 *)ADAS_share_mem[4].va, WIDTH_CAT, HEIGHT_CAT, 1);
		#endif

		#endif //LINE_DETECT

		#if 0 //zmd
		ret = hd_videoproc_release_out_buf(p_stream->proc_path, &video_frame11);  // -1 = blocking mode, 0 = non-blocking mode, >0 = blocking-timeout mode
		if (ret != HD_OK) {
			printf("hd_videoproc_release_out_buf fail (%d)\n\r", ret);
			return 0;
		}
		#endif

		if (left_lane.Confidence == 0) {
			result->ldw_result.ldw_postproc_result.left_lane_x0 = 0;
			result->ldw_result.ldw_postproc_result.left_lane_y0 = 0;
			result->ldw_result.ldw_postproc_result.left_lane_x1 = 0;
			result->ldw_result.ldw_postproc_result.left_lane_y1 = 0;
		} else {
			// result->ldw_result.ldw_postproc_result.left_lane_x0 =(int)(150 - (240 + left_lane.b / left_lane.k) * 5 / 8) * 3 + 510;
			result->ldw_result.ldw_postproc_result.left_lane_x0 =(int)((-left_lane.b / left_lane.k) * 3 * 5 / 8) + 510;
			result->ldw_result.ldw_postproc_result.left_lane_y0 = 750;
			// result->ldw_result.ldw_postproc_result.left_lane_x1 = (int)(150 - (240 - ((80 - left_lane.b) / left_lane.k) * 5 / 8)) * 3 + 510;
			result->ldw_result.ldw_postproc_result.left_lane_x1 = (int)(((80 - left_lane.b) / left_lane.k) * 3 * 5 / 8) + 510;
			result->ldw_result.ldw_postproc_result.left_lane_y1 = 900;
		}
		if (right_lane.Confidence == 0) {
			result->ldw_result.ldw_postproc_result.right_lane_x0 = 0;
			result->ldw_result.ldw_postproc_result.right_lane_y0 = 0;
			result->ldw_result.ldw_postproc_result.right_lane_x1 = 0;
			result->ldw_result.ldw_postproc_result.right_lane_y1 = 0;
		} else {
			result->ldw_result.ldw_postproc_result.right_lane_x0 = (int)((-right_lane.b / right_lane.k) * 3 * 5 / 8) + 510;
			result->ldw_result.ldw_postproc_result.right_lane_y0 = 750;
			result->ldw_result.ldw_postproc_result.right_lane_x1 = (int)(((80 - right_lane.b) / right_lane.k) * 3 * 5 / 8) + 510;
			result->ldw_result.ldw_postproc_result.right_lane_y1 = 900;
		}
	}
}

#if 0 /*ZMD 2021/11/05--15:38:42*/

#endif

