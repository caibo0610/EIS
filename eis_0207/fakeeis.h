#ifndef _FAKEEIS_H
#define _FAKEEIS_H
//------------------------------------------------------------------------------
// EIS API.
//------------------------------------------------------------------------------
struct img_res {
	int width;
	int height;
};

struct impr_cs_data_eis
{
       	//data and size
        uint8_t      size;
	    uint8_t     *data;
};


void * create_instance();

void destroy_instance(void *phandle);

int configure(void *phandle, int bufs_in, struct img_res in, struct img_res out);

void set_output_buffer_pool(void *phandle, char *bufpool[], int buffd[], int nmemb);

// the priv is only used by the upper layer caller.
void set_callback_data(void *phandle, void *priv);

// to tell which output buffer is ok.
typedef void (*pfn_completion_cb_t)(void *priv, int buf_idx, unsigned int frame_cnt);
void set_completion_callback(void *phandle, pfn_completion_cb_t pfn_cb);

typedef void (*pfn_completion_cb_app_t)(void *priv, struct impr_cs_data_eis *d);
void set_completion_callback_APP(void *phandle, pfn_completion_cb_app_t pfn_cb);


// to tell which input buffer can be free.
typedef void (*pfn_free_cb_t)(void *priv, char *yuv_buf);
void set_free_callback(void *phandle, pfn_free_cb_t pfn_cb);

// this function should return quickly, leaving the heavy work to another
// private thread, which will call free_cb() and completion_cb() later.
int process_image(void *phandle, char *yuv_buf, uint64_t timestamp, int buffd);

int8_t impr_eis_process_appdata(void *phandle, struct impr_cs_data_eis *d);

// open :
// 			1  : open eis
// 			0 : close eis
void eis_control(int open);
void set_resolution(img_res in, img_res out);


void setIonBuffMgr(void *mgr);
#endif //_FAKEEIS_H
