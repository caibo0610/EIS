#ifndef __VIDEO_RESIZE_H
#define __VIDEO_RESIZE_H

extern "C" {

#include <stdint.h>

//#define USE_CPU
#define RSDEBUG
#ifdef RSDEBUG
#define rslog(fmt, args...) \
do { \
    struct timeval now; \
    gettimeofday(&now, NULL); \
    fprintf(stderr, "[%5lu.%06lu][ION MGR] %s() %d: " fmt "\n", \
            now.tv_sec, now.tv_usec, __FUNCTION__, __LINE__, ##args); \
} while (0)
#else 
#define rslog(fmt, args...)
#endif

typedef struct c2d_res{
    int w;
    int h;
} c2d_res_t;    

// void convertInit(c2d_res_t in, c2d_res_t out);
// void convertC2D(char *buff_in, int size_in, char **buff_out, int *size_out);

typedef void (*pfn_completion_cb_t) (void *priv, int buf_idx, unsigned int frame_cnt);

void *c2d_create_instance();
void c2d_destroy_instance(void *phandle);
int  c2d_configure(void *phandle, int bufs_in, struct c2d_res in, struct c2d_res out);
void c2d_set_output_buffer_pool(void *phandle, char *ion_pool[], int n_descs[], int nmemb);
void c2d_set_completion_callback(void *phandle, pfn_completion_cb_t pfn_cb);
void c2d_set_callback_data(void *phandle, void *priv);
int  c2d_process_image(void *phandle, char *yuv_buf, uint64_t timestamp, int ion_desc);

void* getIonMgr(void);
}

#endif
