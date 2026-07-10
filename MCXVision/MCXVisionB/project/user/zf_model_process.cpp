#include "zf_model_process.h"

extern "C" {

void ezh_copy_slice_to_model_input(uint32_t idx, uint32_t cam_slice_buffer, uint32_t cam_slice_width, uint32_t cam_slice_height, uint32_t max_idx)
{
    (void)idx;
    (void)cam_slice_buffer;
    (void)cam_slice_width;
    (void)cam_slice_height;
    (void)max_idx;
}

void zf_model_init(void)
{
}

void zf_model_run(void)
{
}

}
