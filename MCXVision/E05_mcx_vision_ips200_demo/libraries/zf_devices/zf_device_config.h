#ifndef _zf_device_config_h_
#define _zf_device_config_h_

#include "zf_common_headfile.h"

extern short int    scc8660_id;
unsigned char       scc8660_set_config_sccb         (void *soft_iic_obj, short int buff[13][2]);
unsigned char       scc8660_set_brightness_sccb     (unsigned short int brightness);
unsigned char       scc8660_set_manual_wb_sccb      (unsigned short int manual_wb_r, unsigned short int manual_wb_g, unsigned short int manual_wb_b);
unsigned char       scc8660_set_reg_sccb            (unsigned char reg, unsigned short int data);
#endif

