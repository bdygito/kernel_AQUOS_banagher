#ifndef _FP_NAVI_INPUT_H_
#define _FP_NAVI_INPUT_H_

void uinput_egis_init(struct egis_data *egis);
void sysfs_egis_init(struct egis_data *egis);
void sysfs_egis_destroy(struct egis_data *egis);
void uinput_egis_destroy(struct egis_data *egis);

#endif
