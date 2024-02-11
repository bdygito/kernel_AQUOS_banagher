#ifndef _cyttsp5_TOUCHPANEL_MODULE_H_
#define _cyttsp5_TOUCHPANEL_MODULE_H_

#define SH_BOOT_TOUCHPANEL_KIND_TT7002		0
#define SH_BOOT_TOUCHPANEL_KIND_TT41701		1
#define SH_BOOT_TOUCHPANEL_KIND_FT3519		2

struct touchpanel_module_data {
	struct device *dev;
	char *pwr_reg_name;
	char *bus_reg_name;
	struct pinctrl *pinctrl;
	struct pinctrl_state *panel_kind_state_pullup;
	struct pinctrl_state *panel_kind_state_pulldown;
	int panel_kind_gpio;
};

void cyttsp5_touchpanel_module_set_data(struct device *dev, struct touchpanel_module_data *data);
void cyttsp5_touchpanel_module_clear_data(struct device *dev);

int cyttsp5_touchpanel_module_update_panel_kind(void);
int cyttsp5_touchpanel_module_get_panel_kind(void);

int cyttsp5_touchpanel_module_init(void);
void cyttsp5_touchpanel_module_exit(void);
#endif /* _cyttsp5_TOUCHPANEL_MODULE_H_ */
