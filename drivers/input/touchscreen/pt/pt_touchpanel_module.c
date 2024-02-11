#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <soc/qcom/sh_smem.h>

#include "pt_touchpanel_module.h"

static struct touchpanel_module_data touchpanel_module_data;
static struct device *touchpanel_module_data_dev = NULL;
static struct mutex touchpanel_module_data_lock;

void pt_touchpanel_module_set_data(struct device *dev, struct touchpanel_module_data *data)
{
	mutex_lock(&touchpanel_module_data_lock);

	if(touchpanel_module_data_dev == NULL) {
		touchpanel_module_data_dev = dev;

		touchpanel_module_data.dev 							= data->dev;
		touchpanel_module_data.pwr_reg_name 				= data->pwr_reg_name;
		touchpanel_module_data.bus_reg_name 				= data->bus_reg_name;
		touchpanel_module_data.pinctrl 						= data->pinctrl;
		touchpanel_module_data.panel_kind_state_pullup 		= data->panel_kind_state_pullup;
		touchpanel_module_data.panel_kind_state_pulldown	= data->panel_kind_state_pulldown;
		touchpanel_module_data.panel_kind_gpio 				= data->panel_kind_gpio;
	}

	mutex_unlock(&touchpanel_module_data_lock);

	return;
}
EXPORT_SYMBOL(pt_touchpanel_module_set_data);

void pt_touchpanel_module_clear_data(struct device *dev)
{
	mutex_lock(&touchpanel_module_data_lock);

	if(touchpanel_module_data_dev == dev) {
		touchpanel_module_data_dev = NULL;
	}

	mutex_unlock(&touchpanel_module_data_lock);

	return;
}
EXPORT_SYMBOL(pt_touchpanel_module_clear_data);

int pt_touchpanel_module_update_panel_kind(void)
{
	int val = 0;
	sharp_smem_common_type *sh_smem_addr = NULL;
	int touchpanel_kind;
	struct regulator *pwr_reg;
	struct regulator *bus_reg;

	mutex_lock(&touchpanel_module_data_lock);

	if(touchpanel_module_data_dev == NULL) {
		mutex_unlock(&touchpanel_module_data_lock);
		pr_err("touchpanel_update_module_kind error. touchpanel_module_data_dev == NULL\n");
		return -EFAULT;
	}

	pwr_reg = regulator_get(touchpanel_module_data.dev, touchpanel_module_data.pwr_reg_name);
	if (IS_ERR(pwr_reg)) {
		mutex_unlock(&touchpanel_module_data_lock);
		pr_err("touchpanel_update_module_kind error. regulator_get(pwr_reg_name) error.\n");
		return -EFAULT;
	}
	bus_reg = regulator_get(touchpanel_module_data.dev, touchpanel_module_data.bus_reg_name);
	if (IS_ERR(bus_reg)) {
		regulator_put(pwr_reg);
		mutex_unlock(&touchpanel_module_data_lock);
		pr_err("touchpanel_update_module_kind error. regulator_get(bus_reg_name) error.\n");
		return -EFAULT;
	}

	sh_smem_addr = sh_smem_get_common_address();

	regulator_enable(pwr_reg);
	regulator_enable(bus_reg);

	pinctrl_select_state(touchpanel_module_data.pinctrl, touchpanel_module_data.panel_kind_state_pullup);

	udelay(10000);

	gpio_request(touchpanel_module_data.panel_kind_gpio, "panel_kind");
	val = gpio_get_value(touchpanel_module_data.panel_kind_gpio);
	gpio_free(touchpanel_module_data.panel_kind_gpio);

	if(val == 0) {
		touchpanel_kind = SH_BOOT_TOUCHPANEL_KIND_TT41701;
	}
	else {
		pinctrl_select_state(touchpanel_module_data.pinctrl, touchpanel_module_data.panel_kind_state_pulldown);

		udelay(10000);

		gpio_request(touchpanel_module_data.panel_kind_gpio, "panel_kind");
		val = gpio_get_value(touchpanel_module_data.panel_kind_gpio);
		gpio_free(touchpanel_module_data.panel_kind_gpio);


		if(val != 0) {
			touchpanel_kind = SH_BOOT_TOUCHPANEL_KIND_FT3519;
		}
		else {
			touchpanel_kind = SH_BOOT_TOUCHPANEL_KIND_TT7002;
		}
	}

	regulator_disable(pwr_reg);
	regulator_disable(bus_reg);

	regulator_put(pwr_reg);
	regulator_put(bus_reg);

	pr_err("update touchpanel_kind (%d -> %d)\n", sh_smem_addr->shtps_touchpanel_kind, touchpanel_kind);

	sh_smem_addr->shtps_touchpanel_kind = touchpanel_kind;


	mutex_unlock(&touchpanel_module_data_lock);

	return sh_smem_addr->shtps_touchpanel_kind;
}
EXPORT_SYMBOL(pt_touchpanel_module_update_panel_kind);

int pt_touchpanel_module_get_panel_kind(void)
{
	sharp_smem_common_type *sh_smem_addr = NULL;

	sh_smem_addr = sh_smem_get_common_address();

	return sh_smem_addr->shtps_touchpanel_kind;
}
EXPORT_SYMBOL(pt_touchpanel_module_get_panel_kind);

int pt_touchpanel_module_init(void)
{
	mutex_init(&touchpanel_module_data_lock);
	return 0;
}
EXPORT_SYMBOL(pt_touchpanel_module_init);

void pt_touchpanel_module_exit(void)
{
	return;
}
EXPORT_SYMBOL(pt_touchpanel_module_exit);

MODULE_VERSION("1.0");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_DESCRIPTION("TOUCH EVENT NOTIFIER");
