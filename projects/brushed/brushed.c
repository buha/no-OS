#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "maxim_gpio.h"

int main()
{
	int ret;
	uint8_t state = 1;
	struct max_gpio_init_param xsw3 = {
		.vssel = MXC_GPIO_VSSEL_VDDIOH,
	};
	struct no_os_gpio_init_param sw3_config = {
		.number = 19,
		.port = 1,
		.platform_ops = &max_gpio_ops,
		.extra = &xsw3,
		.pull = NO_OS_PULL_UP,
	};

	struct no_os_gpio_desc *sw3;
	ret = no_os_gpio_get(&sw3, &sw3_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_input(sw3);
	if (ret)
		return ret;

	struct no_os_gpio_init_param out_config = {
		.number = 18,
		.port = 1,
		.platform_ops = &max_gpio_ops,
		.extra = &xsw3,
	};
	struct no_os_gpio_desc *out;
	ret = no_os_gpio_get(&out, &out_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(out, 1);
	if (ret)
		return ret;

	struct no_os_gpio_init_param motor_config = {
		.number = 17,
		.port = 2,
		.platform_ops = &max_gpio_ops,
		.extra = &xsw3,
	};
	struct no_os_gpio_desc *motor;
	ret = no_os_gpio_get(&motor, &motor_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(motor, 0);
	if (ret)
		return ret;

	while(1) {
		
		ret = no_os_gpio_get_value(sw3, &state);
		if (ret)
			return ret;
		
		no_os_mdelay(20);
		
		ret = no_os_gpio_set_value(out, state);
		if (ret)
			return ret;
		
		ret = no_os_gpio_set_value(motor, !state);
		if (ret)
			return ret;
	}
}
