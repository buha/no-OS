#include "no_os_delay.h"
#include "no_os_gpio.h"
#include "maxim_gpio.h"
#include "no_os_util.h"
#include "no_os_timer.h"
#include "maxim_timer.h"
#include "no_os_irq.h"
#include "maxim_irq.h"
#include <errno.h>
#include <string.h>
#include <stdlib.h>

enum control_mode {
	/* In one phase mode, each successive coil is energized in turn. One phase mode pro-
	* duces smooth rotations and the lowest power comsumption of the three modes due to
	* the fact that only 1 coil is energized at any moment.
	*/
	UNIPOLAR_ONEPHASE,
	/* In two phase mode, successive pairs of adjacent coils are energised in turn, so
	* the rotor settles in between two coils. The motion is not as smooth as in one phase
	* mode, power comsumption is higher but it produces greater torque.
	*/
	UNIPOLAR_TWOPHASE,
	/* The half step sequence energizes successive coils L1 and L2 in the sequence L1 ->
	* L1+L2 -> L2. In doing so, a half step granularity is achieved as well as a smoother
	* motion.
	*/
	UNIPOLAR_TWOPHASE_HALFSTEP
};

struct control_config {
	bool reverse;
	enum control_mode mode;
	unsigned int pulse_period_us;
	struct no_os_gpio_desc *_1a;
	struct no_os_gpio_desc *_1b;
	struct no_os_gpio_desc *_2a;
	struct no_os_gpio_desc *_2b;
};

struct control {
	bool spinning;
	struct no_os_gpio_desc *set[8];
	struct no_os_gpio_desc *clear[8];
	unsigned int len;
	unsigned int idx;
	unsigned int step;
	unsigned int steps;
	struct no_os_timer_desc *timer;
	enum control_mode mode;
};

void tick(void *context)
{
	struct control *control = context;
	no_os_gpio_set_value(control->clear[control->idx], 0);
	if (control->step < control->steps)
		no_os_gpio_set_value(control->set[control->idx], 1);
	control->step++;

	if(control->step <= control->steps + 1) {
		control->idx++;
		control->idx &= (control->len - 1); // make sure it overflows at 4 or 8, works because they are powers of 2
		return;
	}
	else {
		no_os_gpio_set_value(control->set[control->idx], 0);
		no_os_timer_stop(control->timer);
		control->spinning = false;
	}
}

int control_create(struct control **control)
{
	int ret;
	struct control *c = calloc(1, sizeof(*c));
	if (!c)
		return -ENOMEM;

	struct no_os_irq_ctrl_desc *nvic;
	struct no_os_irq_init_param nvic_config = {
		.irq_ctrl_id = 0,
		.platform_ops = &max_irq_ops,
	};
	ret = no_os_irq_ctrl_init(&nvic, &nvic_config);
	if (ret)
		goto error;

	ret = no_os_irq_set_priority(nvic, TMR0_IRQn, 1);
	if (ret)
		goto error_1;

	struct no_os_callback_desc timer_callback = {
		.callback = tick,
		.ctx = c,
		.event = NO_OS_EVT_TIM_ELAPSED,
		.peripheral = NO_OS_TIM_IRQ,
		.handle = MXC_TMR0,
	};
	ret = no_os_irq_register_callback(nvic, TMR0_IRQn, &timer_callback);
	if (ret)
		goto error_1;

	ret = no_os_irq_enable(nvic, TMR0_IRQn);
	if (ret)
		goto error_1;

	*control = c;
	return 0;

error_1:
	no_os_irq_ctrl_remove(nvic);
error:
	free(c);
	return ret;
}

int control_config(struct control *control, struct control_config *config)
{
	int ret;
	if (control->spinning)
		return -EBUSY;
	
	memset(control->set, 0, 8 * sizeof(control->set[0]));
	memset(control->set, 0, 8 * sizeof(control->set[0]));

	if (config->mode == UNIPOLAR_ONEPHASE) {
		control->len = 4;
		control->idx = 0;
		if (config->reverse) {
			control->set[0] = config->_1a;
			control->clear[0] = config->_2b;
			control->set[1] = config->_1b;
			control->clear[1] = config->_1a;
			control->set[2] = config->_2a;
			control->clear[2] = config->_1b;
			control->set[3] = config->_2b;
			control->clear[3] = config->_2a;
		}
		else {
			control->set[0] = config->_2b;
			control->clear[0] = config->_1a;
			control->set[1] = config->_2a;
			control->clear[1] = config->_2b;
			control->set[2] = config->_1b;
			control->clear[2] = config->_2a;
			control->set[3] = config->_1a;
			control->clear[3] = config->_1b;
		}
	}
	else if (config->mode == UNIPOLAR_TWOPHASE) {
		control->len = 4;
		control->idx = 0;
		if (config->reverse) {
			control->set[0] = config->_1a;
			control->clear[0] = config->_2a;
			control->set[1] = config->_1b;
			control->clear[1] = config->_2b;
			control->set[2] = config->_2a;
			control->clear[2] = config->_1a;
			control->set[3] = config->_2b;
			control->clear[3] = config->_1b;
		}
		else {
			control->set[0] = config->_2b;
			control->clear[0] = config->_1b;
			control->set[1] = config->_2a;
			control->clear[1] = config->_1a;
			control->set[2] = config->_1b;
			control->clear[2] = config->_2b;
			control->set[3] = config->_1a;
			control->clear[3] = config->_2a;
		}
	}
	else if (config->mode == UNIPOLAR_TWOPHASE_HALFSTEP) {
		control->len = 8;
		control->idx = 0;
		if (config->reverse) {
			control->set[0] = config->_1a;
			control->clear[1] = config->_2b;
			control->set[2] = config->_1b;
			control->clear[3] = config->_1a;
			control->set[4] = config->_2a;
			control->clear[5] = config->_1b;
			control->set[6] = config->_2b;
			control->clear[7] = config->_2a;
		}
		else {
			control->set[0] = config->_2b;
			control->clear[1] = config->_1a;
			control->set[2] = config->_2a;
			control->clear[3] = config->_2b;
			control->set[4] = config->_1b;
			control->clear[5] = config->_2a;
			control->set[6] = config->_1a;
			control->clear[7] = config->_1b;
		}
	}
	else {
		return -EINVAL;
	}
	control->mode = config->mode;

	struct no_os_timer_init_param timer_config = {
		.id = 0,
		.freq_hz = 234375,
		.ticks_count = 234375 * config->pulse_period_us / 1000000 / (control->len / 4),
		.platform_ops = &max_timer_ops,
	};

	if (control->timer)
		no_os_timer_remove(control->timer);
	
	ret = no_os_timer_init(&control->timer, &timer_config);
	if (ret)
		return ret;

	return 0;
}

int control_spin(struct control *control, unsigned int steps)
{
	if (control->spinning)
		return -EBUSY;

	control->spinning = true;
	control->step = 0;
	control->steps = steps;

	if (control->mode == UNIPOLAR_TWOPHASE_HALFSTEP)
		control->steps *= 2;

	return no_os_timer_start(control->timer);
}

int control_remove(struct control *control)
{
	free(control);
	return 0;
}

struct no_os_gpio_desc *pin1, *pin2, *pin4, *pin5;

int main()
{
	int ret;
	uint8_t state = 1;
	struct max_gpio_init_param xip = {
		.vssel = MXC_GPIO_VSSEL_VDDIOH,
	};
	struct no_os_gpio_init_param right_config = {
		.number = 19,
		.port = 1,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
		.pull = NO_OS_PULL_UP,
	};

	struct no_os_gpio_desc *right;
	ret = no_os_gpio_get(&right, &right_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_input(right);
	if (ret)
		return ret;

	struct no_os_gpio_init_param left_config = {
		.number = 21,
		.port = 1,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
		.pull = NO_OS_PULL_UP,
	};

	struct no_os_gpio_desc *left;
	ret = no_os_gpio_get(&left, &left_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_input(left);
	if (ret)
		return ret;

	struct no_os_gpio_init_param green_led_config = {
		.number = 18,
		.port = 1,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
	};
	struct no_os_gpio_desc *green_led;
	ret = no_os_gpio_get(&green_led, &green_led_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(green_led, 1);
	if (ret)
		return ret;

	struct no_os_gpio_init_param red_led_config = {
		.number = 14,
		.port = 1,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
	};
	struct no_os_gpio_desc *red_led;
	ret = no_os_gpio_get(&red_led, &red_led_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(red_led, 1);
	if (ret)
		return ret;

	struct no_os_gpio_init_param pin1_config = {
		.number = 15,
		.port = 2,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
	};
	ret = no_os_gpio_get(&pin1, &pin1_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(pin1, 0);
	if (ret)
		return ret;

	struct no_os_gpio_init_param pin2_config = {
		.number = 13,
		.port = 2,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
	};
	ret = no_os_gpio_get(&pin2, &pin2_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(pin2, 0);
	if (ret)
		return ret;

	struct no_os_gpio_init_param pin4_config = {
		.number = 18,
		.port = 2,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
	};
	ret = no_os_gpio_get(&pin4, &pin4_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(pin4, 0);
	if (ret)
		return ret;

	struct no_os_gpio_init_param pin5_config = {
		.number = 17,
		.port = 2,
		.platform_ops = &max_gpio_ops,
		.extra = &xip,
	};
	ret = no_os_gpio_get(&pin5, &pin5_config);
	if (ret)
		return ret;
	
	ret = no_os_gpio_direction_output(pin5, 0);
	if (ret)
		return ret;

	struct control *motor;
	ret = control_create(&motor);
	if (ret)
		return ret;

	while(1) {
		ret = no_os_gpio_get_value(right, &state);
		if (ret)
			return ret;
		
		if (!state) {
			ret = no_os_gpio_set_value(green_led, 0);
			if (ret)
				return ret;
			
			struct control_config motor_config = {
				.mode = UNIPOLAR_TWOPHASE_HALFSTEP,
				.reverse = true,
				.pulse_period_us = 5500,
				._1a = pin1,
				._1b = pin5,
				._2a = pin2,
				._2b = pin4,
			};
			ret = control_config(motor, &motor_config);
			if (ret)
				return ret;

			control_spin(motor, 48); while(motor->spinning);
		}

		ret = no_os_gpio_get_value(left, &state);
		if (ret)
			return ret;

		if (!state) {
			ret = no_os_gpio_set_value(red_led, 0);
			if (ret)
				return ret;
			
			struct control_config motor_config = {
				.mode = UNIPOLAR_TWOPHASE_HALFSTEP,
				.reverse = false,
				.pulse_period_us = 5500,
				._1a = pin1,
				._1b = pin5,
				._2a = pin2,
				._2b = pin4,
			};
			ret = control_config(motor, &motor_config);
			if (ret)
				return ret;
			
			control_spin(motor, 48); while(motor->spinning);
		}

		no_os_mdelay(100);
		no_os_gpio_set_value(green_led, 1);
		no_os_gpio_set_value(red_led, 1);
	}
}
