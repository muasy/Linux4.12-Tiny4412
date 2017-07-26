#ifndef __USB4604_H__
#define __USB4604_H__

#define USB4604_I2C_NAME	"USB4604"

#define USB4604_OFF_PORT1	(1 << 1)
#define USB4604_OFF_PORT2	(1 << 2)
#define USB4604_OFF_PORT3	(1 << 3)

enum USB4604_mode {
	USB4604_MODE_UNKNOWN,
	USB4604_MODE_HUB,
	USB4604_MODE_STANDBY,
};

struct USB4604_platform_data {
	enum USB4604_mode	initial_mode;
	u8	port_off_mask;
	int	gpio_intn;
	int	gpio_connect;
	int	gpio_reset;
};

#endif

