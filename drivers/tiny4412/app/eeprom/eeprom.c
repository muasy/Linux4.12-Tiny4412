/*
 * eeprom driver for tiny4412
 *
 * Copyright (c) 2017
 * Author: SY <1530454315@qq.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version. 
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h> 
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#if 1
static void help(void)
{
	printf("Usage:\n");
	printf("    read: ./eeprom r [i2c_addr] [dev_addr] [lenth]\n");
	printf("   write: ./eeprom w [i2c_addr] [dev_addr] [lenth] ...\n");
}
#endif


#pragma pack(1)

struct i2c_data {
	uint8_t addr;
	uint8_t data[0];
};

#pragma pack()

bool i2c_write(int fd, uint8_t i2c_addr, uint16_t dev_addr, uint8_t *buf, uint16_t len)
{
	bool ret = true;
	int i;
	struct i2c_rdwr_ioctl_data i2c;
	i2c.nmsgs = 1;
	i2c.msgs = (struct i2c_msg *)calloc(i2c.nmsgs, sizeof(struct i2c_msg));
		
	struct i2c_msg *p = &i2c.msgs[0];
	p->addr = i2c_addr;
	p->flags = 0;
	p->len = sizeof(struct i2c_data) + len;
	p->buf = calloc(1, p->len);
	struct i2c_data *data = (struct i2c_data *)p->buf;
	data->addr = dev_addr;
	memcpy(data->data, buf, len);	
	
	int res = ioctl(fd, I2C_RDWR, &i2c);
	if (res < 0) {
		perror("Write ioctl error");
		ret = false;
		goto error;
	}
	printf("WRITE\n");
	for (i=0; i<len; ++i) {
		printf("%x ", buf[i]);
	}
	printf("\n");

error:
	free(p->buf);
	free(i2c.msgs);

	return ret;
}


bool i2c_read(int fd, uint8_t i2c_addr, uint16_t dev_addr, uint8_t *buf, uint16_t len)
{
	bool ret = true;
	struct i2c_rdwr_ioctl_data i2c;
	i2c.nmsgs = 2;
	i2c.msgs = (struct i2c_msg *)calloc(i2c.nmsgs, sizeof(struct i2c_msg));

	/* First Write addr */
	struct i2c_msg *p1 = &i2c.msgs[0];
	p1->addr = i2c_addr;
	p1->flags = 0;
	p1->len = sizeof(struct i2c_data);
	p1->buf = calloc(1, p1->len);
	((struct i2c_data *)p1->buf)->addr = dev_addr;

	/* Read */	
	struct i2c_msg *p2 = &i2c.msgs[1];
	p2->addr = i2c_addr;
	p2->flags = I2C_M_RD;
	p2->len = len;
	p2->buf = buf;
	
 	int res = ioctl(fd, I2C_RDWR, &i2c);
	if (res < 0) {
		perror("Read ioctl error");
		ret = false;
		goto error;
	}
	
error:
	free(p1->buf);
	free(i2c.msgs);
	
	return ret;
}


int main(int argc, char **argv)
{
	if (argc < 2) {
		help();
		exit(0);
	}
	char rw = *argv[1];
	if (rw == 'r') {
		if (argc != 5) {
			help();
			exit(0);
		}
	} else if (rw == 'w') {
		if (argc != 6) {
			help();
			exit(0);
		}
	} else {
		help();
		exit(0);
	}

	int i2c_addr;
	sscanf(argv[2], "%x", &i2c_addr);
	int dev_addr;
	sscanf(argv[3], "%x", &dev_addr);
	uint16_t len = atoi(argv[4]);
	printf("> i2c_addr = %x, dev_addr = %x, lenth = %d\n", i2c_addr, dev_addr, len);

	int fd = open("/dev/i2c-0", O_RDWR);
	if(!fd) {
		printf("open /dev/i2c-0 return error\n");
		exit(0);
	}

	int i;
	switch (rw) {
	case 'r': {
		uint8_t *buff = calloc(1, len);
		if (buff == NULL) {
			printf("calloc error!\n");
			goto error1;
		}
		if (i2c_read(fd, i2c_addr, dev_addr, buff, len) == false) {
			free(buff);
			goto error1;
		}

		printf("READ:\n");
		for (i=0; i<len; ++i) {
			printf("%x ", buff[i]);
		}
		printf("\n");
		free(buff);
		break;
	}
	case 'w': {
		uint8_t *buff = calloc(1, len);
		if (buff == NULL) {
			printf("calloc error!\n");
			goto error1;
		}
		memcpy(buff, argv[5], len);
		if (i2c_write(fd, i2c_addr, dev_addr, buff, len) == false) {
			free(buff);
			goto error1;
		}
		free(buff);
		break;
	}
	default:
		help();
		break;
	}

error1:
	close(fd);

	return 0;
}

