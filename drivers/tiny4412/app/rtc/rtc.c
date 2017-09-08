/*
 * rtc driver for tiny4412
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
#include <linux/rtc.h>


#if 1
static void help(void)
{
	printf("Usage:\n");
	printf("    read: ./rtc r\n");
	printf("    write: ./rtc w [year] [month] [day] [hour] [minuter] [second]\n");
}
#endif

bool set_time(int fd, struct rtc_time *time)
{
	int ret = ioctl(fd, RTC_SET_TIME, time);
	if (ret < 0) {
		perror("set rtc time error");
		return false;
	}

	return true;
}

bool get_time(int fd, struct rtc_time *time)
{
	int ret = ioctl(fd, RTC_RD_TIME, time);
	if (ret < 0) {
		perror("get rtc time error");
		return false;
	}

	printf("> %04d-%02d-%02d %02d:%02d:%02d\n", 1900+time->tm_year, time->tm_mon, time->tm_mday,
		time->tm_hour, time->tm_min, time->tm_sec);

	return true;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		help();
		exit(0);
	}
	char rw = argv[1][0];
	if (rw == 'w') {
		if (argc != 8) {
			help();
			exit(0);
		}
	}
	
	int fd = open("/dev/rtc0", O_RDWR);
	if(!fd) {
		printf("open /dev/rtc0 return error\n");
		exit(0);
	}

	struct rtc_time time = {0};
	switch (rw) {
	case 'r': {
		get_time(fd, &time);
		break;
	}
	case 'w': {
		time.tm_year = atoi(argv[2]) - 1900;
		time.tm_mon = atoi(argv[3]);
		time.tm_mday = atoi(argv[4]);
		time.tm_hour = atoi(argv[5]);
		time.tm_min = atoi(argv[6]);
		time.tm_sec = atoi(argv[7]);
		
		set_time(fd, &time);
		break;
	}
	default:
		help();
		break;
	}

	close(fd);

	return 0;
}

