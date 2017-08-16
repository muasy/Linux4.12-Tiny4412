/*
 * beep driver for tiny4412
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
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h> 

#define EV_SND			0x12
#define SND_BELL		0x01
#define SND_TONE		0x02

struct input_event {
	struct timeval time;
	unsigned short int type;
	unsigned short int code;
	signed int value;
};

static void help(void)
{
	printf("Usage: ./beep <duty>\n");
}


int main(int argc, char **argv)
{
	if (argc < 2) {
		help();
		return 1;
	}

	int fd = open("/dev/input/event0", O_RDWR);
	if (fd < 0) {
		perror("[open]");
		return 1;
	}

	int duty = atoi(argv[1]);

	struct input_event event;	
	event.type = EV_SND;
	event.code = SND_TONE;
	event.value = duty;
	
	if (write(fd, &event, sizeof(event)) < 0) {
		perror("[write]");
		goto error;	
	}
	close(fd);

	printf("done!\n");

	return 0;

error:
	close(fd);
	return 1;
}



