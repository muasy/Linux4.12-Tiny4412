/*
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

static void help(void)
{
	printf("Usage: ./backlight <brightness>\n");
}

static void test_backlight(int argc, char **argv)
{
	if (argc < 2) {
		help();
		return;
	}

	int value = 0;
	sscanf(argv[1], "%d", &value);

	int evfd = open("/dev/backlight", O_RDWR);
    if (evfd < 0) {
    	perror("[open]");
    }

    uint8_t buff = value;
	for (int i=0; i<1; i++) {
  	 	if (write(evfd, &buff, 1) < 0) {
     		perror("write");
   		}
		usleep(20000);
	}

	close(evfd);
}

int main(int argc, char **argv)
{
	test_backlight(argc, argv);
	
	printf("done!\n");

	return 0;
}



