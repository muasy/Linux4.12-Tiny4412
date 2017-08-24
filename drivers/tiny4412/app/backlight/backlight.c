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


#if 0
static void help(void)
{
	printf("Usage: ./key <id>\n");
}
#endif

int main(int argc, char **argv)
{
	int evfd = open("/dev/backlight_1wire", O_RDWR);
    if (evfd < 0) {
    	perror("[open]");
    }

	for (int i=0; i<=127; i++) {
        uint8_t buff = i;
        if (write(evfd, &buff, 1) < 0) {
            perror("write");
        }
		printf("backlight: %d\n", i);

		usleep(50000);
	}

	close(evfd);
	printf("done!\n");

	return 0;
}



