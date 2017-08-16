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
#include <unistd.h>
#include <fcntl.h> 
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/epoll.h>

#define KEY_1			2
#define KEY_2			3
#define KEY_3			4
#define KEY_4			5

struct input_event {
	struct timeval time;
	unsigned short int type;
	unsigned short int code;
	signed int value;
};

#if 0
static void help(void)
{
	printf("Usage: ./key <id>\n");
}
#endif

bool esc = false;

static void sigint_handler(int dunno)
{
	switch (dunno) {
	case SIGINT:
		esc = true;
		printf("< Ctrl+C > Press.\n");
		break;
	default:
		break;
	}
}

static void* read_handler(void* data)
{
	printf("thread run.\n");

	int epfd = epoll_create1(0);
	if (epfd < 0) {
		perror("epoll_create1");
		return NULL;
	}
	
	int evfd = open("/dev/input/event1", O_RDONLY);
	if (evfd < 0) {
		perror("[open]");
		esc = true;
	}

	struct epoll_event epoll_event;
	epoll_event.events = EPOLLIN;
	epoll_event.data.fd = evfd;
	if (epoll_ctl(epfd, EPOLL_CTL_ADD, evfd, &epoll_event) < 0) {
		perror("[epoll_ctl]");
		esc = true;
	}
	
	printf("start epoll...\n");

	struct input_event event;
	const int MAX_EVENT_NUMS = 10;
	const int TIMEOUT = 100;
	struct epoll_event *events = calloc(MAX_EVENT_NUMS, sizeof(struct epoll_event));
	if (!events) {
		perror("mem calloc");
		esc = true;
	}
	
	while (esc == false) {
		int nums = epoll_wait(epfd, events, MAX_EVENT_NUMS, TIMEOUT);
		for (int i=0; i<nums; ++i) {
			if (events[i].events & (EPOLLERR | EPOLLHUP)) {
				perror("epoll");
				continue;			
			} else if ((events[i].data.fd == evfd) && (events[i].events & EPOLLIN)) {
				int ret = read(evfd, &event, sizeof(event));
				if (ret < 0) {
					break;
				}
				//printf("[key] nums=%d code=%d value=%d\n", nums, event.code, event.value);

				switch (event.code) {
            	case KEY_1:
                	if (event.value) {
                 		printf("[KEY1] Press.\n");
                	} else {
                		printf("[KEY1] Release.\n");
            		}
            		break;
            	case KEY_2:
                	if (event.value) {
                    	printf("[KEY2] Press.\n");
            		} else {
                    	printf("[KEY2] Release.\n");
            		}
                	break;
            	case KEY_3:
                	if (event.value) {
                		printf("[KEY3] Press.\n");
            		} else {
               			printf("[KEY3] Release.\n");
           	 		}
            		break;
            	case KEY_4:
                	if (event.value) {
                		printf("[KEY4] Press.\n");
                	} else {
                  		printf("[KEY4] Release.\n");
               		}
                	break;
            	default:
                	break;
            	}
			}
		}
	}

	if (events) {
		free(events);
	}
	close(epfd);
	close(evfd);
	printf("thread exit.\n");

	pthread_exit(NULL);	

	return NULL;
}

int main(int argc, char **argv)
{
	pthread_t thread_read;
	int ret = pthread_create(&thread_read, NULL, read_handler, NULL);
	if (ret) {
		perror("[thread_create]");
		return 1;
	}

	/* Register signal */
	signal(SIGINT, sigint_handler);
	
	


	pthread_join(thread_read, NULL);
	printf("done!\n");

	return 0;
}



