/*
 * adc driver for tiny4412
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


#define DEVICE_NAME		"/sys/bus/iio/devices/iio\:device0/"
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

struct adc_dev {
	uint8_t chn;
	const char *dev;
};

const struct adc_dev adc_dev_table[] = {
	{0, 	"in_voltage0_raw"},
	{1, 	"in_voltage1_raw"},
	{2, 	"in_voltage2_raw"},
	{3, 	"in_voltage3_raw"},
};


#if 1
static void help(void)
{
	printf("Usage: ./adc <chn>\n");
}
#endif

int adc_chn;

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

const char* match_adc_dev(uint8_t chn)
{
	for (uint8_t i=0; i<ARRAY_SIZE(adc_dev_table); ++i) {
		if (chn == adc_dev_table[i].chn) {
			return adc_dev_table[i].dev;
		}
	}

	return NULL;
}

static bool read_adc(char *dev, int32_t *code)
{
	int fd = open(dev, O_RDONLY);
	if (fd < 0) {
		perror("open error");
		return false;
	}
	
	char value[32];
	int ret = read(fd, value, sizeof(value));
	if (ret < 0) {
		perror("read error");
		return false;	
	}
	int32_t adc = 0;
	sscanf(value, "%d", &adc);

	close(fd);	

	*code = adc;
	return true;
}

static void* read_handler(void* data)
{
	printf("thread run.\n");
	char dev_name[64];
	strcpy(dev_name, DEVICE_NAME);
	const char *dev = match_adc_dev(adc_chn);
		
	if (!dev) {
		printf("it can't find device!\n");
		return NULL;
	}
	strcat(dev_name, dev);

	while (esc == false) {	
		int32_t code = 0;
		if (read_adc(dev_name, &code) == false) {
			printf("read_adc error!\n");
			return NULL;
		}

		printf("[%d] %d\n", adc_chn, code);
		usleep(20*1000);
	}
	pthread_exit(NULL);	

	return NULL;
}

int main(int argc, char **argv)
{
	if (argc != 2) {
		help();
		exit(0);
	}

	adc_chn = atoi(argv[1]);

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



