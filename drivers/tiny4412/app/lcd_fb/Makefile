# Linux modules compile
# Author : SY
# Time   : 2017-8-10 22:29:24
#############################################################################

CC            = arm-none-linux-gnueabi-gcc
CFLAGS        = -Wall -std=gnu99 -pthread
				

TARGET        = lcd_fb
OBJS          = $(TARGET).o
INSTALL_PATH  = /opt/fs/rootfs/rootfs/tmp/

$(TARGET) : $(OBJS)
	$(CC) $(CFLAGS) -o $@ $<

install:
	chmod 755 $(TARGET)
	cp $(TARGET) $(INSTALL_PATH) 	
	
clean:
	rm -rf *.o $(TARGET) 
 
