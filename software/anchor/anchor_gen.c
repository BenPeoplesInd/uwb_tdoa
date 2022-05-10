#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include <hiredis/hiredis.h>

void DumpHex(const void* data, size_t size) {
	char ascii[17];
	size_t i, j;
	ascii[16] = '\0';
	for (i = 0; i < size; ++i) {
		printf("%02X ", ((unsigned char*)data)[i]);
		if (((unsigned char*)data)[i] >= ' ' && ((unsigned char*)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char*)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i+1) % 8 == 0 || i+1 == size) {
			printf(" ");
			if ((i+1) % 16 == 0) {
				printf("|  %s \n", ascii);
			} else if (i+1 == size) {
				ascii[(i+1) % 16] = '\0';
				if ((i+1) % 16 <= 8) {
					printf(" ");
				}
				for (j = (i+1) % 16; j < 16; ++j) {
					printf("   ");
				}
				printf("|  %s \n", ascii);
			}
		}
	}
}

int main(int argc, char *argv[])
{

		if(argc < 2)
		{
			printf("Usage: anchor_gen tag_id\n");
			exit(-1);
		}

    redisContext *c;
    redisReply *reply;

		char mac_string[32];
		memset(mac_string,0x00,32);
		strncpy(mac_string,argv[1],32);

    struct timeval timeout = { 0, 500000 }; // 0.5 seconds
  		c = redisConnectWithTimeout("127.0.0.1", 6379, timeout);
  	 if (c == NULL || c->err) {
  			 if (c) {
  					 printf("Connection error: %s\n", c->errstr);
  					 redisFree(c);
  			 } else {
  					 printf("Connection error: can't allocate redis context\n");
  			 }
  			 exit(3);
  	 }



/*
reply = redisCommand(c,"HGET %s key",mac_string);
memset(key,0x00,32);
if(reply->len <= 32)
 memcpy(key,reply->str,reply->len);
freeReplyObject(reply);
*/

}
