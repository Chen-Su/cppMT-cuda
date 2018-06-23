#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <algorithm>
#ifndef _SHA1_H_
#define _SHA1_H_
typedef struct SHA1Context{
	unsigned Message_Digest[5];
	unsigned Length_Low;
	unsigned Length_High;
	unsigned char Message_Block[64];
	int Message_Block_Index;
	int Computed;
	int Corrupted;
} SHA1Context;

void SHA1Reset(SHA1Context *);
int SHA1Result(SHA1Context *);
void SHA1Input(SHA1Context *, const char *, unsigned);



#define SHA1CircularShift(bits,word) ((((word) << (bits)) & 0xFFFFFFFF) | ((word) >> (32-(bits))))

void SHA1ProcessMessageBlock(SHA1Context *);
void SHA1PadMessage(SHA1Context *);

bool sha1_hash(const char *source, char *buf123);
#endif
