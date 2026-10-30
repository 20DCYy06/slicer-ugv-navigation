/* 无libc的裸机目标需要的结构体清零支持；不引入动态内存。 */
#include <stddef.h>
void *memset(void *destination,int value,size_t count){
 unsigned char *out=destination;
 for(size_t n=0;n<count;n++)out[n]=(unsigned char)value;
 return destination;
}
