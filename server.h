#ifndef SERVER_H
#define SERVER_H
#include <sys/types.h>

int create_socket();
int read_socket(char* data);
int write_socket(char* data, size_t size);
void close_socket();

#endif // SERVER_H
