#include "VescUart.h"
#include "server.h"

int main()
{
    create_socket();

    char send_buffer[256];
    for (int i=0;i<256;i++)
        send_buffer[i] = i;

    int n = write_socket(send_buffer, 256);


    while (1) {
       VescUartSetCurrent(2.5);
    }
    return 0;
}


