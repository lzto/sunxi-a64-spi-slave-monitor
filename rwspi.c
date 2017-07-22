/*
 * SPI monitor userland program
 * 2017 Tong Zhang<ztong@vt.edu>
 */
#include <stdio.h>
#include <stdlib.h>

unsigned char c;

int main()
{
    FILE *fp = fopen("/dev/spi_mon", "r");
    if (!fp)
    {
        return -1;
    }
    int cnt = 0;
    while(1)
    {
        if (fread(&c, 1, 1, fp))
        {
            printf("0x%02x ", c);
            cnt++;
            if(cnt%16==0)
            {
                printf("\n");
            }
        }
    }
    fclose(fp);
    return 0;
}

