/*************************************************************************
	> File Name: main.c
	> Author: 
	> Mail: 
	> Created Time: Wednesday 13 July 2016 02:21:01 PM CST
 ************************************************************************/

#include<stdio.h>

void main()
{
    MPU6050Init();
    UltraInit();
    Delaynms(50);

    while(1)
    {
        if(g_ucUart2Flag>=1)
        {
            BlutoothControl();
            g_ucUart2Flag = 0;
        }

        if(EchoFlag)
        {
            g_ucUltraDis = UltraDis();
        }
    }
}
