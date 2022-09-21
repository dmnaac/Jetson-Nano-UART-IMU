extern "C"
{
#include "JY901.h"
}
extern "C"
{
    extern void CopeSerialData(unsigned char ucRxBuffer[]);
}

#include <sstream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <iconv.h>
#include <string>
#include <iostream>

/* IMU Model: JY901
 * Three steps are required to set the sensor using registers:
 * 1. Unlock the registers: 0xFF 0xAA 0x69 0x88 0xB5.
 * 2. Send the setting command to the corresponding register.
 * 3. Save: 0xFF 0xAA 0x00 0x00 0x00
*/

/* Function: UnlockRegister
 * Unlock the registers to change settings.
 * Return 1 when unlocking is successful; return 0 when unlocking fails.
*/
void UnlockRegister(unsigned int handle)
{
    unsigned char buffer[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    unsigned char byteBuffer[1];
    int cnt=0;

    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }
    if (cnt==0)
    {
        perror("Unlock registers Failed.\n");
    }
    else
    {
        printf("Unlock registers Succeed.\n");
    }
}

/* Function: HardwareSetting
 * Command the sensor to save the changes, to reboot, or to be reset to factory default.
 * Save changes: mode=0 / reboot: mode=1 / reset: mode=2
*/ 
void HardwareSetting(unsigned int handle, int mode)
{
    unsigned char buffer[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    unsigned char byteBuffer[1];
    int cnt=0;

    switch (mode)
    {
        case 0: buffer[3] = 0x00; // Save changes
                break;
        case 1: buffer[3] = 0xFF; // Reboot
                break;
        case 2: buffer[3] = 0x01; // Reset to factory default
                break;
    }
    
    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }
    if (cnt==0)
    {
        switch (mode)
        {
            case 0: perror("Failed to save setting changes.\n");
                    break;
            case 1: perror("Rebooting failed.\n");
                    break;
            case 2: perror("Failed to reset to factory default.\n");
                    break;
        }        
    }
    else
    {
        switch (mode)
        {
            case 0: printf("Setting changes saved.\n");
                    break;
            case 1: printf("Reboot.\n");
                    break;
            case 2: printf("Reset to factory default.\n");
                    break;
        }
    }
}

/* Function: CalibrateMode
 * Set the sensor to different calibration modes.
 * mode=0 : Normal working mode / mode=1 : Gyo calibration / mode=2 : Height clear / mode=3 : Yaw clear
*/ 
void CalibrateMode(unsigned int handle, int mode)
{
    unsigned char buffer[5] = {0xFF, 0xAA, 0x01, 0x00, 0x00};
    unsigned char byteBuffer[1];
    int cnt=0;

    switch (mode)
    {
        case 0: buffer[3] = 0x00; // Normal working mode
                break;
        case 1: buffer[3] = 0x01; // Gyo calibration
                break;
        case 2: buffer[3] = 0x03; // Height clear
                break;
        case 3: buffer[3] = 0x04; // Yaw clear
    }
    
    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }
    if (cnt==0)
    {
        switch (mode)
        {
            case 0: perror("Failed to switch to Normal working mode.\n");
                    break;
            case 1: perror("Failed to switch to Gyo calibration mode.\n");
                    break;
            case 2: perror("Failed to clear Height.\n");
                    break;
            case 3: perror("Failed to clear Yaw.\n");
                    break;
        }
    }
    else
    {
        switch (mode)
        {
            case 0: printf("Switch to Normal working mode.\n");
                    break;
            case 1: printf("Switch to Gyo calibration mode.\n");
                    break;
            case 2: printf("Switch to Height clearing mode.\n");
                    break;
            case 3: printf("Switch to Yaw clearing mode.\n");
                    break;
        }
    }
}

/* Function: SetBaudrate
 * Set the baudrate of the sensor.
 * Supported baudrate (bps): 4800/9600/19200/38400/57600/115200/230400
*/
void SetBaudrate(unsigned int handle, int baudrate)
{
    unsigned char buffer[5] = {0xFF, 0xAA, 0x04, 0x00, 0x00};
    unsigned char byteBuffer[1];
    int cnt=0;

    switch (baudrate)
    {
        case 4800: buffer[3] = 0x01;
                   break;
        case 9600: buffer[3] = 0x02;
                   break;
        case 19200: buffer[3] = 0x03;
                    break;
        case 38400: buffer[3] = 0x04;
                    break;
        case 57600: buffer[3] = 0x05;
                    break;
        case 115200: buffer[3] = 0x06;
                     break;
        case 230400: buffer[3] = 0x07;
                     break;
    }

    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }

    if (cnt==0)
    {
        perror("Failed to set baudrate.\n");
    }
    else
    {
        printf("Baudrate is set to %d.\n", baudrate);
    }
}

/* Function: SetRate
 * Set the transmission rate of the sensor.
 * Supported rate (Hz): 0.2/0.5/1/2/5/10/20/50/100/200/ONCE/NO
*/
void SetRate(unsigned int handle, int rate)
{
    unsigned char buffer[5] = {0xFF, 0xAA, 0x03, 0x00, 0x00};
    unsigned char byteBuffer[1];
    int cnt=0;

    switch (rate)
    {
        case 1: buffer[3] = 0x03;
                break;
        case 2: buffer[3] = 0x04;
                break;
        case 5: buffer[3] = 0x05;
                break;
        case 10: buffer[3] = 0x06;
                 break;
        case 20: buffer[3] = 0x07;
                 break;
        case 50: buffer[3] = 0x08;
                 break;
        case 100: buffer[3] = 0x09;
                  break;
        case 200: buffer[3] = 0x0B;
                  break;
        case 0: buffer[3] = 0x0D;
                break;
    }

    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }

    if (cnt==0)
    {
        perror("Failed to set rate.\n");
    }
    else
    {
        printf("Rate is set to %d.\n", rate);
    }
}

void setMountDirection(unsigned int handle, int OnOff)
{ 
    unsigned char buffer[5] = {0xFF, 0xAA, 0x23, 0x00, 0x00};
    unsigned char byteBuffer[1];

    //Horizontal Mounting
    if (OnOff==0)
    {
        buffer[3] = 0x00;
    }
    //Vertical Mounting
    else if (OnOff>0)
    {
        buffer[3] = 0x01;
    }
    else
    {
        perror("Mounting Direction State Error.\n");
    }

    int cnt=0;
    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }
    if (cnt==0)
    {
        perror("Setting Mounting Direction Failed.\n");
    }
    else
    {
        printf("Setting Mounting Direction Succeed.\n");
    }
}

void setAlgorithm(unsigned int handle, int numofaxis)
{
    unsigned char buffer[5] = {0xFF, 0xAA, 0x24, 0x00, 0x00};
    unsigned char byteBuffer[1];

    if (numofaxis==9)
    {
        buffer[3] = 0x00;
    }
    else if (numofaxis==6)
    {
        buffer[3] = 0x01;
    }
    else
    {
        perror("Setting Algorithm Parameter Error.\n");
    }

    int cnt=0;
    for (int i = 0; i < 5; i++)
    {
        byteBuffer[0] = buffer[i];
        cnt += write(handle, byteBuffer, 1);
    }
    if (cnt==0)
    {
        perror("Setting Algorithm Failed.\n");
    }
    else
    {
        printf("Setting Algorithm Succeed.\n");
    }
}

int open_port(int com_port)
{
    int fd;
    /* 使用普通串口 */
    // TODO::在此处添加串口列表
    char const* dev[] = { "/dev/ttyTHS1", "/dev/ttyUSB0" };
 
    //O_NDELAY 同 O_NONBLOCK。
    fd = open(dev[com_port], O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror("open serial port");
        return(-1);
    }
 
    //恢复串口为阻塞状态 
    //非阻塞：fcntl(fd,F_SETFL,FNDELAY)  
    //阻塞：fcntl(fd,F_SETFL,0) 
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        perror("fcntl F_SETFL\n");
    }
    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        perror("standard input is not a terminal device");
    }
 
    return fd;
}

int set_uart_config(int fd, int baud_rate, char parity)
{
    struct termios opt;
    int speed;
    if (tcgetattr(fd, &opt) != 0)
    {
        perror("tcgetattr");
        return -1;
    }
 
    /*设置波特率*/
    switch (baud_rate)
    {
    case 2400:  speed = B2400;  break;
    case 4800:  speed = B4800;  break;
    case 9600:  speed = B9600;  break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    default:    speed = B115200; break;
    }
    cfsetispeed(&opt, speed);
    cfsetospeed(&opt, speed);
    tcsetattr(fd, TCSANOW, &opt);
 
    opt.c_cflag &= ~CSIZE;
 
    
 
    /*设置奇偶校验位*/
    switch (parity) //N
    {
    case 'n':case 'N':
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能  
    }break;
    case 'o':case 'O':
    {
        opt.c_cflag |= (PARODD | PARENB);//PARODD使用奇校验而不使用偶校验 
        opt.c_iflag |= INPCK;
    }break;
    case 'e':case 'E':
    {
        opt.c_cflag |= PARENB;
        opt.c_cflag &= ~PARODD;
        opt.c_iflag |= INPCK;
    }break;
    case 's':case 'S': /*as no parity*/
    {
        opt.c_cflag &= ~PARENB;
        opt.c_cflag &= ~CSTOPB;
    }break;
    default:
    {
        opt.c_cflag &= ~PARENB;//校验位使能     
        opt.c_iflag &= ~INPCK; //奇偶校验使能          	
    }break;
    }

 
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
 
    /*设置等待时间和最小接收字符*/
    opt.c_cc[VTIME] = 1000;
    opt.c_cc[VMIN] = 0;
 
    /*关闭串口回显*/
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | NOFLSH);
 
    /*禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)*/
    opt.c_iflag &= ~ICRNL;
    /*禁止将所有接收的字符裁减为7比特*/
    opt.c_iflag &= ~ISTRIP;
 
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &opt)) != 0)
    {
        perror("tcsetattr");
        return -1;
    }
 
    return 0;
}


int main(int argc, char *argv[])
{
    int UART_fd = open_port(1);
    //if (set_uart_config(UART_fd, 9600, 'S') < 0)
    {
        //perror("Failed to set UART.");
        //exit(1);
    }

    UnlockRegister(UART_fd);
    
    //CalibrateMode(UART_fd, 2);

    SetBaudrate(UART_fd, 9600);
    SetRate(UART_fd, 10);

    //HardwareSetting(UART_fd, 2);

    HardwareSetting(UART_fd, 0);

    sleep(5); // Wait for the command to finish

    //CalibrateMode(UART_fd, 0);

    return 0;
}
