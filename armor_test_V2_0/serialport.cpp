/**
 * @file serialport.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2020 步兵视觉串口部分头文件
 * @version test2.0
 * @date 2020-01-5
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 * @author jiaming
 */
#include "serialport.h"


int SerialPort::fd;
char SerialPort::send_buf[BUFF_LENGTH];//out
char SerialPort::send_buf_temp[BUFF_TEMP_LENGTH];//out_temp
char SerialPort::rec_buf[REC_BUFF_LENGTH];//in
char *SerialPort::file_name;


SerialPort::SerialPort(void)
{
    switch (PORT_NUM)
    {
    case 1: file_name = (char*)"";             break;
    case 2: file_name = (char*)"/dev/ttyUSB0"; break;
    case 3: file_name = (char*)"/dev/ttyACM0"; break;
    default:file_name = (char*)"";             break;
    }


    fd = open(file_name, O_RDWR|O_NONBLOCK|O_NOCTTY|O_NDELAY); //开串口，就像文件操作
    if (fd == -1){
        perror("Can't Open Serial Port");// 输出错误
    }else{
        #if COUT_STATE == 1
        cout << "open" << file_name << " successful"<<endl;
        #endif
    }

    struct termios SerialData;//通过设置termios类型的数据结构中的值和使用一小组函数调用，你就可以对终端接口进行控制。
     /*打开串口*/
    bzero(&SerialData, sizeof(SerialData));//清零

    /*设置发送波特率*/
    cfsetospeed(&SerialData, B115200);  // 发送
    cfsetispeed(&SerialData, B115200);  // 接收

    //本地连线, 取消控制功能 | 开始接收
    SerialData.c_cflag |= CLOCAL | CREAD;
    //设置字符大小
    SerialData.c_cflag &= ~CSIZE;
    //设置停止位1
    SerialData.c_cflag &= ~CSTOPB;
    //设置数据位8位
    SerialData.c_cflag |= CS8;
    //设置无奇偶校验位，N
    SerialData.c_cflag &= ~PARENB;

    /*阻塞模式的设置*/
    SerialData.c_cc[VTIME]=0;  // 设置超时时间
    SerialData.c_cc[VMIN]=0;

    tcflush(fd,TCIOFLUSH);   // 清空输入端

    tcsetattr(fd, TCSANOW, &SerialData);  // 让设置立刻生效

}


SerialPort::~SerialPort(void)
{
    if (!close(fd))
        printf("Close Serial Port Successful\n");
    else
        close(fd);
}


/**
*  @brief: 串口数据读取函数
**/
void SerialPort::receiveData(int arr[REC_BUFF_LENGTH])//接收数据
{
    memset(rec_buf, '0', REC_BUFF_LENGTH); //清空缓存
    char rec_buf_temp[REC_BUFF_LENGTH*2];   //数据10位
    memset(rec_buf_temp,'0',sizeof(rec_buf_temp)); //清0
    read(fd,rec_buf_temp,sizeof(rec_buf_temp)); // 读取数据
    for(int i = 0; i < (int)sizeof(rec_buf_temp); ++i) // 接收数据位
    {
        if(rec_buf_temp[i]=='S' && rec_buf_temp[i+sizeof(rec_buf)-1] == 'E')  // 获取有效数据
        {
            for(int j = 0;j<((int)sizeof(rec_buf));++j)
            {
                rec_buf[j] = rec_buf_temp[i+j];  // 把临时数据存到g_rec_buf中
            }
            break;  // 每次只读取一串有效数据
        }
    }
    for(size_t i = 0;i < sizeof(rec_buf);++i){
        arr[i] = (rec_buf[i] - '0');
    }

    tcflush(fd,TCIFLUSH);  // 清空文件数据
    rec_buf_temp[10] = '\0';   // 数据末尾
    #if COUT_STATE == 1
    cout<<"rec_buf_temp: "<< rec_buf_temp << endl;
    cout<<"rec_buf: "<< rec_buf << endl;
    #endif
}



/**
*  @brief: 串口数据发送函数
**/
void SerialPort::sendData(int model,int model_select,int x,int y,int depth){
//    int model = 1;
//    int model_select = 1;
//    int depth = 1;
    sprintf(send_buf_temp, "%c%1d%1d%03d%03d%c", 'S',model,model_select, x, y, depth);
    uint8_t CRC = Check_CRC8(send_buf_temp, sizeof(send_buf_temp));
    sprintf(send_buf, "%c%01d%01d%03d%03d%03d%03d%c", 'S',model, model_select,x, y,depth,CRC, 'E');
    write(fd, send_buf, sizeof(send_buf));
    #if COUT_STATE == 1
    cout << "send_buff:" << send_buf << endl;
    #endif
    usleep(5);  // 防止发送过快出现乱码
}


/**
*  @brief: CRC8校验
**/
uint8_t SerialPort::Check_CRC8(char *buf,uint16_t len)
{
    uint8_t check = 0;

    while(len--)
    {
        check = CRC8_TAB[check^(*buf++)];
    }

    return (check)&0x00ff;

}

