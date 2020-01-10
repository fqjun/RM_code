#ifndef RM_VIDEOCAPTURE_H
#define RM_VIDEOCAPTURE_H

#include "configure.h"

class RM_VideoCapture
{
public:
    /***************************相机驱动自带*******************************/
    unsigned char *g_pRgbBuffer; //处理后数据缓存区

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    IplImage                *iplImage = nullptr;
    int                     channel=3;
    BOOL                    AEstate=FALSE;
    tSdkImageResolution     pImageResolution;       //相机分辨率信息
 /**********************************************************/
    bool                    iscamera0_open = false;

public:
    RM_VideoCapture(int cameramode);
    ~RM_VideoCapture();
    bool isindustryimgInput();
    void cameraReleasebuff();
    int cameraSet();

};

#endif // RM_VIDEOCAPTURE_H
