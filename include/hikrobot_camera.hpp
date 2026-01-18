#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "ros/ros.h"
#include <stdio.h>
#include <pthread.h>
#include <cstdint>
#include <cmath>
#include <string>
#include <opencv2/opencv.hpp>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"

namespace camera
{
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4 * 2048 * 3072)
    //********** frame ************************************/
    cv::Mat frame;
    //********** frame_empty ******************************/
    bool frame_empty = 0;
    //********** frame_stamp ******************************/
    ros::Time frame_stamp;
    bool frame_stamp_valid = false;
    //********** mutex ************************************/
    pthread_mutex_t mutex;
    //********** timestamp config ******************************/
    bool use_device_timestamp = true;
    std::string device_ts_offset_calib = "first";
    long double dev_ts_tick_hz = 0.0L;
    long double dev_ts_offset_epoch_sec = 0.0L;
    bool dev_ts_inited = false;
    uint64_t last_dev_ticks = 0;
    bool last_dev_ticks_valid = false;
    //********** CameraProperties config ************************************/
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线

    };

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class************************************ //
    //^ *********************************************************************************** //
    static int TrySetInt(void *handle, const char *key, int64_t value)
    {
        int nRet = MV_CC_SetIntValue(handle, key, value);
        if (nRet != MV_OK)
        {
            ROS_WARN("MV_CC_SetIntValue(%s=%ld) failed, nRet=0x%x", key, (long)value, nRet);
        }
        else
        {
            ROS_INFO("Set %s = %ld", key, (long)value);
        }
        return nRet;
    }

    static bool TryGetInt(void *handle, const char *key, uint64_t &out)
    {
        MVCC_INTVALUE v = {0};
        int nRet = MV_CC_GetIntValue(handle, key, &v);
        if (nRet != MV_OK)
        {
            ROS_WARN("MV_CC_GetIntValue(%s) failed, nRet=0x%x", key, nRet);
            return false;
        }
        out = static_cast<uint64_t>(v.nCurValue);
        ROS_INFO("Get %s = %lu", key, static_cast<unsigned long>(out));
        return true;
    }

    static int TrySetBool(void *handle, const char *key, bool value)
    {
        int nRet = MV_CC_SetBoolValue(handle, key, value ? 1 : 0);
        if (nRet != MV_OK)
        {
            ROS_WARN("MV_CC_SetBoolValue(%s=%d) failed, nRet=0x%x", key, value ? 1 : 0, nRet);
        }
        else
        {
            ROS_INFO("Set %s = %s", key, value ? "true" : "false");
        }
        return nRet;
    }

    static int TrySetEnum(void *handle, const char *key, uint32_t value)
    {
        int nRet = MV_CC_SetEnumValue(handle, key, value);
        if (nRet != MV_OK)
        {
            ROS_WARN("MV_CC_SetEnumValue(%s=%u) failed, nRet=0x%x", key, value, nRet);
        }
        else
        {
            ROS_INFO("Set %s = %u", key, value);
        }
        return nRet;
    }

    [[maybe_unused]] static int TrySetBoolByInt(void *handle, const char *key, bool on)
    {
        return TrySetInt(handle, key, on ? 1 : 0);
    }

    static inline uint64_t DevTsTicks(uint32_t hi, uint32_t lo)
    {
        return (static_cast<uint64_t>(hi) << 32) | static_cast<uint64_t>(lo);
    }

    static ros::Time DevTsToRosTime(uint64_t ticks)
    {
        long double t = dev_ts_offset_epoch_sec + static_cast<long double>(ticks) / dev_ts_tick_hz;
        if (t < 0)
        {
            t = 0;
        }
        const uint32_t sec = static_cast<uint32_t>(std::floor(t));
        long double frac = (t - static_cast<long double>(sec)) * 1e9L;
        uint32_t nsec = static_cast<uint32_t>(std::llround(frac));
        uint32_t sec2 = sec;
        if (nsec >= 1000000000U)
        {
            sec2 += 1;
            nsec -= 1000000000U;
        }
        return ros::Time(sec2, nsec);
    }

    static void SetPtp1588(void *cam_handle, bool enable)
    {
        const char *keys[] = {
            "GevIEEE1588",
            "Std::GevIEEE1588",
            "GevIEEE1588Enable",
            "Std::GevIEEE1588Enable"};

        for (const char *key : keys)
        {
            if (TrySetBool(cam_handle, key, enable) == MV_OK)
            {
                return;
            }
            if (TrySetEnum(cam_handle, key, enable ? 1u : 0u) == MV_OK)
            {
                return;
            }
        }
        ROS_WARN("PTP enable failed for all known keys. Please confirm node name/type.");
    }

    static void SetGigeTransportParamsIfNeeded(void *cam_handle,
                                               const MV_CC_DEVICE_INFO *dev_info,
                                               int gev_scps_packet_size,
                                               int gev_scpd,
                                               int gev_heartbeat_timeout_ms)
    {
        if (dev_info == NULL)
        {
            return;
        }

        if (dev_info->nTLayerType != MV_GIGE_DEVICE)
        {
            ROS_INFO("Non-GigE device, skip GigE transport params.");
            return;
        }

        int packet_size_to_set = gev_scps_packet_size;
        if (packet_size_to_set <= 0)
        {
            int nPacketSize = MV_CC_GetOptimalPacketSize(cam_handle);
            if (nPacketSize > 0)
            {
                packet_size_to_set = nPacketSize;
                ROS_INFO("Optimal packet size from SDK: %d", packet_size_to_set);
            }
            else
            {
                packet_size_to_set = 1500;
                ROS_WARN("GetOptimalPacketSize failed (%d), fallback to %d", nPacketSize, packet_size_to_set);
            }
        }

        if (packet_size_to_set > 0)
        {
            TrySetInt(cam_handle, "GevSCPSPacketSize", packet_size_to_set);
            TrySetInt(cam_handle, "Std::GevSCPSPacketSize", packet_size_to_set);
        }

        if (gev_scpd >= 0)
        {
            TrySetInt(cam_handle, "GevSCPD", gev_scpd);
            TrySetInt(cam_handle, "Std::GevSCPD", gev_scpd);
        }

        if (gev_heartbeat_timeout_ms > 0)
        {
            TrySetInt(cam_handle, "GevHeartbeatTimeout", gev_heartbeat_timeout_ms);
            TrySetInt(cam_handle, "Std::GevHeartbeatTimeout", gev_heartbeat_timeout_ms);
        }

        // Optional: TrySetBoolByInt(cam_handle, "GevSCPSDoNotFragment", true);
    }

    class Camera
    {
    public:
        //********** 构造函数  ****************************/
        Camera(ros::NodeHandle &node);
        //********** 析构函数  ****************************/
        ~Camera();
        //********** 原始信息转换线程 **********************/
        static void *HKWorkThread(void *p_handle);

        //********** 输出摄像头信息 ***********************/
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);
        //********** 设置摄像头参数 ***********************/
        bool set(camera::CamerProperties type, float value);
        //********** 恢复默认参数 *************************/
        bool reset();
        //********** 读图10个相机的原始图像 ********************************/
        void ReadImg(cv::Mat &image);
        void ReadImg(cv::Mat &image, ros::Time &stamp);

    private:
        //********** handle ******************************/
        void *handle;
        //********** nThreadID ******************************/
        pthread_t nThreadID;
        //********** yaml config ******************************/
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;
        int BurstFrameCount;
        int ExposureTime;
        bool GammaEnable;
        float Gamma;
        int GainAuto;
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;
    };
    //^ *********************************************************************************** //

    //^ ********************************** Camera constructor************************************ //
    Camera::Camera(ros::NodeHandle &node)
    {
        handle = NULL;

        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        node.param("width", width, 3072);
        node.param("height", height, 2048);
        node.param("FrameRateEnable", FrameRateEnable, false);
        node.param("FrameRate", FrameRate, 10);
        node.param("BurstFrameCount", BurstFrameCount, 10); // 一次触发采集的次数
        node.param("ExposureTime", ExposureTime, 50000);
        node.param("GammaEnable", GammaEnable, false);
        node.param("Gamma", Gamma, (float)0.7);
        node.param("GainAuto", GainAuto, 2);
        node.param("SaturationEnable", SaturationEnable,true);
        node.param("Saturation", Saturation, 128);
        node.param("Offset_x", Offset_x, 0);
        node.param("Offset_y", Offset_y, 0);
        node.param("TriggerMode", TriggerMode, 1);
        node.param("TriggerSource", TriggerSource, 2);
        node.param("LineSelector", LineSelector, 2);
        bool ptp_enable = true;
        node.param("ptp_enable", ptp_enable, ptp_enable);
        node.param("use_device_timestamp", use_device_timestamp, true);
        node.param("device_ts_offset_calib", device_ts_offset_calib, std::string("first"));
        int gev_scps_packet_size = 0;
        int gev_scpd = 0;
        int gev_heartbeat_timeout_ms = 30000;
        node.param("gev_scps_packet_size", gev_scps_packet_size, gev_scps_packet_size);
        node.param("gev_scpd", gev_scpd, gev_scpd);
        node.param("gev_heartbeat_timeout_ms", gev_heartbeat_timeout_ms, gev_heartbeat_timeout_ms);

        //********** 枚举设备 ********************************/
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        unsigned int nIndex = 0;
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                }
                PrintDeviceInfo(pDeviceInfo);
            }
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        //********** 选择设备并创建句柄 *************************/

        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[0]);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        //********** frame **********/

        nRet = MV_CC_OpenDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        MV_CC_DEVICE_INFO *device_info = stDeviceList.pDeviceInfo[0];
        if (device_info->nTLayerType == MV_GIGE_DEVICE)
        {
            SetPtp1588(handle, ptp_enable);
        }
        else
        {
            ROS_INFO("Non-GigE device, skip PTP enable.");
        }

        dev_ts_tick_hz = 0.0L;
        dev_ts_inited = false;
        last_dev_ticks_valid = false;
        frame_stamp_valid = false;
        if (use_device_timestamp)
        {
            if (device_ts_offset_calib != "first")
            {
                ROS_WARN("device_ts_offset_calib=%s not supported, using 'first'.",
                         device_ts_offset_calib.c_str());
            }
            uint64_t tick_hz_value = 0;
            if (device_info->nTLayerType == MV_GIGE_DEVICE &&
                (TryGetInt(handle, "GevTimestampTickFrequency", tick_hz_value) ||
                 TryGetInt(handle, "Std::GevTimestampTickFrequency", tick_hz_value)))
            {
                dev_ts_tick_hz = static_cast<long double>(tick_hz_value);
            }
            else
            {
                ROS_WARN("Failed to read GevTimestampTickFrequency, falling back to ros::Time::now().");
            }
        }

        SetGigeTransportParamsIfNeeded(handle,
                                       device_info,
                                       gev_scps_packet_size,
                                       gev_scpd,
                                       gev_heartbeat_timeout_ms);

        //设置 yaml 文件里面的配置
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAP_PROP_FRAMERATE, FrameRate);
        // this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount);
        this->set(CAP_PROP_HEIGHT, height);
        this->set(CAP_PROP_WIDTH, width);
        this->set(CAP_PROP_OFFSETX, Offset_x);
        this->set(CAP_PROP_OFFSETY, Offset_y);
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
        // printf("\n%d\n",GammaEnable);
        this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
        // printf("\n%d\n",GammaEnable);
        if (GammaEnable)
            this->set(CAP_PROP_GAMMA, Gamma);
        this->set(CAP_PROP_GAINAUTO, GainAuto);
        // this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);
        // this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);
        // this->set(CAP_PROP_LINE_SELECTOR, LineSelector);

        //********** frame **********/
        //白平衡 非自适应（给定参数0）
        nRet = MV_CC_SetEnumValue(handle, "BalanceWhiteAuto", 0);
        // //白平衡度
        // int rgb[3] = {1742, 1024, 2371};
        // for (int i = 0; i < 3; i++)
        // {
        //     //********** frame **********/

        //     nRet = MV_CC_SetEnumValue(handle, "BalanceRatioSelector", i);
        //     nRet = MV_CC_SetIntValue(handle, "BalanceRatio", rgb[i]);
        // }
        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n",0.0 );
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }
        this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
        if (SaturationEnable)
            this->set(CAP_PROP_SATURATION, Saturation);
        //软件触发
        // ********** frame **********/
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK == nRet)
        {
            printf("set TriggerMode OK!\n");
        }
        else
        {
            printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
        }

        //********** 图像格式 **********/
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014); // 目前 RGB  

        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = RGB\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
        }
        MVCC_ENUMVALUE t = {0};
        //********** frame **********/

        nRet = MV_CC_GetEnumValue(handle, "PixelFormat", &t);

        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }
        // 开始取流
        //********** frame **********/

        nRet = MV_CC_StartGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        //初始化互斥量
        nRet = pthread_mutex_init(&mutex, NULL);
        if (nRet != 0)
        {
            perror("pthread_create failed\n");
            exit(-1);
        }
        //********** frame **********/

        nRet = pthread_create(&nThreadID, NULL, HKWorkThread, handle);

        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n", nRet);
            exit(-1);
        }
    }

    //^ ********************************** Camera constructor************************************ //
    Camera::~Camera()
    {
        int nRet;
        //********** frame **********/

        pthread_join(nThreadID, NULL);

        //********** frame **********/

        nRet = MV_CC_StopGrabbing(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        //********** frame **********/

        nRet = MV_CC_CloseDevice(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        // 销毁句柄
        //********** frame **********/

        nRet = MV_CC_DestroyHandle(handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");
        // 销毁互斥量
        pthread_mutex_destroy(&mutex);
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {
        case CAP_PROP_FRAMERATE_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "AcquisitionBurstFrameCount", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionBurstFrameCount OK!\n");
            }
            else
            {
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Height", value); //图像高度

            if (MV_OK == nRet)
            {
                printf("set Height OK!\n");
            }
            else
            {
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_WIDTH:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Width", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Width OK!\n");
            }
            else
            {
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "OffsetX", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset X OK!\n");
            }
            else
            {
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "OffsetY", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset Y OK!\n");
            }
            else
            {
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "ExposureTime", value); //曝光时间

            if (MV_OK == nRet)
            {
                printf("set ExposureTime OK! value=%f\n",value);
            }
            else
            {
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）

            if (MV_OK == nRet)
            {
                printf("set GammaEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(handle, "Gamma", value); //伽马越小 亮度越大

            if (MV_OK == nRet)
            {
                printf("set Gamma OK! value=%f\n",value);
            }
            else
            {
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAINAUTO:
        {
            //********** frame **********/

            nRet = MV_CC_SetEnumValue(handle, "GainAuto", value); //亮度 越大越亮

            if (MV_OK == nRet)
            {
                printf("set GainAuto OK! value=%f\n",value);
            }
            else
            {
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)

            if (MV_OK == nRet)
            {
                printf("set SaturationEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(handle, "Saturation", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set Saturation OK! value=%f\n",value);
            }
            else
            {
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        case CAP_PROP_TRIGGER_MODE:
        {

            nRet = MV_CC_SetEnumValue(handle, "TriggerMode", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set TriggerMode OK!\n");
            }
            else
            {
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {

            nRet = MV_CC_SetEnumValue(handle, "TriggerSource", value); //饱和度 默认128 最大255255

            if (MV_OK == nRet)
            {
                printf("set TriggerSource OK!\n");
            }
            else
            {
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {

            nRet = MV_CC_SetEnumValue(handle, "LineSelector", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set LineSelector OK!\n");
            }
            else
            {
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        default:
            return 0;
        }
        return nRet;
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
    }

    //^ ********************************** PrintDeviceInfo ************************************ //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ ********************************** Camera constructor************************************ //
    void Camera::ReadImg(cv::Mat &image)
    {
        ros::Time dummy;
        ReadImg(image, dummy);
    }

    void Camera::ReadImg(cv::Mat &image, ros::Time &stamp)
    {

        pthread_mutex_lock(&mutex);
        if (frame_empty)
        {
            image = cv::Mat();
            stamp = ros::Time(0);
        }
        else
        {
            image = camera::frame.clone();
            frame_empty = 1;
            stamp = frame_stamp_valid ? frame_stamp : ros::Time(0);
        }
        pthread_mutex_unlock(&mutex);
    }

    //^ ********************************** HKWorkThread1 ************************************ //
    void *Camera::HKWorkThread(void *p_handle)
    {
        double start;
        int nRet;
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0; //空图帧数
        while (ros::ok())
        {
            start = static_cast<double>(cv::getTickCount());
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数
            //转换图像格式为BGR8

            stConvertParam.nWidth = stImageInfo.nWidth;                 //ch:图像宽 | en:image width
            stConvertParam.nHeight = stImageInfo.nHeight;               //ch:图像高 | en:image height
            stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
            if (stImageInfo.nFrameLen > 0)
            {
                stConvertParam.nSrcDataLen = stImageInfo.nFrameLen;     //ch:输入数据大小 | en:input data size
            }
            else
            {
                stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;       //ch:输入数据大小 | en:input data size
            }
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format                      //! 输出格式 RGB
            stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
            uint64_t dst_size = static_cast<uint64_t>(stImageInfo.nWidth) *
                                static_cast<uint64_t>(stImageInfo.nHeight) * 3;
            if (dst_size > MAX_IMAGE_DATA_SIZE)
            {
                dst_size = MAX_IMAGE_DATA_SIZE;
            }
            stConvertParam.nDstBufferSize = static_cast<unsigned int>(dst_size); //ch:输出缓存大小 | en:output buffer size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format                       //! 输入格式 RGB
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);
            pthread_mutex_lock(&mutex);
            camera::frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage).clone(); //tmp.clone();
            if (use_device_timestamp && dev_ts_tick_hz > 0.0L)
            {
                uint64_t ticks = DevTsTicks(stImageInfo.nDevTimeStampHigh, stImageInfo.nDevTimeStampLow);
                if (ticks == 0 || (last_dev_ticks_valid && ticks == last_dev_ticks))
                {
                    frame_stamp = ros::Time::now();
                    frame_stamp_valid = true;
                }
                else if (!dev_ts_inited || !last_dev_ticks_valid || ticks <= last_dev_ticks)
                {
                    ros::Time now = ros::Time::now();
                    dev_ts_offset_epoch_sec = static_cast<long double>(now.toSec()) -
                                              static_cast<long double>(ticks) / dev_ts_tick_hz;
                    dev_ts_inited = true;
                    last_dev_ticks = ticks;
                    last_dev_ticks_valid = true;
                    frame_stamp = DevTsToRosTime(ticks);
                    frame_stamp_valid = true;
                }
                else
                {
                    frame_stamp = DevTsToRosTime(ticks);
                    frame_stamp_valid = true;
                    last_dev_ticks = ticks;
                    last_dev_ticks_valid = true;
                }
            }
            else
            {
                frame_stamp = ros::Time::now();
                frame_stamp_valid = true;
            }
            frame_empty = 0;
            pthread_mutex_unlock(&mutex);
            double time = ((double)cv::getTickCount() - start) / cv::getTickFrequency();
            //*************************************testing img********************************//
            //std::cout << "HK_camera,Time:" << time << "\tFPS:" << 1 / time << std::endl;
            //imshow("HK vision",frame);
            //waitKey(1);
        }
        free(m_pBufForDriver);
        free(m_pBufForSaveImage);
        return 0;
    }

} // namespace camera
#endif
