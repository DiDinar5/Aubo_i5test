﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Input;
using auboi5_sdk_for_windows_x86_csharp;
using MoveObjectWithMouse;

public  class RobotAubo{
    const int RSERR_SUCC = 0;
    
    static UInt16 rshd = 0xffff;
    //机械臂IP地址

    //const string robotIP = "192.168.50.10";
    const string robotIP = "192.168.12.132";
    //机械臂端口号
    const int serverPort = 8899;
    //M_PI
    const double M_PI = 3.14159265358979323846;
    //接口板用户DI地址
    const int ROBOT_IO_F1 = 30;
    const int ROBOT_IO_F2 = 31;
    const int ROBOT_IO_F3 = 32;
    const int ROBOT_IO_F4 = 33;
    const int ROBOT_IO_F5 = 34;
    const int ROBOT_IO_F6 = 35;
    const int ROBOT_IO_U_DI_00 = 36;
    const int ROBOT_IO_U_DI_01 = 37;
    const int ROBOT_IO_U_DI_02 = 38;
    const int ROBOT_IO_U_DI_03 = 39;
    const int ROBOT_IO_U_DI_04 = 40;
    const int ROBOT_IO_U_DI_05 = 41;
    const int ROBOT_IO_U_DI_06 = 42;
    const int ROBOT_IO_U_DI_07 = 43;
    const int ROBOT_IO_U_DI_10 = 44;
    const int ROBOT_IO_U_DI_11 = 45;
    const int ROBOT_IO_U_DI_12 = 46;
    const int ROBOT_IO_U_DI_13 = 47;
    const int ROBOT_IO_U_DI_14 = 48;
    const int ROBOT_IO_U_DI_15 = 49;
    const int ROBOT_IO_U_DI_16 = 50;
    const int ROBOT_IO_U_DI_17 = 51;

    //接口板用户DO地址
    const int ROBOT_IO_U_DO_00 = 32;
    const int ROBOT_IO_U_DO_01 = 33;
    const int ROBOT_IO_U_DO_02 = 34;
    const int ROBOT_IO_U_DO_03 = 35;
    const int ROBOT_IO_U_DO_04 = 36;
    const int ROBOT_IO_U_DO_05 = 37;
    const int ROBOT_IO_U_DO_06 = 38;
    const int ROBOT_IO_U_DO_07 = 39;
    const int ROBOT_IO_U_DO_10 = 40;
    const int ROBOT_IO_U_DO_11 = 41;
    const int ROBOT_IO_U_DO_12 = 42;
    const int ROBOT_IO_U_DO_13 = 43;
    const int ROBOT_IO_U_DO_14 = 44;
    const int ROBOT_IO_U_DO_15 = 45;
    const int ROBOT_IO_U_DO_16 = 46;
    const int ROBOT_IO_U_DO_17 = 47;

    //接口板用户AI地址
    const int ROBOT_IO_VI0 = 0;
    const int ROBOT_IO_VI1 = 1;
    const int ROBOT_IO_VI2 = 2;
    const int ROBOT_IO_VI3 = 3;

    //接口板用户AO地址
    const int ROBOT_IO_VO0 = 0;
    const int ROBOT_IO_VO1 = 1;
    const int ROBOT_IO_CO0 = 2;
    const int ROBOT_IO_CO1 = 3;

    //接口板IO类型
    //
    //接口板用户DI(数字量输入)　可读可写
    const int Robot_User_DI = 4;
    //接口板用户DO(数字量输出)  可读可写
    const int Robot_User_DO = 5;
    //接口板用户AI(模拟量输入)  可读可写
    const int Robot_User_AI = 6;
    //接口板用户AO(模拟量输出)  可读可写
    const int Robot_User_AO = 7;

    //工具端IO类型
    //
    //工具端DI
    const int Robot_Tool_DI = 8;
    //工具端DO
    const int Robot_Tool_DO = 9;
    //工具端AI
    const int Robot_Tool_AI = 10;
    //工具端AO
    const int Robot_Tool_AO = 11;
    //工具端DI
    const int Robot_ToolIoType_DI = Robot_Tool_DI;
    //工具端DO
    const int Robot_ToolIoType_DO = Robot_Tool_DO;

    //工具端IO名称
    const string TOOL_IO_0 = ("T_DI/O_00");
    const string TOOL_IO_1 = ("T_DI/O_01");
    const string TOOL_IO_2 = ("T_DI/O_02");
    const string TOOL_IO_3 = ("T_DI/O_03");

    //工具端数字IO类型
    //
    //输入
    const int TOOL_IO_IN = 0;
    //输出
    const int TOOL_IO_OUT = 0;

    //工具端电源类型
    //
    const int OUT_0V = 0;
    const int OUT_12V = 1;
    const int OUT_24V = 2;

    //IO状态-无效
    const double IO_STATUS_INVALID = 0.0;
    //IO状态-有效
    const double IO_STATUS_VALID = 1.0;

    //坐标系枚举
    const int BaseCoordinate = 0;
    const int EndCoordinate = 1;
    const int WorldCoordinate = 2;

    //坐标系标定方法
    const int Origin_AnyPointOnPositiveXAxis_AnyPointOnPositiveYAxis = 0; // 原点、x轴正半轴、y轴正半轴
    const int Origin_AnyPointOnPositiveYAxis_AnyPointOnPositiveZAxis = 1; // 原点、y轴正半轴、z轴正半轴
    const int Origin_AnyPointOnPositiveZAxis_AnyPointOnPositiveXAxis = 2; // 原点、z轴正半轴、x轴正半轴
    const int Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane = 3; // 原点、x轴正半轴、x、y轴平面的第一象限上任意一点
    const int Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOZPlane = 4; // 原点、x轴正半轴、x、z轴平面的第一象限上任意一点
    const int Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOZPlane = 5; // 原点、y轴正半轴、y、z轴平面的第一象限上任意一点
    const int Origin_AnyPointOnPositiveYAxis_AnyPointOnFirstQuadrantOfYOXPlane = 6; // 原点、y轴正半轴、y、x轴平面的第一象限上任意一点
    const int Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOXPlane = 7; // 原点、z轴正半轴、z、x轴平面的第一象限上任意一点
    const int Origin_AnyPointOnPositiveZAxis_AnyPointOnFirstQuadrantOfZOYPlane = 8; // 原点、z轴正半轴、z、y轴平面的第一象限上任意一点

    //工具标定方法
    const int ToolKinematicsOriCalibrateMathod_xOxy = 0;                   // 原点、x轴正半轴、x、y轴平面的第一象限上任意一点
    const int ToolKinematicsOriCalibrateMathod_yOyz = 1;                  // 原点、y轴正半轴、y、z轴平面的第一象限上任意一点
    const int ToolKinematicsOriCalibrateMathod_zOzx = 2;                  // 原点、z轴正半轴、z、x轴平面的第一象限上任意一点
    const int ToolKinematicsOriCalibrateMathod_TxRBz_TxyPBzAndTyABnz = 3;  // 工具x轴平行反向于基坐标系z轴; 工具xOy平面平行于基坐标系z轴、工具y轴与基坐标系负z轴夹角为锐角
    const int ToolKinematicsOriCalibrateMathod_TyRBz_TyzPBzAndTzABnz = 4;  // 工具y轴平行反向于基坐标系z轴; 工具yOz平面平行于基坐标系z轴、工具z轴与基坐标系负z轴夹角为锐角
    const int ToolKinematicsOriCalibrateMathod_TzRBz_TzxPBzAndTxABnz = 5;  // 工具z轴平行反向于基坐标系z轴; 工具zOx平面平行于基坐标系z轴、工具x轴与基坐标系负z轴夹角为锐角

    //运动轨迹类型
    //
    //圆
    const int ARC_CIR = 2;
    //圆弧
    const int CARTESIAN_MOVEP = 3;

    //机械臂状态
    const int RobotStopped = 0;
    const int RobotRunning = 1;
    const int RobotPaused = 2;
    const int RobotResumed = 3;

    //机械臂工作模式
    const int RobotModeSimulator = 0; //机械臂仿真模式
    const int RobotModeReal = 1; //机械臂真实模式

    
    static int checkUsercoordExample()
    {
        cSharpBinding.CoordCalibrate user_coord = new cSharpBinding.CoordCalibrate();
        IntPtr pt_user_coord = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.CoordCalibrate)));
        user_coord = (cSharpBinding.CoordCalibrate)Marshal.PtrToStructure(pt_user_coord, typeof(cSharpBinding.CoordCalibrate));

        //检查用户坐标系参数设置是否合理
        user_coord.coordType = WorldCoordinate;
        //坐标系标定方法xoxy
        user_coord.methods = Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;
        //为了简化，使用法兰盘中心为工具端
        user_coord.toolDesc.cartPos.x = 0;
        user_coord.toolDesc.cartPos.y = 0;
        user_coord.toolDesc.cartPos.z = 0;
        user_coord.toolDesc.orientation.w = 1;
        user_coord.toolDesc.orientation.x = 0;
        user_coord.toolDesc.orientation.y = 0;
        user_coord.toolDesc.orientation.z = 0;
        //原点
        user_coord.jointPara[0].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[1] = -7.291862 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[2] = -75.694718 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[3] = 21.596727 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[5] = -0.00458 / 180 * M_PI;
        //x轴正半轴上一点
        user_coord.jointPara[1].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[1] = -1.106149 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[2] = -55.497882 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[3] =  35.608266 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[5] = -0.000458 / 180 * M_PI;
        //x、y轴平面的第一象限上任意一点
        user_coord.jointPara[2].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[1] = -9.503956 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[2] = -113.279220 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[3] = -13.775266 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[4] = -89.999982  / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[5] = -0.000458 / 180 * M_PI;

        //检查用户坐标系参数设置是否合理
        if (RSERR_SUCC == cSharpBinding.rs_check_user_coord(rshd, ref user_coord))
        {
            CustomLog.Log($"user coord check ok.");
        }
        else
        {
            CustomLog.Log($"user coord check failed!");
        }      
        return 0;
    }

    public static int relativeOffsetExample(double offsetX, double offsetY)
    {
        cSharpBinding.CoordCalibrate user_coord = new cSharpBinding.CoordCalibrate();
        IntPtr pt_user_coord = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.CoordCalibrate)));
        user_coord = (cSharpBinding.CoordCalibrate)Marshal.PtrToStructure(pt_user_coord, typeof(cSharpBinding.CoordCalibrate));

        cSharpBinding.MoveRelative relative = new cSharpBinding.MoveRelative();
        IntPtr pt_relative = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.MoveRelative)));
        relative = (cSharpBinding.MoveRelative)Marshal.PtrToStructure(pt_relative, typeof(cSharpBinding.MoveRelative));
        relative.enable = 1;
        relative.orientation.w = 1;
        relative.orientation.x = 0;
        relative.orientation.y = 0;
        relative.orientation.z = 0;
        relative.pos[0] = 0;
        relative.pos[1] = 0;
        relative.pos[2] = 0.1F;

        double[] target0 = { 0, 0, 0, 0, 0, 0 }; //注意这个里面的值是弧度！
        target0[0] += -90.000172 * offsetX > 0 ? 1 : -1 / 180 * M_PI;
        target0[1] = -7.291862 / 180 * M_PI;
        target0[2] += -75.694718 * offsetY > 0 ? 1 : -1 / 180 * M_PI;
        target0[3] = 21.596727 / 180 * M_PI;
        target0[4] = -89.999982 / 180 * M_PI;
        target0[5] = -0.00458 / 180 * M_PI;

        user_coord.coordType = BaseCoordinate;
        //移动到坐标系原点
        cSharpBinding.rs_move_joint(rshd, target0, true);
        //相对坐标系原点沿z轴正向运动
        cSharpBinding.rs_set_relative_offset_on_user(rshd, ref relative, ref user_coord);
        cSharpBinding.rs_move_line(rshd, target0, true);
        //check function
        return 0;
    }

    static int baseToUserExample()
    {
        //基座坐标系转用户坐标系                        
        cSharpBinding.Ori ori_onbase = new cSharpBinding.Ori();
        cSharpBinding.Pos pos_onbase = new cSharpBinding.Pos();
        cSharpBinding.ToolInEndDesc tool_pos = new cSharpBinding.ToolInEndDesc();
        cSharpBinding.Pos pos_onuser = new cSharpBinding.Pos();
        cSharpBinding.Ori ori_onuser = new cSharpBinding.Ori();
        cSharpBinding.wayPoint_S waypoint = new cSharpBinding.wayPoint_S();
        cSharpBinding.CoordCalibrate user_coord = new cSharpBinding.CoordCalibrate();
        IntPtr pt_user_coord = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.CoordCalibrate)));
        user_coord = (cSharpBinding.CoordCalibrate)Marshal.PtrToStructure(pt_user_coord, typeof(cSharpBinding.CoordCalibrate));
        user_coord.coordType = WorldCoordinate;
        user_coord.methods= Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;
        //为了简化，使用法兰盘中心为工具端
        user_coord.toolDesc.cartPos.x = 0;
        user_coord.toolDesc.cartPos.y = 0;
        user_coord.toolDesc.cartPos.z = 0;
        user_coord.toolDesc.orientation.w = 1;
        user_coord.toolDesc.orientation.x = 0;
        user_coord.toolDesc.orientation.y = 0;
        user_coord.toolDesc.orientation.z = 0;
        //原点
        user_coord.jointPara[0].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[1] = -7.291862 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[2] = -75.694718 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[3] = 21.596727 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[5] = -0.00458 / 180 * M_PI;
        //x轴正半轴上一点
        user_coord.jointPara[1].jointRadian[0] = 11.116932 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[1] = -0.858816 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[2] = -64.008663 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[3] = 26.850182 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[4] = -90.000064 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[5] = 11.116645 / 180 * M_PI;
        //x、y轴平面的第一象限上任意一点
        user_coord.jointPara[2].jointRadian[0] = 9.012562 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[1] = 13.378931 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[2] = -47.871256 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[3] = 28.749813 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[4] = -90.000064 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[5] = 9.012275 / 180 * M_PI;
        user_coord.coordType = WorldCoordinate;
        //首先获取当前位置
        cSharpBinding.rs_get_current_waypoint(rshd, ref waypoint);
        //打印路点信息
        cSharpBinding.PrintWaypoint(waypoint);
        ori_onbase = waypoint.orientation;
        pos_onbase = waypoint.cartPos;
        tool_pos.orientation.w = 1;
        tool_pos.orientation.x = 0;
        tool_pos.orientation.y = 0;
        tool_pos.orientation.z = 0;
        tool_pos.cartPos.x = 0;
        tool_pos.cartPos.y = 0;
        tool_pos.cartPos.z = 0;

        //基座坐标系转用户坐标系
        cSharpBinding.rs_base_to_user(rshd, ref pos_onbase, ref ori_onbase, ref user_coord, ref tool_pos, ref pos_onuser, ref ori_onuser);

        return 0;
    }
    static int userToBaseExample()
    {
        cSharpBinding.Ori ori_onbase = new cSharpBinding.Ori();
        cSharpBinding.Pos pos_onbase = new cSharpBinding.Pos();
        cSharpBinding.Pos pos_onuser = new cSharpBinding.Pos();
        cSharpBinding.Ori ori_onuser = new cSharpBinding.Ori();
        cSharpBinding.ToolInEndDesc tool_pos = new cSharpBinding.ToolInEndDesc();
        cSharpBinding.CoordCalibrate user_coord = new cSharpBinding.CoordCalibrate();
        IntPtr pt_user_coord = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.CoordCalibrate)));
        user_coord = (cSharpBinding.CoordCalibrate)Marshal.PtrToStructure(pt_user_coord, typeof(cSharpBinding.CoordCalibrate));
        //设置工具坐标系
        tool_pos.orientation.w = 1;
        tool_pos.orientation.x = 0;
        tool_pos.orientation.y = 0;
        tool_pos.orientation.z = 0;
        tool_pos.cartPos.x = 0;
        tool_pos.cartPos.y = 0;
        tool_pos.cartPos.z = 0;
        //用坐标系转基座坐标系
        pos_onbase.x = 0;
        pos_onbase.y = 0;
        pos_onbase.z = 0;
        ori_onbase.w = 1;
        ori_onbase.x = 0;
        ori_onbase.y = 0;
        ori_onbase.z = 0;
        cSharpBinding.rs_user_to_base(rshd, ref pos_onuser, ref ori_onuser, ref user_coord, ref tool_pos, ref pos_onbase, ref ori_onbase);
        CustomLog.Log($"onbase pos.x={pos_onbase.x}\npos.y={pos_onbase.y}\npos.z={pos_onbase.z}\n");
        return 0;
    }

    static int forwardAndInverseExample()
    {
        cSharpBinding.wayPoint_S waypoint = new cSharpBinding.wayPoint_S();
        cSharpBinding.Pos pos = new cSharpBinding.Pos();
        cSharpBinding.Ori ori = new cSharpBinding.Ori();
        //正解测试
        double[] temp_joint = {-4.358721 / 180 * M_PI,
                                -6.4477722 / 180 * M_PI,
                                -68.298347 / 180 * M_PI,
                                19.191783 / 180 * M_PI,
                                -81.194208 / 180 * M_PI,
                                -4.366669 / 180 * M_PI
                            };
        //正解
        cSharpBinding.rs_forward_kin(rshd, temp_joint, ref waypoint);
        //打印路点信息
        cSharpBinding.PrintWaypoint(waypoint);
        //逆解测试                        
        pos.x = -0.484595;
        pos.y = 0.030930;
        pos.z = 0.341558;
        //参考当前机械臂姿态
        cSharpBinding.rs_get_current_waypoint(rshd, ref waypoint);
        ori = waypoint.orientation;
        double[] zier1 = { 0, 0, 0, 0, 0, 0 };
        //逆解
        cSharpBinding.rs_inverse_kin(rshd, waypoint.jointpos, ref pos, ref ori, ref waypoint);
        cSharpBinding.PrintWaypoint(waypoint);

        cSharpBinding.rs_move_joint(rshd, zier1, true);
        cSharpBinding.rs_move_joint(rshd, waypoint.jointpos, true);
       
        return 0;
    }
    static int quaternion_rpy_example()
    {
        cSharpBinding.wayPoint_S waypoint = new cSharpBinding.wayPoint_S();
        cSharpBinding.Ori ori = new cSharpBinding.Ori();
        cSharpBinding.Rpy rpy = new cSharpBinding.Rpy();
        cSharpBinding.rs_get_current_waypoint(rshd, ref waypoint);
        //四元数转欧拉角
        ori = waypoint.orientation;
        cSharpBinding.rs_quaternion_to_rpy(rshd, ref ori, ref rpy);
        CustomLog.Log($"rpy.rx={rpy.rx * 180 / M_PI}\nrpy.ry={rpy.ry * 180 / M_PI}\nrpy.rz={rpy.rz * 180 / M_PI}\n");
        //欧拉角转四元数
        cSharpBinding.rs_rpy_to_quaternion(rshd, ref rpy, ref ori);
        CustomLog.Log($"ori.w={ori.w} x={ori.x} y={ori.y} z={ori.z}");
        return 0;
    }
    static int jointMoveExample()
    {
        double[] target0 = {0,0,0,0,0,0 };
        double[] target1 = { 1, 1, 1, 1, 1, 1 };
        
        cSharpBinding.ToolInEndDesc tool_pos = new cSharpBinding.ToolInEndDesc();
        tool_pos.orientation.w = 1;
        tool_pos.orientation.x = 0;
        tool_pos.orientation.y = 0;
        tool_pos.orientation.z = 0;
        tool_pos.cartPos.x = 0;
        tool_pos.cartPos.y = 0;
        tool_pos.cartPos.z = 0;
        //轴动机械臂
        cSharpBinding.rs_move_joint(rshd, target0, true);
        for (int i = 0; i < 5; i++)
        {
            cSharpBinding.rs_move_joint(rshd, target1, true);
            cSharpBinding.rs_move_joint(rshd, target0, true);
        }

        return 0;
    }
    public static int moveLineToTargetExample(double offsetX, double offsetY)
    {
        cSharpBinding.Pos pos = new cSharpBinding.Pos();
        //轴动到目标位置                   
        // pos.x = -0.502071;
        //pos.y = -0.104238;
        pos.x = -0.502071 + 0.1 * offsetX > 0 ? 1 :-1 ;
        pos.y = -0.104238 + 0.1 * offsetY > 0 ? 1 :-1 ;
        //pos.x = position.X / 180 * M_PI;
        //pos.y = position.Y / 180 * M_PI; 
        pos.z = 0.476826;
        cSharpBinding.ToolInEndDesc tool_pos = new cSharpBinding.ToolInEndDesc();
        tool_pos.orientation.w = 1;
        tool_pos.orientation.x = 0;
        tool_pos.orientation.y = 0;
        tool_pos.orientation.z = 0;
        tool_pos.cartPos.x = 0;
        tool_pos.cartPos.y = 0;
        tool_pos.cartPos.z = 0;
        cSharpBinding.rs_move_line_to(rshd, ref pos, ref tool_pos, true);
        return 0;
    }
    static int getTargetWaypointByPositionExample()
    {
        cSharpBinding.wayPoint_S target_waypoint_on_basecoord = new cSharpBinding.wayPoint_S();
        cSharpBinding.wayPoint_S source_waypoint_on_basecoord = new cSharpBinding.wayPoint_S();
        cSharpBinding.Pos tool_End_Position = new cSharpBinding.Pos();
        cSharpBinding.ToolInEndDesc tool_pos = new cSharpBinding.ToolInEndDesc();
        cSharpBinding.CoordCalibrate user_coord = new cSharpBinding.CoordCalibrate();
        IntPtr pt_user_coord = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.CoordCalibrate)));
        user_coord = (cSharpBinding.CoordCalibrate)Marshal.PtrToStructure(pt_user_coord, typeof(cSharpBinding.CoordCalibrate));

        cSharpBinding.rs_get_current_waypoint(rshd ,ref source_waypoint_on_basecoord);
        
        user_coord.coordType = WorldCoordinate;
        //坐标系标定方法xoxy
        user_coord.methods = Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;
        //为了简化，使用法兰盘中心为工具端
        user_coord.toolDesc.cartPos.x = 0;
        user_coord.toolDesc.cartPos.y = 0;
        user_coord.toolDesc.cartPos.z = 0;
        user_coord.toolDesc.orientation.w = 1;
        user_coord.toolDesc.orientation.x = 0;
        user_coord.toolDesc.orientation.y = 0;
        user_coord.toolDesc.orientation.z = 0;
        //原点
        user_coord.jointPara[0].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[1] = -7.291862 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[2] = -75.694718 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[3] = 21.596727 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[5] = -0.00458 / 180 * M_PI;
        //x轴正半轴上一点
        user_coord.jointPara[1].jointRadian[0] = 14.283838 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[1] = -1.310068 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[2] = -70.235286 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[3] = 21.074762 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[4] = -90.000098 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[5] = 14.283551 / 180 * M_PI;
        //x、y轴平面的第一象限上任意一点
        user_coord.jointPara[2].jointRadian[0] = 11.701804 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[1] = 11.909616 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[2] = -55.591688 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[3] = 22.498677 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[4] = -90.000037 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[5] = 11.701517 / 180 * M_PI;
        //设置工具坐标系
        tool_pos.orientation.w = 1;
        tool_pos.orientation.x = 0;
        tool_pos.orientation.y = 0;
        tool_pos.orientation.z = 0;
        tool_pos.cartPos.x = 0;
        tool_pos.cartPos.y = 0;
        tool_pos.cartPos.z = 0;
        //工具末端目标位置
        tool_End_Position.x = 0.2;
        tool_End_Position.y = 0;
        tool_End_Position.z = 0;

        cSharpBinding.rs_get_target_waypoint_by_position(rshd, ref source_waypoint_on_basecoord, ref user_coord, ref tool_End_Position, ref tool_pos, ref target_waypoint_on_basecoord );
        cSharpBinding.rs_move_line(rshd,target_waypoint_on_basecoord.jointpos,true);
        return 0;
    }
    static int toolCalibrationExample()
    {
        
        cSharpBinding.ToolInEndDesc tool_pos = new cSharpBinding.ToolInEndDesc();
        cSharpBinding.ToolCalibrate toolCalibrate = new cSharpBinding.ToolCalibrate();
        cSharpBinding.wayPoint_S currentWaypoint = new cSharpBinding.wayPoint_S();
        toolCalibrate.posCalibrateWaypoint = new cSharpBinding.wayPoint_S[4];
        toolCalibrate.oriCalibrateWaypoint = new cSharpBinding.wayPoint_S[3];

        CustomLog.Log($"---------------------------------------------------------------------------------------");
        CustomLog.Log($"工具标定选点：\n");
        double[] jointpos = { 0, 0, 0, 0, 0, 0 };

        jointpos[0] = -0.000172 / 180 * M_PI;
        jointpos[1] = -7.291862 / 180 * M_PI;
        jointpos[2] = -75.694718 / 180 * M_PI;
        jointpos[3] = 21.596727 / 180 * M_PI;
        jointpos[4] = -89.999982 / 180 * M_PI;
        jointpos[5] = -0.000458 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.posCalibrateWaypoint[0] = currentWaypoint;
        CustomLog.Log($"Position_1:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        jointpos[0] = -7.157335 / 180 * M_PI;
        jointpos[1] = -9.198799 / 180 * M_PI;
        jointpos[2] = -78.285142 / 180 * M_PI;
        jointpos[3] = 25.303605 / 180 * M_PI;
        jointpos[4] = -58.632352 / 180 * M_PI;
        jointpos[5] = -8.390680 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.posCalibrateWaypoint[1] = currentWaypoint;
        CustomLog.Log($"Position_2:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        jointpos[0] = 8.423011 / 180 * M_PI;
        jointpos[1] = -4.640062 / 180 * M_PI;
        jointpos[2] = -74.922312 / 180 * M_PI;
        jointpos[3] = 26.760133 / 180 * M_PI;
        jointpos[4] = -129.625885 / 180 * M_PI;
        jointpos[5] = 10.962565 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.posCalibrateWaypoint[2] = currentWaypoint;
        CustomLog.Log($"Position_3:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        jointpos[0] = 1.981716 / 180 * M_PI;
        jointpos[1] = 0.742380 / 180 * M_PI;
        jointpos[2] = -63.190745 / 180 * M_PI;
        jointpos[3] = 47.592509 / 180 * M_PI;
        jointpos[4] = -98.514182 / 180 * M_PI;
        jointpos[5] = -11.502224 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.posCalibrateWaypoint[3] = currentWaypoint;
        CustomLog.Log($"Position_4:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        jointpos[0] = -0.000172 / 180 * M_PI;
        jointpos[1] = -7.291862 / 180 * M_PI;
        jointpos[2] = -75.694718 / 180 * M_PI;
        jointpos[3] = 21.596727 / 180 * M_PI;
        jointpos[4] = -89.999982 / 180 * M_PI;
        jointpos[5] = -0.000458 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.oriCalibrateWaypoint[0] = currentWaypoint;
        CustomLog.Log($"Orientation_1:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        jointpos[0] = 14.285429 / 180 * M_PI;
        jointpos[1] = -1.309190 / 180 * M_PI;
        jointpos[2] = -70.234425 / 180 * M_PI;
        jointpos[3] = 21.074762 / 180 * M_PI;
        jointpos[4] = -90.000084 / 180 * M_PI;
        jointpos[5] = 14.285143 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.oriCalibrateWaypoint[1] = currentWaypoint;
        CustomLog.Log($"Orientation_2:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        jointpos[0] = 12.018674 / 180 * M_PI;
        jointpos[1] = 9.866897 / 180 * M_PI;
        jointpos[2] = -58.086873 / 180 * M_PI;
        jointpos[3] = 22.046228 / 180 * M_PI;
        jointpos[4] = -90.000071 / 180 * M_PI;
        jointpos[5] = 12.018387 / 180 * M_PI;
        cSharpBinding.rs_move_line(rshd, jointpos, true);
        cSharpBinding.rs_get_current_waypoint(rshd, ref currentWaypoint);
        toolCalibrate.oriCalibrateWaypoint[2] = currentWaypoint;
        CustomLog.Log($"Orientation_3:");
        cSharpBinding.PrintWaypoint(currentWaypoint);

        toolCalibrate.posCalibrateNum = 4;
        toolCalibrate.oriCalibrateNum = 3;
        toolCalibrate.CalibMethod = ToolKinematicsOriCalibrateMathod_xOxy;
        //IntPtr ptr_toolCalibrate = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.ToolCalibrate)));
        //Marshal.StructureToPtr(toolCalibrate, ptr_toolCalibrate, true);
        cSharpBinding.rs_tool_calibration(rshd, ref toolCalibrate, ref tool_pos);
        //CustomLog.Log($"工具标定结果:\n");
        //CustomLog.Log($"pos: x={0},y={1},z={2}", tool_pos.cartPos.x, tool_pos.cartPos.y, tool_pos.cartPos.z);
        //CustomLog.Log($"ori: x={0},y={1},z={2},w={3}", tool_pos.orientation.x, tool_pos.orientation.y, tool_pos.orientation.z, tool_pos.orientation.w);
        //CustomLog.Log($"---------------------------------------------------------------------------------------");
        return 0;
    }

    static int getJointStatusExample()
    {
        /* c#向c++封送结构体问题
         * 要先分配一段非托管内存，接口中结构体参数应传入IntPtr
         * 调用接口后，再将非托管内容数据读取到托管结构体数据中
         */
        int size = Marshal.SizeOf(typeof(cSharpBinding.JointStatus)) * 6;
        byte[] bytes = new byte[size];
        IntPtr pBuff = Marshal.AllocHGlobal(size);
        cSharpBinding.JointStatus[] jointStatus = new cSharpBinding.JointStatus[6];
        cSharpBinding.rs_get_joint_status(rshd, pBuff);
        for (int i = 0; i < 6; i++)
        {
            IntPtr pPonitor = new IntPtr(pBuff.ToInt64() + Marshal.SizeOf(typeof(cSharpBinding.JointStatus)) * i);
            jointStatus[i] = (cSharpBinding.JointStatus)Marshal.PtrToStructure(pPonitor, typeof(cSharpBinding.JointStatus));
        }
        //for (int i = 0; i < 6; i++)
        //{
        //    CustomLog.Log($"jointStatus {0}",i+1);
        //    CustomLog.Log($"joint I: {0}\njoint Speed: {1}\njoint Angel: {2}", jointStatus[i].jointCurrentI, jointStatus[i].jointSpeedMoto, jointStatus[i].jointPosJ);
        //    CustomLog.Log($"joint vol: {0}\njoint temp: {1}\njoint target I: {2}", jointStatus[i].jointCurVol, jointStatus[i].jointCurTemp, jointStatus[i].jointTagCurrentI);
        //    CustomLog.Log($"joint target Speed: {0}\njoint Target Angel: {1}\njoint Errnum: {2}", jointStatus[i].jointTagSpeedMoto, jointStatus[i].jointTagPosJ, jointStatus[i].jointErrorNum);
        //    CustomLog.Log($"---------------------------------------------------------------------------------------");

        //}
        Marshal.FreeHGlobal(pBuff);

        return 0;
    }

    static int getRobotDiagnosisExample()
    {
        cSharpBinding.RobotDiagnosis robotDiagnosis = new cSharpBinding.RobotDiagnosis();
        cSharpBinding.rs_get_diagnosis_info(rshd,ref robotDiagnosis);

        //CustomLog.Log($"armCanbusStatus:{0}\n", robotDiagnosis.armCanbusStatus);
        //CustomLog.Log($"armPowerCurrent:{0}\n", robotDiagnosis.armPowerCurrent);
        //CustomLog.Log($"armPowerVoltage:{0}\n", robotDiagnosis.armPowerVoltage);
        //CustomLog.Log($"armPowerStatus:{0}\n", robotDiagnosis.armPowerStatus);
        //CustomLog.Log($"contorllerTemp:{0}\n", robotDiagnosis.contorllerTemp);
        //CustomLog.Log($"contorllerHumidity:{0}\n", robotDiagnosis.contorllerHumidity);
        //CustomLog.Log($"remoteHalt:{0}\n", robotDiagnosis.remoteHalt);
        //CustomLog.Log($"softEmergency:{0}\n", robotDiagnosis.softEmergency);
        //CustomLog.Log($"remoteEmergency:{0}\n", robotDiagnosis.remoteEmergency);
        //CustomLog.Log($"robotCollision:{0}\n", robotDiagnosis.robotCollision);
        //CustomLog.Log($"forceControlMode:{0}\n", robotDiagnosis.forceControlMode);
        //CustomLog.Log($"brakeStuats:{0}\n", robotDiagnosis.brakeStuats);
        //CustomLog.Log($"robotEndSpeed:{0}\n", robotDiagnosis.robotEndSpeed);
        //CustomLog.Log($"robotMaxAcc:{0}\n", robotDiagnosis.robotMaxAcc);
        //CustomLog.Log($"orpeStatus:{0}\n", robotDiagnosis.orpeStatus);
        //CustomLog.Log($"enableReadPose:{0}\n", robotDiagnosis.enableReadPose);
        //CustomLog.Log($"robotMountingPoseChanged:{0}\n", robotDiagnosis.robotMountingPoseChanged);
        //CustomLog.Log($"encoderErrorStatus:{0}\n", robotDiagnosis.encoderErrorStatus);
        //CustomLog.Log($"staticCollisionDetect:{0}\n", robotDiagnosis.staticCollisionDetect);
        //CustomLog.Log($"jointCollisionDetect:{0}\n", robotDiagnosis.jointCollisionDetect);
        //CustomLog.Log($"encoderLinesError:{0}\n", robotDiagnosis.encoderLinesError);
        //CustomLog.Log($"jointErrorStatus:{0}\n", robotDiagnosis.jointErrorStatus);
        //CustomLog.Log($"singularityOverSpeedAlarm:{0}\n", robotDiagnosis.singularityOverSpeedAlarm);
        //CustomLog.Log($"robotCurrentAlarm:{0}\n", robotDiagnosis.robotCurrentAlarm);
        //CustomLog.Log($"toolIoError:{0}\n", robotDiagnosis.toolIoError);
        //CustomLog.Log($"robotMountingPoseWarning:{0}\n", robotDiagnosis.robotMountingPoseWarning);
        //CustomLog.Log($"macTargetPosBufferSize:{0}\n", robotDiagnosis.macTargetPosBufferSize);
        //CustomLog.Log($"macTargetPosDataSize:{0}\n", robotDiagnosis.macTargetPosDataSize);
        //CustomLog.Log($"macDataInterruptWarning:{0}\n", robotDiagnosis.macDataInterruptWarning);
        //CustomLog.Log($"controlBoardAbnormalStateFlag:{0}\n", robotDiagnosis.controlBoardAbnormalStateFlag);

        return 0;
    }

    static int getDeviceInfoExample()
    {
        cSharpBinding.RobotDevInfo dev = new cSharpBinding.RobotDevInfo();
        cSharpBinding.rs_get_device_info(rshd,ref dev);
        return 0;
    }

    static int userCoordCalibrateExample()
    {
        cSharpBinding.CoordCalibrate user_coord = new cSharpBinding.CoordCalibrate();
        IntPtr pt_user_coord = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(cSharpBinding.CoordCalibrate)));
        user_coord = (cSharpBinding.CoordCalibrate)Marshal.PtrToStructure(pt_user_coord, typeof(cSharpBinding.CoordCalibrate));

        user_coord.coordType = WorldCoordinate;
        //坐标系标定方法xoxy
        user_coord.methods = Origin_AnyPointOnPositiveXAxis_AnyPointOnFirstQuadrantOfXOYPlane;
        //为了简化，使用法兰盘中心为工具端
        user_coord.toolDesc.cartPos.x = 0;
        user_coord.toolDesc.cartPos.y = 0;
        user_coord.toolDesc.cartPos.z = 0;
        user_coord.toolDesc.orientation.w = 1;
        user_coord.toolDesc.orientation.x = 0;
        user_coord.toolDesc.orientation.y = 0;
        user_coord.toolDesc.orientation.z = 0;
        
        //传正常坐标系
        //原点
        user_coord.jointPara[0].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[1] = -7.291862 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[2] = -75.694718 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[3] = 21.596727 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[5] = -0.00458 / 180 * M_PI;
        //x轴正半轴上一点
        user_coord.jointPara[1].jointRadian[0] = 14.283838 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[1] = -1.310068 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[2] = -70.235286 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[3] = 21.074762 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[4] = -90.000098 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[5] = 14.283551 / 180 * M_PI;
        //x、y轴平面的第一象限上任意一点
        user_coord.jointPara[2].jointRadian[0] = 11.701804 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[1] = 11.909616 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[2] = -55.591688 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[3] = 22.498677 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[4] = -90.000037 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[5] = 11.701517 / 180 * M_PI;
        int ret= cSharpBinding.rs_check_user_coord(rshd, ref user_coord);
        if (ret == 0)
        {
            double[] bInWPos = { 0, 0, 0 };
            double[] bInWOri = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] wInBPos = { 0, 0, 0 };

            cSharpBinding.rs_user_coord_calibrate(rshd, ref user_coord, bInWPos, bInWOri, wInBPos);
            CustomLog.Log($"bInWPos: x={bInWPos[0]:f5}  y={bInWPos[1]:f5}  z={bInWPos[2]:f5}");
            CustomLog.Log($"wInBPos: x={wInBPos[0]:f5}  y={wInBPos[1]:f5}  z={wInBPos[2]:f5}");
            CustomLog.Log($"bInWOri:");
            for (int i = 0; i < 9; i++)
            {
                Console.Out.Write("{0,10:f5}", bInWOri[i]);
                if (i == 2 || i == 5 || i == 8)
                {
                    Console.Out.Write("\n");
                }
            }
            CustomLog.Log($"---------------------------------------------------------------------------------------");
        }
        else {
            Console.Out.Write("UserCoord Calibrate Fail!\n");
            CustomLog.Log($"---------------------------------------------------------------------------------------");
        }

        //传一条直线
        user_coord.jointPara[0].jointRadian[0] = -0.000172 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[1] = -7.291862 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[2] = -75.694718 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[3] = 21.596727 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[0].jointRadian[5] = -0.00458 / 180 * M_PI;
        
        user_coord.jointPara[1].jointRadian[0] = -0.0000172 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[1] = 11.351293 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[2] = -23.993406 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[3] = 54.655300 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[1].jointRadian[5] = -0.000458 / 180 * M_PI;
        
        user_coord.jointPara[2].jointRadian[0] = -0.0000172 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[1] = -10.055338 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[2] = -111.085131 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[3] = -11.029791 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[4] = -89.999982 / 180 * M_PI;
        user_coord.jointPara[2].jointRadian[5] = -0.000458 / 180 * M_PI;
        ret = cSharpBinding.rs_check_user_coord(rshd, ref user_coord);
        if (ret == 0)
        {
            double[] bInWPos = { 0, 0, 0 };
            double[] bInWOri = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            double[] wInBPos = { 0, 0, 0 };

            cSharpBinding.rs_user_coord_calibrate(rshd, ref user_coord, bInWPos, bInWOri, wInBPos);
            CustomLog.Log($"bInWPos: x={bInWPos[0]:f5}  y={bInWPos[1]:f5}  z={bInWPos[2]:f5}");
            CustomLog.Log($"wInBPos: x={wInBPos[0]:f5}  y={wInBPos[1]:f5}  z={wInBPos[2]:f5}");
            CustomLog.Log($"bInWOri:");
            for (int i = 0; i < 9; i++)
            {
                Console.Out.Write("{0,10:f5}", bInWOri[i]);
                if (i == 2 || i == 5 || i == 8)
                {
                    Console.Out.Write("\n");
                }
            }
            CustomLog.Log($"---------------------------------------------------------------------------------------");
        }
        else
        {
            Console.Out.Write("UserCoord Calibrate Fail!\n");
            CustomLog.Log($"---------------------------------------------------------------------------------------");
        }

        return 0;
    }

    public static void RunRobot ()
    {
        cSharpBinding csharpbingding = new cSharpBinding();
        double end_max_angle_velc = 3;
        double end_max_angle_acc = 3;
        double[] joint_max_velc = { 1.5, 1.5, 1.5, 1.5, 1.5, 1.5 };
        double[] joint_max_acc = { 5, 5, 5, 5, 5, 5 };
        double io_status = 0;
        int result = 0xffff;
        CustomLog.Log($"call rs_initialize");

        //初始化机械臂控制库
        result = cSharpBinding.rs_initialize ();
        CustomLog.Log($"rs_initialize.ret={result}");

        if (RSERR_SUCC == result)
        {
           //创建机械臂控制上下文句柄
           if (cSharpBinding.rs_create_context (ref rshd) == RSERR_SUCC)
            {
               CustomLog.Log($"rshd={rshd}");
               //链接机械臂服务器
               if (cSharpBinding.rs_login(rshd, robotIP, serverPort) == RSERR_SUCC)
               {
                   CustomLog.Log($"login succ.");
                   cSharpBinding.ToolDynamicsParam tool = new cSharpBinding.ToolDynamicsParam { positionX = 0, positionY = 0,positionZ=0,payload=0,};
                   tool.toolInertia = new cSharpBinding.ToolInertia { xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz=0 };
                   int state = 0;

                   result=cSharpBinding.rs_robot_startup( rshd,
                                                        ref tool,
                                                        6        /*Collision level*/,
                                                        true     /*Whether to allow reading poses defaults to true*/,
                                                        true,    /*Leave the default to true */
                                                        1000,    /*Leave the default to 1000 */
                                                        ref state);
                        
                    //设置是否允许实时路点信息推送
                    //cSharpBinding.rs_enable_push_realtime_roadpoint(rshd, true);

                    //函数指针实例化
                    //cSharpBinding.REALTIME_ROADPOINT_CALLBACK RobotPosCallBack = new cSharpBinding.REALTIME_ROADPOINT_CALLBACK(cSharpBinding.CurrentPositionCallback);
                    //cSharpBinding.rs_setcallback_realtime_roadpoint(rshd, RobotPosCallBack, IntPtr.Zero);

                    //机械臂事件回调
                    //cSharpBinding.ROBOT_EVENT_CALLBACK RobotEventCallbackPtr = new cSharpBinding.ROBOT_EVENT_CALLBACK(cSharpBinding.RobotEventCallback);
                    //cSharpBinding.rs_setcallback_robot_event(rshd, RobotEventCallbackPtr, IntPtr.Zero);

                    //cSharpBinding.REALTIME_ENDSPEED_CALLBACK RobotEndSpeedCallbackPtr = new cSharpBinding.REALTIME_ENDSPEED_CALLBACK(cSharpBinding.CurrentEndSpeedCallback);
                    //cSharpBinding.rs_setcallback_realtime_end_speed(rshd, RobotEndSpeedCallbackPtr, IntPtr.Zero);

                    //cSharpBinding.ROBOT_JOINT_STATUS_CALLBACK RobotJointStatusCallbackPtr = new cSharpBinding.ROBOT_JOINT_STATUS_CALLBACK(cSharpBinding.CurrentJointStatusCallback);
                    //cSharpBinding.rs_setcallback_realtime_joint_status(rshd, RobotJointStatusCallbackPtr , IntPtr.Zero);

                    //Thread.Sleep(50 * 1000);

                    cSharpBinding.wayPoint_S waypoint = new cSharpBinding.wayPoint_S();
                    cSharpBinding.JointVelcAccParam temp = new cSharpBinding.JointVelcAccParam();
                    //直接获取机械臂当前位置信息
                    cSharpBinding.rs_get_current_waypoint(rshd, ref waypoint);
                    //打印路点信息
                    cSharpBinding.PrintWaypoint(waypoint);

                    //根据错误号返回错误信息
                    int err_code = 10024;
                    IntPtr _errinfo = cSharpBinding.rs_get_error_information_by_errcode(rshd, err_code);
                    string err_information = Marshal.PtrToStringAnsi(_errinfo);
                    CustomLog.Log($"err_information is:{err_information}");
                    CustomLog.Log($"---------------------------------------------------------------------------------------");
                    //设置碰撞等级
                    int grade = 2;
                    cSharpBinding.rs_set_collision_class(rshd, grade);
                    CustomLog.Log($"set robot collision ret is:{+grade}");
                    //获取当前碰撞等级
                    int current_grade = 0;
                    cSharpBinding.rs_get_collision_class(rshd, ref current_grade);
                    CustomLog.Log($"current grade is:{current_grade}");
                    CustomLog.Log($"---------------------------------------------------------------------------------------");
                    //初始化全局的运动属性
                    cSharpBinding.rs_init_global_move_profile(rshd);

                    //设置六个关节轴动的最大加速度
                    cSharpBinding.rs_set_global_joint_maxacc(rshd, joint_max_acc);

                    //设置六个关节轴动的最大速度
                    cSharpBinding.rs_set_global_joint_maxvelc(rshd, joint_max_velc);

                    //获取六个关节轴动的最大速度
                    cSharpBinding.rs_get_global_joint_maxvelc(rshd, ref temp);
                    CustomLog.Log($"joint max velc = {temp.jointPara[0]},{temp.jointPara[1]}," +
                        $"{temp.jointPara[2]},{temp.jointPara[3]}," +
                        $"{temp.jointPara[4]},{temp.jointPara[5]}");

                    //获取六个关节轴动的最大加速度
                    cSharpBinding.rs_get_global_joint_maxacc(rshd, ref temp);
                    CustomLog.Log($"joint max acc = {temp.jointPara[0]},{temp.jointPara[1]}," +
                        $"{temp.jointPara[2]},{temp.jointPara[3]}," +
                        $"{temp.jointPara[4]},{temp.jointPara[5]}");

                    //设置末端型运动旋转最大角加速度
                    cSharpBinding.rs_set_global_end_max_angle_acc(rshd, end_max_angle_acc);
                    //设置末端型运动旋转最大角速度
                    cSharpBinding.rs_set_global_end_max_angle_velc(rshd, end_max_angle_velc);
                    //获取末端型运动旋转最大角加速度
                    double get_end_max_angle_acc = 0;
                    cSharpBinding.rs_get_global_end_max_angle_acc(rshd, ref get_end_max_angle_acc);
                    CustomLog.Log($"end max angle acc = {get_end_max_angle_acc}");
                    //获取末端型运动旋转最大角速度
                    double get_end_max_angle_velc = 0;
                    cSharpBinding.rs_get_global_end_max_angle_velc(rshd, ref get_end_max_angle_velc);
                    CustomLog.Log($"end max angle velc = {get_end_max_angle_velc}");
                    CustomLog.Log($"---------------------------------------------------------------------------------------");
                    //检查用户坐标系是否合理
                    //checkUsercoordExample();

                    //坐标系偏移Example
                    //relativeOffsetExample();

                    //基坐标系转用户坐标系
                    //baseToUserExample();

                    //用户坐标系转基坐标系
                    //userToBaseExample();

                    //正逆解测试
                    //forwardAndInverseExample();

                    //四元数与欧拉角转换
                    //quaternion_rpy_example();

                    //轴动
                    //jointMoveExample();
                    
                    //直线运动到目标位置
                    //moveLineToTargetExample(1,1);

                    //根据位置获取目标路点信息(获取基于基座标下的目标路点通过基于用户坐标系的位置，目标路点保持起点姿态)
                    //getTargetWaypointByPositionExample();

                    //工具标定
                    //toolCalibrationExample();

                    //获取关节状态信息
                    //getJointStatusExample();

                    //获取机器人诊断信息
                    //getRobotDiagnosisExample();

                    //获取设备信息
                    //getDeviceInfoExample();

                    //坐标系标定检查
                    //userCoordCalibrateExample();


                    ////设置用户IO（必须连接控制柜！！！）                        
                    //if (RSERR_SUCC == (result = cSharpBinding.rs_set_board_io_status_by_addr(rshd, Robot_User_DO, ROBOT_IO_U_DO_00, IO_STATUS_VALID)))
                    //    CustomLog.Log($"set io succ");
                    //else
                    //    CustomLog.Log($"set io error, error code={0}", result);

                    ////获取用户IO（必须连接控制柜！！！）
                    //if (RSERR_SUCC == (result = cSharpBinding.rs_get_board_io_status_by_addr(rshd, Robot_User_DI, ROBOT_IO_U_DI_00, ref io_status)))
                    //    CustomLog.Log($"get io status = {0}", io_status);
                    //else
                    //    CustomLog.Log($"get io status error, error code={0}", result);

                    ////设置工具端IO
                    //if (RSERR_SUCC == (result = cSharpBinding.rs_set_tool_do_status(rshd, TOOL_IO_0, 1)))
                    //    CustomLog.Log($"set io succ");
                    //else
                    //    CustomLog.Log($"set io error, error code={0}", result);

                    ////获取设置工具端IO
                    //if (RSERR_SUCC == (result = cSharpBinding.rs_get_tool_io_status(rshd, TOOL_IO_0, ref io_status)))
                    //    CustomLog.Log($"get tool_io status = {0}", io_status);
                    //else
                    //    CustomLog.Log($"get tool_io status error, error code={0}", result);

                    //Thread.Sleep(1*1000);

                    //机器人下电
                    //cSharpBinding.rs_robot_shutdown(rshd);
                    //断开机械臂服务器链接
                    //cSharpBinding.rs_logout(rshd);
                    
               } else {
                          CustomLog.Log($"login failed!");
                       }
               //注销机械臂控制上下文句柄
             //  cSharpBinding.rs_destory_context (rshd);
          } else {
                     CustomLog.Log($"rs_create_context failed!");
                  }

         //反初始化机械臂控制库
         //cSharpBinding.rs_uninitialize ();
    } else {
              CustomLog.Log($"rs_initialize failed!");
            }

        CustomLog.Log($"bye");
    }
    public static void StopRobot()
    {
        cSharpBinding.rs_logout(rshd);
        cSharpBinding.rs_destory_context(rshd);
        cSharpBinding.rs_uninitialize();

    }
    
}
