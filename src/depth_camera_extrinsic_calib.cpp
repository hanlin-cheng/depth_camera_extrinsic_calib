#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>//PLY相关头文件
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>// ICP 相关头文件
#include <pcl/visualization/pcl_visualizer.h>//可视化类头文件
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

typedef pcl::PointXYZ PointT;// x,y,z点
typedef pcl::PointCloud<PointT> PointCloudT;//点云　申明pcl::PointXYZ数据

bool next_iteration = false;

 //打印旋转矩阵和平移矩阵
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing)
{//使用空格键来增加迭代次数，并更新显示
    if (event.getKeySym () == "space" && event.keyDown ())
        next_iteration = true;
}

void toPointCloud(const Mat & depthImg, PointCloudT::Ptr & cloud, const Mat & intrinsic， const float & depth)
{
    for (unsigned int u = 0; u < depthImg.rows; ++u) 
    {
            for (unsigned int v = 0; v < depthImg.cols; ++v) 
            {
                double z = (double)(depthImg.at<ushort>(u, v)) * depth;
                if (z <= 0.) 
                {
                    continue;
                }
                double x = (v - intrinsic.at<double>(0, 2)) * z / intrinsic.at<double>(0, 0);
                double y = (u - intrinsic.at<double>(1, 2)) * z / intrinsic.at<double>(1, 1);

                pcl::PointXYZ node;
                node.x = x;
                node.y = y;
                node.z = z;
                cloud->points.push_back(node);
            }
        }
        cloud->width = 1;
        cloud->height = cloud->points.size();
        cloud->is_dense = false;
}

// //加载点云并保存在总体的点云列表中
// void loadData (int argc, char **argv, PointCloudT::Ptr & model)
// {
//     std::string extension (".pcd");
//     // 第一个参数是命令本身，所以要从第二个参数开始解析
//     std::string fname = std::string (argv[1]);
//     // PCD文件名至少为5个字符大小字符串（因为后缀名.pcd就已经占了四个字符位置）
//     if (fname.size () <= extension.size ())
//         continue;

//     std::transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
//     //检查参数是否为一个pcd后缀的文件
//     if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
//     {
//         if(pcl::io::loadPCDFile (argv[1], *model) == -1)
//         {
//             PCL_ERROR ("Couldn't read file example.pcd \n");
//             return (-1);
//         } 
//         // //从点云中移除NAN点也就是无效点
//         // std::vector<int> indices;
//         // pcl::removeNaNFromPointCloud(*m.cloud,*m.cloud, indices);
//         // models.push_back (m);
//     }
// }


int main (int argc, char* argv[])
{
    // 声明将要使用的 
    PointCloudT::Ptr cloud_in (new PointCloudT);
    PointCloudT::Ptr cloud_tr (new PointCloudT);
    PointCloudT::Ptr cloud_icp (new PointCloudT); 
    PointCloudT::Ptr cloud_in_origin (new PointCloudT);
    PointCloudT::Ptr cloud_icp_origin (new PointCloudT);

    // 检查程序输入命令的合法性  
    if (argc != 6)  //如果只有一个命令说明没有指定目标点云，所以会提示用法
    {
        PCL_ERROR ("usage: depth_camera_extrinsic_calib depth1 depth2 depth filter_dis iterations\n");
        return (-1);
    }

    float filter_dis = 1.0;
    filter_dis = stof (argv[4]);
    int iterations = 1; // 默认的ICP迭代次数
    iterations = atoi (argv[5]);//传递参数的格式转化为int型
    
    if (iterations < 1)  //同时不能设置迭代次数为1
    {
        PCL_ERROR ("Number of initial iterations must be >= 1\n");
        return (-1);
    }

    // 建立3D点
    Mat depth1 = imread(argv[1], cv::IMREAD_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat depth2 = imread(argv[2], cv::IMREAD_UNCHANGED);       // 深度图为16位无符号数，单通道图像
    Mat K = (Mat_<double>(3, 3) << 425, 0, 320, 0, 425, 200, 0, 0, 1);

    float imageDepth = stof (argv[3]);
    toPointCloud(depth1, cloud_in_origin, K, imageDepth);
    toPointCloud(depth2, cloud_icp_origin, K, imageDepth);

    pcl::io::savePCDFileASCII("1_depth_origin.pcd", *cloud_in_origin);
    pcl::io::savePCDFileASCII("2_depth_origin.pcd", *cloud_icp_origin);

    pcl::PassThrough<PointT> pass_cloud_in;
    pcl::PassThrough<PointT> pass_cloud_icp;
    pass_cloud_in.setInputCloud(cloud_in_origin);
    pass_cloud_in.setFilterFieldName("z"); // 选择过滤的维度
    pass_cloud_in.setFilterLimits(0.0, filter_dis); // 设置过滤的范围，这里过滤掉 z 坐标大于 1.0 的点
    pass_cloud_in.filter(*cloud_in); // 应用滤波器
    pass_cloud_in.setInputCloud(cloud_icp_origin);
    pass_cloud_in.setFilterFieldName("z"); // 选择过滤的维度
    pass_cloud_in.setFilterLimits(0.0, filter_dis); // 设置过滤的范围，这里过滤掉 z 坐标大于 1.0 的点
    pass_cloud_in.filter(*cloud_icp); // 应用滤波器
    pcl::io::savePCDFileASCII("1_depth.pcd", *cloud_in);
    pcl::io::savePCDFileASCII("2_depth.pcd", *cloud_icp);
    std::cout << "Saved depth image as point pcd file." << std::endl;

    //  定义旋转矩阵和平移向量Matrix4d是为4*4的矩阵
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();//初始化

    *cloud_tr = *cloud_icp;// 备份cloud_icp赋值给cloud_tr为后期使用

    // 迭代最近点算法
    pcl::console::TicToc time; //申明时间记录
    time.tic ();//时间
    pcl::IterativeClosestPoint<PointT, PointT> icp; // 配准对象
    icp.setMaximumIterations (iterations);//设置最大的迭代次数，即每迭代N次就认为收敛，停止内部迭代
    icp.setTransformationEpsilon(1e-8);//设置变换阈值。当两次迭代之间的变换小于此阈值时，算法认为已经收敛
    icp.setEuclideanFitnessEpsilon(1e-6);//设置欧几里得适应性阈值。用于确定算法是否收敛
    icp.setInputSource (cloud_icp);// 设置源点云
    icp.setInputTarget (cloud_in);// 设置目标点云
    icp.align (*cloud_icp);//匹配后源点云
    icp.setMaximumIterations (1); // 设置为1以便下次调用

    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

    if (icp.hasConverged ())// 输出变换矩阵的适合性评估
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
        std::cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix (transformation_matrix);
    }
    else
    {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    // 可视化ICP的过程与结果
    pcl::visualization::PCLVisualizer viewer ("ICP demo");
    // 创建两个观察视点
    int v1 (0);
    int v2 (1);
    viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);//用左半窗口创建视口vp_1
    viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);//用右半窗口创建视口vp_2

    // 定义显示的颜色信息
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;

    // 原始的点云设置为白色的
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 
                                (int) 255 * txt_gray_lvl, 
                                (int) 255 * txt_gray_lvl,
                                                                (int) 255 * txt_gray_lvl);
    //设置原始的点云都是显示为白色
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

    // 转换后的点云显示为绿色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud (cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

    // ICP配准后的点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 180, 20, 20);
    viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

    // 加入文本的描述在各自的视口界面
    //在指定视口viewport=v1添加字符串“white 。。。”
    //其中"icp_info_1"是添加字符串的ID标志，
    //（10，15）为坐标
    //16为字符大小 
    //后面分别是RGB值
    viewer.addText ("White: Original point cloud\nGreen: Target point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
    viewer.addText ("White: Original point cloud\nRed: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2", v2);

    std::stringstream ss;
    ss << iterations;//输入的迭代的次数
    std::string iterations_cnt = "ICP iterations = " + ss.str ();
    viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

    // 设置背景颜色
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

    // 设置相机的坐标和方向
    viewer.setCameraPosition (0, 0, 0, 1, 0, 0, 0, 0, 1);
    viewer.setSize (1280, 1024);// 可视化窗口的大小

    // 注册按键回调函数
    viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

    // 显示
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();

        //按下空格键的函数
        if (next_iteration)
        {
            // 最近点迭代算法
            time.tic ();
            icp.align (*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc () << " ms" << std::endl;

            if (icp.hasConverged ())
            {
                //printf ("\033[11A");  // Go up 11 lines in terminal output.
                printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_icp -> cloud_in" << std::endl;
                transformation_matrix *= icp.getFinalTransformation ().cast<double>();  
            // WARNING /!\ This is not accurate! For "educational" purpose only! 舍入，凑整

            // matrix[ICP 0->1]*matrix[ICP 1->2]*matrix[ICP 2->3] = matrix[ICP 0->3]
                // 打印矩阵变换
                print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
            // matrix with 20 initial iterations is much more accurate than the one multiplied 19 times.
                ss.str ("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str ();
                viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
                viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }
            else
            {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }
    return (0);
}