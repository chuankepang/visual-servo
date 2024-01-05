#include "z1_control/control/unitreeArm.h"
#include "z1_control/demo_visual_servo.hpp"

using namespace UNITREE_ARM;

using namespace std::chrono_literals;

Eigen::MatrixXd pinv(Eigen::MatrixXd  &A)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double  pinvtoler = 1.e-8; //tolerance
    int row = A.rows();
    int col = A.cols();
    int k = std::min(row,col);
    Eigen::MatrixXd X = Eigen::MatrixXd::Zero(col,row);
    Eigen::MatrixXd singularValues_inv = svd.singularValues();//奇异值
    Eigen::MatrixXd singularValues_inv_mat = Eigen::MatrixXd::Zero(col, row);
    for (long i = 0; i<k; ++i) {
        if (singularValues_inv(i) > pinvtoler)
            singularValues_inv(i) = 1.0 / singularValues_inv(i);
        else singularValues_inv(i) = 0;
    }
    for (long i = 0; i < k; ++i) 
    {
        singularValues_inv_mat(i, i) = singularValues_inv(i);
    }
    X=(svd.matrixV())*(singularValues_inv_mat)*(svd.matrixU().transpose());
 
    return X;
}

Mat6 TransMotionVector(HomoMat &X)
{
    Mat3 c,d,r;
    Mat6 J;   
    c << 0, -X(2,3), X(1,3),
         X(2,3), 0, -X(0,3),
         -X(1,3), X(0,3), 0;
    // d = Vec3(1,1,1).asDiagonal();
    r = getHomoRotMat(X);
    // std::cout<< c << std::endl;
    // std::cout<< X << std::endl;
         
    J << r, -r * c,
         Mat3::Zero(), r;
    return J;     
}

class Z1ARM : public unitreeArm{
public:
    Z1ARM():unitreeArm(true){
        runThread = new LoopFunc("Z1LowCmd", 0.002, boost::bind(&Z1ARM::run, this));

        kp_ = Eigen::Matrix<double, 6, 1>(50,50,50,50,50,50).asDiagonal() * 0.5;
        kd_ = Eigen::Matrix<double, 6, 1>(10,10,10,10,10,10).asDiagonal();
        ki_ = Eigen::Matrix<double, 6, 1>(0.2,0.2,0.2,0.2,0.2,0.2).asDiagonal() * 100;

        std::cout<< kp_ << std::endl;
        std::cout<< kd_ << std::endl;
        std::cout<< ki_ << std::endl;

        q_des << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
        qd_des << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        qdd_des << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    };
    ~Z1ARM(){delete runThread;};
    void run();
    LoopFunc *runThread;
    Eigen::Matrix<double, 6, 6> kp_, kd_, ki_;

    Vec6 q_des, qd_des, qdd_des;
    Vec6 q_now, qd_now, qdd_c;
};

void Z1ARM::run(){
    // 一直循环，试图跟踪期望轨迹(位置、速度、加速度)，视觉伺服只用跟踪速度/加速度
    // q_now = lowstate->getQ();
    // qd_now = lowstate->getQd();

    // qdd_c = -kp_ * (q_now - q_des) - kp_ * (qd_now - q_des) + qdd_des;
    // Vec6 gTemp = _ctrlComp->armModel->inverseDynamics(q_now, qd_now, qdd_c, Vec6::Zero());

    // tau = gTemp;
    
    // tau(0) = 3.;
    // tau(1) = 0.;
    // tau(2) = 0.;
    // tau(3) = 0.;
    // tau(4) = 0.;
    // tau(5) = 0.;

    // tau(1) = 3;
    // tau(2) = -3;

    // for(int i(0); i<6; i++){
    //     std::cout << tau(i) << ', ';
    // }
    // std::cout << std::endl;

    qd = qd_des;
    // std::cout<< q.transpose() << std::endl;
    // std::cout<< tau.transpose() << std::endl;

    sendRecv();
}

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
        // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::callback_send_recv, this));

        rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
        // sub_image_ = image_transport::create_subscription(
        //     this, 
        //     "/color/video/image",
        //     std::bind(&MinimalPublisher::callback_image, this, std::placeholders::_1), 
        //     "raw", 
        //     custom_qos);

        sub_cam_ = image_transport::create_camera_subscription(
            this,
            "/color/video/image", 
            // &MinimalPublisher::callback_image,
            std::bind(&MinimalPublisher::callback_image, this, std::placeholders::_1, std::placeholders::_2), 
            // std::bind(&MinimalPublisher::callback_image, this, std::placeholders::_1), 
            "raw", 
            custom_qos);

        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
        params_ = cv::aruco::DetectorParameters::create();
        params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;

        // subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        //     "/color/video/camera_info", 
        //     2, 
        //     std::bind(&MinimalPublisher::callback_camera_info, this, std::placeholders::_1));

        // RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubRightGray.getTopic());
        RCLCPP_INFO(get_logger(), "********************************");
        // RCLCPP_INFO_STREAM(get_logger(), " * [DYN] Publish framerate [Hz]:  " << mPubFrameRate);
        RCLCPP_DEBUG(get_logger(), "Stopping temperatures timer");
        
        // https://www.theconstructsim.com/how-to-integrate-opencv-with-a-ros2-c-node/        
        arm_.sendRecvThread->start();

        arm_.backToStart();
        arm_.setFsm(ArmFSMState::PASSIVE);
        arm_.setFsm(ArmFSMState::LOWCMD);

        std::vector<double> KP, KW;
        KP = arm_._ctrlComp->lowcmd->kp;
        KW = arm_._ctrlComp->lowcmd->kd;

        for(int i(0); i<6; i++){
            KP.at(i) = 0.0;
            // KW.at(i) = 0.0;
        }
        std::cout<< KP.at(0) << std::endl;

        arm_._ctrlComp->lowcmd->setControlGain(KP, KW);
        arm_.sendRecvThread->shutdown();
        arm_.runThread->start();// using runThread instead of sendRecvThread

        // while(1){
        //     // visual servoing
        //     // arm_.q_des << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
        //     arm_.qd_des << 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
        //     // arm_.qdd_des << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        //     // std::cout<< arm_.lowstate->getQ() << std::endl;
        //     usleep(arm_._ctrlComp->dt*1000000);
        // }
        double v;
        for(int i(0); i<1000; i++){
            v = i < 1000 ? 0.5 : -0.5;
            arm_.qd_des << v, 1, -1, -1, 0.5, v;
            usleep(arm_._ctrlComp->dt*1000000);
        }
        arm_.qd_des << 0, 0.0, 0.0, 0.0, 0.0, 0;
        usleep(arm_._ctrlComp->dt*1000000);
    }

    ~MinimalPublisher()
    {
        arm_.runThread->shutdown();
        arm_.sendRecvThread->start();

        arm_.setFsm(ArmFSMState::JOINTCTRL);
        arm_.backToStart();
        arm_.setFsm(ArmFSMState::PASSIVE);
        arm_.sendRecvThread->shutdown();    
    }

    private:
    void normalize_pixel(std::vector<cv::Point2f> & corners, Eigen::Matrix<double, 8, 1> & s)
    {
        for (int i=0;i<corners.size();i++)
        {
            Eigen::Matrix<double, 2, 1> s_tmp;
            s_tmp << (corners[i].x - cam_.cx()) / cam_.fx(),
                     (corners[i].y - cam_.cy()) / cam_.fy();
            s.block<2, 1>(2*i, 0) = s_tmp;

            // std::cout << corners[i].x << std::endl;
            // std::cout << corners[i].y << std::endl;
        }
    }

    void callback_send_recv()
    {
        // 在这里获取s/v，发送qd
        // 外参数
        HomoMat Tc;
        Mat6 Jc;
        Tc << 1,0,0,0,
            0,1,0,0,
            0,0,1,0.1,
            0,0,0,1;
        Jc = TransMotionVector(Tc);
        RCLCPP_INFO_STREAM(this->get_logger(), "HomoMat to Jc:  " << Jc);

        // 雅克比,机械臂本体系吗,仿佛是末端系
        Mat6 J = arm_._ctrlComp->armModel->CalcJacobian(arm_.lowstate->getQ());
                
        // RCLCPP_INFO_STREAM(this->get_logger(), "[Visual Servo] Cartesian velocity:  " << vel_.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "[Visual Servo] Joint velocity:  " << (J.inverse() * vel_).transpose());
        // arm_.qd_des = pinv(J) * vel_;
    }

    // void callback_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    void callback_image(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
    {
        if (!got_cam_info_){
            cam_.fromCameraInfo(*info);

            full_resolution_ = cam_.fullResolution();
            width_ = full_resolution_.width;
            height_ = full_resolution_.height;
            K_ << cam_.fx(), 0.f, cam_.cx(),
                0.f, cam_.fy(), cam_.cy(),
                0.f, 0.f, 1.f;
            // cv::Point2d rectified_point = cam_.rectifyPoint(cv::Point2d(x,y));

            std::cout << "height: " << height_ << ", "
                    << "width: " << width_ << ", "
                    << "k1: " << info->d[0] << ", "
                    << "k2: " << info->d[1] << ", "
                    << "p1: " << info->d[2] << ", " 
                    << "p2: " << info->d[3] << ", "
                    << "k3: " << info->d[4] << std::endl
                    << "K: " << K_ << std::endl;

            got_cam_info_ = true;

            double size = 100.;
            std::vector<cv::Point2f> corner_des;
            cv::Point2f c1 = cv::Point2f(width_/2-size,height_/2-size);corner_des.push_back(c1);
            cv::Point2f c2 = cv::Point2f(width_/2+size,height_/2-size);corner_des.push_back(c2);
            cv::Point2f c3 = cv::Point2f(width_/2+size,height_/2+size);corner_des.push_back(c3);
            cv::Point2f c4 = cv::Point2f(width_/2-size,height_/2+size);corner_des.push_back(c4);
            
            normalize_pixel(corner_des,s_des_);
        }

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cv_ptr->image;

        // 原文链接：https://blog.csdn.net/zhou4411781/article/details/103262675
        // https://blog.csdn.net/weixin_41010198/article/details/114843647
        // https://blog.csdn.net/qq_27865227/article/details/123200246
        // https://github.com/ros-perception/image_common/wiki/ROS2-Migration

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners, rejected;
        cv::aruco::detectMarkers(img, dictionary_, corners, ids, params_);

        if (ids.size() > 0) 
        {
            cv::aruco::drawDetectedMarkers(img, corners, ids);
            // 只存在一个二维码
            for (int ci=0;ci<corners.size();ci++) 
            {
                if (ids[ci] == 33 && corners[ci].size() == 4)
                {
                    Eigen::Matrix<double, 8, 1> s, s_error;
                    normalize_pixel(corners[ci],s);

                    // Eigen::Matrix<double, 8, 6> L;
                    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(8, 6);

                    // L = Eigen::MatrixXf::Zero(corners_norm.size()*2, 6);
                    // s = Eigen::MatrixXf::Zero(corners_norm.size()*2, 1);
                    
                    for (int pi=0;pi<corners[ci].size();pi++)
                    {
                        double x = s[2*pi];
                        double y = s[2*pi+1];
                        double zi = 1;
                        Eigen::MatrixXd L_tmp = Eigen::MatrixXd::Zero(2, 6);
                        L_tmp << -zi, 0, x * zi, x * y, -(1 + x * x), y,
                                0, -zi, y * zi, 1 + y * y, -x * y, -x;

                        // std::cout << L_tmp << std::endl;
                        L.block<2, 6>(pi*2, 0) = L_tmp;
                    }
                    s_error = s - s_des_;
                    vel_ = -0.1 * pinv(L) * s_error;
                    // std::cout << L << std::endl;
                    // std::cout << s_des_ << std::endl;
                    // std::cout << s << std::endl;
                    // std::cout << "===========" << std::endl;
                }
            }
        }
        else
        {
            // 有S就有vel,没有就取0
            vel_.setZero(); // 固定大小
        }

        cv::imshow(OPENCV_WINDOW, img);
        cv::waitKey(3);

        // std::string filename;
        // if (!saveImage(image_msg, filename))
        // return;        
    }

    // https://blog.csdn.net/tfb760/article/details/115935289
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
    // image_transport::Subscriber sub_image_;
    // rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_info_;
    image_transport::CameraSubscriber sub_cam_;
    // camera_info_manager::CameraInfoManager cam_info_manager_;
    image_geometry::PinholeCameraModel cam_;
    cv::Size full_resolution_;
    int width_ ;
    int height_;
    Eigen::Matrix<double, 3, 3> K_;
    bool got_cam_info_ = false;
    // camera_calibration_parsers 

    const std::string OPENCV_WINDOW = "Image window";
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;

    Z1ARM arm_;
    Eigen::Matrix<double, 6, 1> vel_;
    Eigen::Matrix<double, 8, 1> s_des_;
    
    size_t count_; 
};


int main(int argc, char *argv[]) {
    std::cout << std::fixed << std::setprecision(3);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}