#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0],
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1,-eye_pos[2],
                 0, 0, 0, 1;

    view = translate * view;

    return view;
}
Eigen::Matrix4f get_rotation(Eigen::Vector3f n,float angle){
    angle=angle/180.0*MY_PI;
    Eigen::Vector4f column_n;
    Eigen::RowVector4f row_n;
    
    //n=n/(sqrt(n.x()*n.x()+n.y()*n.y()+n.z()*n.z()));
    column_n<<n.x(),n.y(),n.z(),0;
    row_n<<n.x(),n.y(),n.z(),0;
    
    Eigen::Matrix4f N=Eigen::Matrix4f::Identity();
    Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f R= Eigen::Matrix4f::Identity();
    
    I<<std::cos(angle),0,0,0,
       0,std::cos(angle),0,0,
       0,0,std::cos(angle),0,
       0,0,0,1;
    
    N=(1-std::cos(angle))*column_n*row_n;
    
    R<<0,-column_n.z(),column_n.y(),0,
       column_n.z(),0,-column_n.x(),0,
       -column_n.y(),column_n.x(),0,0,
       0,0,0,0;

    R=I+N+std::sin(angle)*R;

    return R;
}
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.这里暂时只体现了model变换的旋转，平移和变形没有体现
    rotation_angle = rotation_angle / 180 * MY_PI;
    Eigen::Matrix4f rotation;
    rotation<<std::cos(rotation_angle),-std::sin(rotation_angle),0,0,
              std::sin(rotation_angle),std::cos(rotation_angle),0,0,
              0,0,1,0,
              0,0,0,1;
    model =rotation*model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f persp;
    Eigen::Matrix4f translate;
    Eigen::Matrix4f scale;
    eye_fov=(eye_fov/180.0)*MY_PI;
    float n=abs(zNear);
    float f=abs(zFar);
    float t=n*tanf(eye_fov/2);
    float r=aspect_ratio*t;
    persp<<zNear,0,0,0,
           0,zNear,0,0,
           0,0,zFar+zNear,-zFar*zNear,
           0,0,1,0;
    translate<<1,0,0,0,
               0,1,0,0,
               0,0,1,-(zFar+zNear)/2,
               0,0,0,1;
    scale<<1/r,0,0,0,
           0,1/t,0,0,
           0,0,2/(f-n),0,
           0,0,0,1;
    projection=scale*translate*persp;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    return projection;
}

int main(int argc, const char** argv)
{   

    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }
    
    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 2, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1,-0.1,-50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }
    Eigen::Vector3f raxis;
    bool rflag = false;
    float ra;
    double rangle = 0;

    std::cout << "Please enter the axis and angle:" << std::endl;
    std::cin >> raxis.x() >> raxis.y() >> raxis.z() >> ra;

    while (key != 27) {
        
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, -0.1, -50));
        
        if (rflag) //如果按下r了，就开始绕给定任意轴旋转
            r.set_rodrigues(get_rotation(raxis, rangle));
        else
            r.set_rodrigues(get_rotation({ 0,0,1 }, 0));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'r') {//按下r，绕给定任意轴旋转
            rflag = true;
            rangle += ra;
        }
    }
    std::cout<<2442;
    return 0;
}
