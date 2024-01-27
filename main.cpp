#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}


Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // set the model matrix to identity matrix
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // change the angle to radian
    float radian = rotation_angle*MY_PI/180;
    /* In order to rotate around Z axis, the matrix needs to be changed to: 
    cosθ -sinθ 0
    sinθ  cosθ 0
     0     0   1
    */ 
    model(0,0) = cos(radian);
    model(0,1) = -sin(radian);
    model(1,0) = sin(radian);
    model(1,1) = cos(radian);
    // return the model matrix
    return model;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float rotation_angle)
{
    // set model to be identity matrix
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    // change angle to radian
    float radian = rotation_angle*MY_PI/180;
    // normalize the vector
    axis.normalize();
    // calculate the q0, q1, q2 and q3 of Quaternion
    float q0 = cos(radian/2);
    float q1 = axis[0]*sin(radian/2);
    float q2 = axis[1]*sin(radian/2);
    float q3 = axis[2]*sin(radian/2);
    // set the matrix value
    model(0,0) = 1-2*pow(q2,2)-2*pow(q3,2);
    model(0,1) = 2*q1*q2+2*q0*q3;
    model(0,2) = 2*q1*q3-2*q0*q2;
    model(1,0) = 2*q1*q2-2*q0*q3;
    model(1,1) = 1-2*pow(q1,2)-2*pow(q3,2);
    model(1,2) = 2*q2*q3+2*q0*q1; 
    model(2,0) = 2*q1*q3+2*q0*q2;
    model(2,1) = 2*q2*q3-2*q0*q1;
    model(2,2) = 1-2*pow(q1,2)-2*pow(q2,2);
    //return the model matrix
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // set the projection matrix to identity matrix
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    // change the degree to radian
    float radian = eye_fov * MY_PI/180;
    // calculate the top and right value
    float top = zNear*tan(radian/2);
    float right = top*aspect_ratio;
    // set the projection matrix
    projection(0,0) = zNear/right;
    projection(1,1) = zNear/top;
    projection(2,2) = -(zFar+zNear)/(zFar-zNear);
    projection(3,2) = -1;
    projection(2,3) = (-2*zFar*zNear)/(zFar-zNear);
    // return the projection matrix
    return projection;
}

int main(int argc, const char **argv)
{
    float angle = 0;
    // bonus
    // float angle = 60;
    // Vector3f axis = {1,1,0};

    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_model_matrix(angle));
        // bonus
        // r.set_model(get_rotation(axis,angle));

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        
        r.set_model(get_model_matrix(angle));
        // bonus
        // r.set_model(get_rotation(axis,angle));
        
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';
        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
