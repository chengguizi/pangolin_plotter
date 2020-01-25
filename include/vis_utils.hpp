#pragma once

#include <Eigen/Eigen>
#include <pangolin/gl/gldraw.h>
#include <pangolin/gl/colour.h>

#include <eigen_utils.hpp>

namespace visualisation{

    const u_int8_t cam_color[3]{250, 0, 26};
    const u_int8_t state_color[3]{250, 0, 26};
    const u_int8_t pose_color[3]{0, 50, 255};
    const u_int8_t gt_color[3]{0, 171, 47};

    inline void render_camera(const Eigen::Matrix4d& T_w_c, float lineWidth, pangolin::Colour color, float sizeFactor) 
    {
        const float sz = sizeFactor;
        const float width = 640, height = 480, fx = 500, fy = 500, cx = 320, cy = 240;

        const Eigen::aligned_vector<Eigen::Vector3f> lines = {
            {0, 0, 0},
            {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
            {0, 0, 0},
            {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
            {0, 0, 0},
            {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
            {0, 0, 0},
            {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
            {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz},
            {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
            {sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
            {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
            {sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz},
            {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
            {sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz},
            {sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz}};

        glPushMatrix();
        glMultMatrixd(T_w_c.data());
        glColor4fv(color.Get());
        glLineWidth(lineWidth);
        pangolin::glDrawLines(lines);
        glPopMatrix();
    }
}

