#ifndef RANSAC_TYPES_H
#define RANSAC_TYPES_H

#include "ransac.h"

class RansacPlane3D : public Ransac {
public:
    RansacPlane3D(const std::vector<Eigen::VectorXd> &data, double distThreshold, double probGoodSample, int maxIter);

protected:
    /**
     * 计算平面参数
     * @param data 输入3个数据点
     * @param model 平面方程。形式：ax+by+cz+d=0
     */
    void fit(const std::vector<Eigen::VectorXd> &data, Eigen::VectorXd &model) override;

    std::vector<Eigen::VectorXd> generateData() override;

    double calDist(const Eigen::VectorXd &data, const Eigen::VectorXd &model) override;

};


#endif // RANSAC_TYPES_H