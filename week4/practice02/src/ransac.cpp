#include "ransac.h"

Ransac::Ransac(const std::vector<Eigen::VectorXd> &data, double distThreshold, int minSizeSamplesToFit, double probGoodSample, int maxIter) : data(
        data), distThreshold(distThreshold), minSizeSamplesToFit(minSizeSamplesToFit), probGoodSample(probGoodSample), maxIter(maxIter) {
    // 检查参数设置有效性
    assert(distThreshold > 0 && minSizeSamplesToFit > 0 && probGoodSample > 0 && probGoodSample < 1 && maxIter > 0 && "Input parameters invalid\n");

    // 检查数据有效性
    assert(data.size() > 0 && "Data not valid\n");
    numDimensions = data[0].size();
    numPoints = data.size();
    for (auto &item:data) {
        if (item.size() != numDimensions) {
            std::cerr << "Data dimension not match\n";
            return;
        }
    }
}

// 部分参考：https://github.com/MRPT/mrpt/blob/develop/libs/math/include/mrpt/math/ransac_impl.h
bool Ransac::execute(std::vector<int> &inliersIdx, Eigen::VectorXd &model) {
    Eigen::VectorXd currBestModel = Eigen::VectorXd::Zero(minSizeSamplesToFit, 1);
    std::vector<int> currBestInliersIdx;

    int iter = maxIter;
    int count = 0;
    while (count++ < iter) {
        // 随机产生数据点
        auto randData = generateData();
        if (randData.empty()) return false;

        Eigen::VectorXd tempModel = Eigen::VectorXd::Zero(minSizeSamplesToFit, 1);
        std::vector<int> tempInliersIdx;

        // 拟合模型
        fit(randData, tempModel);

        // 统计内点个数
        calInlierIdx(tempModel, tempInliersIdx);

        // 更新当前最优模型、当前最优内点、迭代次数（为了提前结束）
        bool findBetterModel = tempInliersIdx.size() > currBestInliersIdx.size();
        if (findBetterModel) {
            currBestInliersIdx = tempInliersIdx;
            currBestModel = tempModel;

            // TODO: 加入早停机制
            // iter = std::min(maxIter, calIteration(tempInliersIdx.size() / data.size()));
        }
    }

    // 输出最优的模型
    inliersIdx = currBestInliersIdx;
    model = currBestModel;
    return true;
}

void Ransac::calInlierIdx(const Eigen::VectorXd &model, std::vector<int> &inliersIdx) {
    inliersIdx.clear();
    inliersIdx.resize(0);

    for (int idx = 0; idx < data.size(); ++idx) {
        double dist = calDist(data[idx], model);
        if (dist < distThreshold) inliersIdx.push_back(idx);
    }
}

int Ransac::calIteration(double pInliers) {
    double pHasOutlier = 1 - std::pow(pInliers, static_cast<double>(minSizeSamplesToFit));
    pHasOutlier = std::max(pHasOutlier, std::numeric_limits<double>::epsilon());
    pHasOutlier = std::min(pHasOutlier, 1 - std::numeric_limits<double>::epsilon());

    return log(1 - probGoodSample) / log(pHasOutlier);
}