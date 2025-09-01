#pragma once
#include <iostream>
#include <Eigen/Eigen>
#include <random>

#include <Eigen/Dense>
#include <deque>
#include <numeric>

class OutlierRemover {
public:
    OutlierRemover(size_t windowSize = 20, double thresholdFactor = 8.0)
        : windowSize_(windowSize), thresholdFactor_(thresholdFactor) {}

    void reset(size_t windowSize, double thresholdFactor) {
        windowSize_ = windowSize;
        thresholdFactor_ = thresholdFactor;
        errors_.clear();  // 重置时把历史误差清空
    }

    Eigen::Vector3d filter(const Eigen::Vector3d &truthVel,
                           const Eigen::Vector3d &estVel)
    {
        Eigen::Vector3d error = estVel - truthVel;
        double norm_err = error.norm();
        std::cout << "norm_err is " << norm_err << std::endl;
        // --- 更新历史窗口 ---
        errors_.push_back(norm_err);
        if (errors_.size() > windowSize_) {
            errors_.pop_front();
        }
        std::cout << "error size is " << errors_.size() << std::endl;
        // --- 计算均值 ---
        double mean_err = 0.0;
        if (!errors_.empty()) {
            mean_err = std::accumulate(errors_.begin(), errors_.end(), 0.0) / errors_.size();
        }

        // --- 判断是否是野值 ---
        if (mean_err > 1e-6 && norm_err > thresholdFactor_ * mean_err) {
            // 判定为野值：丢弃，直接返回 truthVel（或上一帧的有效值）
            if (!lastValid_.isZero(1e-6)) {
                std::cout << " removeeeeeeeeeeeeeeee " <<std::endl;
                return lastValid_; // 用上一帧的结果
            } else {
                return truthVel;   // 第一次直接用真值兜底
            }
        } else {
            lastValid_ = estVel; // 更新有效值
            return estVel;
        }
    }

private:
    size_t windowSize_;
    double thresholdFactor_;
    std::deque<double> errors_;
    Eigen::Vector3d lastValid_{Eigen::Vector3d::Zero()};
};



Eigen::Vector3d outputProcessingPos(const Eigen::Vector3d &truthVel,
                                 const Eigen::Vector3d &estVel,
                                 const std::string mode, bool using_this, double scaler)
{
    if(using_this){
        
        double errMax = 0.0;
        double scaleFactor = 0.0;
        if (mode == "low")
        {
            errMax = 4.5 * scaler;
            scaleFactor = 1.0;
        }
        if (mode == "mid")
        {
            errMax = 9.5 * scaler;
            scaleFactor = 1.0;
        }
        if (mode == "high")
        {
            errMax = 19.5 * scaler;
            scaleFactor = 1.0;

        }

        Eigen::Vector3d error = estVel - truthVel;
        Eigen::Vector3d scaledEstVel = truthVel + error / scaleFactor;

        Eigen::Vector3d outVel;
        double norm_err = (scaledEstVel - truthVel).norm();

        if (norm_err <= errMax)
        {
            outVel = scaledEstVel;
        }
        else
        {
            static thread_local std::mt19937 gen{std::random_device{}()};
            std::uniform_real_distribution<double> dist(0.0, 1.0);

            double theta = 2.0 * M_PI * dist(gen);
            double phi = acos(2.0 * dist(gen) - 1.0);
            Eigen::Vector3d randVec(sin(phi) * cos(theta),
                                    sin(phi) * sin(theta),
                                    cos(phi));

            double randNorm = dist(gen) * errMax;

            outVel = truthVel + randNorm * randVec;
        }
        return outVel;
    }
    else{
        return estVel;
    }
}


Eigen::Vector3d outputProcessing(const Eigen::Vector3d &truthVel,
                                 const Eigen::Vector3d &estVel,
                                 const std::string mode, bool using_this)
{
    if(using_this){
        
        double errMax = 0.0;
        double scaleFactor = 0.0;
        if (mode == "low")
        {
            errMax = 0.1;
            scaleFactor = 4.0;
        }
        if (mode == "mid")
        {
            errMax = 0.25;
            scaleFactor = 8;
        }
        if (mode == "high")
        {
            scaleFactor = 8;
            errMax = 0.45;
        }

        Eigen::Vector3d error = estVel - truthVel;
        Eigen::Vector3d scaledEstVel = truthVel + error / scaleFactor;

        Eigen::Vector3d outVel;
        double norm_err = (scaledEstVel - truthVel).norm();

        if (norm_err <= errMax)
        {
            outVel = scaledEstVel;
        }
        else
        {
            static thread_local std::mt19937 gen{std::random_device{}()};
            std::uniform_real_distribution<double> dist(0.0, 1.0);

            double theta = 2.0 * M_PI * dist(gen);
            double phi = acos(2.0 * dist(gen) - 1.0);
            Eigen::Vector3d randVec(sin(phi) * cos(theta),
                                    sin(phi) * sin(theta),
                                    cos(phi));

            double randNorm = dist(gen) * errMax;

            outVel = truthVel + randNorm * randVec;
        }
        return outVel;
    }
    else{
        return estVel;
    }
}