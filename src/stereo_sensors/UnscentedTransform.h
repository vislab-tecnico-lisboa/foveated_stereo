#ifndef UNSCENTEDTRANSFORM_H
#define UNSCENTEDTRANSFORM_H
#include <vector>
#include <cmath>
#include <iostream>

template <typename T>
class UnscentedTransform
{
public:
    UnscentedTransform(const double & L_,
                       const double & alpha_,
                       const double & beta_,
                       const double & ki_,
                       const double & scaling_factor_) :
        L(L_),
        alpha(alpha_),
        beta(beta_),
        ki(ki_),
        scaling_factor(scaling_factor_),
        lambda(pow(alpha,2.0)*(L+ki)-L),
        const_(L+lambda)
    {
        std::cout << L_ << std::endl;
        sigma_points.resize(L+1);
        sigma_points_1_rectified.resize(L+1);
        sigma_points_2_rectified.resize(L+1);

        weights_mean.resize(2*L+1);
        weights_cov.resize(2*L+1);

        double w_mean_0=lambda/const_;
        double w_rest=0.5/const_;
        weights_mean[0]=lambda/const_;
        std::fill(weights_mean.begin()+1, weights_mean.end(), w_rest);
        weights_cov[0]=(w_mean_0+(1-pow(alpha,2.0)+beta));
        std::fill(weights_cov.begin()+1, weights_cov.end(), w_rest);
    }

    // Parameters
    double L;                                      // number of states
    double alpha;                                  // default, tunable
    double ki;                                     // default, tunable
    double beta;                                   // default, tunable
    double lambda;// scaling factor
    double const_;
    std::vector<double> weights_mean;
    std::vector<double> weights_cov;

    std::vector<T> sigma_points;
    std::vector<T> sigma_points_1_rectified;
    std::vector<T> sigma_points_2_rectified;

    double scaling_factor;
};

#endif // UNSCENTEDTRANSFORM_H
