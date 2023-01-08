/*
 * Copyright (C) HoloMatic Technology(Beijing) Co., Ltd. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */
/**
 * @file histogram.cpp
 * @brief implement of histogram
 * @author Shuaijie Li @ lishuaijie@holomatic.com
 * @date 2022-12-16
 */

#include "holo_histogram.hpp"
#include <iostream>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HoloHistogram::Parameters::Parameters(std::string const& _name, Scalar const _resolution,
                                  std::pair<Scalar, Scalar> const _histogram_boundary,
                                  Scalar const _min_number_of_samples_for_kurtosis, bool_t const _verbose)
  : name(_name)
  , resolution(_resolution)
  , histogram_boundary(_histogram_boundary)
  , min_number_of_samples_for_kurtosis(_min_number_of_samples_for_kurtosis)
  , verbose(_verbose)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HoloHistogram::Parameters HoloHistogram::Parameters::GenerageExample()
{
    return Parameters("test", 0.1, {-1, 1}, 50, true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, HoloHistogram::Parameters const& parameters)
{
    os << "HoloHistogram::Parameters --- "
       << "\n name = " << parameters.name << "\n resolution = " << parameters.resolution << "\n histogram_boundary = ["
       << parameters.histogram_boundary.first << ", " << parameters.histogram_boundary.second << "]"
       << "\n min_number_of_samples_for_kurtosis = " << parameters.min_number_of_samples_for_kurtosis
       << "\n verbose = " << parameters.verbose << std::endl;
    return os;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HoloHistogram::IntervalInfo::IntervalInfo(Scalar const left_boundary, Scalar const right_boundary)
  : mean(0.0), num_of_samples(0.0), interval_boundary({left_boundary, right_boundary})
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, HoloHistogram::IntervalInfo const& interval_info)
{
    os << "IntervalInfo --- "
       << "\n mean = " << interval_info.mean << "\n num_of_samples = " << interval_info.num_of_samples
       << "\n interval_boundary = [" << interval_info.interval_boundary.first << ", "
       << interval_info.interval_boundary.second << "]" << std::endl;
    return os;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HoloHistogram::HoloHistogram(Parameters const& parameters) : parameters_(parameters)
{
    initialize(parameters_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HoloHistogram::~HoloHistogram()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HoloHistogram::Run(Scalar const sample)
{
    Scalar const left_boundary  = parameters_.histogram_boundary.first;
    Scalar const right_boundary = parameters_.histogram_boundary.second;
    Scalar const resolution     = parameters_.resolution;

    ///@brief step1: check whether the sample is in the boundary of histogram
    if (sample < left_boundary || sample > right_boundary)
    {
        return;
    }

    ///@brief step2: update histogram
    uint64_t const index                 = (sample - left_boundary) / resolution;
    IntervalInfo&  current_interval_info = multi_interval_info_.at(index);
    /**
     * @details mean_k = (mean_k_1 * num_of_samples_k_1 + sample) / (num_of_samples_k_1 + 1)
     *                 = mean_k_1 + (sample - mean_k_1) / (num_of_samples_k_1 + 1)
     *          num_of_samples_k = num_of_samples_k_1 + 1
     */
    current_interval_info.mean += (sample - current_interval_info.mean) / (current_interval_info.num_of_samples + 1.0);
    current_interval_info.num_of_samples += 1.0;

    ///@brief check and update the information of interval whose samples are max
    if (current_interval_info.num_of_samples > interval_index_with_max_samples_.second)
    {
        interval_index_with_max_samples_.first  = index;
        interval_index_with_max_samples_.second = current_interval_info.num_of_samples;
    }

    num_of_all_samples_++;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HoloHistogram::Reset()
{
    initialize(parameters_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
HoloHistogram::Parameters const& HoloHistogram::GetParameters() const
{
    return parameters_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HoloHistogram::SetCenterAndRange(Scalar const center, Scalar const range)
{
    parameters_.histogram_boundary = std::make_pair(center - range, center + range);
    Reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::tuple<Scalar, Scalar, Scalar> HoloHistogram::GetCenterAndHistogramBoundary() const
{
    Scalar const left_boundary  = parameters_.histogram_boundary.first;
    Scalar const right_boundary = parameters_.histogram_boundary.second;
    Scalar const center         = (left_boundary + right_boundary) / 2.0;
    return std::make_tuple(center, left_boundary, right_boundary);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HoloHistogram::SetResolution(Scalar const resolution)
{
    parameters_.resolution = resolution;
    Reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Scalar HoloHistogram::GetResolution() const
{
    return parameters_.resolution;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Scalar HoloHistogram::GetNumOfSamples() const
{
    return num_of_all_samples_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Scalar HoloHistogram::GetNumOfIntervals() const
{
    return multi_interval_info_.size();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::pair<Scalar, HoloHistogram::IntervalInfo> HoloHistogram::GetIntervalInfoWithMaxSamples() const
{
    // return the first interval with zero probability if there is not sample
    if (interval_index_with_max_samples_.first < 0)
    {
        return std::make_pair(0.0, multi_interval_info_.at(0u));
    }

    Scalar const probability = interval_index_with_max_samples_.second / num_of_all_samples_;
    return std::make_pair(probability, multi_interval_info_.at(interval_index_with_max_samples_.first));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
std::pair<Scalar, HoloHistogram::IntervalInfo> HoloHistogram::GetIntervalInfo(uint64_t const index) const
{
    if (index >= GetNumOfIntervals())
    {
        return std::make_pair(-1, IntervalInfo(0.0, 0.0));
    }

    Scalar const probability = multi_interval_info_.at(index).num_of_samples / num_of_all_samples_;
    return std::make_pair(probability, multi_interval_info_.at(index));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool_t HoloHistogram::IsApproxGaussian() const
{
    return computeKurtosis() >= 0.0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void HoloHistogram::initialize(Parameters const parameters)
{
    Scalar const left_boundary  = parameters.histogram_boundary.first;
    Scalar const right_boundary = parameters.histogram_boundary.second;
    Scalar const resolution     = parameters.resolution;

    if (left_boundary >= right_boundary || resolution <= 0.0)
    {
        std::stringstream ss;
        ss << "Histogram --- failed to initialize histogram for " << parameters.name
           << ", the histogram parameters is invalid! the parameters is " << parameters;
        std::cerr << ss.str();
        throw std::runtime_error(ss.str());
    }

    uint64_t const num_of_intervals = (right_boundary - left_boundary) / resolution + 1u;
    multi_interval_info_.clear();
    multi_interval_info_.reserve(num_of_intervals);

    for (uint64_t index = 0u; index < num_of_intervals - 1u; index++)
    {
        multi_interval_info_.emplace_back(left_boundary + resolution * index,
                                          left_boundary + resolution * (index + 1u));
    }

    multi_interval_info_.emplace_back(left_boundary + resolution * (num_of_intervals - 1u), right_boundary);
    num_of_all_samples_              = 0.0;
    interval_index_with_max_samples_ = std::make_pair(-1, 0.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Scalar HoloHistogram::computeKurtosis() const
{
    ///@brief step1: do not compute kurtosis if the number of samples is too small
    if (num_of_all_samples_ < parameters_.min_number_of_samples_for_kurtosis)
    {
        return -100.0;
    }

    ///@brief step2: compute mu and sigma
    std::vector<Scalar> multi_interval_probability;
    multi_interval_probability.reserve(multi_interval_info_.size());
    Scalar mu                  = 0.0;
    Scalar moment2             = 0.0;

    for (uint64_t index = 0u; index < multi_interval_info_.size(); index++)
    {
        IntervalInfo const& current_interval_info = multi_interval_info_.at(index);
        Scalar const        probability           = current_interval_info.num_of_samples / num_of_all_samples_;
        multi_interval_probability.push_back(probability);
        mu += current_interval_info.mean * probability;
        moment2 += current_interval_info.mean * current_interval_info.mean * probability;
    }

    Scalar const sigma2   = moment2 - mu * mu;
    Scalar const sigma4   = sigma2 * sigma2;
    Scalar kurtosis = 0.0;

    ///@brief step3: compute kurtosis
    for (uint64_t index = 0u; index < multi_interval_info_.size(); index++)
    {
        kurtosis +=
            multi_interval_probability.at(index) * std::pow(multi_interval_info_.at(index).mean - mu, 4) / sigma4;
    }

    return kurtosis - 3.0;  ///< make normal distribution to zero
}

