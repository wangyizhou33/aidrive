/*
 * Copyright (C) HoloMatic Technology(Beijing) Co., Ltd. - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Proprietary and confidential
 */

/**
 * @file histogram.h
 * @brief header file for histogram tool
 * @author Shuaijie Li @ lishuaijie@holomatic.com
 * @date 2022-12-16
 */

#ifndef HOLO_LOCALIZATION_TOOLS_HISTOGRAM_H_
#define HOLO_LOCALIZATION_TOOLS_HISTOGRAM_H_

#include <aidrive/Types.hpp>

/**
 * @addtogroup tools
 * @{
 *
 */

/**
 * @brief histogram class
 *
 * @details this class mainly aims to do histogram statistics for a scalar variable
 */
using Scalar = float32_t;
using bool_t = bool;

class HoloHistogram
{
public:
    /**
     * @brief parameters for the histogram
     */
    struct Parameters
    {
        std::string name;                             ///< name of the variable in this histogram
        Scalar resolution;                            ///< resolution of each interval of this histogram
        std::pair<Scalar, Scalar> histogram_boundary; ///< left and right boundary of this histogram
        Scalar min_number_of_samples_for_kurtosis;    ///< the minimum number of samples for kurtosis computation
        bool_t verbose;                               ///< whether output additional log

        /**
         * @brief constructor of parameters
         *
         * @param[in] _name name of the variable in this histogram
         * @param[in] _resolution resolution of each interval of this histogram
         * @param[in] _histogram_boundary left and right boundary of this histogram
         * @param[in] _min_number_of_samples_for_kurtosis the minimum number of samples for kurtosis computation
         * @param[in] _verbose whether output additional log
         */
        Parameters(std::string const& _name, Scalar const _resolution,
                   std::pair<Scalar, Scalar> const _histogram_boundary,
                   Scalar const _min_number_of_samples_for_kurtosis, bool_t const _verbose = false);

        /**
         * @brief function to generate parameters for unit test
         *
         * @return parameters
         */
        static Parameters GenerageExample();

        /**
         * @brief output parameters to ostream
         *
         * @param[in] os ostream
         * @param[in] parameters parameters to output
         *
         * @return ostream contains parameters
         */
        friend std::ostream& operator<<(std::ostream& os, Parameters const& parameters);
    }; // Parameters

    /**
     * @brief interval information for the histogram
     */
    struct IntervalInfo
    {
        Scalar mean;                                 ///< mean values of all the
        Scalar num_of_samples;                       ///< number of samples in this interval
        std::pair<Scalar, Scalar> interval_boundary; ///< left and right boundary of this interval

        /**
         * @brief constructor, which will initialize ${mean} and ${num_of_samples} to zero
         *
         * @param[in] left_boundary left boundary of this interval
         * @param[in] right_boundary right boundary of this interval
         */
        IntervalInfo(Scalar const left_boundary, Scalar const right_boundary);

        /**
         * @brief output interval info to ostream
         *
         * @param[in] os ostream
         * @param[in] interval_info interval info to output
         *
         * @return   ostream contains interval info
         */
        friend std::ostream& operator<<(std::ostream& os, IntervalInfo const& interval_info);

    }; // IntervalInfo

    /**
     * @brief constructor
     *
     * @param[in] parameters parameters for this histogram
     */
    HoloHistogram(Parameters const& parameters);

    /**
     * @brief destructor
     */
    ~HoloHistogram();

    /**
     * @brief function to add sample into the histogram
     *
     * @param[in] sample new sample value
     */
    void Run(Scalar const sample);

    /**
     * @brief clear all the sample info and ready for new statistics
     */
    void Reset();

    /**
     * @brief get parameters of this histogram
     *
     * @return parameters for this histogram
     */
    Parameters const& GetParameters() const;

    /**
     * @brief change boundary of histogram by set center and range, the new boundary is [center-range, center + range]
     * @note it will change ${parameters_.histogram_boundary} and clear samples
     *
     * @param[in] center center of the histogram
     * @param[in] range range for left and right boundary to the center
     */
    void SetCenterAndRange(Scalar const center, Scalar const range);

    /**
     * @brief get center value and histogram boundary
     *
     * @return tuple<center, left_boundary, right_boundary>
     */
    std::tuple<Scalar, Scalar, Scalar> GetCenterAndHistogramBoundary() const;

    /**
     * @brief set resolution of the histogram
     * @note it will change ${parameters_.resolution} and clear samples
     *
     * @param[in] resolution resolution for the new histogram
     */
    void SetResolution(Scalar const resolution);

    /**
     * @brief get resolution of this histogram
     *
     * @return resolution
     */
    Scalar GetResolution() const;

    /**
     * @brief get number of the samples in this histogram
     *
     * @return number of samples
     */
    Scalar GetNumOfSamples() const;

    /**
     * @brief get number of interval info in this histogram
     *
     * @return number of interval info
     */
    Scalar GetNumOfIntervals() const;

    /**
     * @brief get interval info with max number of samples
     * @note if there is not any sample, return the first interval info with zero probability
     *
     * @return pair<probability, interval info>
     */
    std::pair<Scalar, IntervalInfo> GetIntervalInfoWithMaxSamples() const;

    /**
     * @brief get interval info of the given index in ${multi_interval_info_}
     * @note if the index is invalid, return [0, 0] interval info with -1 probability
     *
     * @param[in] index index of interval info in ${multi_interval_info_}
     *
     * @return pair<probability, interval info>
     */
    std::pair<Scalar, IntervalInfo> GetIntervalInfo(uint64_t const index) const;

    /**
     * @brief check whether the histogram is approx to gaussian
     *
     * @return true -> approx gaussian, otherwise false
     */
    bool_t IsApproxGaussian() const;

protected:
    /**
     * @brief initialize internal variables
     *
     * @param[in] parameters histogram parameters
     */
    void initialize(Parameters const parameters);

public:
    /**
     * @brief compute Kurtosis of the histogram with ${multi_interval_info_} and ${num_of_all_samples_}
     * @details kurtosis == 0.0 -> standard normal distribution
     *          kurtosis < 0.0 -> platykurtic
     *          kurtosis > 0.0 -> leptokurtic
     *
     * @return kurtosis of the samples
     */
    Scalar computeKurtosis() const;

protected:
    Parameters parameters_;                                      ///< parameters for histogram, which might be changed by SetXXX function
    std::vector<IntervalInfo> multi_interval_info_;              ///< all the interval infos for this histogram
    Scalar num_of_all_samples_;                                  ///< number of all the samples in this histogram
    std::pair<int64_t, Scalar> interval_index_with_max_samples_; ///< index of ${multi_interval_info_} whose samples is
                                                                 ///< maximum

}; // Histogram

#endif