/* -*-c++-*- SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: header of semi-global matching class
*/

#pragma once

#include "sgm_types.h"
#include <vector>

/**
 * \brief SemiGlobalMatching class General implementation of Semi-Global Matching 
 */
class SemiGlobalMatching
{
public:
	SemiGlobalMatching();
	~SemiGlobalMatching();


	/** \brief Census Window Size Type */
	enum CensusSize {
		Census5x5 = 0,
		Census9x7
	};

	/** \brief SGM parameter structure */
	struct SGMOption {
		uint8	num_paths;			// Aggregate Path Number 4 and 8
		sint32  min_disparity;		// Minimum parallax
		sint32	max_disparity;		// Maximum parallax

		CensusSize census_size;		// Census window size

		bool	is_check_unique;	// Whether to check uniqueness
		float32	uniqueness_ratio;	// Uniqueness constraint threshold (minimum cost - sub-minimum cost)/minimum cost > threshold is a valid pixel

		bool	is_check_lr;		// Whether to check left-right consistency
		float32	lrcheck_thres;		// Left-right consistency constraint threshold

		bool	is_remove_speckles;	// Whether to remove small connected areas
		int		min_speckle_aera;	// Minimum connected area (number of pixels)

		bool	is_fill_holes;		// Whether to fill parallax holes

		// P1,P2 
		// P2 = P2_init / (Ip-Iq)
		sint32  p1;				// Penalty parameter P1
		sint32  p2_init;		// Penalty parameter P2

		SGMOption(): num_paths(8), min_disparity(0), max_disparity(64), census_size(Census5x5),
		             is_check_unique(true), uniqueness_ratio(0.95f),
		             is_check_lr(true), lrcheck_thres(1.0f),
		             is_remove_speckles(true), min_speckle_aera(20),
		             is_fill_holes(true),
		             p1(10), p2_init(150) { }
	};
public:
	/**
	 * \brief Initialize the class, complete some memory pre-allocation, parameter pre-setting, etc.
	 * \param width		Input, epipolar image width
	 * \param height	Input, epipolar image to image height
	 * \param option	Input, SemiGlobalMatching parameters
	 */
	bool Initialize(const sint32& width, const sint32& height, const SGMOption& option);

	/**
	 * \brief Perform matching
	 * \param img_left	Input, left image data pointer
	 * \param img_right	Input, right image data pointer
	 * \param disp_left	Output, left image disparity map pointer, pre-allocated memory space of the same size as the image
	 */
	bool Match(const uint8* img_left, const uint8* img_right, float32* disp_left);

	/**
	 * \brief Reset
	 * \param width		Input, epipolar image width
	 * \param height	Input, epipolar image to image height
	 * \param option	Input, SemiGlobalMatching parameters
	 */
	bool Reset(const uint32& width, const uint32& height, const SGMOption& option);

private:

	/** \brief Census Transformation */
	void CensusTransform() const;

	/** \brief Cost Calculation	 */
	void ComputeCost() const;

	/** \brief Cost Aggregation	 */
	void CostAggregation() const;

	/** \brief Parallax calculation	 */
	void ComputeDisparity() const;

	/** \brief Parallax calculation	 */
	void ComputeDisparityRight() const;

	/** \brief Consistency Check	 */
	void LRCheck();

	/** \brief Disparity map filling */
	void FillHolesInDispMap();

	/** \brief Memory release	 */
	void Release();

private:
	/** \brief SGM parameters	 */
	SGMOption option_;

	/** \brief image width	 */
	sint32 width_;

	/** \brief image height	 */
	sint32 height_;

	/** \brief Left image data	 */
	const uint8* img_left_;

	/** \brief Right image data	 */
	const uint8* img_right_;
	
	/** \brief Left image census value	*/
	void* census_left_;
	
	/** \brief Right image census value	*/
	void* census_right_;
	
	/** \brief Initial matching cost	*/
	uint8* cost_init_;
	
	/** \brief Aggregate matching cost	*/
	uint16* cost_aggr_;

	// ↘ ↓ ↙ 5 3 7
	// → ← 1 2
	// ↗ ↑ ↖ 8 4 6
	/** \brief Aggregate matching cost-direction 1	*/
	uint8* cost_aggr_1_;
	/** \brief Aggregate matching cost-direction 2	*/
	uint8* cost_aggr_2_;
	/** \brief Aggregate matching cost-direction 3	*/
	uint8* cost_aggr_3_;
	/** \brief Aggregate matching cost-direction 4	*/
	uint8* cost_aggr_4_;
	/** \brief Aggregate matching cost-direction 5	*/
	uint8* cost_aggr_5_;
	/** \brief Aggregate matching cost-direction 6	*/
	uint8* cost_aggr_6_;
	/** \brief Aggregate matching cost-direction 7	*/
	uint8* cost_aggr_7_;
	/** \brief Aggregate matching cost-direction 8	*/
	uint8* cost_aggr_8_;

	/** \brief Left image disparity map	*/
	float32* disp_left_;
	/** \brief Right image disparity map	*/
	float32* disp_right_;

	/** \brief Whether to initialize the flag	*/
	bool is_initialized_;

	/** \brief Occlusion area pixel set	*/
	std::vector<std::pair<int, int>> occlusions_;
	/** \brief Mismatched pixel set	*/
	std::vector<std::pair<int, int>> mismatches_;
};

