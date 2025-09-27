/* -*-c++-*- SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: implement of sgm_util
*/

#pragma once
#include "sgm_types.h"

#ifndef SAFE_DELETE
#define SAFE_DELETE(P) {if(P) delete[](P);(P)=nullptr;}
#endif

namespace sgm_util
{
	//������������ Census Tools
	// Census Transformation

	/**
	* \brief census transformation
	* \param source input, image data
	* \param census output, array of census values
	* \param width input, image width
	* \param height input, image height
	*/
	void census_transform_5x5(const uint8* source, uint32* census, const sint32& width, const sint32& height);
	void census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height);
	// Hamming distance
	uint8 Hamming32(const uint32& x, const uint32& y);
	uint8 Hamming64(const uint64& x, const uint64& y);

	/**
	* \brief Left-right path aggregation �� ��
	* \param img_data Input, image data
	* \param width Input, image width
	* \param height Input, image height
	* \param min_disparity Input, minimum disparity
	* \param max_disparity Input, maximum disparity
	* \param p1 Input, penalty term P1
	* \param p2_init Input, penalty term P2_Init
	* \param cost_init Input, initial cost data
	* \param cost_aggr Output, path aggregation cost data
	* \param is_forward Input, whether the path is in the forward direction (forward is from left to right, reverse is from right to left)
	*/
	void CostAggregateLeftRight(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1,const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	/**
	* \brief Up and down path aggregation �� ��
	* \param img_data Input, image data
	* \param width Input, image width
	* \param height Input, image height
	* \param min_disparity Input, minimum disparity
	* \param max_disparity Input, maximum disparity
	* \param p1 Input, penalty term P1
	* \param p2_init Input, penalty term P2_Init
	* \param cost_init Input, initial cost data
	* \param cost_aggr Output, path aggregation cost data
	* \param is_forward Input, whether the path is in the forward direction (forward is from top to bottom, reverse is from bottom to top)
	*/
	void CostAggregateUpDown(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	/**
	* \brief Diagonal 1-path aggregation (upper left <-> lower right) �K �I
	* \param img_data Input, image data
	* \param width Input, image width
	* \param height Input, image height
	* \param min_disparity Input, minimum disparity
	* \param max_disparity Input, maximum disparity
	* \param p1 Input, penalty term P1
	* \param p2_init Input, penalty term P2_Init
	* \param cost_init Input, initial cost data
	* \param cost_aggr Output, path aggregation cost data
	* \param is_forward Input, whether the path is in the forward direction (forward direction is from upper left to lower right, reverse direction is from lower right to upper left)
	*/
	void CostAggregateDagonal_1(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	/**
	* \brief Diagonal 2-path aggregation (upper right <-> lower left) �L �J
	* \param img_data Input, image data
	* \param width Input, image width
	* \param height Input, image height
	* \param min_disparity Input, minimum disparity
	* \param max_disparity Input, maximum disparity
	* \param p1 Input, penalty term P1
	* \param p2_init Input, penalty term P2_Init
	* \param cost_init Input, initial cost data
	* \param cost_aggr Output, path aggregation cost data
	* \param is_forward Input, whether the path is in the forward direction (forward is from top to bottom, reverse is from bottom to top)
	*/
	void CostAggregateDagonal_2(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
		const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward = true);

	
	/**
	* \param median filter
	* \param in (input, source data)
	* \param out (output, target data)
	* \param width (input, width)
	* \param height (input, height)
	* \param wnd_size (input, window width)
	*/
	void MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height, const sint32 wnd_size);


	/**
	* \brief Remove small connected regions
	* \param disparity_map Input, disparity map
	* \param width Input, width
	* \param height Input, height
	* \param diff_insame Input, local pixel differences within the same connected region
	* \param min_speckle_aera Input, minimum connected region area
	* \param invalid_val Input, invalid value
	*/
	void RemoveSpeckles(float32* disparity_map, const sint32& width, const sint32& height, const sint32& diff_insame,const uint32& min_speckle_aera, const float32& invalid_val);
}