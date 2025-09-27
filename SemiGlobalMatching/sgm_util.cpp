/* -*-c++-*- SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: implement of sgm_util
*/

#include "stdafx.h"
#include "sgm_util.h"
#include <algorithm>
#include <cassert>
#include <vector>
#include <queue>
#include <cstring>

void sgm_util::census_transform_5x5(const uint8* source, uint32* census, const sint32& width,
	const sint32& height)
{
	if (source == nullptr || census == nullptr || width <= 5 || height <= 5) {
		return;
	}

	// Calculate census value pixel by pixel
	for (sint32 i = 2; i < height - 2; i++) {
		for (sint32 j = 2; j < width - 2; j++) {
			
			// Center pixel value
			const uint8 gray_center = source[i * width + j];
			
			// Traverse the neighborhood pixels in a 5x5 window, compare the pixel values with the center pixel value one by one, 
			// and calculate the census value
			uint32 census_val = 0u;
			for (sint32 r = -2; r <= 2; r++) {
				for (sint32 c = -2; c <= 2; c++) {
					census_val <<= 1;
					const uint8 gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}

			//census value of the center pixel
			census[i * width + j] = census_val;		
		}
	}
}

void sgm_util::census_transform_9x7(const uint8* source, uint64* census, const sint32& width, const sint32& height)
{
	if (source == nullptr || census == nullptr || width <= 9 || height <= 7) {
		return;
	}

	// Calculate census value pixel by pixel
	for (sint32 i = 4; i < height - 4; i++) {
		for (sint32 j = 3; j < width - 3; j++) {

			// Center pixel value
			const uint8 gray_center = source[i * width + j];

			// Traverse the neighborhood pixels in a 5x5 window, compare the pixel values ??with the center pixel value one by one, and calculate the census value
			uint64 census_val = 0u;
			for (sint32 r = -4; r <= 4; r++) {
				for (sint32 c = -3; c <= 3; c++) {
					census_val <<= 1;
					const uint8 gray = source[(i + r) * width + j + c];
					if (gray < gray_center) {
						census_val += 1;
					}
				}
			}

			//census value of the center pixel
			census[i * width + j] = census_val;
		}
	}
}

uint8 sgm_util::Hamming32(const uint32& x, const uint32& y)
{
	uint32 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}

uint8 sgm_util::Hamming64(const uint64& x, const uint64& y)
{
	uint64 dist = 0, val = x ^ y;

	// Count the number of set bits
	while (val) {
		++dist;
		val &= val - 1;
	}

	return static_cast<uint8>(dist);
}


void sgm_util::CostAggregateLeftRight(const uint8* img_data, const sint32& width, const sint32& height, const sint32& min_disparity, const sint32& max_disparity,
	const sint32& p1, const sint32& p2_init, const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 0 && height > 0 && max_disparity > min_disparity);

	// Parallax range
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// Forward (left->right): is_forward = true; direction = 1
	// Reverse (right->left): is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// Aggregation
	for (sint32 i = 0u; i < height; i++) {
		// The path header is the first (last, dir=-1) column of pixels in each row
		auto cost_init_row = (is_forward) ? (cost_init + i * width * disp_range) : (cost_init + i * width * disp_range + (width - 1) * disp_range);
		auto cost_aggr_row = (is_forward) ? (cost_aggr + i * width * disp_range) : (cost_aggr + i * width * disp_range + (width - 1) * disp_range);
		auto img_row = (is_forward) ? (img_data + i * width) : (img_data + i * width + width - 1);

		// The current grayscale value and the previous grayscale value on the path
		uint8 gray = *img_row;
		uint8 gray_last = *img_row;

		// The cost array of the previous pixel on the path. The extra two elements are to avoid boundary overflow (one at the beginning and one at the end)
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// Initialization: The aggregation cost of the first pixel is equal to the initial cost
		memcpy(cost_aggr_row, cost_init_row, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(uint8));
		cost_init_row += direction * disp_range;
		cost_aggr_row += direction * disp_range;
		img_row += direction;

		// The minimum cost of the previous pixel on the path
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// Aggregate sequentially starting from the second pixel in the direction
		for (sint32 j = 0; j < width - 1; j++) {
			gray = *img_row;
			uint8 min_cost = UINT8_MAX;
			for (sint32 d = 0; d < disp_range; d++){
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const uint8  cost = cost_init_row[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));
				
				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);
				
				cost_aggr_row[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Reset the minimum cost value and cost array of the previous pixel
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_row, disp_range * sizeof(uint8));

			// Next pixel
			cost_init_row += direction * disp_range;
			cost_aggr_row += direction * disp_range;
			img_row += direction;
			
			// Reassign pixel values
			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateUpDown(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 0 && height > 0 && max_disparity > min_disparity);

	// Parallax range
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// Forward (up->down): is_forward = true; direction = 1
	// Reverse (down->up): is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// polymerization
	for (sint32 j = 0; j < width; j++) {
		// The path header is the first (last, dir=-1) row of pixels in each column
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// The current grayscale value and the previous grayscale value on the path
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// The cost array of the previous pixel on the path. The extra two elements are to avoid boundary overflow (one at the beginning and one at the end)
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// Initialization: The aggregation cost of the first pixel is equal to the initial cost
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));
		cost_init_col += direction * width * disp_range;
		cost_aggr_col += direction * width * disp_range;
		img_col += direction * width;

		// The minimum cost of the previous pixel on the path
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// Aggregate sequentially starting from the second pixel in the direction
		for (sint32 i = 0; i < height - 1; i ++) {
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;
			for (sint32 d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Reset the minimum cost value and cost array of the previous pixel
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// Next pixel
			cost_init_col += direction * width * disp_range;
			cost_aggr_col += direction * width * disp_range;
			img_col += direction * width;

			// Reassign pixel values
			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateDagonal_1(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// Parallax range
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// Forward (upper left -> lower right): is_forward = true; direction = 1
	// Reverse direction (lower right -> upper left): is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// aggregation

	// Store the current row and column number to determine whether the image boundary has been reached
	sint32 current_row = 0;
	sint32 current_col = 0;

	for (sint32 j = 0; j < width; j++) {
		// The path header is the first (last, dir=-1) row of pixels in each column
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// The cost array of the previous pixel on the path. The extra two elements are to avoid boundary overflow (one at the beginning and one at the end)
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// Initialization: The aggregation cost of the first pixel is equal to the initial cost
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

		// The current grayscale value and the previous grayscale value on the path
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// The next pixel on the diagonal path, with a gap of width + 1 pixels between them
		// Here we need one more boundary processing
		// When moving diagonally, you will encounter the image column boundary. The strategy is to continue moving in the original direction with the row number and jump to the other boundary with the column number.
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == width - 1 && current_row < height - 1) {
			// Upper left -> lower right, touch the right border
			cost_init_col = cost_init + (current_row + direction) * width * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
            current_col = 0;
		}
		else if (!is_forward && current_col == 0 && current_row > 0) {
			// Bottom right -> top left, touch the left border
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
            current_col = width - 1;
		}
		else {
			cost_init_col += direction * (width + 1) * disp_range;
			cost_aggr_col += direction * (width + 1) * disp_range;
			img_col += direction * (width + 1);
		}

		// The minimum cost of the previous pixel on the path
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// Aggregate sequentially starting from the second pixel in the direction
		for (sint32 i = 0; i < height - 1; i ++) {
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;
			for (sint32 d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Reset the minimum cost value and cost array of the previous pixel
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// Row and column number of the current pixel
			current_row += direction;
			current_col += direction;
			
			// Next pixel, one more boundary to process here
			// One more boundary to process here
			// When moving diagonally, we'll hit an image column boundary. The strategy is to continue moving in the original direction for the row number and jump to the next boundary for the column number.
			if (is_forward && current_col == width - 1 && current_row < height - 1) {
				// Upper left -> lower right, touch the right border
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
                current_col = 0;
			}
			else if (!is_forward && current_col == 0 && current_row > 0) {
				// Bottom right -> top left, touch the left border
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
                current_col = width - 1;
			}
			else {
				cost_init_col += direction * (width + 1) * disp_range;
				cost_aggr_col += direction * (width + 1) * disp_range;
				img_col += direction * (width + 1);
			}

			// Reassign pixel values
			gray_last = gray;
		}
	}
}

void sgm_util::CostAggregateDagonal_2(const uint8* img_data, const sint32& width, const sint32& height,
	const sint32& min_disparity, const sint32& max_disparity, const sint32& p1, const sint32& p2_init,
	const uint8* cost_init, uint8* cost_aggr, bool is_forward)
{
	assert(width > 1 && height > 1 && max_disparity > min_disparity);

	// Parallax range
	const sint32 disp_range = max_disparity - min_disparity;

	// P1,P2
	const auto& P1 = p1;
	const auto& P2_Init = p2_init;

	// Forward (upper right -> lower left): is_forward = true; direction = 1
	// Reverse (lower left -> upper right): is_forward = false; direction = -1;
	const sint32 direction = is_forward ? 1 : -1;

	// Aggregation

	// Store the current row and column numbers to determine if the image boundary has been reached
	sint32 current_row = 0;
	sint32 current_col = 0;

	for (sint32 j = 0; j < width; j++) {
		// The path header is the first (last, dir=-1) row of pixels in each column
		auto cost_init_col = (is_forward) ? (cost_init + j * disp_range) : (cost_init + (height - 1) * width * disp_range + j * disp_range);
		auto cost_aggr_col = (is_forward) ? (cost_aggr + j * disp_range) : (cost_aggr + (height - 1) * width * disp_range + j * disp_range);
		auto img_col = (is_forward) ? (img_data + j) : (img_data + (height - 1) * width + j);

		// The cost array of the previous pixel on the path. The extra two elements are to avoid boundary overflow (one at the beginning and one at the end)
		std::vector<uint8> cost_last_path(disp_range + 2, UINT8_MAX);

		// Initialization: The aggregation cost of the first pixel is equal to the initial cost
		memcpy(cost_aggr_col, cost_init_col, disp_range * sizeof(uint8));
		memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

		// The current grayscale value and the previous grayscale value on the path
		uint8 gray = *img_col;
		uint8 gray_last = *img_col;

		// Next pixel on the diagonal path, with a gap of width - 1 pixels between them
		// Additional boundary handling is required here
		// When moving diagonally, we will encounter an image column boundary. The strategy is to continue moving in the original direction for the row number and jump to the next boundary when the column number reaches it.
		current_row = is_forward ? 0 : height - 1;
		current_col = j;
		if (is_forward && current_col == 0 && current_row < height - 1) {
			// Upper right -> lower left, touching the left border
			cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
			img_col = img_data + (current_row + direction) * width + (width - 1);
            current_col = width - 1;
		}
		else if (!is_forward && current_col == width - 1 && current_row > 0) {
			// Lower left -> upper right, touch the right border
			cost_init_col = cost_init + (current_row + direction) * width * disp_range ;
			cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
			img_col = img_data + (current_row + direction) * width;
            current_col = 0;
		}
		else {
			cost_init_col += direction * (width - 1) * disp_range;
			cost_aggr_col += direction * (width - 1) * disp_range;
			img_col += direction * (width - 1);
		}

		// The minimum cost of the previous pixel on the path
		uint8 mincost_last_path = UINT8_MAX;
		for (auto cost : cost_last_path) {
			mincost_last_path = std::min(mincost_last_path, cost);
		}

		// Aggregate sequentially starting from the second pixel on the path
		for (sint32 i = 0; i < height - 1; i++) {
			gray = *img_col;
			uint8 min_cost = UINT8_MAX;
			for (sint32 d = 0; d < disp_range; d++) {
				// Lr(p,d) = C(p,d) + min( Lr(p-r,d), Lr(p-r,d-1) + P1, Lr(p-r,d+1) + P1, min(Lr(p-r))+P2 ) - min(Lr(p-r))
				const uint8  cost = cost_init_col[d];
				const uint16 l1 = cost_last_path[d + 1];
				const uint16 l2 = cost_last_path[d] + P1;
				const uint16 l3 = cost_last_path[d + 2] + P1;
				const uint16 l4 = mincost_last_path + std::max(P1, P2_Init / (abs(gray - gray_last) + 1));

				const uint8 cost_s = cost + static_cast<uint8>(std::min(std::min(l1, l2), std::min(l3, l4)) - mincost_last_path);

				cost_aggr_col[d] = cost_s;
				min_cost = std::min(min_cost, cost_s);
			}

			// Reset the minimum cost value and cost array of the previous pixel
			mincost_last_path = min_cost;
			memcpy(&cost_last_path[1], cost_aggr_col, disp_range * sizeof(uint8));

			// Row and column number of the current pixel
			current_row += direction;
			current_col -= direction;

			// Next pixel, one more boundary to process here
			// One more boundary to process here
			// When moving diagonally, we'll hit an image column boundary. The strategy is to continue moving in the original direction for the row number and jump to the next boundary for the column number.
			if (is_forward && current_col == 0 && current_row < height - 1) {
				// Upper right -> lower left, touching the left border
				cost_init_col = cost_init + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range + (width - 1) * disp_range;
				img_col = img_data + (current_row + direction) * width + (width - 1);
                current_col = width - 1;
			}
			else if (!is_forward && current_col == width - 1 && current_row > 0) {
				// Lower left -> upper right, touch the right border
				cost_init_col = cost_init + (current_row + direction) * width * disp_range;
				cost_aggr_col = cost_aggr + (current_row + direction) * width * disp_range;
				img_col = img_data + (current_row + direction) * width;
                current_col = 0;
			}
			else {
				cost_init_col += direction * (width - 1) * disp_range;
				cost_aggr_col += direction * (width - 1) * disp_range;
				img_col += direction * (width - 1);
			}

			// Reassign pixel values
			gray_last = gray;
		}
	}
}

void sgm_util::MedianFilter(const float32* in, float32* out, const sint32& width, const sint32& height,
	const sint32 wnd_size)
{
	const sint32 radius = wnd_size / 2;
	const sint32 size = wnd_size * wnd_size;

	//Store data in the local window
	std::vector<float32> wnd_data;
	wnd_data.reserve(size);

	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			wnd_data.clear();

			// Get local window data
			for (sint32 r = -radius; r <= radius; r++) {
				for (sint32 c = -radius; c <= radius; c++) {
					const sint32 row = i + r;
					const sint32 col = j + c;
					if (row >= 0 && row < height && col >= 0 && col < width) {
						wnd_data.push_back(in[row * width + col]);
					}
				}
			}

			// Sort
			std::sort(wnd_data.begin(), wnd_data.end());
			// Get the median
			out[i * width + j] = wnd_data[wnd_data.size() / 2];
		}
	}
}

void sgm_util::RemoveSpeckles(float32* disparity_map, const sint32& width, const sint32& height,
	const sint32& diff_insame, const uint32& min_speckle_aera, const float32& invalid_val)
{
	assert(width > 0 && height > 0);
	if (width < 0 || height < 0) {
		return;
	}

	// Defines the array that marks whether pixels are accessed
	std::vector<bool> visited(uint32(width*height),false);
	for(sint32 i=0;i<height;i++) {
		for(sint32 j=0;j<width;j++) {
			if (visited[i * width + j] || disparity_map[i*width+j] == invalid_val) {
				// Skip visited pixels and invalid pixels
				continue;
			}
			// Breadth-first traversal, region tracking
			// Set the disparity of connected regions whose area is less than the threshold to an invalid value
			std::vector<std::pair<sint32, sint32>> vec;
			vec.emplace_back(i, j);
			visited[i * width + j] = true;
			uint32 cur = 0;
			uint32 next = 0;
			do {
				// Breadth-first traversal area tracking
				next = vec.size();
				for (uint32 k = cur; k < next; k++) {
					const auto& pixel = vec[k];
					const sint32 row = pixel.first;
					const sint32 col = pixel.second;
					const auto& disp_base = disparity_map[row * width + col];
					//8 neighborhood traversal
					for(int r=-1;r<=1;r++) {
						for(int c=-1;c<=1;c++) {
							if(r==0&&c==0) {
								continue;
							}
							int rowr = row + r;
							int colc = col + c;
							if (rowr >= 0 && rowr < height && colc >= 0 && colc < width) {
								if(!visited[rowr * width + colc] &&
									(disparity_map[rowr * width + colc] != invalid_val) &&
									abs(disparity_map[rowr * width + colc] - disp_base) <= diff_insame) {
									vec.emplace_back(rowr, colc);
									visited[rowr * width + colc] = true;
								}
							}
						}
					}
				}
				cur = next;
			} while (next < vec.size());

			// Set the disparity of the connected domain area smaller than the threshold to invalid value
			if(vec.size() < min_speckle_aera) {
				for(auto& pix:vec) {
					disparity_map[pix.first * width + pix.second] = invalid_val;
				}
			}
		}
	}
}
