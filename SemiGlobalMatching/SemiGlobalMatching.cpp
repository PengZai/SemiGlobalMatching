/*c++ SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: implement of semi-global matching class
*/

#include "stdafx.h"
#include "SemiGlobalMatching.h"
#include "sgm_util.h"
#include <algorithm>
#include <vector>
#include <cassert>
#include <chrono>
#include <cstring>
#include <cmath>

using namespace std::chrono;

SemiGlobalMatching::SemiGlobalMatching(): width_(0), height_(0), img_left_(nullptr), img_right_(nullptr),
                                          census_left_(nullptr), census_right_(nullptr),
                                          cost_init_(nullptr), cost_aggr_(nullptr),
                                          cost_aggr_1_(nullptr), cost_aggr_2_(nullptr),
                                          cost_aggr_3_(nullptr), cost_aggr_4_(nullptr),
                                          cost_aggr_5_(nullptr), cost_aggr_6_(nullptr),
                                          cost_aggr_7_(nullptr), cost_aggr_8_(nullptr),
                                          disp_left_(nullptr), disp_right_(nullptr),
                                          is_initialized_(false)
{
}


SemiGlobalMatching::~SemiGlobalMatching()
{
    Release();
    is_initialized_ = false;
}

bool SemiGlobalMatching::Initialize(const sint32& width, const sint32& height, const SGMOption& option)
{
    
    width_ = width;
    height_ = height;
    // SGM parameters
    option_ = option;

    if(width == 0 || height == 0) {
        return false;
    }


    // census value left and right image
    const sint32 img_size = width * height;
    if (option.census_size == Census5x5) {
        census_left_ = new uint32[img_size]();
        census_right_ = new uint32[img_size]();
    }
    else {
        census_left_ = new uint64[img_size]();
        census_right_ = new uint64[img_size]();
    }

    // disparity range
    const sint32 disp_range = option.max_disparity - option.min_disparity;
    if (disp_range <= 0) {
        return false;
    }

    // matching cost (initial/aggregation)
    const sint32 size = width * height * disp_range;
    cost_init_  = new uint8[size]();
    cost_aggr_  = new uint16[size]();
    cost_aggr_1_ = new uint8[size]();
    cost_aggr_2_ = new uint8[size]();
    cost_aggr_3_ = new uint8[size]();
    cost_aggr_4_ = new uint8[size]();
    cost_aggr_5_ = new uint8[size]();
    cost_aggr_6_ = new uint8[size]();
    cost_aggr_7_ = new uint8[size]();
    cost_aggr_8_ = new uint8[size]();

    // dipsarity
    disp_left_ = new float32[img_size]();
    disp_right_ = new float32[img_size]();

    is_initialized_ = census_left_ && census_right_ && cost_init_ && cost_aggr_ && disp_left_;

    return is_initialized_;
}


void SemiGlobalMatching::Release()
{
    // release memory
    SAFE_DELETE(census_left_);
    SAFE_DELETE(census_right_);
    SAFE_DELETE(cost_init_);
    SAFE_DELETE(cost_aggr_);
    SAFE_DELETE(cost_aggr_1_);
    SAFE_DELETE(cost_aggr_2_);
    SAFE_DELETE(cost_aggr_3_);
    SAFE_DELETE(cost_aggr_4_);
    SAFE_DELETE(cost_aggr_5_);
    SAFE_DELETE(cost_aggr_6_);
    SAFE_DELETE(cost_aggr_7_);
    SAFE_DELETE(cost_aggr_8_);
    SAFE_DELETE(disp_left_);
    SAFE_DELETE(disp_right_);
}

bool SemiGlobalMatching::Match(const uint8* img_left, const uint8* img_right, float32* disp_left)
{
    if(!is_initialized_) {
        return false;
    }
    if (img_left == nullptr || img_right == nullptr) {
        return false;
    }

    img_left_ = img_left;
    img_right_ = img_right;

    auto start = std::chrono::steady_clock::now();

    // census transformation
    CensusTransform();

    // cost calculation
    ComputeCost();

    auto end = steady_clock::now();
    auto tt = duration_cast<milliseconds>(end - start);
    printf("computing cost! timing :	%lf s\n", tt.count() / 1000.0);
    start = steady_clock::now();

    // cost aggregation
    CostAggregation();

    // end = steady_clock::now();
    // tt = duration_cast<milliseconds>(end - start);
    // printf("cost aggregating! timing :	%lf s\n", tt.count() / 1000.0);
    // start = steady_clock::now();

    // disparity calculation
    ComputeDisparity();

    end = steady_clock::now();
    tt = duration_cast<milliseconds>(end - start);
    printf("computing disparities! timing :	%lf s\n", tt.count() / 1000.0);
    start = steady_clock::now();

    // left right consistency check
    if (option_.is_check_lr) {
        // right disaprity calculation
        ComputeDisparityRight();
        // left right consistency check
        LRCheck();
    }

    // remove small connection region
    if (option_.is_remove_speckles) {
        sgm_util::RemoveSpeckles(disp_left_, width_, height_, 1, option_.min_speckle_aera, Invalid_Float);
    }

    // disparity padding
	if(option_.is_fill_holes) {
		FillHolesInDispMap();
	}

    // median filter
    sgm_util::MedianFilter(disp_left_, disp_left_, width_, height_, 3);

    end = steady_clock::now();
    tt = duration_cast<milliseconds>(end - start);
    printf("postprocessing! timing :        %lf s\n", tt.count() / 1000.0);
    start = steady_clock::now();

    // output dispairy map
    memcpy(disp_left, disp_left_, height_ * width_ * sizeof(float32));

	return true;
}

bool SemiGlobalMatching::Reset(const uint32& width, const uint32& height, const SGMOption& option)
{
    // release memory
    Release();

    // reset intialized flag
    is_initialized_ = false;

    // intialize
    return Initialize(width, height, option);
}

void SemiGlobalMatching::CensusTransform() const
{
	// left right image census transformation
    if (option_.census_size == Census5x5) {
        sgm_util::census_transform_5x5(img_left_, static_cast<uint32*>(census_left_), width_, height_);
        sgm_util::census_transform_5x5(img_right_, static_cast<uint32*>(census_right_), width_, height_);
    }else {
        sgm_util::census_transform_9x7(img_left_, static_cast<uint64*>(census_left_), width_, height_);
        sgm_util::census_transform_9x7(img_right_, static_cast<uint64*>(census_right_), width_, height_);
    }
}

void SemiGlobalMatching::ComputeCost() const
{
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }


	// cost calculation (base on hamming distance)
    for (sint32 i = 0; i < height_; i++) {
        for (sint32 j = 0; j < width_; j++) {
            // Calculate the cost value per parallax
        	for (sint32 d = min_disparity; d < max_disparity; d++) {
                auto& cost = cost_init_[i * width_ * disp_range + j * disp_range + (d - min_disparity)];
                if (j - d < 0 || j - d >= width_) {
                    cost = UINT8_MAX/2;
                    continue;
                }
                if (option_.census_size == Census5x5) {
                    // Left image census value
                    const auto& census_val_l = static_cast<uint32*>(census_left_)[i * width_ + j];
                    // Census value of the corresponding image point in the right image
                    const auto& census_val_r = static_cast<uint32*>(census_right_)[i * width_ + j - d];
                    // Calculate matching cost
                    cost = sgm_util::Hamming32(census_val_l, census_val_r);
                }
        		else {
                    const auto& census_val_l = static_cast<uint64*>(census_left_)[i * width_ + j];
                    const auto& census_val_r = static_cast<uint64*>(census_right_)[i * width_ + j - d];
                    cost = sgm_util::Hamming64(census_val_l, census_val_r);
                }
            }
        }
    }
}

void SemiGlobalMatching::CostAggregation() const
{
    // Path aggregation
    // 1. Left -> Right / Right -> Left
    // 2. Up -> Down / Down -> Up
    // 3. Upper Left -> Lower Right / Lower Right -> Upper Left
    // 4. Upper Right -> Upper Left / Lower Left -> Upper Right
    //
    // ↘ ↓ ↙ 5 3 7
    // → ← 1 2
    // ↗ ↑ ↖ 8 4 6
    //
    const auto& min_disparity = option_.min_disparity;
    const auto& max_disparity = option_.max_disparity;
    assert(max_disparity > min_disparity);

    const sint32 size = width_ * height_ * (max_disparity - min_disparity);
    if(size <= 0) {
        return;
    }

    const auto& P1 = option_.p1;
    const auto& P2_Int = option_.p2_init;

    if (option_.num_paths == 4 || option_.num_paths == 8) {
        // Left and right aggregation
        sgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_1_, true);
        sgm_util::CostAggregateLeftRight(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_2_, false);
        // Up and down aggregation
		sgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_3_, true);
        sgm_util::CostAggregateUpDown(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_4_, false);
    }

    if (option_.num_paths == 8) {
        //Diagonal 1 aggregation
        sgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_5_, true);
        sgm_util::CostAggregateDagonal_1(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_6_, false);
        //Diagonal 2 aggregation
        sgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_7_, true);
        sgm_util::CostAggregateDagonal_2(img_left_, width_, height_, min_disparity, max_disparity, P1, P2_Int, cost_init_, cost_aggr_8_, false);
    }

    // Add up 4/8 directions
    for (sint32 i = 0; i < size; i++) {
        if (option_.num_paths == 4 || option_.num_paths == 8) {
            cost_aggr_[i] = cost_aggr_1_[i] + cost_aggr_2_[i] + cost_aggr_3_[i] + cost_aggr_4_[i];
        }
        if (option_.num_paths == 8) {
            cost_aggr_[i] += cost_aggr_5_[i] + cost_aggr_6_[i] + cost_aggr_7_[i] + cost_aggr_8_[i];
        }
    }
}

void SemiGlobalMatching::ComputeDisparity() const
{
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if(disp_range <= 0) {
        return;
    }

    // Left image disparity map
    const auto disparity = disp_left_;
	// Left image aggregation cost array
	const auto cost_ptr = cost_aggr_;

    const sint32 width = width_;
    const sint32 height = height_;
    const bool is_check_unique = option_.is_check_unique;
	const float32 uniqueness_ratio = option_.uniqueness_ratio;

	// To speed up reading, store all cost values ​​of a single pixel in a local array
    std::vector<uint16> cost_local(disp_range);
    
	// ---Calculate the optimal disparity pixel by pixel
	for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            uint16 min_cost = UINT16_MAX;
            uint16 sec_min_cost = UINT16_MAX;
            sint32 best_disparity = 0;

            // ---Traverse all cost values ​​within the disparity range and output the minimum cost value and the corresponding disparity value
            for (sint32 d = min_disparity; d < max_disparity; d++) {
	            const sint32 d_idx = d - min_disparity;
                const auto& cost = cost_local[d_idx] = cost_ptr[i * width * disp_range + j * disp_range + d_idx];
                if(min_cost > cost) {
                    min_cost = cost;
                    best_disparity = d;
                }
            }

            if (is_check_unique) {
                // Traverse again and output the next minimum cost value
                for (sint32 d = min_disparity; d < max_disparity; d++) {
                    if (d == best_disparity) {
                        // Skip the minimum cost value
                        continue;
                    }
                    const auto& cost = cost_local[d - min_disparity];
                    sec_min_cost = std::min(sec_min_cost, cost);
                }

                // Check uniqueness constraint
                // If (min-sec)/min < min*(1-uniquness), then the estimate is invalid.
                if (sec_min_cost - min_cost <= static_cast<uint16>(min_cost * (1 - uniqueness_ratio))) {
                    disparity[i * width + j] = Invalid_Float;
                    continue;
                }
            }

            // ---Sub-pixel fitting
            if (best_disparity == min_disparity || best_disparity == max_disparity - 1) {
                disparity[i * width + j] = Invalid_Float;
                continue;
            }
            // The cost value of the parallax before the optimal parallax is cost_1, and the cost value of the parallax after the optimal parallax is cost_2
            const sint32 idx_1 = best_disparity - 1 - min_disparity;
            const sint32 idx_2 = best_disparity + 1 - min_disparity;
            const uint16 cost_1 = cost_local[idx_1];
            const uint16 cost_2 = cost_local[idx_2];
            // Solve the extreme value of the quadratic curve
            const uint16 denom = std::max(1, cost_1 + cost_2 - 2 * min_cost);
            disparity[i * width + j] = static_cast<float32>(best_disparity) + static_cast<float32>(cost_1 - cost_2) / (denom * 2.0f);
        }
    }
}

void SemiGlobalMatching::ComputeDisparityRight() const
{
    const sint32& min_disparity = option_.min_disparity;
    const sint32& max_disparity = option_.max_disparity;
    const sint32 disp_range = max_disparity - min_disparity;
    if (disp_range <= 0) {
        return;
    }

    // Right image disparity map
    const auto disparity = disp_right_;
    // Left image aggregation cost array
	const auto cost_ptr = cost_aggr_;

    const sint32 width = width_;
    const sint32 height = height_;
    const bool is_check_unique = option_.is_check_unique;
    const float32 uniqueness_ratio = option_.uniqueness_ratio;

    // To speed up reading, store all cost values ​​of a single pixel in a local array
    std::vector<uint16> cost_local(disp_range);

    // --- Calculate optimal disparity pixel by pixel
    // Get the cost of the right image using the cost of the left image
    // Right cost (xr, yr, d) = Left cost (xr + d, yl, d)
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            uint16 min_cost = UINT16_MAX;
            uint16 sec_min_cost = UINT16_MAX;
            sint32 best_disparity = 0;

            // ---Count the cost values ​​under candidate disparity
        	for (sint32 d = min_disparity; d < max_disparity; d++) {
                const sint32 d_idx = d - min_disparity;
        		const sint32 col_left = j + d;
        		if (col_left >= 0 && col_left < width) {
                    const auto& cost = cost_local[d_idx] = cost_ptr[i * width * disp_range + col_left * disp_range + d_idx];
                    if (min_cost > cost) {
                        min_cost = cost;
                        best_disparity = d;
                    }
        		}
                else {
                    cost_local[d_idx] = UINT16_MAX;
                }
            }

            if (is_check_unique) {
                // Traverse again and output the next minimum cost value
                for (sint32 d = min_disparity; d < max_disparity; d++) {
                    if (d == best_disparity) {
                        // Skip the minimum cost value
                        continue;
                    }
                    const auto& cost = cost_local[d - min_disparity];
                    sec_min_cost = std::min(sec_min_cost, cost);
                }

                // Check uniqueness constraint
                // If (min-sec)/min < min*(1-uniquness), then the estimate is invalid.
                if (sec_min_cost - min_cost <= static_cast<uint16>(min_cost * (1 - uniqueness_ratio))) {
                    disparity[i * width + j] = Invalid_Float;
                    continue;
                }
            }
            
            // ---Sub-pixel fitting
            if (best_disparity == min_disparity || best_disparity == max_disparity - 1) {
                disparity[i * width + j] = Invalid_Float;
                continue;
            }

            // The cost value of the parallax before the optimal parallax is cost_1, and the cost value of the parallax after the optimal parallax is cost_2
            const sint32 idx_1 = best_disparity - 1 - min_disparity;
            const sint32 idx_2 = best_disparity + 1 - min_disparity;
            const uint16 cost_1 = cost_local[idx_1];
            const uint16 cost_2 = cost_local[idx_2];
            // Solve the extreme value of the quadratic curve
            const uint16 denom = std::max(1, cost_1 + cost_2 - 2 * min_cost);
            disparity[i * width + j] = static_cast<float32>(best_disparity) + static_cast<float32>(cost_1 - cost_2) / (denom * 2.0f);
        }
    }
}

void SemiGlobalMatching::LRCheck()
{
    const sint32 width = width_;
    const sint32 height = height_;

    const float32& threshold = option_.lrcheck_thres;

	// Pixels in the occluded area and pixels in the mismatched area
	auto& occlusions = occlusions_;
	auto& mismatches = mismatches_;
	occlusions.clear();
	mismatches.clear();

    // --- Left and right consistency check
    for (sint32 i = 0; i < height; i++) {
        for (sint32 j = 0; j < width; j++) {
            // Left image disparity value
        	auto& disp = disp_left_[i * width + j];
			if(disp == Invalid_Float){
				mismatches.emplace_back(i, j);
				continue;
			}

            // Find the corresponding pixel with the same name on the right image based on the disparity value
        	const auto col_right = static_cast<sint32>(j - disp + 0.5);
            
        	if(col_right >= 0 && col_right < width) {
                // Disparity value of the pixel with the same name on the right image
                const auto& disp_r = disp_right_[i * width + col_right];
                
        		// Determine whether the two disparity values ​​are consistent (the difference is within the threshold)
        		if (abs(disp - disp_r) > threshold) {
					// Distinguish occlusions from mismatches
                    // Calculate the matching pixel in the left image using the right image's disparity and obtain the disparity disp_rl
                    // if (disp_rl > disp)
                    // Pixel in occlusions
                    // else
                    // Pixel in mismatches
					const sint32 col_rl = static_cast<sint32>(col_right + disp_r + 0.5);
					if(col_rl > 0 && col_rl < width){
						const auto& disp_l = disp_left_[i*width + col_rl];
						if(disp_l > disp) {
							occlusions.emplace_back(i, j);
						}
						else {
							mismatches.emplace_back(i, j);
						}
					}
					else{
						mismatches.emplace_back(i, j);
					}

                    // Invalidate the parallax value
					disp = Invalid_Float;
                }
            }
            else{
                // The disparity value cannot be found on the right image with the same pixel name (out of image range)
                disp = Invalid_Float;
				mismatches.emplace_back(i, j);
            }
        }
    }

}

void SemiGlobalMatching::FillHolesInDispMap()
{
	const sint32 width = width_;
	const sint32 height = height_;

	std::vector<float32> disp_collects;

	// Define 8 directions
	const float32 pi = 3.1415926f;
	float32 angle1[8] = { pi, 3 * pi / 4, pi / 2, pi / 4, 0, 7 * pi / 4, 3 * pi / 2, 5 * pi / 4 };
	float32 angle2[8] = { pi, 5 * pi / 4, 3 * pi / 2, 7 * pi / 4, 0, pi / 4, pi / 2, 3 * pi / 4 };
	float32 *angle = angle1;
    // Maximum search distance, no need to search pixels too far away
    const sint32 max_search_length = 1.0*std::max(abs(option_.max_disparity), abs(option_.min_disparity));

	float32* disp_ptr = disp_left_;
	for (sint32 k = 0; k < 3; k++) {
		// The first loop processes the occluded area, and the second loop processes the mismatched area
		auto& trg_pixels = (k == 0) ? occlusions_ : mismatches_;
        if (trg_pixels.empty()) {
            continue;
        }
		std::vector<float32> fill_disps(trg_pixels.size());
		std::vector<std::pair<sint32, sint32>> inv_pixels;
		if (k == 2) {
			// The third loop processes pixels that were not processed cleanly in the first two loops
			for (sint32 i = 0; i < height; i++) {
				for (sint32 j = 0; j < width; j++) {
					if (disp_ptr[i * width + j] == Invalid_Float) {
						inv_pixels.emplace_back(i, j);
					}
				}
			}
			trg_pixels = inv_pixels;
		}

		// Traverse the pixels to be processed
        for (auto n = 0u; n < trg_pixels.size(); n++) {
            auto& pix = trg_pixels[n];
            const sint32 y = pix.first;
            const sint32 x = pix.second;

			if (y == height / 2) {
				angle = angle2; 
			}

			// Collect the first valid disparity value encountered in 8 directions
			disp_collects.clear();
			for (sint32 s = 0; s < 8; s++) {
				const float32 ang = angle[s];
				const float32 sina = float32(std::sin(ang));
				const float32 cosa = float32(std::cos(ang));
				for (sint32 m = 1; m < max_search_length; m++) {
					const sint32 yy = std::lround(y + m * sina);
					const sint32 xx = std::lround(x + m * cosa);
					if (yy<0 || yy >= height || xx<0 || xx >= width) {
						break;
					}
					const auto& disp = *(disp_ptr + yy*width + xx);
					if (disp != Invalid_Float) {
						disp_collects.push_back(disp);
						break;
					}
				}
			}
			if(disp_collects.empty()) {
				continue;
			}

			std::sort(disp_collects.begin(), disp_collects.end());

			// If it is an occluded area, select the second smallest disparity value
            // If it is a mismatched area, select the median value
			if (k == 0) {
				if (disp_collects.size() > 1) {
                    fill_disps[n] = disp_collects[1];
				}
				else {
                    fill_disps[n] = disp_collects[0];
				}
			}
			else{
                fill_disps[n] = disp_collects[disp_collects.size() / 2];
			}
		}
        for (auto n = 0u; n < trg_pixels.size(); n++) {
            auto& pix = trg_pixels[n];
            const sint32 y = pix.first;
            const sint32 x = pix.second;
            disp_ptr[y * width + x] = fill_disps[n];
        }
	}
}
