/* -*-c++-*- SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: header of sgm_types
*/

#pragma once

#include <cstdint>
#include <limits>

constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

typedef int8_t			sint8;		
typedef uint8_t			uint8;		
typedef int16_t			sint16;		
typedef uint16_t		uint16;		
typedef int32_t			sint32;		
typedef uint32_t		uint32;		
typedef int64_t			sint64;		
typedef uint64_t		uint64;		
typedef float			float32;	
typedef double			float64;	