/**
 * MIT License
 * 
 * Copyright (c) 2018 Yash Shah
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file talker.hpp
 * @Auther Yash Shah
 * @version 1.0
 * @brief This is a hpp file for talker.cpp
 * 
 * This program is a part of talker.cpp to have a global string variable. 
 * 
 * @copyright MIT License (c) 2018  
 */

#ifndef INCLUDE_TALKER_HPP_
#define INCLUDE_TALKER_HPP_
#include <sstream>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/change_talker_string.h"

// This was declared to solve global string errors in cpplint.
struct globalString {
    // Initialize the base string to print
    std::string str_text = "ENPM808X: Software Development."
                          " The count has reached: ";
};

#endif  // INCLUDE_TALKER_HPP_
