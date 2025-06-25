/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* ...
*********************************************************************/

#ifndef SHARED_CONST_BUFFER_HPP__
#define SHARED_CONST_BUFFER_HPP__

#include <memory>
#include <string>
#include <vector>

#include <boost/asio.hpp>

namespace hw_interface_support_types
{
    // A reference-counted non-modifiable buffer sequence.
    class shared_const_buffer
    {
    public:
        // Construct from a std::string.
        explicit shared_const_buffer(const std::string & data)
            : data_(std::make_shared<std::vector<char>>(data.begin(), data.end())),
              buffer_(std::make_shared<boost::asio::const_buffer>(boost::asio::buffer(*data_)))
        {
        }

        shared_const_buffer(const char * data, int length)
            : data_(std::make_shared<std::vector<char>>(data, data + length)),
              buffer_(std::make_shared<boost::asio::const_buffer>(boost::asio::buffer(*data_)))
        {
        }

        shared_const_buffer(const char * data, int length, bool LEtoBE)
            : data_(std::make_shared<std::vector<char>>(length)),
              buffer_(std::make_shared<boost::asio::const_buffer>(boost::asio::buffer(*data_)))
        {
            if (LEtoBE) {
                for (int i = 0; i < length; ++i) {
                    (*data_)[i] = data[length - 1 - i];
                }
            }
        }

        explicit shared_const_buffer(const std::vector<char> & data)
            : data_(std::make_shared<std::vector<char>>(data)),
              buffer_(std::make_shared<boost::asio::const_buffer>(boost::asio::buffer(*data_)))
        {
        }

        template<typename T>
        explicit shared_const_buffer(const std::vector<T> & data)
            : data_(std::make_shared<std::vector<char>>(data.begin(), data.end())),
              buffer_(std::make_shared<boost::asio::const_buffer>(boost::asio::buffer(*data_)))
        {
        }

        template<typename S>
        explicit shared_const_buffer(const S & data)
            : data_(std::make_shared<std::vector<char>>(
                  reinterpret_cast<const char *>(&data),
                  reinterpret_cast<const char *>(&data) + sizeof(S))),
              buffer_(std::make_shared<boost::asio::const_buffer>(boost::asio::buffer(*data_)))
        {
        }

        // ConstBufferSequence requirements
        using value_type    = boost::asio::const_buffer;
        using const_iterator = const boost::asio::const_buffer *;

        const_iterator begin() const { return buffer_.get(); }
        const_iterator end()   const { return buffer_.get() + 1; }

    protected:
        // For derived classes that might need a blank ctor
        shared_const_buffer() = default;

        std::shared_ptr<std::vector<char>> data_;
        std::shared_ptr<boost::asio::const_buffer> buffer_;
    };
}  // namespace hw_interface_support_types

#endif  // SHARED_CONST_BUFFER_HPP__
