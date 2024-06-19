///
//! @file 			Exception.cpp
//! @author 		Vulcan YJX <vulcanai@163.com> 
//! @created		2022-06-07
//! @last-modified 	2022-06-10
//! @brief			The main serial port class.



#ifndef VULCAN_CPP_LINUX_SERIAL_EXCEPTION_H_
#define VULCAN_CPP_LINUX_SERIAL_EXCEPTION_H_

// System includes

#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>



namespace VulcanSerial {

        class Exception : public std::runtime_error {

        public:
            Exception(const char *file, int line, const std::string &arg) :
                    std::runtime_error(arg) {
                msg_ = std::string(file) + ":" + std::to_string(line) + ": " + arg;
            }

            ~Exception() throw() {}

            const char *what() const throw() override {
                return msg_.c_str();
            }

        private:
            std::string msg_;
        };

} // namespace VulcanSerial


#define THROW_EXCEPT(arg) throw Exception(__FILE__, __LINE__, arg);


#endif // VULCAN_CPP_LINUX_SERIAL_EXCEPTION_H_
