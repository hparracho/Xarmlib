// ----------------------------------------------------------------------------
// @file    xarmlib.hpp
// @brief   Xarmlib main header file.
// @date    8 October 2020
// ----------------------------------------------------------------------------
//
// Xarmlib 0.2.0 - https://github.com/hparracho/Xarmlib
// Copyright (c) 2018-2020 Helder Parracho (hparracho@gmail.com)
// PDX-License-Identifier: MIT License
//
// See README.md file for additional credits and acknowledgments.
//
// ----------------------------------------------------------------------------

#ifndef XARMLIB_HPP
#define XARMLIB_HPP




// API interface

#include "api/api_crc.hpp"
#include "api/api_digital_in.hpp"
#include "api/api_digital_in_bus.hpp"
#include "api/api_digital_out.hpp"
#include "api/api_high_res_clock.hpp"
//#include "api/api_digital_out_bus.hpp"
//#include "api/api_input_debouncer.hpp"
//#include "api/api_io_debouncer.hpp"
//#include "api/api_output_driver.hpp"
#include "api/api_pin_bus.hpp"
//#include "api/api_pin_scanner.hpp"


// Targets HAL interface

//#include "hal/hal_can.hpp"
//#include "hal/hal_enc.hpp"
#include "hal/hal_flash.hpp"
#include "hal/hal_gpio.hpp"
//#include "hal/hal_i2c.hpp"
#include "hal/hal_pin.hpp"
#include "hal/hal_pin_int.hpp"
#include "hal/hal_port.hpp"
#include "hal/hal_spi.hpp"
#include "hal/hal_system.hpp"
#include "hal/hal_timer.hpp"
#include "hal/hal_usart.hpp"
#include "hal/hal_us_ticker.hpp"
#include "hal/hal_watchdog.hpp"


#include "core/target_flash.hpp"
#include "core/target_gpio.hpp"
#include "core/target_pin.hpp"
#include "core/target_pin_int.hpp"
#include "core/target_port.hpp"
#include "core/target_specs.hpp"
#include "core/target_spi.hpp"
#include "core/target_system.hpp"
#include "core/target_timer.hpp"
#include "core/target_usart.hpp"
#include "core/target_watchdog.hpp"




#endif // XARMLIB_HPP
