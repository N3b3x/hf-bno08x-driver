/*
 * Copyright 2015-21 CEVA, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and
 * any applicable agreements you may have with CEVA, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * DFU (Download Firmware Update) interface
 */

#ifndef DFU_H
#define DFU_H

#include "HcBin.h"
#include "IDfuTransport.hpp"

/**
 * @brief Run DFU process using the provided transport.
 *
 * @param transport Hardware transport implementation.
 * @return Err code from sh2_err.h indicating DFU result.
 */
int dfu(IDfuTransport &transport, const HcBin_t &firmware);
int dfu(IDfuTransport &transport);

#endif
