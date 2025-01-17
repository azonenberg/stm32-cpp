/***********************************************************************************************************************
*                                                                                                                      *
* STM32-CPP                                                                                                            *
*                                                                                                                      *
* Copyright (c) 2020-2024 Andrew D. Zonenberg                                                                          *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#ifndef stm32_common_h
#define stm32_common_h

#include <stdint.h>
#include <cstddef>

/**
	@brief Disables interrupts without saving the previous enable state
 */
extern "C" void DisableInterrupts();

/**
	@brief Enables interrupts without saving the previous enable state
 */
extern "C" void EnableInterrupts();

/**
	@brief Enters a critical section, disables interrupts, and returns the previous PRIMASK value
 */
extern "C" uint32_t EnterCriticalSection();

/**
	@brief Leaves a critical section and restores the previous PRIMASK value
 */
extern "C" void LeaveCriticalSection(uint32_t cpu_sr);

//Linker variables
extern uint8_t __data_romstart;
extern uint8_t __data_start;
extern uint8_t __data_end;
extern uint8_t __itcm_start;
extern uint8_t __itcm_romstart;
extern uint8_t __itcm_end;

/**
	@brief Enables an IRQ lane in the NVIC
 */
void NVIC_EnableIRQ(int lane);

/**
	@brief Disables bus fault errors for data accesses
 */
uint32_t SCB_DisableDataFaults();

/**
	@brief Re-enables bus fault errors for data accesses
 */
void SCB_EnableDataFaults(uint32_t faultmask);

///@brief Disable faults via FAULTMASK
extern "C" uint32_t DisableFaults();

///@brief Enable faults via FAULTMASK
extern "C" uint32_t EnableFaults(uint32_t faultmask);

///@brief Invalidate the instruction cache
extern "C" void InvalidateInstructionCache();

///@brief Enable the instruction cache
extern "C" void EnableInstructionCache();

///@brief Invalidate the data cache
extern "C" void InvalidateDataCache();

///@brief Enable the data cache
extern "C" void EnableDataCache();

///@brief Invalidate one section of the cache
void CleanDataCache(void* baseAddr, size_t size);

#endif

