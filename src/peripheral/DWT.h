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

#ifndef dwt_h
#define dwt_h

/**
	@brief The CoreSight Data Watchpoint and Trace block
 */
class DWT
{
public:

	enum SampleType
	{
		PC_SAMPLE_FAST,
		PC_SAMPLE_SLOW
	};

	/**
		@brief Enable PC sampling over trace

		If cyctapFast is set, POSTCNT tap at CYCCNT[6]
		If not set, POSTCNT tap at CYCCNT[10]
	 */
	static void EnablePCSampling(DWT::SampleType sampleType)
	{
		uint32_t tapsel = 0;
		if(sampleType == PC_SAMPLE_SLOW)
			tapsel = DWT_CTRL_CYCTAP;

		_DWT.CTRL |=
			DWT_CTRL_CYCEVTENA |	//enable POSTCNT underflow event counter packets
			DWT_CTRL_PCSAMPLENA	|	//enable POSTCNT periodic PC packet sampling
			tapsel |
			DWT_CTRL_CYCCNTENA;		//enable cycle counting
	}
};

#endif
