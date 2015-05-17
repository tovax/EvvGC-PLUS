/*
    EvvGC-PLUS - Copyright (C) 2013-2015 Sarunas Vaitekonis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _MISC_H_
#define _MISC_H_

#define constrain(val,min,max)  ((val)<(min)?(min):((val)>(max)?(max):(val)))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  Resets the CRC Data register (DR).
 * @param  None
 * @retval None
 */
static inline void crcResetDR(void) {
  /* Resets CRC generator. */
  CRC->CR = CRC_CR_RESET;
}

/**
 * @brief  Computes the 32-bit CRC of a given buffer of data word(32-bit).
 * @param  pBuf: pointer to the buffer containing the data to be computed
 * @param  length: length of the buffer to be computed
 * @retval 32-bit CRC
 */
static inline uint32_t crcCRC32(const uint32_t pBuf[], uint32_t length) {
  uint32_t i;
  /* Resets CRC generator. */
  CRC->CR = CRC_CR_RESET;
  /* Calculates CRC32 checksum. */
  for(i = 0; i < length; i++) {
    CRC->DR = pBuf[i];
  }
  return CRC->DR;
}

#ifdef __cplusplus
}
#endif

#endif /* _MISC_H_ */
