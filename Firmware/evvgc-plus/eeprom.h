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

#ifndef _EEPROM_H_
#define _EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif
uint8_t eepromLoadSettings(void);
uint8_t eepromSaveSettings(void);
uint8_t eepromContinueSaving(void);
uint8_t eepromIsDataLeft(void);
#ifdef __cplusplus
}
#endif

#endif /* _EEPROM_H_ */
