/* Network interface defaults
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "netsocket/NetworkInterface.h"

#include "netsocket/EthInterface.h"

/* Weak default instance static classes for the various abstract classes.
 * Applications can override these.
 */

MBED_WEAK EthInterface *EthInterface::get_default_instance()
{
    return get_target_default_instance();
}

/* For other types, we can provide a reasonable get_target_default_instance
 * in some cases. This is done in EthernetInterface.cpp, mbed-mesh-api and
 * OnboardCellularInterface.cpp. We have no implementation for WiFi, so a
 * default empty one lives here.
 */


/* The top-level get_default_instance() call. Weak for application override. */
MBED_WEAK NetworkInterface *NetworkInterface::get_default_instance()
{
    return get_target_default_instance();
}


/* Helpers to set default parameters - used by NetworkInterface::get_default_instance,
 * but exposed for apps which want to get these defaults after requesting a specific type.
 */
void NetworkInterface::set_default_parameters()
{

}

/* Finally the dispatch from the JSON default interface type to the specific
 * subclasses. It's our job to configure - the default NetworkInterface is
 * preconfigured - the specific subtypes' defaults are not (necessarily).
 */
#define ETHERNET 1
#define MESH 3
#if MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE == ETHERNET
MBED_WEAK NetworkInterface *NetworkInterface::get_target_default_instance()
{
    return EthInterface::get_default_instance();
}
#elif defined(MBED_CONF_TARGET_NETWORK_DEFAULT_INTERFACE_TYPE)
/* If anyone invents a new JSON value, they must have their own default weak
 * implementation.
 */
#else
/* When the default type is null */
MBED_WEAK NetworkInterface *NetworkInterface::get_target_default_instance()
{
    return NULL;
}
#endif
