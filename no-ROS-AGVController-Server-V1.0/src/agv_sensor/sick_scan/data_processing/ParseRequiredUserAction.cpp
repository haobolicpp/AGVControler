// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2019, SICK AG, Waldkirch
*  Copyright (C) 2019, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.

*/

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file ParseRequiredUserAction.cpp
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2019-07-22
 */
//----------------------------------------------------------------------

#include <data_processing/ParseRequiredUserAction.h>

#include <cola2/Command.h>

namespace sick {
namespace data_processing {

ParseRequiredUserActionData::ParseRequiredUserActionData() {}


bool ParseRequiredUserActionData::parseTCPSequence(
  const datastructure::PacketBuffer& buffer,
  sick::datastructure::RequiredUserAction& required_user_action) const
{
  // Keep our own copy of the shared_ptr to keep the iterators valid
  const std::shared_ptr<std::vector<uint8_t> const> vec_ptr = buffer.getBuffer();
  std::vector<uint8_t>::const_iterator data_ptr             = vec_ptr->begin();
  readRequiredUserAction(data_ptr, required_user_action);
  return true;
}

bool ParseRequiredUserActionData::readRequiredUserAction(
  std::vector<uint8_t>::const_iterator data_ptr,
  sick::datastructure::RequiredUserAction& required_user_action) const
{
  uint16_t word = read_write_helper::readUint16LittleEndian(data_ptr + 0);

  required_user_action.setConfirmConfiguration(static_cast<bool>(word & (0x01 << 0)));
  required_user_action.setCheckConfiguration(static_cast<bool>(word & (0x01 << 1)));
  required_user_action.setCheckEnvironment(static_cast<bool>(word & (0x01 << 2)));
  required_user_action.setCheckApplicationInterfaces(static_cast<bool>(word & (0x01 << 3)));
  required_user_action.setCheckDevice(static_cast<bool>(word & (0x01 << 4)));
  required_user_action.setRunSetupProcedure(static_cast<bool>(word & (0x01 << 5)));
  required_user_action.setCheckFirmware(static_cast<bool>(word & (0x01 << 6)));
  required_user_action.setWait(static_cast<bool>(word & (0x01 << 7)));
  return true;
}

} // namespace data_processing
} // namespace sick
