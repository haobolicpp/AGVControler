// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

/*!
*  Copyright (C) 2018, SICK AG, Waldkirch
*  Copyright (C) 2018, FZI Forschungszentrum Informatik, Karlsruhe, Germany
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
 * \file ChangeCommSettingsCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-09-24
 */
//----------------------------------------------------------------------


#ifndef SICK_SAFETYSCANNERS_COLA2_CHANGECOMMSETTINGSCOMMAND_H
#define SICK_SAFETYSCANNERS_COLA2_CHANGECOMMSETTINGSCOMMAND_H


#include <cola2/MethodCommand.h>
#include <datastructure/CommSettings.h>

namespace sick {
namespace cola2 {


/*!
 * \brief Method command class for changing the settings in the sensor. Such as which data
 * packages to enable.
 */
class ChangeCommSettingsCommand : public MethodCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::MethodCommand base_class;

  /*!
   * \brief Constructor of the Command, takes the current session and the new set of parameters for
   * the sensor.
   *
   * \param session The current Cola2 session, in which the parameters should be transferred.
   * \param settings The new settings for the sensor. While the command will be executed in the
   * session, these parameters will be set in the sensor.
   */
  ChangeCommSettingsCommand(Cola2Session& session, const datastructure::CommSettings& settings);


  /*!
   * \brief Adds the settings as  data to the packetbuffer.
   *
   * \param telegram The telegram, which will be modified the settings as data.
   * \returns Completed telegram
   */
  std::vector<uint8_t> addTelegramData(const std::vector<uint8_t>& telegram) const;


  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;


  /*!
   * \brief Processes the return from the sensor. Checks if the method was acknowledged by the
   * sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();


private:
  sick::datastructure::CommSettings m_settings;

  void writeDataToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeChannelToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeEnabledToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeEInterfaceTypeToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeIPAddresstoDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writePortToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeFrequencyToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeStartAngleToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeEndAngleToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
  void writeFeaturesToDataPtr(std::vector<uint8_t>::iterator data_ptr) const;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_CHANGECOMMSETTINGSCOMMAND_H
