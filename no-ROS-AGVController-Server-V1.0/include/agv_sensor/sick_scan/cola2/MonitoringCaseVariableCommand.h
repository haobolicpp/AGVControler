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
 * \file MonitoringCaseVariableCommand.h
 *
 * \author  Lennart Puck <puck@fzi.de>
 * \date    2018-11-28
 */
//----------------------------------------------------------------------

#ifndef SICK_SAFETYSCANNERS_COLA2_MONITORINGCASEVARIABLECOMMAND_H
#define SICK_SAFETYSCANNERS_COLA2_MONITORINGCASEVARIABLECOMMAND_H


#include <cola2/VariableCommand.h>
#include <data_processing/ParseMonitoringCaseData.h>
#include <datastructure/CommSettings.h>

namespace sick {
namespace cola2 {

/*!
 * \brief Command to read the header of the monitoring case table.
 */
class MonitoringCaseVariableCommand : public VariableCommand
{
public:
  /*!
   * \brief Typedef to reference the base class.
   */
  typedef sick::cola2::VariableCommand base_class;


  /*!
   * \brief Constructor of the command.
   *
   * \param session The current cola2 session.
   * \param monitoring_case_data Reference to the monitoring case data.
   * \param index Index of the variable to ask from the sensor, in range [0-254]
   *
   */
  MonitoringCaseVariableCommand(Cola2Session& session,
                                datastructure::MonitoringCaseData& monitoring_case_data,
                                const uint16_t& index);

  /*!
   * \brief Returns if the command can be executed without a session ID. Will return false for most
   * commands except the commands to establish a connection.
   *
   * \returns If the command needs a session ID to be executed.
   */
  bool canBeExecutedWithoutSessionID() const;

  /*!
   * \brief Processes the return from the sensor.
   *
   * \returns If processing of the returned data was successful.
   */
  bool processReply();


private:
  std::shared_ptr<sick::data_processing::ParseMonitoringCaseData> m_monitoring_case_parser_ptr;

  sick::datastructure::MonitoringCaseData& m_monitoring_case_data;
};

} // namespace cola2
} // namespace sick

#endif // SICK_SAFETYSCANNERS_COLA2_MONITORINGCASEVARIABLECOMMAND_H
