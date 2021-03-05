////////////////////////////////////////////////////////////////////////////////
// /****************************************************************************
// **
// ** Copyright (C) 2015-2021 M-Way Solutions GmbH
// ** Contact: https://www.blureange.io/licensing
// **
// ** This file is part of the Bluerange/FruityMesh implementation
// **
// ** $BR_BEGIN_LICENSE:GPL-EXCEPT$
// ** Commercial License Usage
// ** Licensees holding valid commercial Bluerange licenses may use this file in
// ** accordance with the commercial license agreement provided with the
// ** Software or, alternatively, in accordance with the terms contained in
// ** a written agreement between them and M-Way Solutions GmbH.
// ** For licensing terms and conditions see https://www.bluerange.io/terms-conditions. For further
// ** information use the contact form at https://www.bluerange.io/contact.
// **
// ** GNU General Public License Usage
// ** Alternatively, this file may be used under the terms of the GNU
// ** General Public License version 3 as published by the Free Software
// ** Foundation with exceptions as appearing in the file LICENSE.GPL3-EXCEPT
// ** included in the packaging of this file. Please review the following
// ** information to ensure the GNU General Public License requirements will
// ** be met: https://www.gnu.org/licenses/gpl-3.0.html.
// **
// ** $BR_END_LICENSE$
// **
// ****************************************************************************/
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <Module.h>
#include <Logger.h>

#include <Terminal.h>

//This should be set to the correct vendor and subId
constexpr VendorModuleId SENSE_MODULE_ID = GET_VENDOR_MODULE_ID(0xABCD, 3);
constexpr u8 SENSE_MODULE_CONFIG_VERSION = 1;
constexpr u16 SENSE_MODULE_MAX_HOPS = NODE_ID_HOPS_BASE + NODE_ID_HOPS_BASE_SIZE - 1;

constexpr size_t LIGHT_INTENSITY_SAMPLES_IN_BUFFER = 1; //Number of SAADC samples in RAM before returning a SAADC event. For low power SAADC set this constant to 1. Otherwise the EasyDMA will be enabled for an extended time which consumes high current.

enum class senseModuleComponent :u16 {
    TIME = 0xABCD,
};

enum class senseModuleRegister : u16 {
    TIME = 0x1234,
};


#pragma pack(push, 1)
//Module configuration that is saved persistently
struct senseModuleConfiguration: ModuleConfiguration
{
        u16 statusReportingIntervalDs;
};
#pragma pack(pop)


class senseModule: public Module
{
public:
        
        static constexpr u32 lightIntensityMeasurementIntervalDs = SEC_TO_DS(6);

        enum class senseModuleTriggerActionMessages : u8
        {
            GET_LIGHT_INTENSITY = 0
        };

        enum class senseModuleActionResponseMessages : u8
        {
            LIGHT_INTENSITY = 0
        };

private:

        //####### Module specific message structs (these need to be packed)
        #pragma pack(push)
        #pragma pack(1)

        typedef struct
            {
                NodeId nodeId;
            } nodeMeasurement;

            //This message delivers often changing information and info about the incoming connection
            static constexpr int SIZEOF_SENSE_MODULE_STATUS_MESSAGE = 1;
            typedef struct
            {
                u8 lightIntensityInfo;
            } senseModuleStatusMessage;
            STATIC_ASSERT_SIZE(senseModuleStatusMessage, 1);


        #pragma pack(pop)

        //####### Module messages end

        static constexpr int NUM_NODE_MEASUREMENTS = 20;
        nodeMeasurement nodeMeasurements[NUM_NODE_MEASUREMENTS];

        u8 avgAdcValue;
        bool isADCInitialized;
        u8 number_of_adc_channels;
        i16 m_buffer[LIGHT_INTENSITY_SAMPLES_IN_BUFFER];

        void SendStatus(NodeId toNode, u8 requestHandle, MessageType messageType) const;
        void SendErrors(NodeId toNode, u8 requestHandle) const;
        

        static void AdcEventHandler();
        void InitLightIntensityADC();
        void LightIntensityADC();
        void AvgADCValue();

        bool periodicTimeSendWasActivePreviousTimerEventHandler = false;
        u32 periodicTimeSendStartTimestampDs = 0;
        constexpr static u32 PERIODIC_TIME_SEND_AUTOMATIC_DEACTIVATION = SEC_TO_DS(/*10 minutes*/ 10 * 60);
        constexpr static u32 TIME_BETWEEN_PERIODIC_TIME_SENDS_DS = SEC_TO_DS(/*5 seconds*/5);
        u32 timeSinceLastPeriodicTimeSendDs = 0;
        NodeId periodicTimeSendReceiver = 0;
        decltype(ComponentMessageHeader::requestHandle) periodicTimeSendRequestHandle = 0;
        bool IsPeriodicTimeSendActive();

    public:

        static constexpr int SIZEOF_SENSE_MODULE_CONNECTIONS_MESSAGE = 12;
        DECLARE_CONFIG_AND_PACKED_STRUCT(senseModuleConfiguration);
        senseModule();
        void ConfigurationLoadedHandler(u8* migratableConfig, u16 migratableConfigLength) override final;
        void ResetToDefaultConfiguration() override final;
        void TimerEventHandler(u16 passedTimeDs) override final;
        #ifdef TERMINAL_ENABLED
        TerminalCommandHandlerReturnType TerminalCommandHandler(const char* commandArgs[], u8 commandArgsSize) override final;
        #endif
        void MeshMessageReceivedHandler(BaseConnection* connection, BaseConnectionSendData* sendData, ConnPacketHeader const * packetHeader) override final;
        u8 GetLightIntensityInfo() const;
};

