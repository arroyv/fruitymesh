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


#include <cstdlib>


#include "senseModule.h"
#include "Logger.h"
#include "Utility.h"
#include "Node.h"
#include "Config.h"
#include "GlobalState.h"
#include "MeshAccessModule.h"

senseModule::senseModule()
    : Module(SENSE_MODULE_ID, "sense")
{
    //Enable the logtag for our vendor module template
    GS->logger.EnableTag("SENSEMOD");

    isADCInitialized = false;
    this->avgAdcValue = 0;
    number_of_adc_channels = 0;
    //Register callbacks n' stuff
    //Save configuration to base class variables
    //sizeof configuration must be a multiple of 4 bytes
    configurationPointer = &configuration;
    configurationLength = sizeof(senseModuleConfiguration);

    //Set defaults
    ResetToDefaultConfiguration();
}

void senseModule::ResetToDefaultConfiguration()
{
    //Set default configuration values
    configuration.moduleId = moduleId;
    configuration.moduleActive = true;
    configuration.moduleVersion = SENSE_MODULE_CONFIG_VERSION;
    configuration.statusReportingIntervalDs = 0;

    CheckedMemset(nodeMeasurements, 0x00, sizeof(nodeMeasurements));

    SET_FEATURESET_CONFIGURATION(&configuration, this);
}

void senseModule::ConfigurationLoadedHandler(u8* migratableConfig, u16 migratableConfigLength)
{
    //Start the Module...

}

void senseModule::TimerEventHandler(u16 passedTimeDs)
{
    //Peridoic Message sending does not make sense for Assets as they are not connected most of the time.
    //So instead, the asset fully relies on manual querying these messages. Other than "not making sense"
    //this can lead to issues on the Gateway if it receives a messages through a MA-Connection that has a
    //virtual partnerId as the gateway gets confused by the unknown nodeId.
    // if (GET_DEVICE_TYPE() != DeviceType::ASSET)
    // {

    //     if (SHOULD_IV_TRIGGER(GS->appTimerDs + GS->appTimerRandomOffsetDs, passedTimeDs, configuration.statusReportingIntervalDs)) {
    //         SendStatus(NODE_ID_BROADCAST, 0, MessageType::MODULE_ACTION_RESPONSE);
    //     }
    // }
    //Measurement (measure short after reset and then priodically)
    if( (GS->appTimerDs < SEC_TO_DS(40) && Boardconfig->lightIntensityAdcInputPin != -1 )
        || SHOULD_IV_TRIGGER(GS->appTimerDs, passedTimeDs, lightIntensityMeasurementIntervalDs)){
            // logt("ERROR", "In side TimerEventHandler and right before the call to the LightIntensityADC function ");
        LightIntensityADC();
    }

    if (IsPeriodicTimeSendActive() != periodicTimeSendWasActivePreviousTimerEventHandler)
    {
        MeshAccessModule* maMod = (MeshAccessModule*)GS->node.GetModuleById(ModuleId::MESH_ACCESS_MODULE);
        if (maMod != nullptr) {
            maMod->UpdateMeshAccessBroadcastPacket();
        }

        periodicTimeSendWasActivePreviousTimerEventHandler = IsPeriodicTimeSendActive();
        logt("SENSEMOD", "Periodic Time Send is now: %u", (u32)periodicTimeSendWasActivePreviousTimerEventHandler);
    }

    if (IsPeriodicTimeSendActive())
    {
        timeSinceLastPeriodicTimeSendDs += passedTimeDs;
        if(timeSinceLastPeriodicTimeSendDs > TIME_BETWEEN_PERIODIC_TIME_SENDS_DS){
            timeSinceLastPeriodicTimeSendDs = 0;

            constexpr size_t bufferSize = sizeof(ComponentMessageHeaderVendor) + sizeof(GetLightIntensityInfo());
            alignas(u32) u8 buffer[bufferSize];
            CheckedMemset(buffer, 0x00, sizeof(buffer));

            ConnPacketComponentMessageVendor *outPacket = (ConnPacketComponentMessageVendor*)buffer;

            outPacket->componentHeader.header.messageType = MessageType::COMPONENT_SENSE;
            outPacket->componentHeader.header.sender = GS->node.configuration.nodeId;
            outPacket->componentHeader.header.receiver = periodicTimeSendReceiver;

            outPacket->componentHeader.moduleId = SENSE_MODULE_ID;
            outPacket->componentHeader.requestHandle = periodicTimeSendRequestHandle;
            outPacket->componentHeader.actionType = (u8)SensorMessageActionType::READ_RSP;
            outPacket->componentHeader.component = (u16)senseModuleComponent::TIME;
            outPacket->componentHeader.registerAddress = (u16)senseModuleRegister::TIME;
            *(decltype(GetLightIntensityInfo())*)outPacket->payload = GetLightIntensityInfo();
            GS->cm.SendMeshMessage(buffer, bufferSize);
        }
    }
}

//This method sends the node's status over the network
void senseModule::SendStatus(NodeId toNode, u8 requestHandle, MessageType messageType) const
{
    MeshConnections conn = GS->cm.GetMeshConnections(ConnectionDirection::DIRECTION_IN);
    MeshConnectionHandle inConnection;

    for (u32 i = 0; i < conn.count; i++) {
        if (conn.handles[i].IsHandshakeDone()) {
            inConnection = conn.handles[i];
        }
    }

    senseModuleStatusMessage data;

    data.lightIntensityInfo = GetLightIntensityInfo();
    
    SendModuleActionMessage(
        messageType,
        toNode,
        (u8)senseModuleActionResponseMessages::LIGHT_INTENSITY,
        requestHandle,
        (u8*)&data,
        SIZEOF_SENSE_MODULE_STATUS_MESSAGE,
        false
    );
}

#ifdef TERMINAL_ENABLED
TerminalCommandHandlerReturnType senseModule::TerminalCommandHandler(const char* commandArgs[], u8 commandArgsSize)
{
    //modules commands
        // action [nodeId] sense get_light_intensity
        // component_act 546 2882339824 1 0xABCD 0x1234 01
        // component_act 546 2882339824 1 0xABCD 0x1234 00
        // component_act 546 3 1 0xABCD 0x1234 01
        // component_act 546 3 1 0xABCD 0x1234 00
        // component_act 546 2882339824 1 0xABCD 0x1234 01
        // component_act 207 2882339824 1 0xABCD 0x1234 01
        // component_act 142 2882339824 1 0xABCD 0x1234 01
        // component_act 503 2882339824 1 0xABCD 0x1234 01
    //React on commands, return true if handled, false otherwise
    if(commandArgsSize >= 4 && TERMARGS(2, moduleName)) //module name is sense
    {
        if(TERMARGS(0, "action"))
        {
            NodeId destinationNode = Utility::TerminalArgumentToNodeId(commandArgs[1]);

            if(TERMARGS(3, "get_light_intensity"))
            {
                SendModuleActionMessage(
                    MessageType::MODULE_TRIGGER_ACTION,
                    destinationNode,
                    (u8)senseModuleTriggerActionMessages::GET_LIGHT_INTENSITY,
                    0,
                    nullptr,
                    0,
                    false
                );

                return TerminalCommandHandlerReturnType::SUCCESS;
            }
        }
    }

    //Must be called to allow the module to get and set the config
    return Module::TerminalCommandHandler(commandArgs, commandArgsSize);
}
#endif

void senseModule::MeshMessageReceivedHandler(BaseConnection* connection, BaseConnectionSendData* sendData, ConnPacketHeader const * packetHeader)
{
    //Must call superclass for handling
    Module::MeshMessageReceivedHandler(connection, sendData, packetHeader);

    if(packetHeader->messageType == MessageType::MODULE_TRIGGER_ACTION){
        ConnPacketModuleVendor const * packet = (ConnPacketModuleVendor const *)packetHeader;

        //Check if our module is meant and we should trigger an action
        if(packet->moduleId == vendorModuleId){

            //We were queried for our status
            senseModuleTriggerActionMessages actionType = (senseModuleTriggerActionMessages)packet->actionType;
            if(actionType == senseModuleTriggerActionMessages::GET_LIGHT_INTENSITY)
            {
                SendStatus(packet->header.sender, packet->requestHandle, MessageType::MODULE_ACTION_RESPONSE);

            }
        }
    }

    //Parse Module responses
    if(packetHeader->messageType == MessageType::MODULE_ACTION_RESPONSE){

        ConnPacketModuleVendor const * packet = (ConnPacketModuleVendor const *)packetHeader;

        //Check if our module is meant and we should trigger an action
        if(packet->moduleId == vendorModuleId)
        {
            senseModuleActionResponseMessages actionType = (senseModuleActionResponseMessages)packet->actionType;
           if(actionType == senseModuleActionResponseMessages::LIGHT_INTENSITY)
            {
                //Print packet to console
                senseModuleStatusMessage const * data = (senseModuleStatusMessage const *) (packet->data);

                logjson_partial("SENSEMOD", "{\"nodeId\":%u,\"module\":%u,", packet->header.sender, (u32) SENSE_MODULE_ID);
                logjson_partial("SENSEMOD", "\"lightIntensityInfo\":%u,", data->lightIntensityInfo);
                logjson("SENSEMOD", "}" SEP);
            }
        }
    }

    if (packetHeader->messageType == MessageType::COMPONENT_ACT && sendData->dataLength >= SIZEOF_CONN_PACKET_COMPONENT_MESSAGE)
    {
        ConnPacketComponentMessageVendor const * packet = (ConnPacketComponentMessageVendor const *)packetHeader;
        if (packet->componentHeader.moduleId == vendorModuleId)
        {
            if (packet->componentHeader.actionType == (u8)ActorMessageActionType::WRITE)
            {
                if (packet->componentHeader.component == (u16)senseModuleComponent::TIME)
                {
                    if (packet->componentHeader.registerAddress == (u16)senseModuleRegister::TIME)
                    {
                        if (packet->payload[0] != 0)
                        {
                            periodicTimeSendStartTimestampDs = GS->appTimerDs;
                            timeSinceLastPeriodicTimeSendDs = TIME_BETWEEN_PERIODIC_TIME_SENDS_DS;
                            periodicTimeSendReceiver = packet->componentHeader.header.sender;
                            periodicTimeSendRequestHandle = packet->componentHeader.requestHandle;
                        }
                        else
                        {
                            periodicTimeSendStartTimestampDs = 0;
                        }
                    }
                }
            }
        }
    }
}

void senseModule::AdcEventHandler()
{
    // logt("ERROR", "In side AdcEventHandler");
    senseModule * p_senseModule = (senseModule *)GS->node.GetModuleById(SENSE_MODULE_ID);
    if (p_senseModule == nullptr) return;
    p_senseModule->AvgADCValue();
    FruityHal::AdcUninit();
}

void senseModule::InitLightIntensityADC() {
    // logt("ERROR", "In side InitLightIntensityADC");
    if(Boardconfig->lightIntensityAdcInputPin == -1 || isADCInitialized){
        // logt("ERROR", "  battry pin is == -1: %u", (u32)Boardconfig->lightIntensityAdcInputPin);
        // logt("ERROR", "  ADC is initialized?: %u", (u32)isADCInitialized);
        return;
    }
    ErrorType error = FruityHal::AdcInit(AdcEventHandler);
    FRUITYMESH_ERROR_CHECK((u32)error);

    u32 pin = Boardconfig->lightIntensityAdcInputPin;

    ErrorType err = ErrorType::SUCCESS;
#if FEATURE_AVAILABLE(ADC_INTERNAL_MEASUREMENT)
    // logt("ERROR", "  ADC_INTERNAL_MEASUREMENT_AVAILABLE");
    // logt("ERROR", "   AdcReference::ADC_REFERENCE_0_6V, AdcResoultion::ADC_10_BIT, AdcGain::ADC_GAIN_1_6");
    err = FruityHal::AdcConfigureChannel(pin,
                                    FruityHal::AdcReference::ADC_REFERENCE_0_6V, 
                                    FruityHal::AdcResoultion::ADC_10_BIT, 
                                    FruityHal::AdcGain::ADC_GAIN_1_6);
#else
    // logt("ERROR", "  ADC_INTERNAL_MEASUREMENT_NOT_AVAILABLE");
    // logt("ERROR", "  AdcReference::ADC_REFERENCE_1_2V, AdcResoultion::ADC_8_BIT, AdcGain::ADC_GAIN_1");
    err = FruityHal::AdcConfigureChannel(pin, 
                                   FruityHal::AdcReference::ADC_REFERENCE_1_2V, 
                                   FruityHal::AdcResoultion::ADC_8_BIT, 
                                   FruityHal::AdcGain::ADC_GAIN_1);
#endif // FEATURE_AVAILABLE(ADC_INTERNAL_MEASUREMENT)
    if (err != ErrorType::SUCCESS)
    {
        logt("LIGHT_INTENSITY", "Failed to configure adc because %u", (u32)err);
        // logt("ERROR",  "Failed to configure adc because %u", (u32)err);
    }
    isADCInitialized = true;
}
void senseModule::LightIntensityADC(){
    // logt("ERROR", "Inside LightIntensityADC function");
    // logt("ERROR", "Boardconfig->lightIntensityAdcInputPin: %u", (u32)Boardconfig->lightIntensityAdcInputPin);
    InitLightIntensityADC();
    //Check if initialization did work
    if(!isADCInitialized || Boardconfig->lightIntensityAdcInputPin == -1) return;

#ifndef SIM_ENABLED
    // logt("ERROR", "SIM_ENABLED");
    if (Boardconfig->lightIntensityAdcInputPin >= 0) {
        FruityHal::GpioConfigureOutput(Boardconfig->lightIntensityMeasurementEnablePin);
        FruityHal::GpioPinSet(Boardconfig->lightIntensityMeasurementEnablePin);
    }
    
    ErrorType err = FruityHal::AdcSample(*m_buffer, 1);
    FRUITYMESH_ERROR_CHECK((u32)err);

    isADCInitialized = false;
#endif
}

void senseModule::AvgADCValue()
{
    // logt("ERROR", "In side AvgADCValue");
    u32 adc_sum_value = 0;
    // logt("ERROR", "LIGHT_INTENSITY_SAMPLES_IN_BUFFER: %u", (u32)LIGHT_INTENSITY_SAMPLES_IN_BUFFER);
    for (u16 i = 0; i < LIGHT_INTENSITY_SAMPLES_IN_BUFFER; i++) {
        //Buffer implemented for future use
        adc_sum_value += m_buffer[i];               //Sum all values in ADC buffer
    }
    // logt("ERROR", "adc_sum_value: %u", (u32)adc_sum_value);

    avgAdcValue = (adc_sum_value / LIGHT_INTENSITY_SAMPLES_IN_BUFFER);
    //Convert Adc to lumens
    // pointed a iphone x directly at the photoresistor (max 50 lumnes and got a reading of 150 max)
    //covered teh photoresistor and got a min value of 2
    //derived this formula y = 0.5x-0.5
    double temp = (0.5) * avgAdcValue - (0.5);
    avgAdcValue = (u8) temp; 

}

bool senseModule::IsPeriodicTimeSendActive()
{
    return periodicTimeSendStartTimestampDs != 0 && GS->appTimerDs < periodicTimeSendStartTimestampDs + PERIODIC_TIME_SEND_AUTOMATIC_DEACTIVATION;
}

u8 senseModule::GetLightIntensityInfo() const
{   
    // logt("ERROR", "In side GetLightIntensityInfo");
    // logt("ERROR", "avgAdcValue: %u", (u32)avgAdcValue);
    return avgAdcValue;
}
