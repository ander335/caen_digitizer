//
// Created by user on 09.03.16.
//

#pragma once

#include "../flug/src/kernel/drivers/DeviceDriver.h"
#include <CAENDigitizer.h>
#include <CAENDigitizerType.h>


namespace SciKit {
    class CaenDigitizer : public DeviceDriver {
    public:
        CaenDigitizer() = delete;
        CaenDigitizer(const std::string & deviceInstanceName, const std::string & devType);

        virtual ~CaenDigitizer();


        virtual bool initModule ();
        virtual bool destroyModule ();
        virtual bool handleRequest (Request & req, Response & resp);
        virtual bool loadConfig (Json::Value & config);
        virtual State getState ();

        bool handleHardDownloadData (Request & req, Response & resp);

        bool handleSetPostTriggerPercent(Request &req, Response &resp);
        bool handleGetPostTriggerPercent(Request &req, Response &resp);

        bool handleSetChannelsDcOffset (Request & req, Response & resp);
        bool handleGetChannelsDcOffset (Request & req, Response & resp);

        bool handleSetDigitizerFreq (Request & req, Response & resp);
        bool handleGetDigitizerFreq (Request & req, Response & resp);

        bool handleStartAcquisition (Request & req, Response & resp);
        bool handleStopAcquisition (Request & req, Response & resp);
        bool handleSetMaxEvents (Request & req, Response & resp);
        bool handleGetData (Request & req, Response & resp);

        bool handleSetTriggerConfig (Request & req, Response & resp);
        bool handleGetTriggerConfig (Request & req, Response & resp);

    protected:

        void writeRegister (uint32_t reg, uint32_t data);
        uint32_t readRegister (uint32_t reg);
        void getDeviceInfo ();
        void clearData ();
        void setMaxEvents (uint32_t num);
        void mallocReadoutBuffer ();
        void readData ();
        void drsCorrection (bool enble = true);
        void parseEvent (CAEN_DGTZ_X742_EVENT_t & evt);
        void enableGroups ();
        void readCorrectionTable ();
        void setFrequency (CAEN_DGTZ_DRS4Frequency_t freq);
        CAEN_DGTZ_DRS4Frequency_t getFrequency ();

        void setTriggerLevel (float level);

        void configDigitizer ();


        void loadCorrectionTables();

        void reset ();
    private:
        int m_handle;
        bool m_applyCorrection;
        CAEN_DGTZ_DRS4Correction_t m_correction[MAX_X742_GROUP_SIZE];
        CAEN_DGTZ_BoardInfo_t m_info;
        CAEN_DGTZ_DRS4Frequency_t m_freq;
        int m_channles;
        int m_channelGroups;
        char * m_readoutBuffer;
        uint32_t m_readoutBufferSize;

        std::vector<std::vector<float>> m_data;
        std::vector<uint32_t> m_startingPoints;

        float m_chanMul[16];
        float m_chanAdd[16];
        float m_trMul;
        float m_trAdd;

        uint32_t m_trDcOffset;
        uint32_t m_trTreshold;

    };


    extern "C" {
        int32_t X742_DecodeEvent(char *evtPtr, void **Evt);
        int32_t GetEventPtr(char *buffer, uint32_t buffsize,
                            int32_t numEvent, char **EventPtr);
        int32_t GetNumEvents(char *buffer, uint32_t buffsize, uint32_t *numEvents);
        int V1742UnpackEventGroup(uint32_t group, uint32_t *datain, CAEN_DGTZ_X742_GROUP_t *dataout);
        int32_t getNumberOfBits(uint8_t byte);
        int LoadCorrectionTable(char *baseInputFileName, CAEN_DGTZ_DRS4Correction_t *tb);
        int SaveCorrectionTables(char *outputFileName, uint32_t groupMask, CAEN_DGTZ_DRS4Correction_t *tables);
        void ApplyDataCorrection(CAEN_DGTZ_DRS4Correction_t *CTable, CAEN_DGTZ_DRS4Frequency_t frequency,
                                 int CorrectionLevelMask, CAEN_DGTZ_X742_GROUP_t *data);
        void PeakCorrection(CAEN_DGTZ_X742_GROUP_t *dataout);
    }

}

