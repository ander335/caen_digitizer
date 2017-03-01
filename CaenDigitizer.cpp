//
// Created by user on 09.03.16.
//

#include <CAENDigitizerType.h>
#include "CaenDigitizer.h"

#define DEFAULT_CAEN_BUF_SIZE (10 * 1024 * 1024)

#define EVENT_HEADER_SIZE        0x10
#define X742_MAX_GROUPS            0x04
#define X742_FIXED_SIZE            0x400

FLUG_DYNAMIC_DRIVER(SciKit::CaenDigitizer);

namespace SciKit {


#define MAX_READ_CHAR               1000
#define MAX_BASE_INPUT_FILE_LENGTH  1000


    CaenDigitizer::CaenDigitizer(const std::string &deviceInstanceName,
                                 const std::string &devType)
            : DeviceDriver(deviceInstanceName, devType) {

    }

    CaenDigitizer::~CaenDigitizer() {

    }


    bool CaenDigitizer::initModule() {
        m_channles = 0x8;
        m_channelGroups = 0x2;

        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_OpenDigitizer(CAEN_DGTZ_USB, 0, 0, 0, &m_handle);
        if (err) {
            throw std::runtime_error("Failed to open USB connection to CAEN Digitizer");
        }

        configDigitizer();

        clearData();
        getDeviceInfo();
        enableGroups();
        setMaxEvents(1);
        getFrequency();
        drsCorrection(m_applyCorrection);
        mallocReadoutBuffer();
        setTriggerLevel(2000);
        loadCorrectionTables();

        configDigitizer();

        return true;
    }

    bool CaenDigitizer::destroyModule() {
        CAEN_DGTZ_FreeReadoutBuffer(&m_readoutBuffer);
        CAEN_DGTZ_CloseDigitizer(m_handle);
        return true;
    }

    bool CaenDigitizer::handleRequest(Request &req, Response &resp) {
        //throw std::runtime_error("Not implemented");
        std::string reqtype = req.m_json["reqtype"].asString();
        Json::Value root;

        if (reqtype == "downloadData") {
            return handleHardDownloadData(req, resp);
        } else if (reqtype == "setFrequency") {
            return handleSetDigitizerFreq(req, resp);
        } else if (reqtype == "setPostTrigger") {
            return handleSetPostTriggerPercent(req, resp);
        } else if (reqtype == "getPostTrigger") {
            return handleGetPostTriggerPercent(req, resp);
        } else if (reqtype == "setFreq") {
            return handleSetDigitizerFreq(req, resp);
        } else if (reqtype == "getFreq") {
            return handleGetDigitizerFreq(req, resp);
        }

        return false;
    }

    void CaenDigitizer::writeRegister(uint32_t reg, uint32_t data) {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_WriteRegister(m_handle, reg, data);
        if (err) {
            throw std::runtime_error("Error while writing value to register");
        }
    }

    uint32_t CaenDigitizer::readRegister(uint32_t reg) {
        uint32_t data;
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_ReadRegister(m_handle, reg, &data);
        if (err) {
            throw std::runtime_error("Error while read value to register");
        }
        return data;
    }


    void CaenDigitizer::reset() {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_Reset(m_handle);
        if (err) {
            throw std::runtime_error("Failed to reset");
        }
    }

    void CaenDigitizer::getDeviceInfo() {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_GetInfo(m_handle, &m_info);
        if (err) {
            throw std::runtime_error("Failed to get info");
        }

        //m_channles = m_info.Channels;

        std::cout << "CAEN Model: " << m_info.ModelName << std::endl;
        std::cout << "CAEN Channles: " << m_info.Channels << std::endl;
    }

    void CaenDigitizer::clearData() {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_ClearData(m_handle);
        if (err) {
            throw std::runtime_error("Can't clear digitizer data");
        }
    }

    void CaenDigitizer::setMaxEvents(uint32_t num) {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_SetMaxNumEventsBLT(m_handle, num);
        if (err) {
            throw std::runtime_error("Failed to set max. number of events");
        }
    }


    void CaenDigitizer::mallocReadoutBuffer() {
        CAEN_DGTZ_ErrorCode err;
        m_readoutBuffer = NULL;
        m_readoutBufferSize = DEFAULT_CAEN_BUF_SIZE;
        err = CAEN_DGTZ_MallocReadoutBuffer(m_handle, &m_readoutBuffer,
                                            &m_readoutBufferSize);
        if (err) {
            throw std::runtime_error("Can't allocate readout buffer");
        }
    }


    void CaenDigitizer::readData() {
        CAEN_DGTZ_ErrorCode err;
        uint32_t readSize = m_readoutBufferSize;
        err = CAEN_DGTZ_ReadData(m_handle, CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
                                 m_readoutBuffer, &readSize);
        if (err) {
            throw std::runtime_error("Can't read data to readout buffer");
        }

        int32_t ret;
        uint32_t numEvents;
        ret = GetNumEvents(m_readoutBuffer, readSize, &numEvents);
        if (ret) {
            throw std::runtime_error("Can't get number of events in readout buffer");
        }

        if (numEvents == 0) {
            throw std::runtime_error("Got zero events");
        }

        char *evPtr = NULL;
        ret = GetEventPtr(m_readoutBuffer, readSize,
                          0, &evPtr);
        if (ret) {
            throw std::runtime_error("Can't get event info in readout buffer");
        }


        CAEN_DGTZ_X742_EVENT_t *evt = NULL;
        ret = X742_DecodeEvent(evPtr, (void **) &evt);
        if (ret) {
            throw std::runtime_error("Can't decode event");
        }

        parseEvent(*evt);

        CAEN_DGTZ_FreeEvent(m_handle, (void **) &evt);

    }

    void CaenDigitizer::parseEvent(CAEN_DGTZ_X742_EVENT_t &evt) {
        //m_data.resize(0x10);
        std::cout << "Channels count: " << m_channles << std::endl;
        m_data.clear();
        for (int gr = 0; gr < MAX_X742_GROUP_SIZE; gr++) {
            std::cout << "New group" << std::endl;
            if (evt.GrPresent[gr]) {
                ApplyDataCorrection(&(m_correction[gr]), m_freq, 0x3, &(evt.DataGroup[gr]));
                for (int chan = 0; chan < m_channles; chan++) {
                    int chanSize = evt.DataGroup[gr].ChSize[chan];
                    std::cout << "Chan size: " << chanSize << std::endl;
                    std::cout << "First value: " <<
                    evt.DataGroup[gr].DataChannel[chan][0] << std::endl;
                    //m_data[chan + gr * m_channelGroups].clear();
                    m_data.push_back(
                            std::vector<float>(
                                    evt.DataGroup[gr].DataChannel[chan],
                                    evt.DataGroup[gr].DataChannel[chan] + chanSize)
                    );
                }
            }
        }
    }

    void CaenDigitizer::drsCorrection(bool enble) {
        CAEN_DGTZ_ErrorCode err;
        if (enble) {
            err = CAEN_DGTZ_EnableDRS4Correction(m_handle);
        } else {
            err = CAEN_DGTZ_DisableDRS4Correction(m_handle);
        }

        if (err) {
            throw std::runtime_error("Failed to enable/disable DRS4 correction");
        }
    }


    bool CaenDigitizer::handleHardDownloadData(Request &req, Response &resp) {
        Json::Value root;


        CAEN_DGTZ_SWStartAcquisition(m_handle);
        CAEN_DGTZ_SendSWtrigger(m_handle);
        usleep(10);
        CAEN_DGTZ_SWStopAcquisition(m_handle);

        readData();

        for (int chan = 0; chan < m_channles * m_channelGroups; chan++) {
            int i = 0;
            for (auto iter: m_data[chan]) {
                root["data"][chan][i++] = iter * m_chanMul[chan] + m_chanAdd[chan];
            }
        }


        root["status"] = "success";
        resp = root;
        return true;
    }

    void CaenDigitizer::enableGroups() {
        CAEN_DGTZ_ErrorCode err;
        uint32_t grEn;
        err = CAEN_DGTZ_GetGroupEnableMask(m_handle, &grEn);
        std::cout << "Enabled bits: " << grEn << std::endl;
        err = CAEN_DGTZ_SetGroupEnableMask(m_handle, 0x3);
        if (err) {
            throw std::runtime_error("Failed to set groups enable mask");
        }
    }

    void CaenDigitizer::readCorrectionTable() {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_LoadDRS4CorrectionData(m_handle, m_freq);
        if (err) {
            throw std::runtime_error("Failed to load DRS4 correction tables");
        }
    }

    void CaenDigitizer::setFrequency(CAEN_DGTZ_DRS4Frequency_t freq) {
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_SetDRS4SamplingFrequency(m_handle, freq);
        if (err) {
            throw std::runtime_error("Failed to set DRS4 samp. frequency");
        }
        m_freq = freq;
    }

    CAEN_DGTZ_DRS4Frequency_t CaenDigitizer::getFrequency() {
        CAEN_DGTZ_DRS4Frequency_t freq;
        CAEN_DGTZ_ErrorCode err;
        err = CAEN_DGTZ_GetDRS4SamplingFrequency(m_handle, &freq);
        if (err) {
            throw std::runtime_error("Failed to get DRS4 samp. frequency");
        }
        m_freq = freq;
        return freq;
    }

    Module::State CaenDigitizer::getState() {
        return ST_ONLINE;
    }

    void CaenDigitizer::loadCorrectionTables() {
        int ret = CAEN_DGTZ_EnableDRS4Correction(m_handle);
        if (ret != CAEN_DGTZ_Success) {
            throw std::runtime_error("DRS4 correction failed");
        }

        ret = CAEN_DGTZ_GetCorrectionTables(m_handle, m_freq, (void *) m_correction);
        if (ret != CAEN_DGTZ_Success) {
            throw std::runtime_error("Failed to load correction tables from device");
        }
    }

    bool CaenDigitizer::loadConfig(Json::Value &config) {
        if (config.isMember("apply_correction") && config["apply_correction"].isBool()) {
            m_applyCorrection = config["apply_correction"].asBool();
        } else {
            m_applyCorrection = false;
        }

        for (int chan = 0; chan < 16; chan++) {
            m_chanMul[chan] = config["user_calibration"][chan][0].asFloat();
            m_chanAdd[chan] = config["user_calibration"][chan][1].asFloat();
        }

        m_trMul = config["trigger_calibration"][0].asFloat();
        m_trAdd = config["trigger_calibration"][1].asFloat();

        m_trDcOffset = config["tr0_dcOffset"].asUInt();
        m_trTreshold = config["tr0_treshold"].asUInt();

        return true;
    }

    void CaenDigitizer::setTriggerLevel(float level) {
        int ret = 0;

        ret |= CAEN_DGTZ_SetFastTriggerDigitizing(m_handle, CAEN_DGTZ_ENABLE);
        ret |= CAEN_DGTZ_SetFastTriggerMode(m_handle, CAEN_DGTZ_TRGMODE_ACQ_AND_EXTOUT);

        ret = CAEN_DGTZ_SetPostTriggerSize(m_handle, 50);
        if (ret) {
            throw std::runtime_error("Failed to set post-trigger percent");
        }

        for (uint32_t gr = 0; gr < m_channelGroups; gr++) {
            uint32_t defOffset;
            ret |= CAEN_DGTZ_SetGroupFastTriggerDCOffset(m_handle, gr, 2000);
            ret |= CAEN_DGTZ_GetGroupFastTriggerDCOffset(m_handle, gr, &defOffset);
            std::cout << "Default trig offset: " << defOffset << std::endl;
            ret |= CAEN_DGTZ_SetGroupFastTriggerThreshold(m_handle, gr, level);
            if (ret) {
                throw std::runtime_error("Failed to set group trigger level");
            }
        }

        CAEN_DGTZ_SetDRS4SamplingFrequency(m_handle, CAEN_DGTZ_DRS4_5GHz);
        ret |= CAEN_DGTZ_SetExtTriggerInputMode(m_handle, CAEN_DGTZ_TRGMODE_ACQ_AND_EXTOUT);
        if (ret) {
            throw std::runtime_error("shit");
        }

    }


    void CaenDigitizer::configDigitizer() {

        int i, j, ret = 0;


        ret |= CAEN_DGTZ_SetIOLevel(m_handle, CAEN_DGTZ_IOLevel_TTL);
        ret |= CAEN_DGTZ_SetMaxNumEventsBLT(m_handle, 1);
        ret |= CAEN_DGTZ_SetAcquisitionMode(m_handle, CAEN_DGTZ_SW_CONTROLLED);
        ret |= CAEN_DGTZ_SetGroupEnableMask(m_handle, 0x3);
        ret |= CAEN_DGTZ_SetRecordLength(m_handle, 1024);
        ret |= CAEN_DGTZ_SetPostTriggerSize(m_handle, 80);
        for (i = 0; i < 2; i++) {
            ret |= CAEN_DGTZ_SetTriggerPolarity(m_handle, i, CAEN_DGTZ_TriggerOnRisingEdge);
        }
        ret |= CAEN_DGTZ_SetFastTriggerDigitizing(m_handle, CAEN_DGTZ_ENABLE);
        ret |= CAEN_DGTZ_SetFastTriggerMode(m_handle, CAEN_DGTZ_TRGMODE_ACQ_ONLY);
        ret |= CAEN_DGTZ_SetDRS4SamplingFrequency(m_handle, CAEN_DGTZ_DRS4_5GHz);
        for (i = 0; i < 2; i++) {
            ret |= CAEN_DGTZ_SetGroupFastTriggerDCOffset(m_handle, i, m_trDcOffset);
            ret |= CAEN_DGTZ_SetGroupFastTriggerThreshold(m_handle, i, m_trTreshold);
        }


        if (ret)
            throw std::runtime_error(
                    "[WARNING]: errors found during the programming of the digitizer.\nSome settings may not be executed");

        loadCorrectionTables();
    }

    bool CaenDigitizer::handleSetPostTriggerPercent(Request &req, Response &resp) {
        Json::Value root;

        uint32_t perc = req.m_json["percent"].asUInt();
        int ret = CAEN_DGTZ_SetPostTriggerSize(m_handle, perc);
        if (ret != CAEN_DGTZ_Success)
            throw std::runtime_error("Failed to set pre-trig percent size");

        root["status"] = "success";
        resp = root;
        return true;
    }


    bool CaenDigitizer::handleGetPostTriggerPercent(Request &req, Response &resp) {
        Json::Value root;

        uint32_t perc;
        int ret = CAEN_DGTZ_GetPostTriggerSize(m_handle, &perc);
        if (ret != CAEN_DGTZ_Success)
            throw std::runtime_error("Failed to get pre-trig percent size");

        root["status"] = "success";
        root["percent"] = perc;
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleSetTriggerConfig(Request &req, Response &resp) {
        Json::Value root;

        float trLvl = req.m_json["level"].asFloat();
        float trOffset = req.m_json["offset"].asFloat();
        int ret = CAEN_DGTZ_Success;
        for (uint32_t ch = 0; ch < 2; ch++) {
            ret |= CAEN_DGTZ_SetGroupFastTriggerDCOffset(m_handle, ch,
                                                         std::max((uint32_t) 0, std::min((uint32_t) 65535,
                                                                                         (uint32_t) (
                                                                                                 (trOffset - m_trAdd) /
                                                                                                 m_trMul))));
            ret |= CAEN_DGTZ_SetGroupFastTriggerThreshold(m_handle, ch,
                                                          std::max((uint32_t) 0, std::min((uint32_t) 65535,
                                                                                          (uint32_t) (
                                                                                                  (trLvl - m_trAdd) /
                                                                                                  m_trMul))));
            if (req.m_json["polarity"].asString() == "falling")
                ret |= CAEN_DGTZ_SetTriggerPolarity(m_handle, ch, CAEN_DGTZ_TriggerOnFallingEdge);
            else
                ret |= CAEN_DGTZ_SetTriggerPolarity(m_handle, ch, CAEN_DGTZ_TriggerOnRisingEdge);
        }
        if (ret != CAEN_DGTZ_Success)
            throw std::runtime_error("Failed to set trigger config");

        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleGetTriggerConfig(Request &req, Response &resp) {
        Json::Value root;

        uint32_t trOffset, trLvl;
        int ret = CAEN_DGTZ_Success;

        ret |= CAEN_DGTZ_GetGroupFastTriggerDCOffset(m_handle, 0, &trOffset);
        ret |= CAEN_DGTZ_GetGroupFastTriggerThreshold(m_handle, 0, &trLvl);

        if (ret != CAEN_DGTZ_Success)
            throw std::runtime_error("Failed to get trigger config");

        root["level"] = trLvl * m_trMul + m_trAdd;
        root["offset"] = trOffset * m_trMul + m_trAdd;
        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleSetDigitizerFreq(Request &req, Response &resp) {
        Json::Value root;
        int ret = CAEN_DGTZ_CommError;
        std::string freqStr = req.m_json["freq"].asString();
        CAEN_DGTZ_DRS4Frequency_t freq;

        if (freqStr == "5ghz") {
            freq = CAEN_DGTZ_DRS4_5GHz;
        } else if (freqStr == "2_5ghz") {
            freq = CAEN_DGTZ_DRS4_2_5GHz;
        } else if (freqStr == "1ghz") {
            freq = CAEN_DGTZ_DRS4_1GHz;
        }

        ret = CAEN_DGTZ_SetDRS4SamplingFrequency(m_handle, freq);

        if (ret != CAEN_DGTZ_Success)
            throw std::runtime_error("Failed to set freq");

        if (m_freq !=  freq) {
            m_freq = freq;
            loadCorrectionTables();
        }

        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleGetDigitizerFreq(Request &req, Response &resp) {
        Json::Value root;
        CAEN_DGTZ_DRS4Frequency_t freq;

        int ret = CAEN_DGTZ_GetDRS4SamplingFrequency(m_handle, &freq);
        if (freq == CAEN_DGTZ_DRS4_5GHz)
            root["freq"] = "5ghz";
        if (freq == CAEN_DGTZ_DRS4_2_5GHz)
            root["freq"] = "2_5ghz";
        if (freq == CAEN_DGTZ_DRS4_1GHz)
            root["freq"] = "1ghz";


        if (ret != CAEN_DGTZ_Success)
            throw std::runtime_error("Failed to get freq");

        if (m_freq !=  freq) {
            m_freq = freq;
            loadCorrectionTables();
        }

        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleStartAcquisition(Request &req, Response &resp) {
        Json::Value root;

        int ret = CAEN_DGTZ_SWStartAcquisition(m_handle);

        if (ret != CAEN_DGTZ_Success) {
            throw std::runtime_error("Failed to start acquisition");
        }

        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleStopAcquisition(Request &req, Response &resp) {
        Json::Value root;

        int ret = CAEN_DGTZ_SWStopAcquisition(m_handle);

        if (ret != CAEN_DGTZ_Success) {
            throw std::runtime_error("Failed to stop acquisition");
        }

        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleSetMaxEvents(Request &req, Response &resp) {
        Json::Value root;

        uint32_t maxEv = req.m_json["max_events"].asUInt();

        int ret = CAEN_DGTZ_SetMaxNumEventsBLT(m_handle, maxEv);

        if (ret != CAEN_DGTZ_Success) {
            throw std::runtime_error("Failed to set maximum events");
        }

        root["status"] = "success";
        resp = root;
        return true;
    }

    bool CaenDigitizer::handleGetData(Request &req, Response &resp) {
        Json::Value root;
//=========================
        CAEN_DGTZ_ErrorCode err;
        uint32_t readSize = m_readoutBufferSize;
        err = CAEN_DGTZ_ReadData(m_handle, CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
                                 m_readoutBuffer, &readSize);
        if (err) {
            throw std::runtime_error("Can't read data to readout buffer");
        }

        int32_t ret;
        uint32_t numEvents;
        ret = GetNumEvents(m_readoutBuffer, readSize, &numEvents);
        if (ret) {
            throw std::runtime_error("Can't get number of events in readout buffer");
        }

        if (numEvents == 0) {
            throw std::runtime_error("Got zero events");
        }

        char *evPtr = NULL;
        ret = GetEventPtr(m_readoutBuffer, readSize,
                          0, &evPtr);
        if (ret) {
            throw std::runtime_error("Can't get event info in readout buffer");
        }


        CAEN_DGTZ_X742_EVENT_t *evt = NULL;
        ret = X742_DecodeEvent(evPtr, (void **) &evt);
        if (ret) {
            throw std::runtime_error("Can't decode event");
        }

        parseEvent(*evt);

        CAEN_DGTZ_FreeEvent(m_handle, (void **) &evt);

//=========================
        for (int chan = 0; chan < m_channles * m_channelGroups; chan++) {
            int i = 0;
            for (auto iter: m_data[chan]) {
                root["data"][chan][i++] = iter * m_chanMul[chan] + m_chanAdd[chan];
            }
        }


        root["status"] = "success";
        resp = root;
        return true;
    }


}


