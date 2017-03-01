//
// Created by user on 02.06.16.
//

#include "CaenDigitizer.h"

#define DEFAULT_CAEN_BUF_SIZE (1024 * 1024)

#define EVENT_HEADER_SIZE		0x10
#define X742_MAX_GROUPS			0x04
#define X742_FIXED_SIZE			0x400
#define MAX_READ_CHAR               1000
#define MAX_BASE_INPUT_FILE_LENGTH  1000

extern "C" {

void PeakCorrection(CAEN_DGTZ_X742_GROUP_t *dataout) {
    int offset;
    int chaux_en;
    unsigned int i;
    int j;

    chaux_en = (dataout->ChSize[8] == 0) ? 0 : 1;
    for (j = 0; j < (8 + chaux_en); j++) {
        dataout->DataChannel[j][0] = dataout->DataChannel[j][1];
    }
    for (i = 1; i < dataout->ChSize[0]; i++) {
        offset = 0;
        for (j = 0; j < 8; j++) {
            if (i == 1) {
                if ((dataout->DataChannel[j][2] - dataout->DataChannel[j][1]) > 30) {
                    offset++;
                }
                else {
                    if (((dataout->DataChannel[j][3] - dataout->DataChannel[j][1]) > 30) &&
                        ((dataout->DataChannel[j][3] - dataout->DataChannel[j][2]) > 30)) {
                        offset++;
                    }
                }
            }
            else {
                if ((i == dataout->ChSize[j] - 1) && ((dataout->DataChannel[j][dataout->ChSize[j] - 2] -
                                                       dataout->DataChannel[j][dataout->ChSize[j] - 1]) > 30)) {
                    offset++;
                }
                else {
                    if ((dataout->DataChannel[j][i - 1] - dataout->DataChannel[j][i]) > 30) {
                        if ((dataout->DataChannel[j][i + 1] - dataout->DataChannel[j][i]) > 30)
                            offset++;
                        else {
                            if ((i == dataout->ChSize[j] - 2) ||
                                ((dataout->DataChannel[j][i + 2] - dataout->DataChannel[j][i]) > 30))
                                offset++;
                        }
                    }
                }
            }
        }

        if (offset == 8) {
            for (j = 0; j < (8 + chaux_en); j++) {
                if (i == 1) {
                    if ((dataout->DataChannel[j][2] - dataout->DataChannel[j][1]) > 30) {
                        dataout->DataChannel[j][0] = dataout->DataChannel[j][2];
                        dataout->DataChannel[j][1] = dataout->DataChannel[j][2];
                    }
                    else {
                        dataout->DataChannel[j][0] = dataout->DataChannel[j][3];
                        dataout->DataChannel[j][1] = dataout->DataChannel[j][3];
                        dataout->DataChannel[j][2] = dataout->DataChannel[j][3];
                    }
                }
                else {
                    if (i == dataout->ChSize[j] - 1) {
                        dataout->DataChannel[j][dataout->ChSize[j] - 1] = dataout->DataChannel[j][dataout->ChSize[j] -
                                                                                                  2];
                    }
                    else {
                        if ((dataout->DataChannel[j][i + 1] - dataout->DataChannel[j][i]) > 30)
                            dataout->DataChannel[j][i] = (
                                    (dataout->DataChannel[j][i + 1] + dataout->DataChannel[j][i - 1]) / 2);
                        else {
                            if (i == dataout->ChSize[j] - 2) {
                                dataout->DataChannel[j][dataout->ChSize[j] - 2] = dataout->DataChannel[j][
                                        dataout->ChSize[j] - 3];
                                dataout->DataChannel[j][dataout->ChSize[j] - 1] = dataout->DataChannel[j][
                                        dataout->ChSize[j] - 3];
                            }
                            else {
                                dataout->DataChannel[j][i] = (
                                        (dataout->DataChannel[j][i + 2] + dataout->DataChannel[j][i - 1]) / 2);
                                dataout->DataChannel[j][i + 1] = (
                                        (dataout->DataChannel[j][i + 2] + dataout->DataChannel[j][i - 1]) / 2);
                            }
                        }
                    }
                }
            }
        }
    }
}

/*! \brief   Corrects 'data' depending on the informations contained in 'CTable'
*
*   \param   CTable              :  Pointer to the Table containing the Data Corrections
*   \param   frequency           :  The operational Frequency of the board
*   \param   CorrectionLevelMask :  Mask of Corrections to be applied
*   \param   data                :  Data to be corrected
*/
void ApplyDataCorrection(CAEN_DGTZ_DRS4Correction_t *CTable, CAEN_DGTZ_DRS4Frequency_t frequency,
                         int CorrectionLevelMask, CAEN_DGTZ_X742_GROUP_t *data) {

    int i, j, rpnt = 0, wpnt = 0, size1, trg = 0, k;
    float Time[1024], t0;
    float Tsamp;
    float vcorr;
    uint16_t st_ind = 0;
    uint32_t freq = frequency;
    float wave_tmp[1024];
    int cellCorrection = CorrectionLevelMask & 0x1;
    int nsampleCorrection = (CorrectionLevelMask & 0x2) >> 1;
    int timeCorrection = (CorrectionLevelMask & 0x4) >> 2;

    switch (frequency) {
        case CAEN_DGTZ_DRS4_2_5GHz:
            Tsamp = (float) ((1.0 / 2500.0) * 1000.0);
            break;
        case CAEN_DGTZ_DRS4_1GHz:
            Tsamp = (float) ((1.0 / 1000.0) * 1000.0);
            break;
        default:
            Tsamp = (float) ((1.0 / 5000.0) * 1000.0);
            break;
    }

    if (data->ChSize[8] != 0) trg = 1;
    st_ind = (uint16_t) (data->StartIndexCell);
    for (i = 0; i < MAX_X742_CHANNEL_SIZE; i++) {
        size1 = data->ChSize[i];
        for (j = 0; j < size1; j++) {
            if (cellCorrection)
                data->DataChannel[i][j] -= CTable->cell[i][((st_ind + j) % 1024)];
            if (nsampleCorrection)
                data->DataChannel[i][j] -= CTable->nsample[i][j];
        }
    }

    if (cellCorrection)
        PeakCorrection(data);
    if (!timeCorrection)
        return;

    t0 = CTable->time[st_ind];
    Time[0] = 0.0;
    for (j = 1; j < 1024; j++) {
        t0 = CTable->time[(st_ind + j) % 1024] - t0;
        if (t0 > 0)
            Time[j] = Time[j - 1] + t0;
        else
            Time[j] = Time[j - 1] + t0 + (Tsamp * 1024);

        t0 = CTable->time[(st_ind + j) % 1024];
    }
    for (j = 0; j < 8 + trg; j++) {
        data->DataChannel[j][0] = data->DataChannel[j][1];
        wave_tmp[0] = data->DataChannel[j][0];
        vcorr = 0.0;
        k = 0;
        i = 0;

        for (i = 1; i < 1024; i++) {
            while ((k < 1024 - 1) && (Time[k] < (i * Tsamp))) k++;
            vcorr = (((float) (data->DataChannel[j][k] - data->DataChannel[j][k - 1]) / (Time[k] - Time[k - 1])) *
                     ((i * Tsamp) - Time[k - 1]));
            wave_tmp[i] = data->DataChannel[j][k - 1] + vcorr;
            k--;
        }
        memcpy(data->DataChannel[j], wave_tmp, 1024 * sizeof(float));
    }
}

/*! \brief   Write the correction table of a x742 boards into the output files
*
*   \param   Filename of output file
*   \param   Group Mask of Tables to be saved
*   \param   Pointer to the DataCorrection group tables
*/
int SaveCorrectionTables(char *outputFileName, uint32_t groupMask, CAEN_DGTZ_DRS4Correction_t *tables) {
    char fnStr[MAX_BASE_INPUT_FILE_LENGTH + 1];
    int ch, i, j, gr;
    FILE *outputfile;

    if ((int) (strlen(outputFileName) - 17) > MAX_BASE_INPUT_FILE_LENGTH)
        return -1; // Too long base filename

    for (gr = 0; gr < MAX_X742_GROUP_SIZE; gr++) {
        CAEN_DGTZ_DRS4Correction_t *tb;

        if (!((groupMask >> gr) & 0x1))
            continue;
        tb = &tables[gr];
        sprintf(fnStr, "%s_gr%d_cell.txt", outputFileName, gr);
        printf("Saving correction table cell values to %s\n", fnStr);
        if ((outputfile = fopen(fnStr, "w")) == NULL)
            return -2;
        for (ch = 0; ch < MAX_X742_CHANNEL_SIZE; ch++) {
            fprintf(outputfile, "Calibration values from cell 0 to 1024 for channel %d:\n\n", ch);
            for (i = 0; i < 1024; i += 8) {
                for (j = 0; j < 8; j++)
                    fprintf(outputfile, "%d\t", tb->cell[ch][i + j]);
                fprintf(outputfile, "cell = %d to %d\n", i, i + 7);
            }
        }
        fclose(outputfile);

        sprintf(fnStr, "%s_gr%d_nsample.txt", outputFileName, gr);
        printf("Saving correction table nsamples values to %s\n", fnStr);
        if ((outputfile = fopen(fnStr, "w")) == NULL)
            return -3;
        for (ch = 0; ch < MAX_X742_CHANNEL_SIZE; ch++) {
            fprintf(outputfile, "Calibration values from cell 0 to 1024 for channel %d:\n\n", ch);
            for (i = 0; i < 1024; i += 8) {
                for (j = 0; j < 8; j++)
                    fprintf(outputfile, "%d\t", tb->nsample[ch][i + j]);
                fprintf(outputfile, "cell = %d to %d\n", i, i + 7);
            }
        }
        fclose(outputfile);

        sprintf(fnStr, "%s_gr%d_time.txt", outputFileName, gr);
        printf("Saving correction table time values to %s\n", fnStr);
        if ((outputfile = fopen(fnStr, "w")) == NULL)
            return -4;
        fprintf(outputfile, "Calibration values (ps) from cell 0 to 1024 :\n\n");
        for (i = 0; i < 1024; i += 8) {
            for (ch = 0; ch < 8; ch++)
                fprintf(outputfile, "%09.3f\t", tb->time[i + ch]);
            fprintf(outputfile, "cell = %d to %d\n", i, i + 7);
        }
        fclose(outputfile);
    }
    return 0;
}

/*! \brief   Reads the correction table of a x742 boards from txt files
*
*   \param   Base Filename of input file. Actual filenames loaded will be:
*               a) baseInputFileName + "_cell.txt"
*               b) baseInputFileName + "_nsample.txt"
*               c) baseInputFileName + "_time.txt"
*   \param   DataCorrection table to be filled
*/
int LoadCorrectionTable(char *baseInputFileName, CAEN_DGTZ_DRS4Correction_t *tb) {
    char fnStr[MAX_BASE_INPUT_FILE_LENGTH + 1];
    int ch, i, j, read;
    FILE *inputfile;
    char Buf[MAX_READ_CHAR + 1], *pread;

    if (strlen(baseInputFileName) - 13 > MAX_BASE_INPUT_FILE_LENGTH)
        return -1; // Too long base filename

    strcpy(fnStr, baseInputFileName);
    strcat(fnStr, "_cell.txt");
    printf("Loading correction table cell values from %s\n", fnStr);
    if ((inputfile = fopen(fnStr, "r")) == NULL)
        return -2;
    for (ch = 0; ch < MAX_X742_CHANNEL_SIZE; ch++) {
        while (strstr(Buf, "Calibration") != Buf)
            pread = fgets(Buf, MAX_READ_CHAR, inputfile);

        for (i = 0; i < 1024; i += 8) {
            for (j = 0; j < 8; j++)
                read = fscanf(inputfile, "%hd", &(tb->cell[ch][i + j]));
            pread = fgets(Buf, MAX_READ_CHAR, inputfile);
        }
    }
    fclose(inputfile);

    strcpy(fnStr, baseInputFileName);
    strcat(fnStr, "_nsample.txt");
    printf("Loading correction table nsamples values from %s\n", fnStr);
    if ((inputfile = fopen(fnStr, "r")) == NULL)
        return -3;
    for (ch = 0; ch < MAX_X742_CHANNEL_SIZE; ch++) {
        while (strstr(Buf, "Calibration") != Buf)
            pread = fgets(Buf, MAX_READ_CHAR, inputfile);

        for (i = 0; i < 1024; i += 8) {
            for (j = 0; j < 8; j++)
                read = fscanf(inputfile, "%hhd", &(tb->nsample[ch][i + j]));
            pread = fgets(Buf, MAX_READ_CHAR, inputfile);
        }
    }
    fclose(inputfile);

    strcpy(fnStr, baseInputFileName);
    strcat(fnStr, "_time.txt");
    printf("Loading correction table time values from %s\n", fnStr);
    if ((inputfile = fopen(fnStr, "r")) == NULL)
        return -4;
    while (strstr(Buf, "Calibration") != Buf)
        pread = fgets(Buf, MAX_READ_CHAR, inputfile);
    pread = fgets(Buf, MAX_READ_CHAR, inputfile);

    for (i = 0; i < 1024; i += 8) {
        for (j = 0; j < 8; j++)
            read = fscanf(inputfile, "%f", &(tb->time[i + j]));
        pread = fgets(Buf, MAX_READ_CHAR, inputfile);
    }
    fclose(inputfile);

    return 0;
}


int32_t getNumberOfBits(uint8_t byte) {
    uint32_t i, count;
    count = 0;
    for (i = 0; i < 8; i++) {
        if ((byte >> i) & 0x1) count++;
    }
    return count;
}

int V1742UnpackEventGroup(uint32_t group, uint32_t *datain, CAEN_DGTZ_X742_GROUP_t *dataout) {

    int i, j, rpnt = 0, wpnt = 0, size1, size2, trg = 0, k;
    long samples;
    float Time[1024], t0;
    float Tsamp;
    float vcorr;
    uint16_t st_ind = 0;
    uint32_t freq;
    float wave_tmp[1024];

    /*if (group == 1) {
        rpnt = size1+size2+2;
    }*/

    freq = (datain[0] >> 16) & 0x3;
    switch (freq) {
        case CAEN_DGTZ_DRS4_2_5GHz:
            Tsamp = (float) ((1.0 / 2500.0) * 1000.0);
            break;
        case CAEN_DGTZ_DRS4_1GHz:
            Tsamp = (float) ((1.0 / 1000.0) * 1000.0);
            break;
        default:
            Tsamp = (float) ((1.0 / 5000.0) * 1000.0);
            break;
    }

    st_ind = (uint16_t) ((datain[0] >> 20) & 0x3FF);
    size1 = datain[0] & 0xFFF;
    if ((trg = (datain[0] >> 12) & 0x1) == 1)
        size2 = (datain[0] >> 3) & 0x1FF;
    else
        size2 = 0;

    dataout->TriggerTimeTag = datain[size1 + size2 + 1] & 0x3FFFFFFF;

    samples = ((long) (size1 / 3));

    while (rpnt < size1) {

        switch (rpnt % 3) {
            case 0 :
                dataout->DataChannel[0][wpnt] = (float) (datain[rpnt + 1] & 0x00000FFF);            /* S0[11:0] - CH0 */
                dataout->DataChannel[1][wpnt] = (float) ((datain[rpnt + 1] & 0x00FFF000) >> 12);    /* S0[11:0] - CH1 */
                dataout->DataChannel[2][wpnt] = (float) ((datain[rpnt + 1] & 0xFF000000) >> 24);    /* S0[ 7:0] - CH2 */
                break;
            case 1 :
                dataout->DataChannel[2][wpnt] += (float) ((datain[rpnt + 1] & 0x0000000F) << 8);
                dataout->DataChannel[3][wpnt] = (float) ((datain[rpnt + 1] & 0x0000FFF0) >> 4);     /* S0[11:0] - CH3 */
                dataout->DataChannel[4][wpnt] = (float) ((datain[rpnt + 1] & 0x0FFF0000) >> 16);    /* S0[11:0] - CH4 */
                dataout->DataChannel[5][wpnt] = (float) ((datain[rpnt + 1] & 0xF0000000) >> 28);    /* S0[3:0]  - CH5 */
                break;
            case 2 :
                dataout->DataChannel[5][wpnt] += (float) ((datain[rpnt + 1] & 0x000000FF) << 4);
                dataout->DataChannel[6][wpnt] = (float) ((datain[rpnt + 1] & 0x000FFF00) >> 8);    /* S0[11:0] - CH6 */
                dataout->DataChannel[7][wpnt] = (float) ((datain[rpnt + 1] & 0xFFF00000) >> 20);    /* S0[11:0] - CH7 */
                wpnt++;
                break;
        }
        rpnt++;
    }
    rpnt++;
    for (k = 0; k < 8; k++) dataout->ChSize[k] = wpnt;
    wpnt = 0;

    for (i = 0; i < size2; i++) {
        switch (i % 3) {
            case 0 :
                dataout->DataChannel[8][wpnt] = (float) (datain[rpnt + i] & 0x00000FFF);          /* S0 - CH8 */
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0x00FFF000) >> 12);  /* S1 - CH8 */
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0xFF000000) >>
                                                           24);   /* S2[ 7:0] - CH8 */
                break;
            case 1 :
                dataout->DataChannel[8][wpnt] += (float) ((datain[rpnt + i] & 0x0000000F) << 8);
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0x0000FFF0) >> 4);    /* S3 - CH8*/
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0x0FFF0000) >> 16);   /* S4 - CH8 */
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0xF0000000) >>
                                                           28);   /* S5[3:0]  - CH8 */
                break;
            case 2 :
                dataout->DataChannel[8][wpnt] += (float) ((datain[rpnt + i] & 0x000000FF) << 4);    /* S5[11:4] - CH8 */
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0x000FFF00) >>
                                                           8);    /* S6[11:0] - CH8 */
                dataout->DataChannel[8][++wpnt] = (float) ((datain[rpnt + i] & 0xFFF00000) >>
                                                           20);   /* S7[11:0] - CH8 */
                wpnt++;
                break;
        }
    }
    dataout->ChSize[8] = wpnt;
    dataout->StartIndexCell = (uint16_t) st_ind;
    return (size1 + size2 + 2);
}

int32_t GetNumEvents(char *buffer, uint32_t buffsize, uint32_t *numEvents) {
    uint32_t i = 0, evtSize;
    int ret;
    int32_t counter = -1;
    if ((buffsize == 0) || (buffer == NULL)) {
        *numEvents = 0;
        return 0;
    }
    if (buffsize < EVENT_HEADER_SIZE) return -1;
    do {
        counter++;
        evtSize = *(long *) (buffer + i) & 0x0FFFFFFF;
        i += (uint32_t) (evtSize * 4);
    } while ((i + EVENT_HEADER_SIZE) < buffsize);
    *numEvents = counter + 1;
    return 0;
}

int32_t GetEventPtr(char *buffer, uint32_t buffsize,
                    int32_t numEvent, char **EventPtr) {
    uint32_t i = 0;
    int32_t counter = -1;
    int ret;
    int evtSize;

    if ((buffer == NULL) || (buffsize < EVENT_HEADER_SIZE)) return -1;
    do {
        counter++;
        evtSize = *(long *) (buffer + i) & 0x0FFFFFFF;
        if (counter == numEvent) {
            if ((i + (uint32_t) evtSize) < buffsize) {
                *EventPtr = (buffer + i);
                return 0;
            }
            else return -1;
        }
        i += (uint32_t) (evtSize * 4);
    } while ((i + EVENT_HEADER_SIZE) < buffsize);
    return -1;
}

int32_t X742_DecodeEvent(char *evtPtr, void **Evt) {
    CAEN_DGTZ_X742_EVENT_t *Event;
    uint32_t *buffer;
    char chanMask;
    uint32_t j, g, size;
    uint32_t *pbuffer;
    uint32_t eventSize;
    int evtSize, h;

    evtSize = *(long *) evtPtr & 0x0FFFFFFF;
    chanMask = *(long *) (evtPtr + 4) & 0x0000000F;

    std::cout << "Channel mask: " << (uint32_t) chanMask << std::endl;

    evtPtr += EVENT_HEADER_SIZE;
    buffer = (uint32_t *) evtPtr;
    pbuffer = (uint32_t *) evtPtr;
    eventSize = (evtSize * 4) - EVENT_HEADER_SIZE;
    if (eventSize == 0) return -1;
    Event = (CAEN_DGTZ_X742_EVENT_t *) malloc(sizeof(CAEN_DGTZ_X742_EVENT_t));
    if (Event == NULL) return -1;
    memset(Event, 0, sizeof(CAEN_DGTZ_X742_EVENT_t));
    for (g = 0; g < X742_MAX_GROUPS; g++) {
        if ((chanMask >> g) & 0x1) {
            for (j = 0; j < MAX_X742_CHANNEL_SIZE; j++) {
                Event->DataGroup[g].DataChannel[j] =
                        (float *) malloc(X742_FIXED_SIZE * sizeof(float));
                if (Event->DataGroup[g].DataChannel[j] == NULL) {
                    for (h = j - 1; h > -1; h++) free(Event->DataGroup[g].DataChannel[h]);
                    return -1;
                }
            }
            size = V1742UnpackEventGroup(g, pbuffer, &(Event->DataGroup[g]));
            pbuffer += size;
            Event->GrPresent[g] = 1;
        }
        else {
            Event->GrPresent[g] = 0;
            for (j = 0; j < MAX_X742_CHANNEL_SIZE; j++) {
                Event->DataGroup[g].DataChannel[j] = NULL;
            }
        }
    }
    *Evt = Event;
    return 0;
}
}