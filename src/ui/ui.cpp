#include "common.h"
#include <FS.h>
#include <LITTLEFS.h>
#include <math.h>
#include "config.h"
#include "drv/cct.h"
#include "sensor/gps.h"
#include "sensor/ms5611.h"
#include "nv/options.h"
#include "ui.h"
#include "route.h"

static const char* TAG = "ui";

bool IsRouteActive = false;
bool IsGpsFixStable = false;
bool IsGpsTrackActive = false;
bool IsLcdBkltEnabled = false;
bool IsLoggingIBG = false;
bool IsFlashDisplayRequired = false;
bool IsGpsHeading = true;
bool EndGpsTrack = false;
bool IsBluetoothEnabled = false;

float SupplyVoltageV = 0.0f;
int32_t GpsCourseHeadingDeg;
int32_t CompassHeadingDeg;

const char szLogType[3][5]    = {"NONE", " GPS", " IBG"};
const char szAltDisplayType[2][5] = {" GPS", "BARO"};
const char szBtMsgType[2][4]  = {"LK8", "XCT"};

static int ParChanged = 0;
static int ScreenParOffset = 0;
static int ParDisplaySel = 0;
static int ParSel = 0;