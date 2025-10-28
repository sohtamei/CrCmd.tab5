#ifndef PTPTDEF
#define PTPTDEF

#include <stdint.h>

void format_f_number(char* linebuf, uint32_t f_number);
void format_iso_sensitivity(char* linebuf, uint32_t iso);
void format_shutter_speed(char* linebuf, uint32_t shutter_speed);
void format_exposure_program_mode(char* linebuf, uint32_t exposure_program_mode);


#define PTP_DT_UNDEF 0x0000
#define PTP_DT_INT8 0x0001
#define PTP_DT_UINT8 0x0002
#define PTP_DT_INT16 0x0003
#define PTP_DT_UINT16 0x0004
#define PTP_DT_INT32 0x0005
#define PTP_DT_UINT32 0x0006
#define PTP_DT_INT64 0x0007
#define PTP_DT_UINT64 0x0008
#define PTP_DT_INT128 0x0009
#define PTP_DT_UINT128 0x000A
#define PTP_DT_AINT8 0x4001
#define PTP_DT_AUINT8 0x4002
#define PTP_DT_AINT16 0x4003
#define PTP_DT_AUINT16 0x4004
#define PTP_DT_AINT32 0x4005
#define PTP_DT_AUINT32 0x4006
#define PTP_DT_AINT64 0x4007
#define PTP_DT_AUINT64 0x4008
#define PTP_DT_AINT128 0x4009
#define PTP_DT_AUINT128 0x400A
#define PTP_DT_STR 0xFFFF

enum {
	PTP_OC_GetDeviceInfo		= 0x1001,
	PTP_OC_OpenSession			= 0x1002,
	PTP_OC_CloseSession			= 0x1003,
	PTP_OC_GetStorageID			= 0x1004,
	PTP_OC_GetStorageInfo		= 0x1005,
	PTP_OC_GetNumObjects		= 0x1006,
	PTP_OC_GetObjectHandles		= 0x1007,
	PTP_OC_GetObjectInfo		= 0x1008,
	PTP_OC_GetObject			= 0x1009,
	PTP_OC_GetThumb 			= 0x100A,
	PTP_OC_DeleteObject			= 0x100B,
	PTP_OC_SendObject			= 0x100D,
	PTP_OC_GetPartialObject		= 0x101B,

	PTP_OC_SDIOConnect					= 0x9201,
	PTP_OC_SDIOGetExtDeviceInfo			= 0x9202,
	PTP_OC_SDIOSetExtDevicePropValue	= 0x9205,
	PTP_OC_SDIOControlDevice			= 0x9207,
	PTP_OC_SDIOGetAllExtDevicePropInfo	= 0x9209,
	PTP_OC_SDIOOpenSession 				= 0x9210,
	PTP_OC_SDIOGetPartialLargeObject	= 0x9211,
	PTP_OC_SDIOSetContentsTransferMode	= 0x9212,
	PTP_OC_SDIOGetDisplayStringList 	= 0x9215,
	PTP_OC_SDIOGetLensInformation		= 0x9223,
	PTP_OC_SDIOOperationResultsSupported = 0x922F,
};


enum {
	PTP_SDIE_ObjectAdded = 0xC201,
	PTP_SDIE_ObjectRemoved = 0xC202,
	PTP_SDIE_DevicePropChanged = 0xC203,
	PTP_SDIE_CapturedEvent = 0xC206,
	PTP_SDIE_CWBCapturedResult = 0xC208,
	PTP_SDIE_CameraSettingReadResult = 0xC209,
	PTP_SDIE_FTPSettingReadResult = 0xC20A,
	PTP_SDIE_MediaFormatResult = 0xC20B,
	PTP_SDIE_FTPDisplayNameListChanged = 0xC20C,
	PTP_SDIE_ContentsTransferEvent = 0xC20D,
	PTP_SDIE_DisplayListChangedEvent = 0xC20F,
	PTP_SDIE_FocusPositionResult = 0xC218,
	PTP_SDIE_LensInformationChanged = 0xC21B,
	PTP_SDIE_OperationResults = 0xC222,
	PTP_SDIE_AFStatus = 0xC223,
	PTP_SDIE_MovieRecOperationResults = 0xC224,
};

enum {
	PTP_CC_S1_Button						= 0xD2C1,
	PTP_CC_S2_Button						= 0xD2C2,
	PTP_CC_AEL_Button						= 0xD2C3,
	PTP_CC_Movie_Rec_Button_Hold			= 0xD2C8,
	PTP_CC_FEL_Button						= 0xD2C9,
	PTP_CC_Remote_Key_UP					= 0xD2CD,
	PTP_CC_Remote_Key_DOWN					= 0xD2CE,
	PTP_CC_Remote_Key_LEFT					= 0xD2CF,
	PTP_CC_Remote_Key_RIGHT					= 0xD2D0,
	PTP_CC_NearFar							= 0xD2D1,
	PTP_CC_AWBL_Button						= 0xD2D9,
	PTP_CC_AF_Area_Position_xy				= 0xD2DC,
	PTP_CC_Zoom_Operation					= 0xD2DD,
	PTP_CC_Custom_WB_Capture_Standby		= 0xD2DF,
	PTP_CC_Custom_WB_Capture_Standby_Cancel	= 0xD2E0,
	PTP_CC_Custom_WB_Capture				= 0xD2E1,
	PTP_CC_Selected_Media_Format			= 0xD2E2,
	PTP_CC_High_Resolution_SS_Adjust		= 0xD2E3,
	PTP_CC_Remote_Touch_Operation_xy		= 0xD2E4,
	PTP_CC_Cancel_Remote_Touch_Operation	= 0xD2E5,
	PTP_CC_S1_S2_Button						= 0xD2E6,
	PTP_CC_Cancel_Media_Format				= 0xD2E7,
	PTP_CC_Save_Zoom_and_Focus_Position		= 0xD2E9,
	PTP_CC_Load_Zoom_and_Focus_Position		= 0xD2EA,
	PTP_CC_APS_C_or_Full_Switching			= 0xD2EB,
	PTP_CC_Color_Temperature_Step			= 0xD2EC,
	PTP_CC_White_Balance_Tint_Step			= 0xD2ED,
	PTP_CC_Reset_Multi_Matrix				= 0xD2EE,
	PTP_CC_Focus_Operation					= 0xD2EF,
	PTP_CC_High_Resolution_SS_Adjust_In_Integral_Multiples = 0xD2F0,
	PTP_CC_Flicker_Scan						= 0xD2F1,
	PTP_CC_Set_PresetInfo_ZoomOnly_Value	= 0xD2F2,
	PTP_CC_REC_Settings_Reset				= 0xD2F3,
	PTP_CC_ContShootSpotBoost_Button		= 0xD2F6,
	PTP_CC_Remote_Key_Cancel_Back_Button	= 0xD2F7,
	PTP_CC_Remote_Key_Display_Button		= 0xD2F8,
	PTP_CC_Remote_Key_SET					= 0xD2F9,
	PTP_CC_Remote_Key_RIGHT_UP				= 0xD2FA,
	PTP_CC_Remote_Key_RIGHT_DOWN			= 0xD2FB,
	PTP_CC_Remote_Key_LEFT_UP				= 0xD2FC,
	PTP_CC_Remote_Key_LEFT_DOWN				= 0xD2FD,
	PTP_CC_Movie_Rec_Button_2nd_Toggle		= 0xD2FE,
	PTP_CC_Remote_Key_Menu_Button			= 0xD2FF,
	PTP_CC_Pixel_Mapping					= 0xD300,
	PTP_CC_Power_Off						= 0xD301,
	PTP_CC_Time_Code_Preset_Reset			= 0xD302,
	PTP_CC_User_Bit_Preset_Reset			= 0xD303,
	PTP_CC_Sensor_Cleaning					= 0xD304,
	PTP_CC_Reset_Picture_Profile			= 0xD305,
	PTP_CC_Reset_Creative_Look				= 0xD306,
	PTP_CC_Stream_Button					= 0xD307,
	PTP_CC_Camera_Button_Function			= 0xD309,
	PTP_CC_Camera_Button_Function_Multi		= 0xD30A,
	PTP_CC_Camera_Dial_Function				= 0xD30B,
	PTP_CC_Camera_Lever_Function			= 0xD30C,
	PTP_CC_Tracking_On_AF_On_Button			= 0xD30D,
	PTP_CC_Forced_File_Number_Reset			= 0xD30E,
	PTP_CC_Set_PostView_Enable				= 0xD312,
	PTP_CC_Set_LiveView_Enable				= 0xD313,
	PTP_CC_Create_New_Folderstill			= 0xD314,
	PTP_CC_Camera_Standby					= 0xD315,
	PTP_CC_Power_On							= 0xD316,
	PTP_CC_Shutter_ECS_Number_Step			= 0xF000,
	PTP_CC_Movie_Rec_Button_Toggle			= 0xF001,
	PTP_CC_Cancel_Focus_Position			= 0xF002,
	PTP_CC_Zoom_Operation_with_INT16		= 0xF003,
	PTP_CC_Focus_Operation_with_INT16		= 0xF004,
	PTP_CC_Cancel_Zoom_Position				= 0xF00C,
	PTP_CC_Preset_PTZF_Recall				= 0xF015,
};

enum {
	DPC_MOVIE_REC				= 0xD2C8,
	DPC_S2_BUTTON				= 0xD2C2,
	DPC_S1_BUTTON				= 0xD2C1,
	DPC_AE_LOCK					= 0xD2C3,
	DPC_AF_LOCK					= 0xD2C9,
	DPC_LIVEVIEW_MODE			= 0xD26A,
	DPC_DRIVE_MODE				= 0x5013,
	DPC_ASPECT_RATIO			= 0xD211,
	DPC_IMAGE_SIZE				= 0xD203,
	DPC_WHITE_BALANCE			= 0x5005,
	DPC_EXPOSURE_COMPENSATION	= 0x5010,		//
	DPC_FNUMBER					= 0x5007,		//
	DPC_SHOOTING_FILE_INFOMATION= 0xD215,
	DPC_FOCUS_MODE				= 0x500A,
	DPC_EXPOSURE_MODE			= 0x500E,		//
	DPC_DRO_HDR_MODE			= 0xD201,
	DPC_FOCUS_AREA_X_Y			= 0xD2DC,
	DPC_FOCUS_AREA				= 0xD22C,
	DPC_LIVEVIEW_STATUS			= 0xD221,
	DPC_SHUTTER_SPEED			= 0xD20D,		//
	DPC_COLOR_TEMP				= 0xD20F,
	DPC_ISO						= 0xD21E,		//
	DPC_FLASH_MODE				= 0x500C,
	DPC_METERING_MODE			= 0x500B,
	DPC_PICTURE_EFFECTS			= 0xD21B,
	DPC_BATTERY_LEVEL			= 0xD20E,
	DPC_POSITION_KEY			= 0xD25A,		//
	DPC_AELOCK_INDICATION		= 0xD217,
	DPC_AFLOCK_INDICATION		= 0xD21F,
	DPC_WHITEBALANCE_AB			= 0xD21C,
	DPC_WHITEBALANCE_GM			= 0xD210,
	DPC_COMPRESSION_SETTING		= 0x5004,
	DPC_IMAGE_FILE_FORMAT		= 0x5004,
	DPC_VIEW					= 0xD231,
	DPC_FLASH_COMP				= 0xD200,
	DPC_WIRELESS_FLASH			= 0xD262,
	DPC_AWB_LOCK				= 0xD2D9,
	DPC_NORMAL					= 0xD2D4,
	DPC_AF_STATUS				= 0xD213,
	DPC_PICTURE_PROFILE			= 0xD23F,
	DPC_CREATIVE_STYLE			= 0xD240,
	DPC_MOVIE_FORMAT			= 0xD241,
	DPC_MOVIE_QUALITY			= 0xD242,
	DPC_SAVE_MEDIA				= 0xD222,		//
	DPC_ZOOM_SETTING			= 0xD25F,
	DPC_ZOOM_SCALE				= 0xD25C,
	DPC_ZOOM_OPTIC				= 0xD25D,
	DPC_NEAR_FAR				= 0xD2D1,
	DPC_ZOOM					= 0xD2DD,
	DPC_MEDIA_FORMAT			= 0xD2CA,
	DPC_JPEG_QUALITY			= 0xD252,
	DPC_FILE_FORMAT				= 0xD253,
	DPC_FOCUS_MAGNIFY			= 0xD254,
	DPC_USB_POWER_SUPPLY		= 0xD150,
};

// FNumber
// type: CrDataType_UInt16
// value = F number * 100
enum
{
	CrFnumber_IrisClose = 0xFFFD, // Iris Close
	CrFnumber_Unknown   = 0xFFFE, // Display "--"
	CrFnumber_Nothing   = 0xFFFF, // Nothing to display
};

// ExposureBiasCompensation
// type: CrDataType_UInt16
// value: compensation value * 1000

// ShutterSpeed
// type: CrDataType_UInt32
// value: upper two bytes = numerator, lower two bytes = denominator.
enum
{
	CrShutterSpeed_Bulb = 0x00000000,
	CrShutterSpeed_Nothing = 0xFFFFFFFF, // Nothing to display
};

// IsoSensitivity
// type: CrDataType_UInt32
// value: bit 28-31 extension, bit 24-27 ISO mode, bit 0-23 ISO value
enum
{
	CrISO_Normal = 0x00,	// ISO setting Normal
	CrISO_MultiFrameNR = 0x01,	// Multi Frame NR
	CrISO_MultiFrameNR_High = 0x02,	// Multi Frame NR High
	CrISO_Ext = 0x10,	// Indicates of extended value
	CrISO_AUTO = 0xFFFFFF,
};

struct __attribute__((packed)) focalFrame {
	uint16_t type;
	uint16_t state;
	uint8_t priority;
	uint8_t reserved[3];
	uint32_t x_numerator;	// 1024x value
	uint32_t y_numerator;
	uint32_t height;
	uint32_t width;
};

struct __attribute__((packed)) focalFrames {
	uint32_t x_denominator;	// 1024x value
	uint32_t y_denominator;
	uint16_t frameNum;
	uint8_t reserved[6];
	struct focalFrame frames[0];
};

/*
type FocalFrameInfo {
	+0		Version(2)		// Data version (100x value)
	+2		reserved(6)
	+8		reserved(8+24)
	+40		reserved Frame(16+24*0)

	+56		FocusFrame:
			FaceFrames:		// Version 1.01 or later
			TrackingFrames:	// Version 1.01 or later
			FramingFrames:	// Version 1.03 or later
};
*/

enum FocusFrameType
{
  PhaseDetection_AFSensor     = 0x0001,
  PhaseDetection_ImageSensor  = 0x0002,
  Wide                        = 0x0003,
  Zone                        = 0x0004,
  CentralEmphasis             = 0x0005,
  ContrastFlexibleMain        = 0x0006,
  ContrastFlexibleAssist      = 0x0007,
  Contrast                    = 0x0008,
  ContrastUpperHalf           = 0x0009,
  ContrastLowerHalf           = 0x000A,
  DualAFMain                  = 0x000B,
  DualAFAssist                = 0x000C,
  NonDualAFMain               = 0x000D,
  NonDualAFAssist             = 0x000E,
  FrameSomewhere              = 0x000F,
  Cross                       = 0x0010,
};
enum FocusFrameState
{
  NotFocused          = 0x0001,
  Focused             = 0x0002,
  FocusFrameSelection = 0x0003,
  Moving              = 0x0004,
  RangeLimit          = 0x0005,
  RegistrationAF      = 0x0006,
  Island              = 0x0007,
};
enum FaceFrameType
{
  DetectedFace              = 0x0001,
  AF_TargetFace             = 0x0002,
  PersonalRecognitionFace   = 0x0003,
  SmileDetectionFace        = 0x0004,
  SelectedFace              = 0x0005,
  AF_TargetSelectionFace    = 0x0006,
  SmileDetectionSelectFace  = 0x0007,
};
/*
enum FaceFrameState
{
  NotFocused  = 0x0001,
  Focused     = 0x0002,
};
*/
enum SelectionState
{
  Unselected  = 0x01,
  Selected    = 0x02,
};
enum TrackingFrameType
{
  NonTargetAF  = 0x0001,
  TargetAF     = 0x0002,
};
/*
enum TrackingFrameState
{
  NotFocused  = 0x0001,
  Focused     = 0x0002,
};
*/
enum FramingFrameType
{
  Auto                = 0x0001,
  None                = 0x0002,
  Single              = 0x0003,
  reserved4           = 0x0004,
  PTZ                 = 0x0005,
  reserved6           = 0x0006,
  reserved7           = 0x0007,
  HoldCurrentPosition = 0x0008,
  ForceZoomOut        = 0x0009,
};

#endif // PTPTDEF
