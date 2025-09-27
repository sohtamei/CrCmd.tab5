#include <cstdio>
#include "PTPDef.h"

/*
double Round(double value, int figure)
{
    bool isNegative = ( value < 0 );
    if (isNegative == true) value = -value;
    double rate = pow(10.0, figure);
    long tmp = (long)(value * rate + 0.5);
    value = tmp/rate;
    if (isNegative == true) value = -value;
    return value;
}
*/
void format_f_number(char* linebuf, uint32_t f_number)
{
    if ((0x0000 == f_number) || (CrFnumber_Unknown == f_number)) {
        sprintf(linebuf, "--");
    } else if(CrFnumber_Nothing == f_number) {

    } else {
        auto modValue = static_cast<uint16_t>(f_number % 100);
        if (modValue > 0) {
          //sprintf(linebuf, "F%.1f", Round((f_number / 100.0), 1));
            sprintf(linebuf, "F%.1f", f_number / 100.0);
        }
        else {
            sprintf(linebuf, "F%ld", f_number / 100);
        }
    }
}

void format_iso_sensitivity(char* linebuf, uint32_t iso)
{
//	uint32_t iso_ext = (iso >> 24) & 0x000000F0;  // bit 28-31
//	uint32_t iso_mode = (iso >> 24) & 0x0000000F; // bit 24-27
	uint32_t iso_value = (iso & 0x00FFFFFF);	  // bit  0-23
/*
	if (iso_mode == CrISO_MultiFrameNR) {
		ss << "MultiFrameNR_";
	}
	else if (iso_mode == CrISO_MultiFrameNR_High) {
		ss << "MultiFrameNR_High_";
	}
*/
	if (iso_value == CrISO_AUTO) {
		sprintf(linebuf, "ISO-AUTO");
	}
	else {
		sprintf(linebuf, "ISO-%ld", iso_value);
	}

	//if (iso_ext == CrISO_Ext) {
	//	ss << " (EXT)";
	//}
}

void format_shutter_speed(char* linebuf, uint32_t shutter_speed)
{
    uint16_t numerator   = static_cast<uint16_t>((shutter_speed >> 16) & 0xFFFF);
    uint16_t denominator = static_cast<uint16_t>(shutter_speed & 0xFFFF);

    if (0 == shutter_speed) {
        sprintf(linebuf, "Bulb");
    } else if (0 == denominator) {
        sprintf(linebuf, "error");
    } else if (1 == numerator) {
        sprintf(linebuf, "%d/%d", numerator, denominator);
    } else if (0 == numerator % denominator) {
        sprintf(linebuf, "%d\"", numerator / denominator);
    } else {
        int32_t numdivision = numerator / denominator;
        int32_t numremainder = numerator % denominator;
        sprintf(linebuf, "%ld.%ld", numdivision, numremainder);
    }
}

struct programMode {
	uint32_t code;
	char str[32];
} static const ProgramModeTable[] = {
	{0x00000001, "M_Manual"},					// Manual (M)
	{0x00010002, "P_Auto"},						// Automatic (P)
	{0x00020003, "A_AperturePriority"},			// Aperture Priority (A)
	{0x00030004, "S_ShutterSpeedPriority"},		// Shutter Priority (S)
	{0x00000005, "Program_Creative"},			// Program Creative (Greater Depth of Field)
	{0x00000006, "Program_Action"},				// Program Action (Faster Shutter Speed)
	{0x00000007, "Portrait"},					// Portrait
	{0x00048000, "Auto"},						// Auto
	{0x00048001, "Auto_Plus"},					// Auto+
	{0x00008008, "P_A"},						// P_A
	{0x00008009, "P_S"},						// P_S
	{0x00058011, "Sports_Action"},				// Sports Action
	{0x00058012, "Sunset"},						// Sunset
	{0x00058013, "Night"},						// Night Scene
	{0x00058014, "Landscape"},					// Landscape
	{0x00058015, "Macro"},						// Macro
	{0x00058016, "HandheldTwilight"},			// Hand-held Twilight
	{0x00058017, "NightPortrait"},				// Night Portrait
	{0x00058018, "AntiMotionBlur"},				// Anti Motion Blur
	{0x00058019, "Pet"},						// Pet
	{0x0005801A, "Gourmet"},					// Gourmet
	{0x0005801B, "Fireworks"},					// Fireworks
	{0x0005801C, "HighSensitivity"},			// High Sensitivity
	{0x00008020, "MemoryRecall"},				// Memory Recall (MR)
	{0x00008030, "ContinuousPriority_AE"},			// Continuous Priority AE
	{0x00008031, "ContinuousPriority_AE_8pics"},	// Tele-Zoom Continuous Priority AE 8pics
	{0x00008032, "ContinuousPriority_AE_10pics"},	// Tele-Zoom Continuous Priority AE 10pics
	{0x00008033, "ContinuousPriority_AE_12pics"},	// Continuous Priority AE12pics
	{0x00068040, "3D_SweepPanorama"},			// 3D Sweep Panorama Shooting
	{0x00068041, "SweepPanorama"},				// Sweep Panorama Shooting
	{0x00078050, "Movie_P"},					// Movie Recording (P)
	{0x00078051, "Movie_A"},					// Movie Recording (A)
	{0x00078052, "Movie_S"},					// Movie Recording (S)
	{0x00078053, "Movie_M"},					// Movie Recording (M)
	{0x00078054, "Movie_Auto"},					// Movie Recording (Auto)
	{0x00098059, "Movie_SQMotion_P"},			// Movie Recording (S&Q Motion (P))
	{0x0009805A, "Movie_SQMotion_A"},			// Movie Recording (S&Q Motion (A))
	{0x0009805B, "Movie_SQMotion_S"},			// Movie Recording (S&Q Motion (S))
	{0x0009805C, "Movie_SQMotion_M"},			// Movie Recording (S&Q Motion (M))
	{0x0009805D, "Movie_SQMotion_AUTO"},		// Movie Recording (S&Q Motion (Auto))
	{0x00008060, "Flash_Off"},					// Flash Off
	{0x00008070, "PictureEffect"},				// Picture Effect
	{0x00088080, "HiFrameRate_P"},				// High Frame Rate (P)
	{0x00088081, "HiFrameRate_A"},				// High Frame Rate (A)
	{0x00088082, "HiFrameRate_S"},				// High Frame Rate (S)
	{0x00088083, "HiFrameRate_M"},				// High Frame Rate (M)
	{0x00008084, "SQMotion_P"},					// S&Q Motion (P)
	{0x00008085, "SQMotion_A"},					// S&Q Motion (A)
	{0x00008086, "SQMotion_S"},					// S&Q Motion (S)
	{0x00008087, "SQMotion_M"},					// S&Q Motion (M)
	{0x000A8088, "MOVIE"},						// Movie
	{0x000A8089, "STILL"},						// Still
	{0x000B808A, "F_MovieOrSQMotion"},			// F (Movie or S&Q)
	{0x00078090, "Movie_F"},					// Movie F Mode
	{0x00098091, "Movie_SQMotion_F"},			// S&Q F Mode
	{0x000C8092, "Movie_IntervalRec_F"},		// Interval REC (Movie) F Mode
	{0x000C8093, "Movie_IntervalRec_P"},		// Interval REC (Movie) (P)
	{0x000C8094, "Movie_IntervalRec_A"},		// Interval REC (Movie) (A)
	{0x000C8095, "Movie_IntervalRec_S"},		// Interval REC (Movie) (S)
	{0x000C8096, "Movie_IntervalRec_M"},		// Interval REC (Movie) (M)
	{0x000C8097, "Movie_IntervalRec_AUTO"},		// Interval REC (Movie) (Auto)
};

void format_exposure_program_mode(char* linebuf, uint32_t exposure_program_mode)
{
	for(int i = 0; i < sizeof(ProgramModeTable)/sizeof(ProgramModeTable[0]); i++) {
		if(ProgramModeTable[i].code == exposure_program_mode) {
			sprintf(linebuf, "%.20s", ProgramModeTable[i].str);
			break;
		}
	}
}
