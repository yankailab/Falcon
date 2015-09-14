//************************************************************************************
// general config parameter access routines
//************************************************************************************
// types of config parameters
/*
enum confType { BOOL, INT8, INT16, INT32, UINT8, UINT16, UINT32 };

#define CONFIGNAME_MAX_LEN	10
typedef struct configDef
{
	char		name[CONFIGNAME_MAX_LEN];	// name of config parameter
	confType	type;						// type of config parameters
	void*		address;					// address of config parameter
	void (*updateFunction) (void);			// function is called when parameter update happens
}
t_configDef;

t_configDef configDef;

// access decriptor as arry of bytes as well
typedef union
{
	t_configDef c;
	char		bytes[sizeof(t_configDef)];
} t_configUnion;

t_configUnion	configUnion;
*/
//
// list of all config parameters
// to be accessed by par command
//
// descriptor is stored in PROGMEN to preserve RAM space
// see http://www.arduino.cc/en/Reference/PROGMEM
// and http://jeelabs.org/2011/05/23/saving-ram-space/
/*
const t_configDef PROGMEM	configListPGM[] = 
{ 
	{ "vers", UINT8, &config.vers, NULL }, 
	{ "accTimeConstant", INT16, &config.accTimeConstant, NULL }, 
	{ "mpuLPF", INT8, &config.mpuLPF, NULL },
	{ "angleOffsetPitch", INT16, &config.angleOffsetPitch, NULL },
	{ "angleOffsetRoll", INT16, &config.angleOffsetRoll, NULL },
	{ "accOutput", BOOL, &config.accOutput, NULL }, 
	{ "enableGyro", BOOL, &config.enableGyro, NULL }, 
	{ "enableACC", BOOL, &config.enableACC, NULL }, 
	{ "bReverseZ", BOOL, &config.axisReverseZ, NULL },
	{ "bSwapXY", BOOL, &config.axisSwapXY, NULL },
    { NULL, BOOL, NULL, NULL } // terminating NULL required !!
};

// read bytes from program memory
void getPGMstring(PGM_P s, char* d, int numBytes)
{
	char	c;
	for (int i = 0; i < numBytes; i++)
	{
		*d++ = pgm_read_byte(s++);
	}
}

// find Config Definition for named parameter
t_configDef* getConfigDef(char* name)
{
	void*			addr = NULL;
	bool			found = false;
	t_configDef*	p = (t_configDef*)configListPGM;

	while (true)
	{
		getPGMstring((PGM_P) p, configUnion.bytes, sizeof(configDef));	// read structure from program memory
		if (configUnion.c.address == NULL)
		{
			break;
		}

		if (strncmp(configUnion.c.name, name, CONFIGNAME_MAX_LEN) == 0)
		{
			addr = configUnion.c.address;
			found = true;
			break;
		}

		p++;
	}

	if (found)
	{
		return &configUnion.c;
	}
	else
	{
		return NULL;
	}
}

// print single parameter value
void printConfig(t_configDef* def)
{
	if (def != NULL)
	{
		g_pSerial->print(def->name);
		g_pSerial->print(F(" "));
		switch (def->type)
		{
			case BOOL:		g_pSerial->print(*(bool *) (def->address)); break;
			case UINT8:		g_pSerial->print(*(uint8_t *) (def->address)); break;
			case UINT16:	g_pSerial->print(*(uint16_t *) (def->address)); break;
			case UINT32:	g_pSerial->print(*(uint32_t *) (def->address)); break;
			case INT8:		g_pSerial->print(*(int8_t *) (def->address)); break;
			case INT16:		g_pSerial->print(*(int16_t *) (def->address)); break;
			case INT32:		g_pSerial->print(*(int32_t *) (def->address)); break;
		}

		g_pSerial->println("");
	}
	else
	{
		g_pSerial->println(F("ERROR: illegal parameter"));
	}
}

// write single parameter with value
void writeConfig(t_configDef* def, int32_t val)
{
	if (def != NULL)
	{
		switch (def->type)
		{
			case BOOL:		*(bool *) (def->address) = val; break;
			case UINT8:		*(uint8_t *) (def->address) = val; break;
			case UINT16:	*(uint16_t *) (def->address) = val; break;
			case UINT32:	*(uint32_t *) (def->address) = val; break;
			case INT8:		*(int8_t *) (def->address) = val; break;
			case INT16:		*(int16_t *) (def->address) = val; break;
			case INT32:		*(int32_t *) (def->address) = val; break;
		}

		// call update function
		if (def->updateFunction != NULL)
		{
			def->updateFunction();
		}
	}
	else
	{
		g_pSerial->println(F("ERROR: illegal parameter"));
	}
}

// print all parameters
void printConfigAll(t_configDef* p)
{
	while (true)
	{
		getPGMstring((PGM_P) p, configUnion.bytes, sizeof(configDef));	// read structure from program memory
		if (configUnion.c.address == NULL)
		{
			break;
		}

		printConfig(&configUnion.c);
		p++;
	}

	g_pSerial->println(F("done."));
}

//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>

//*****************************************************************************
void parameterMod()
{
	char*	paraName = NULL;
	char*	paraValue = NULL;

	int32_t val = 0;

	if ((paraName = sCmd.next()) == NULL)
	{
		// no command parameter, print all config parameters
		printConfigAll((t_configDef*)configListPGM);
	}
	else
	if ((paraValue = sCmd.next()) == NULL)
	{
		// one parameter, print single parameter
		printConfig(getConfigDef(paraName));
	}
	else
	{
		// two parameters, set specified parameter
		val = atol(paraValue);
		writeConfig(getConfigDef(paraName), val);
	}
}
*/
//************************************************************************************
void setDefaultParametersAndUpdate()
{
	setDefaultParameters();
}

/* */
void writeEEPROM()
{
	EEPROM_writeAnything(0, config);
}

/* */
void readEEPROM()
{
	EEPROM_readAnything(0, config);
}

/* */
void transmitActiveConfig()
{
	g_pHostSerial->println(config.vers);
	g_pHostSerial->println(1);	//config.gyroPitchKp);
}

/* */
void gyroRecalibrate()
{
	g_pHostSerial->println(F("recalibration: done"));
}

/* */
void unrecognized(const char* command)
{
	g_pHostSerial->println(F("What?"));
}

/* */
void gyroCalibration()
{
}

/* */
void setDeadzone()
{
	int channel = atoi(sCmd.next());
	int deadzone = atoi(sCmd.next());

	if(channel > 5)return;
	if(deadzone > 100)return;

	config.controlChannel[channel].deadzone = deadzone;

}

/* */
void setFactor()
{
	int channel = atoi(sCmd.next());
	int factor = atoi(sCmd.next());

	if(channel > 5)return;
	if(factor > 100)return;

	config.controlChannel[channel].factor = factor;
}

/* */
void setPrintIMU()
{
	int bSwitch = atoi(sCmd.next());
	g_bPrintIMU = (bSwitch==1)?true:false;

//	tone(BUZZER_PIN, 900, 500);
}

/* */
void setSerialBridge()
{
	g_opeMode = OPE_SERIAL_BRIDGE;
}

/* */
void setSerialProtocol()
{
	// Setup callbacks for SerialCommand commands
	sCmd.addCommand("sd", setDefaultParametersAndUpdate);
	sCmd.addCommand("we", writeEEPROM);
	sCmd.addCommand("re", readEEPROM);
//	sCmd.addCommand("par", parameterMod);
	sCmd.addCommand("gc", gyroRecalibrate);
	sCmd.addCommand("tc", transmitActiveConfig);
	sCmd.addCommand("kgc", gyroCalibration);
	sCmd.addCommand("sdz", setDeadzone);
	sCmd.addCommand("sfc", setFactor);
	sCmd.addCommand("spi", setPrintIMU);
	sCmd.addCommand("ssb", setSerialBridge);

	sCmd.setDefaultHandler(unrecognized);	// Handler for command that isn't matched  (says "What?")
}


