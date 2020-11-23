/*
 * Copyright (c) 2011-2014, fortiss GmbH.
 * Licensed under the Apache License, Version 2.0.
 *
 * Use, modification and distribution are subject to the terms specified
 * in the accompanying license file LICENSE.txt located at the root directory
 * of this software distribution. A copy is available at
 * http://chromosome.fortiss.org/.
 *
 * This file is part of CHROMOSOME.
 *
 * $Id: askDifferenceFunction.c 7805 2014-03-13 09:54:35Z geisinger $
 */

/**
 * \file
 *         Source file for function askDifference in component system.
 *
 * \author
 *         This file has been generated by the CHROMOSOME Modeling Tool (XMT)
 *         (fortiss GmbH).
 */

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "rOSGateway/adv/system/include/askDifferenceFunction.h"

#include "rOSGateway/adv/system/include/askDifferenceFunctionWrapper.h"
#include "rOSGateway/adv/system/include/systemComponent.h"
#include "rOSGateway/adv/system/include/systemComponentWrapper.h"
#include "rOSGateway/adv/system/include/systemManifest.h"

#include "xme/core/logUtils.h"

// PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_C_INCLUDES) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Definitions                                                          ***/
/******************************************************************************/

// PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_C_DEFINITIONS) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Variables                                                            ***/
/******************************************************************************/
/**
 * \brief  Variable holding the value of the required output port 'getDifference'.
 *
 * \details If necessary initialize this in the init function.
 *          The value of this variable will be written to the port at the end of
 *          the step function.
 */
static ROSGateway_topic_differenceRequest_t
portGetDifferenceData;

// PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_C_VARIABLES) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Prototypes                                                           ***/
/******************************************************************************/

// PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_C_PROTOTYPES) ENABLED START
// PROTECTED REGION END

/******************************************************************************/
/***   Implementation                                                       ***/
/******************************************************************************/
xme_status_t
rOSGateway_adv_system_askDifferenceFunction_init
(
    rOSGateway_adv_system_systemComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_INITIALIZE_C) ENABLED START
    XME_UNUSED_PARAMETER(componentConfig);
    
    return XME_STATUS_SUCCESS;
    // PROTECTED REGION END
}

void
rOSGateway_adv_system_askDifferenceFunction_step
(
    rOSGateway_adv_system_systemComponent_config_t* const componentConfig
)
{
    xme_status_t status[1];
    
    ROSGateway_topic_differenceRequest_t* portGetDifferenceDataPtr = &portGetDifferenceData;
    
    {
        // PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_STEP_C) ENABLED START
        if (componentConfig->requestFinished)
        {
			portGetDifferenceData.a = 7;
			portGetDifferenceData.b = 2;

	        xme_fallback_printf("System: askDifference\n");

	        componentConfig->requestFinished = false;
        }
        else
        {
        	portGetDifferenceDataPtr = NULL;
        }
        // PROTECTED REGION END
    }
    
    status[0] = rOSGateway_adv_system_systemComponentWrapper_writePortGetDifference(portGetDifferenceDataPtr);
    
    {
        // PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_STEP_2_C) ENABLED START
    	if (XME_STATUS_SUCCESS != status[0])
		{
			XME_LOG(XME_LOG_ERROR, "askDifferenceFunction: Error sending request!\n");
		}
		// PROTECTED REGION END
    }
}

void
rOSGateway_adv_system_askDifferenceFunction_fini
(
    rOSGateway_adv_system_systemComponent_config_t* const componentConfig
)
{
    // PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_TERMINATE_C) ENABLED START
    XME_UNUSED_PARAMETER(componentConfig);
    // PROTECTED REGION END
}

// PROTECTED REGION ID(ROSGATEWAY_ADV_SYSTEM_ASKDIFFERENCEFUNCTION_IMPLEMENTATION_C) ENABLED START
// PROTECTED REGION END
